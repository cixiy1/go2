"""任务跟踪的离线验证。

两件事：任务要能在提示词里**活下来**（不是只在被下达的那一轮出现一次），
以及任务期间的回话要限流。

2026-11-17 用户的原话：「我发布任务之后，在模型的思考中根本没有去思考我的任务」。
根子是 `build_think(user=...)` 只在那一轮带上用户那句话，下一拍 `user` 是 None，
提示词里一个字都不剩。这里验的就是「补上之后还在不在」。
"""

import sys
import time
from unittest import mock

sys.path.insert(0, ".")

from quadruped.llm import think as T
from quadruped.llm.prompt import build_think

FAILED = []


def check(name: str, ok: bool) -> None:
    print(f"  {'ok  ' if ok else 'FAIL'} {name}")
    if not ok:
        FAILED.append(name)


def make_thinker():
    tk = T.Thinker(motion=object(), percept=None, verbose=False)
    said: list[str] = []
    tk.set_reply_sink(said.append)
    return tk, said


class Ego:
    """`percept.ego` 的最小替身：守位闭环只读 x / y / yaw。"""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x, self.y, self.yaw, self.wz = x, y, yaw, 0.0


class FakePercept:
    """只带里程的假感知。`_odom` 读的就是这两个口（`status` / `pose`）。"""

    def __init__(self) -> None:
        self.travelled = 0.0
        self.xy = (0.0, 0.0)
        self.ego = Ego()

    def status(self) -> dict:
        return {"travelled": self.travelled}

    # **property，不是方法** —— 和 `percept/memory.py` 里的 `Trace.pose` 一致。
    # 写成方法时这套测试会全绿，真机上却因为 `pose()` 抛 TypeError 让进度永远 0.0。
    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.xy[0], self.xy[1], 0.0)


# `_odom` 走的是 `percept.trace.status()/pose()`，假感知把自己当 trace 用
FakePercept.trace = property(lambda self: self)


def main() -> int:
    # ---- 什么算任务 ----
    for text, want in [
        ("过来", True), ("往前走", True), ("调头", True), ("不要动", True),
        ("看着我", True), ("跟着我", True), ("慢一点", True),
        ("你能做什么", False), ("为什么停了", False), ("", False),
    ]:
        check(f"「{text}」{'算' if want else '不算'}任务",
              T.is_command(text) is want)

    # ---- 任务进得了提示词 ----
    msgs = build_think("视野：正前 3.00", task={"text": "过来", "seconds": 12.0})
    payload = msgs[1]["content"]
    check("提示词里有「当前任务」", "当前任务" in payload)
    check("带着任务原话", "过来" in payload)
    check("带着已经过了多久", "12" in payload)
    check("要求先说任务进度", "做到哪一步" in payload)
    check("任务排在触发原因之前", payload.index("当前任务")
          < (payload.index("触发原因") if "触发原因" in payload else 999))

    # 没有任务时不该出现这一段
    msgs = build_think("视野：正前 3.00")
    check("没有任务时不提任务", "当前任务" not in msgs[1]["content"])

    # ---- 任务过期 ----
    tk, _ = make_thinker()
    clock = [1000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = {"text": "过来", "at": 990.0}
        arg = tk._task_arg()
        check("刚接的任务还在", arg is not None and arg["seconds"] == 10.0)

        clock[0] = 990.0 + T.TASK_TIMEOUT + 5.0
        check(f"超过 {T.TASK_TIMEOUT:.0f} 秒自动作废", tk._task_arg() is None)
        check("作废之后状态也清了", tk.task is None)

    # ---- 回话限流 ----
    tk, said = make_thinker()
    clock = [2000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = {"text": "过来", "at": 2000.0}
        tk._reply("第一句")
        tk._reply("第二句")
        check("任务期间 4 秒内只发一条", len(said) == 1 and said[0] == "第一句")

        clock[0] += T.REPLY_GAP + 0.5
        tk._reply("第三句")
        check("隔够了就放行", len(said) == 2 and said[1] == "第三句")

        tk._reply("第四句", force=True)
        check("用户说话那轮不受限流", said[-1] == "第四句")

        # 没有任务时不限流 —— 那种情况本来也不会连着说
        tk.task = None
        tk._reply("第五句")
        check("没有任务时不限流", said[-1] == "第五句")

    # ---- 距离任务：进度必须是量出来的 ----
    for text, want in [("往前走两米", 2.0), ("往前走 2.5 米", 2.5),
                       ("走三米", 3.0), ("走十米", 10.0),
                       ("转头看我", None), ("往前走", None)]:
        check(f"「{text}」里的距离 = {want}", T.distance_of(text) == want)

    fake = FakePercept()
    tk, _ = make_thinker()
    tk.percept = fake
    clock = [3000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = tk._new_task("往前走两米")
        check("新任务记下了想要的米数", tk.task["want"] == 2.0)

        fake.travelled = 1.0
        arg = tk._task_arg()
        payload = build_think("视野：正前 3.00", task=arg)[1]["content"]
        check("进度按里程算，报已走 1.0 米", "已走 1.0 米" in payload)
        check("报还差 1.0 米", "还差 1.0 米" in payload)
        check("没走够时不算完成", arg["done"] is False)

        fake.travelled = 2.1
        fake.xy = (2.0, 0.3)
        arg = tk._task_arg()
        check("走够 2 米就标记完成", arg["done"] is True)
        check("完成时提示词要求停下",
              "vx 给 0" in build_think("视野：正前 3.00", task=arg)[1]["content"])
        check("完成时提示词不再说「还没完成」",
              "还没完成" not in build_think("视野：正前 3.00", task=arg)[1]["content"])
        check("离起点直线距离也算出来了", arg["disp"] is not None
              and abs(arg["disp"] - 2.022) < 0.01)

        # 走够了就撤 —— 但先得让人听见汇报
        tk._settle_task(replied=False)
        check("没汇报就还不撤任务", tk.task is not None)
        tk._settle_task(replied=True)
        check("汇报过就撤任务", tk.task is None)
        check("撤任务后进入罚站窗口", tk._hold_until > 0.0)

    # ---- 超时要说话，不能悄悄作废 ----
    tk, said = make_thinker()
    clock = [4000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = tk._new_task("调头")
        clock[0] = 4000.0 + T.TASK_TIMEOUT + 5.0
        check("超时的任务返回 None", tk._task_arg() is None)
        check("超时要跟人说一句", bool(said) and "没做完" in said[-1])

    # 带距离的任务给更长的时限：按 0.3 m/s 加两个掉头，两米要走 40 秒以上
    tk, _ = make_thinker()
    clock = [5000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = tk._new_task("往前走两米")
        clock[0] = 5000.0 + T.TASK_TIMEOUT + 10.0
        check(f"距离任务 {T.TASK_TIMEOUT:.0f} 秒不作废",
              tk._task_arg() is not None)
        clock[0] = 5000.0 + T.TASK_TIMEOUT_WALK + 5.0
        check(f"到 {T.TASK_TIMEOUT_WALK:.0f} 秒才作废", tk._task_arg() is None)

    # ---- 两类任务：走完就结束 vs 一直保持的状态 ----
    for text, kind in [("看着我", "stay"), ("保持一米距离", "stay"),
                       ("别动", "stay"), ("停下", "stay"),
                       ("跟着我", "stay"), ("一直慢慢走", "stay"),
                       ("往前走两米", "act"), ("调头", "act"),
                       ("往前走两米，走够就停下", "act")]:
        check(f"「{text}」是 {kind}", T.task_kind(text) == kind)

    fake = FakePercept()
    tk, _ = make_thinker()
    tk.percept = fake
    clock = [6000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        # 「保持一米距离」里的 1 米是要维持的间距，不是要走的路程
        tk.task = tk._new_task("保持一米距离")
        check("守状态的任务不取路程", tk.task["want"] is None)
        fake.travelled = 3.0
        arg = tk._task_arg()
        check("守状态的任务走多远都不算完成", arg["done"] is False)
        payload = build_think("视野：正前 3.00", task=arg)[1]["content"]
        check("提示词说明这是要一直保持的状态",
              "一直保持的状态" in payload)
        check("提示词要求一直守着",
              "一直守着" in payload)
        check("提示词禁止自己找方向走", "自己找个方向走" in payload)

        # 时限按守状态算：120 秒、240 秒都不该清
        clock[0] = 6000.0 + T.TASK_TIMEOUT_WALK + 5.0
        check("守状态的任务 240 秒不作废", tk._task_arg() is not None)
        clock[0] = 6000.0 + T.TASK_TIMEOUT_STAY + 5.0
        check(f"到 {T.TASK_TIMEOUT_STAY:.0f} 秒才放开", tk._task_arg() is None)

    # 守状态到时限要说一声，不能安静地解禁然后自己走开
    tk, said = make_thinker()
    clock = [7000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = tk._new_task("别动")
        clock[0] = 7000.0 + T.TASK_TIMEOUT_STAY + 5.0
        tk._task_arg()
        check("守状态到时限要跟人说一句",
              bool(said) and "守了" in said[-1])

    check("「别动」要走执行层硬压速度", T.wants_stop("别动") is True)
    check("「看着我」不用压速度", T.wants_stop("看着我") is False)

    # ---- 守位闭环：漂出去了要往回找，方向不能反 ----
    #
    # 方向反了的话它就越修越远，把「别动」改成比不修还糟。所以这几条必须过：
    # 死区内不动、正前方掉了往前追、机头朝后时往后追、横向漂移按「正 vy 向左」。
    fake = FakePercept()
    tk, _ = make_thinker()
    tk.percept = fake
    clock = [8000.0]
    with mock.patch.object(time, "monotonic", lambda: clock[0]):
        tk.task = tk._new_task("别动")
        tk.task["pos0"] = (0.0, 0.0)

        tk.task["yaw0"] = 0.0
        fake.ego = Ego(0.0, 0.0, 0.0)
        check("死区内不修", tk._hold_fix(fake) == (0.0, 0.0, 0.0))

        fake.ego = Ego(0.0, -0.50, 0.0)     # 横向漂 0.5 米，机头没转
        fix = tk._hold_fix(fake)
        check("横向漂移用 vy 找回来", fix[1] > 0.0)
        check("横向漂移不该混进 vx", abs(fix[0]) < 0.02)

        fake.ego = Ego(-0.50, 0.0, 0.0)     # 沿机头方向掉了 0.5 米
        fix = tk._hold_fix(fake)
        check("正前方掉了往前追", fix[0] > 0.0)
        check("掉在正前方时不横着走", abs(fix[1]) < 0.02)

        fake.ego = Ego(-0.50, 0.0, 3.14159)  # 同一个位置，机头转成朝后
        fix = tk._hold_fix(fake)
        check("机头朝后时改成往后追", fix[0] < 0.0)

        fake.ego = Ego(-3.0, 0.0, 0.0)      # 掉得太远
        check("修正速度不超过上限", abs(tk._hold_fix(fake)[0]) <= T.HOLD_MAX + 1e-9)

        # 朝向也得守：站着不动时机头会慢慢漂，漂着漂着「别动」变成原地转
        fake.ego = Ego(0.0, 0.0, 0.30)      # 偏了 17°
        check("朝向漂了要转回来", tk._hold_fix(fake)[2] < 0.0)
        fake.ego = Ego(0.0, 0.0, 0.0)
        check("朝向在死区内不转", tk._hold_fix(fake)[2] == 0.0)
        tk.task["yaw0"] = 1.0
        fake.ego = Ego(0.0, 0.0, 1.5)       # 多转了 29°，该往回（负）转
        check("多转了就往回转", tk._hold_fix(fake)[2] < 0.0)
        # 跨 ±π 的短路：锚点 -3.0、现在 +3.0，差 0.28 rad，不该绕 6 弧度
        tk.task["yaw0"] = -3.0
        fake.ego = Ego(0.0, 0.0, 3.0)
        fix = tk._hold_fix(fake)
        check("跨 ±π 走短的一边", 0.0 < fix[2] <= T.HOLD_YAW_MAX)
        check("角速度不超过上限", abs(fix[2]) <= T.HOLD_YAW_MAX + 1e-9)

    print()
    if FAILED:
        print(f"{len(FAILED)} 项失败：")
        for name in FAILED:
            print(f"  - {name}")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
