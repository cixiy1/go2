"""扫视找人的离线验证。

真机上人喊「过来」之后，狗会原地一档一档转，转到视觉说「人：有」就停。
这里用假时钟把那个过程跑一遍，看它是不是真的停在人的方向上、转满一圈
是不是真的会放弃。

不接 MuJoCo、不连 DDS、不调模型 —— 只验状态机。
"""

import math
import sys
import time
import types
from unittest import mock

sys.path.insert(0, ".")

from quadruped.llm import think as T

# `TURN_TICK` 是**周期**（0.03 秒），不是频率。写成 1/TURN_TICK 会变成一帧
# 33 秒，yaw 每帧绕地球一圈，状态机看着像死循环。
DT = T.TURN_TICK
# 假视觉的视场。用 `think.SCAN_FOV`（58°）而不是随便写个数 —— 三分块、块中心
# 距离都从它算，测试里另写一个值会让「居中一次够不够」这类断言失去意义。
FOV = T.SCAN_FOV
# 视觉模型的出帧周期。`vla --vlm-period` 默认 3 s，判定只有等来这一帧才可能准确 ——
# 假视觉要是「每一拍都新鲜」，就测不出「拿旧帧判方向」那个问题。
VLM_PERIOD = 3.0


class Clock:
    """替掉 `time.perf_counter`。扫描器用它算「停稳等了多久」。"""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeMotion:
    def __init__(self):
        self.cmd: dict = {}
        self.calls = 0
        self.all: list[dict] = []

    def set_command(self, **kw):
        self.cmd = kw
        self.calls += 1
        self.all.append(kw)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def ideal_turn(err: float, wz: float) -> float | None:
    """理想转向：有误差就给固定角速度，进了容差就停。

    真实的 `turn_command` 在这里不能用 —— 它带提前制动和阻尼项，是给有惯性、
    有执行延迟的实体写的；喂进这个零惯性的假机体里会来回震荡，永远进不了
    「角度到位 + 角速度很低」那个双条件。转向控制律本身由
    `test/turn_overshoot.py` 在 MuJoCo 上验，这个文件只验状态机。
    """
    if abs(err) <= T.TURN_TOL:
        return None
    return math.copysign(2.0, err)


def make_thinker():
    motion = FakeMotion()
    tk = T.Thinker(motion=motion, percept=None, verbose=False)
    said: list[str] = []
    tk.set_reply_sink(said.append)
    return tk, motion, said


def scene_for(yaw: float, person_yaw: float | None) -> str:
    """假视觉描述。人被分到左/中/右哪一块，看当前朝向差多少度。

    和 `vlm.QUESTION` 的第四行一致：画面三等分，块宽 FOV/3，所以
    |偏差| <= FOV/6 算「中」，左半边算「左」，右半边算「右」，出画算「没有」。
    """
    body = "左：空地，能，远(3米外)\n中：空地，能，远(3米外)\n右：空地，能，远(3米外)"
    if person_yaw is None:
        return body + "\n人：没有"
    rel = math.degrees(wrap(person_yaw - yaw))
    if abs(rel) > FOV / 2:
        return body + "\n人：没有"
    third = "中" if abs(rel) <= FOV / 6 else ("左" if rel > 0 else "右")
    return body + f"\n人：{third}"


def run(person_deg: float | None, limit: int = 6000, front: float = 3.0):
    """跑一遍扫描。

    `person_deg` 是人相对起点的方位（度），None = 没有人。
    `front` 是正前方障碍距离 —— 看到人之后要不要说「我朝你走」由它决定。
    """
    tk, motion, said = make_thinker()
    clock = Clock()
    p = types.SimpleNamespace(
        ego=types.SimpleNamespace(yaw=0.0, wz=0.0),
        scene="",
        scene_age=0.02,
        front_gap=lambda: front,
    )
    person_yaw = None if person_deg is None else math.radians(person_deg)
    tk._scan = {
        "kind": "find",
        "mode": "sweep",
        "legs": 0,
        "report_leg": T.SCAN_REPORT_LEGS,
        "target": p.ego.yaw + math.radians(T.SCAN_STEP),
        "phase": "hold",
        "at": clock.t,
        "still_at": clock.t,
        "centering": 0,
        "look_back": False,
        "turn_at": 0.0,
        "aim": None,
    }

    vyaw = 0.0
    steps = 0
    done = False
    swept = 0.0
    frame_at = clock.t - VLM_PERIOD      # 一开始就有一帧
    frame_scene = scene_for(p.ego.yaw, person_yaw)
    with mock.patch.object(time, "perf_counter", clock), \
         mock.patch.object(T, "turn_command", ideal_turn):
        for steps in range(limit):
            if clock.t - frame_at >= VLM_PERIOD:
                frame_at = clock.t
                frame_scene = scene_for(p.ego.yaw, person_yaw)
            p.scene = frame_scene
            p.scene_age = clock.t - frame_at
            tk._run_scan(p)
            if tk._scan is None:
                done = True
                break
            swept = tk._scan["legs"] * T.SCAN_STEP
            vyaw = float(motion.cmd.get("vyaw") or 0.0)
            clock.t += DT
            p.ego.yaw = wrap(p.ego.yaw + vyaw * DT)
            p.ego.wz = vyaw
    if not done:
        raise SystemExit(f"扫描没在 {limit} 拍内结束 —— 状态机卡住了")

    return {
        "yaw": p.ego.yaw,
        "swept": swept,
        "steps": steps,
        "said": said,
        "motion": motion,
        "seconds": steps * DT,
    }


def run_look(seconds: float, leave_at: float | None = None,
             drift_at: float | None = None, drift_deg: float = 0.0,
             person_deg: float | None = 0.0, with_task: bool = False):
    """「看着我」：找到之后进 watch，人走出画面就重扫，超时自己收尾。

    `leave_at` 秒之后把人挪出画面（模拟他走开）。
    """
    tk, motion, said = make_thinker()
    clock = Clock()
    p = types.SimpleNamespace(
        ego=types.SimpleNamespace(yaw=0.0, wz=0.0),
        scene="",
        scene_age=0.02,
        front_gap=lambda: 3.0,
    )
    tk._scan = {
        "kind": "look",
        "mode": "sweep",
        "legs": 0,
        "report_leg": T.SCAN_REPORT_LEGS,
        "target": p.ego.yaw + math.radians(T.SCAN_STEP),
        "phase": "hold",
        "at": clock.t,
        "still_at": clock.t,
        "centering": 0,
        "look_back": False,
        "turn_at": 0.0,
        "aim": None,
    }
    t0 = clock.t
    modes: list[str] = []
    vyaw = 0.0
    steps = int(seconds / DT)
    person0 = None if person_deg is None else math.radians(person_deg)
    frame_at = clock.t - VLM_PERIOD
    frame_scene = scene_for(p.ego.yaw, person0)
    logs: list[str] = []
    tk._log = logs.append
    if with_task:
        # 「看着我」现在是一条守状态任务，得把它放进去才测得到「找不到人时撤任务」。
        tk.task = {"text": "看着我", "kind": "stay", "at": clock.t,
                   "pos0": None, "walked0": 0.0, "want": None, "done": False}
    with mock.patch.object(time, "perf_counter", clock), \
         mock.patch.object(T, "turn_command", ideal_turn):
        for _ in range(steps):
            gone = leave_at is not None and (clock.t - t0) >= leave_at
            person = person0
            if drift_at is not None and (clock.t - t0) >= drift_at:
                person = math.radians(drift_deg)
            if clock.t - frame_at >= VLM_PERIOD:
                frame_at = clock.t
                frame_scene = scene_for(p.ego.yaw, None if gone else person)
            p.scene = frame_scene
            p.scene_age = clock.t - frame_at
            tk._run_scan(p)
            if tk._scan is not None:
                modes.append(tk._scan["mode"])
            vyaw = float(motion.cmd.get("vyaw") or 0.0)
            clock.t += DT
            p.ego.yaw = wrap(p.ego.yaw + vyaw * DT)
            p.ego.wz = vyaw
    return {"said": said, "modes": modes, "scan": tk._scan, "logs": logs,
            "motion": motion, "elapsed": clock.t - t0, "ego": p.ego,
            "task": tk.task}


FAILED = []


def check(name: str, ok: bool) -> None:
    print(f"  {'ok  ' if ok else 'FAIL'} {name}")
    if not ok:
        FAILED.append(name)


def main() -> int:
    # ---- 关键词 ----
    for text, want in [
        ("找到我", True), ("你找我", True), ("看着我", True),
        ("过来", True), ("跟着我", True), ("你在哪", True),
        ("转过来", False), ("别过来", False), ("往前走", False), ("不要动", False),
    ]:
        check(f"「{text}」{'要' if want else '不要'}扫描", T.wants_scan(text) is want)

    # ---- 人的判定 ----
    check("人：有 认得出", T._person_in("左：空地\n人：有"))
    check("人：没有 不算有", not T._person_in("左：空地\n人：没有"))
    check("没有第四行不算有", not T._person_in("左：空地，能，远(3米外)"))

    # ---- 人在正右方（+90°）：第三档就该停 ----
    r = run(90.0)
    deg = math.degrees(r["yaw"])
    # 偏到人还在画面里就算看到，不必正对 —— 假视觉的 FOV 是 20°。
    # 居中之后误差应该落在一个「中块」之内（FOV/6 ≈ 9.7°），不是原来那个
    # 「在画面里就算」的 ±29°，更不是整档 30°。
    check(f"看到人并居中（朝向 {deg:+.1f}°，人的方向 +90，容差 ±{FOV / 6:.0f}°）",
          abs(deg - 90.0) <= FOV / 6 + 1.0)
    check("说「看到你了」", any("看到你" in x for x in r["said"]))
    check("扫了约 90°（不是一圈）", 60.0 <= deg <= 110.0)

    # ---- 人在背后（180°）：转半圈 ----
    r = run(180.0)
    deg = math.degrees(r["yaw"])
    check(f"背后的人也能看到（朝向 {deg:+.0f}°，人的方向 +180）",
          abs(abs(deg) - 180.0) <= FOV)

    # ---- 人就在正前方（0°）：第一档之前就该看见 ----
    r = run(0.0)
    check("正前方的人第一拍就看到，不用转",
          any("看到你" in x for x in r["said"]) and abs(math.degrees(r["yaw"])) <= 12.0)

    # ---- 没有人：转满一圈放弃 ----
    r = run(None)
    check("没人时说「转了一圈」", any("一圈" in x for x in r["said"]))
    # 不能拿最终 yaw 判：转满一圈之后 yaw 回到起点附近，看着像没转。
    check(f"扫满约 360°（累计 {r['swept']:.0f}°）", r["swept"] >= 330.0)
    check(f"耗时可接受（{r['seconds']:.0f} s，上限 45 s）", r["seconds"] <= 45.0)
    # 24 秒一句话没有，屏幕上看着就是死机。每转 90° 要报一次。
    joined = " | ".join(r["said"])
    check("每转 90° 报一次进度（90/180/270）",
          all(f"转了 {d} 度" in joined for d in (90, 180, 270)))

    # ---- 看到但过不去：不许说「我朝你走」 ----
    #
    # 真机上出过：正前 0.78 m 是花池，狗一边说「我朝你那边走」一边被安全闸
    # 压成 vx=-0.25 往后退。人站在那儿看着它就是说一套做一套。
    r = run(90.0, front=0.7)
    check("正前 0.7 m 时不说「朝你走」", not any("朝你那边走" in x for x in r["said"]))
    check("正前 0.7 m 时说明有东西挡着",
          any("挡着" in x for x in r["said"]))
    r = run(90.0, front=2.5)
    check("正前 2.5 m 时说「朝你那边走」", any("朝你那边走" in x for x in r["said"]))
    check("正前够宽时真的发了前进命令",
          abs(float(r["motion"].all[-1].get("vx") or 0.0)) > 0.1)

    # ---- 「看着我」是持续任务 ----
    r = run_look(30.0)
    check("看到之后进入 watch（不是结束）", r["scan"] is not None
          and r["scan"]["mode"] == "watch")
    check("说过「看着你」", any("看着你" in x for x in r["said"]))
    check("盯了 30 秒还会主动报「还在看着你」",
          sum("还在看着你" in x for x in r["said"]) >= 1)
    check("看着的时候一直不动",
          all(abs(float(c.get("vx") or 0.0)) < 1e-9 for c in r["motion"].all))

    # 人走开之后要自己重新找，不是站着盯空处
    r = run_look(30.0, leave_at=8.0)
    check("人走出画面后先回头找", any("回头找一下" in x for x in r["said"]))
    check("回头也没有才扫一圈", any("再扫一圈" in x for x in r["said"]))
    check("重扫时 mode 回到 sweep", "sweep" in r["modes"])

    # 超时自己收尾，不无限占着执行层
    r = run_look(T.SCAN_WATCH + 12.0)
    check(f"{T.SCAN_WATCH:.0f} 秒后自己收尾", r["scan"] is None
          and any("看了这么久" in x for x in r["said"]))

    # ---- 扫视全程不带前进量 ----
    #
    # 人可能就站在半米外，转的时候带 0.25 前进量（普通转向的 `TURN_VX`）会
    # 一边转一边朝他蹭过去。这一条用没人那个场景的长序列验，命令最多。
    r = run(None)
    bad = [c for c in r["motion"].all if abs(float(c.get("vx") or 0.0)) > 1e-9]
    check(f"扫视时 vx 一直是 0（{len(r['motion'].all)} 条命令，越界 {len(bad)} 条）",
          not bad)

    # ---- 陈旧帧不能当「现在看到」 ----
    #
    # 视觉 3 秒一帧，扫视却在转。一帧拍在朝向 +90°，狗转回 0° 之后这帧还说
    # 「人：中」，拿它判就会在错误的方向上「看到人」。
    tk, motion, said = make_thinker()
    clock = Clock()
    p = types.SimpleNamespace(ego=types.SimpleNamespace(yaw=0.0, wz=0.0),
                              scene=scene_for(math.radians(90), math.radians(90)),
                              scene_age=9.0, front_gap=lambda: 3.0)
    tk._scan = {
        "kind": "look", "mode": "sweep", "legs": 0,
        "report_leg": T.SCAN_REPORT_LEGS,
        "target": math.radians(T.SCAN_STEP), "phase": "hold",
        "at": clock.t, "still_at": clock.t - 8.0,
        "centering": 0, "look_back": False, "turn_at": 0.0, "aim": None,
    }
    with mock.patch.object(time, "perf_counter", clock), \
         mock.patch.object(T, "turn_command", ideal_turn):
        tk._run_scan(p)
    check("9 秒前的帧不算「看到人」", not any("看到你" in x for x in said))
    check("等新帧时不动", abs(float(motion.cmd.get("vx") or 0)) < 1e-9
          and abs(float(motion.cmd.get("vyaw") or 0)) < 1e-9)

    # ---- 跟踪：人偏到侧块就转过去，不重扫 ----
    #
    # 原来人往旁边挪一步就丢，然后从当前朝向扫 12 档、最坏 30 秒才回来。
    r = run_look(26.0, drift_at=8.0, drift_deg=24.0)
    check("人偏到侧块时转头跟（出现 track）", "track" in r["modes"])
    check("跟踪时记了「人偏到左边」",
          any("偏到" in l for l in r["logs"]))
    check("跟踪时没有重扫一圈",
          not any("再扫一圈" in x for x in r["said"]))
    check(f"跟过去之后朝向朝人这边（{math.degrees(r['ego'].yaw):+.0f}°）",
          math.degrees(r["ego"].yaw) >= 12.0)
    check("跟过去之后还在 watch", r["scan"] is not None
          and r["scan"]["mode"] in ("watch", "track"))

    # ---- 转一圈没人：守状态任务要撤掉 ----
    #
    # 不撤的话「看着我」会以 900 秒的时限一直挂着，期间它不许自己探索，
    # 人就看着它站在原地说「我找不到你」。
    r = run_look(50.0, person_deg=None, with_task=True)
    check("转一圈没人时说「转了一圈」", any("转了一圈" in x for x in r["said"]))
    check("找不到人就撤掉「看着我」任务", r["task"] is None)

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
