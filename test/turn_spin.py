"""转完就走，走够了才许再转 —— 把「原地连转」这条钉死。

    .venv/bin/python test/turn_spin.py

背景：2026-11-15 仿真迷宫第二段日志，12.2 s → 28.4 s：

    朝向  +5° → -41° → -101° → +139° → +67° → +70°
    16 秒转了 330°，转到位 4 次，位置 (+3.5,+0.3) → (+3.3,+0.2)

`vx` 全程 +0.07（`TURN_VX=0.15` 被 `Guard.filter` 减半），所以每一次「转到位」
之后都还在原地；正前又是墙，下一拍接着转。`TURN_COOL = 1.0` 是时间冷却，
0.07 m/s 下等于 7 cm，而且只挡反向 —— 那五次里有四次同向，全被放行。

这里用假感知 + 假模型重放那几拍，断言四件事：

    1. 转到位后位移 0、正前 1.08 m   → 不许转，并且必须给出前进速度
    2. 位移够 0.5 m                  → 放行
    3. 正前 0.5 m（真堵死）          → 放行，不看位移
    4. 目标点在正前方时转 120°        → 收到 90°；目标点在身后时放行

第 1 条最容易写成「拒绝转向」就完事 —— 那会变成站着不动，正是上一版的毛病
（`_hold()` 清 vx）。所以拒绝转向的同时必须补一个前进速度。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.llm import think as think_mod                  # noqa: E402
from quadruped.llm.drive import Driver                        # noqa: E402
from quadruped.llm.think import Thinker                       # noqa: E402
from quadruped.percept.types import (                          # noqa: E402
    Percept,
    Ray,
    SECTOR_NAMES,
    SelfState,
)

RANGE_MAX = 3.0

# 12.2 s 那一拍的真实八方位（日志原文）。
SECS_122 = {"正前": 1.08, "左前": 1.65, "左侧": 2.33, "左后": 0.80,
            "正后": 3.00, "右后": 1.53, "右侧": 1.16, "右前": 1.43}


class FakeMotion:
    gait = "trot"

    def __init__(self):
        self.cmds: list[dict] = []

    def set_command(self, **kw):
        self.cmds.append(dict(kw))
        return {"clamped": {}}

    def set_gait(self, name):
        pass


class FakeModel:
    """固定回一句话。`reply` 就是模型那一轮的原始输出。"""

    load_seconds = 0.0

    def __init__(self, reply: str):
        self.reply = reply

    def generate(self, _prompt, _max_tokens, cancel=None):
        return self.reply

    def close(self):
        pass


def percept(gaps: dict[str, float], *, x=0.0, y=0.0, yaw=0.0) -> Percept:
    rays = [Ray(bearing=center, distance=min(gaps.get(name, RANGE_MAX), RANGE_MAX))
            for name, center in SECTOR_NAMES]
    return Percept(stamp=0.0, origin="dummy",
                   ego=SelfState(x=x, y=y, yaw=yaw),
                   rays=tuple(rays), range_max=RANGE_MAX)


def thinker(reply: str, gaps: dict[str, float], *,
            target=(1.0, 0.2), free_move=0.0, free_age=0.0) -> tuple[Thinker, FakeMotion]:
    motion = FakeMotion()
    t = Thinker(motion, percept(gaps), verbose=True)
    t._model = FakeModel(reply)
    t.target = target
    t.goal = "朝目标点走"
    t._free_move = free_move
    t._free_at = time.monotonic() - free_age
    return t, motion


def check(name: str, ok: bool, detail: str) -> bool:
    print(f"  {name}：{detail}  {'✓' if ok else '✗'}")
    return ok


def reply(yaw: float) -> str:
    return ('{"thought": "测试", "command": {"vyaw": %g, "vx": 0.8}}' % yaw)


def main() -> int:
    ok = True

    # 0. 正前 3 m 空旷，模型要转 60°「探索」—— 不许，直接走。
    print("前面能走就别转")
    open_all = {n: 3.0 for n, _ in SECTOR_NAMES}
    t, motion = thinker(reply(60), open_all, target=(2.0, 0.0),
                        free_move=9.9, free_age=9.9)
    t._think(t.percept, None, "前方变近", 1.0)
    ok &= check("没有转向", t._turn is None, f"_turn={t._turn}")
    vx = motion.cmds[-1].get("vx") if motion.cmds else None
    ok &= check("在走", vx is not None and vx > 0, f"vx={vx}")

    # 0b. 正前 1.9 m，但目标点在右前 -31°（贴着墙，G 要它回走廊中间）—— 放行。
    #
    # 这条是补的：只看「正前能不能走」会把「贴墙时该横移」也拦掉。2026-11-15
    # 第二段日志，狗贴着北墙走，G 的目标点一直偏右 23°~51°，13 秒只挪 1.3 m。
    print("\n目标点在侧面，正前能走也要转")
    side = dict(SECS_122, 正前=1.90)
    t, _ = thinker(reply(-30), side, target=(1.0, -0.60),
                   free_move=9.9, free_age=9.9)
    t._think(t.percept, None, "目标更新", 1.0)
    ok &= check("允许转向", t._turn is not None, f"_turn={t._turn}")

    # 0c. 同样正前 1.9 m，目标点在正前 +5° —— 拦。
    t, _ = thinker(reply(-30), side, target=(2.0, 0.2),
                   free_move=9.9, free_age=9.9)
    t._think(t.percept, None, "目标更新", 1.0)
    ok &= check("目标在正前，仍然不转", t._turn is None, f"_turn={t._turn}")

    # 1. 刚转完、一步没走、正前 1.40 m（走得动）—— 不许转，但必须给前进速度。
    print("\n转完没走就要再转")
    t, motion = thinker(reply(-45), dict(SECS_122, 正前=1.40), target=(1.0, 0.2),
                        free_move=0.0, free_age=0.5)
    t._think(t.percept, None, "转到位", 1.0)
    ok &= check("没有开始转向", t._turn is None, f"_turn={t._turn}")
    vx = motion.cmds[-1].get("vx") if motion.cmds else None
    ok &= check("补了前进速度", vx is not None and vx >= think_mod.SPIN_VX,
                f"最后一拍 vx={vx}")
    ok &= check("记下了被拦的原因（下一轮告诉模型）", bool(t._blocked),
                t._blocked or "空")

    # 1b. 正前 1.0 m（安全闸压着，走不快），刚转过 —— 放行。
    #
    # 逼一条走不动的路「先走出 0.5 m」只会变成「挡到 4 秒超时再放行」，中间
    # 模型每一拍都重新请求一次同样的转向。175 秒里被挡 54 次就是这么来的。
    print("\n正前 1.0 m 走不动，允许选方向")
    t, _ = thinker(reply(-45), dict(SECS_122, 正前=1.00), target=(1.0, 0.2),
                   free_move=0.0, free_age=0.5)
    t._think(t.percept, None, "转到位", 1.0)
    ok &= check("允许转向", t._turn is not None, f"_turn={t._turn}")

    # 2. 走够了 0.5 m —— 放行。
    print("\n走够 0.5 m 之后")
    t, _ = thinker(reply(-45), SECS_122, target=(1.0, 0.2),
                   free_move=think_mod.TURN_MOVE + 0.01, free_age=2.0)
    t._think(t.percept, None, "转到位", 1.0)
    ok &= check("开始转向", t._turn is not None, f"_turn={t._turn}")
    if t._turn is not None:
        left = (t._turn["target"]) * 57.2958
        ok &= check("转的是 -45°", -60 < left < -30, f"目标 yaw {left:.0f}°")

    # 3. 正前 0.5 m（低于 TURN_CLEAR_OUT）—— 真堵死，放行。
    print("\n正前 0.5 m，真堵死")
    jam = dict(SECS_122, 正前=0.50)
    t, _ = thinker(reply(60), jam, target=(1.0, 0.2), free_move=0.0, free_age=0.2)
    t._think(t.percept, None, "转到位", 1.0)
    ok &= check("允许转", t._turn is not None, f"_turn={t._turn}")

    # 4a. 目标点在前方，模型要转 120° —— 截断到 90°。
    print("\n单次幅度上限")
    t, _ = thinker(reply(120), open_all, target=(2.0, 0.0),
                   free_move=9.9, free_age=9.9)
    t._think(t.percept, None, "接着想", 1.0)
    if t._turn is not None:
        deg = t._turn["target"] * 57.2958
        ok &= check("收到 90°", 80 < deg < 100, f"目标 yaw {deg:.0f}°")
    else:
        ok &= check("收到 90°", False, "_turn=None")

    # 4b. 目标点在身后 —— 掉头是 G 的要求，放行 120°。
    t, _ = thinker(reply(120), open_all, target=(-2.0, 0.0),
                   free_move=9.9, free_age=9.9)
    t._think(t.percept, None, "接着想", 1.0)
    if t._turn is not None:
        deg = t._turn["target"] * 57.2958
        ok &= check("掉头放行 120°", 110 < deg < 130, f"目标 yaw {deg:.0f}°")
    else:
        ok &= check("掉头放行 120°", False, "_turn=None")

    # 5. G 层：追了 16 s 但狗没挪 —— 不换点。
    print("\n目标点超时但狗没动")
    d = Driver(_DriverThinker(), verbose=False)
    d.target = (1.2, 0.0)
    d._aim_at = time.monotonic() - 16.0
    d._aim_best = 1.2
    d._aim_pose = (0.0, 0.0)
    out = d._aim((0.0, 1.5), percept(SECS_122), _Trace((0.0, 0.0, 0.0)))
    ok &= check("保持旧点", out is None and d.target == (1.2, 0.0), f"target={d.target}")

    # 6. 追了 16 s 且狗走了 1.2 m —— 换点。
    d = Driver(_DriverThinker(), verbose=False)
    d.target = (2.4, 0.0)
    d._aim_at = time.monotonic() - 16.0
    d._aim_best = 1.2
    d._aim_pose = (0.0, 0.0)
    d._aim((1.2, 1.5), percept(SECS_122, x=1.2, y=0.0), _Trace((1.2, 0.0, 0.0)))
    ok &= check("换了新点", d.target == (1.2, 1.5), f"target={d.target}")

    # 7. 超时之后，新点方位翻 120° 以上也必须换 —— 否则两条保护互相卡死。
    #
    # 2026-11-15 第三段日志：旧点 (+4.4,+0.8) 追不到（要绕东端缺口），每次巡检
    # 算出的新点都翻 160~179°，被「翻太多就不换」按回去，同一个点连续保持了
    # 14 次 ≈ 140 秒，狗在 row0 里那一小片来回转。
    print("\n超时压过「翻太多不换」")
    d = Driver(_DriverThinker(), verbose=False)
    d.target = (2.4, 0.0)
    d._aim_at = time.monotonic() - 16.0        # 超时
    d._aim_best = 1.2
    d._aim_pose = (0.0, 0.0)
    d._aim((0.6, -0.9), percept(SECS_122, x=1.2, y=0.0), _Trace((1.2, 0.0, 0.0)))
    ok &= check("换了", d.target == (0.6, -0.9), f"target={d.target}")

    # 8. 没超时的时候，翻转抑制还在（这是它本来的用途）。
    d = Driver(_DriverThinker(), verbose=False)
    d.target = (2.4, 0.0)
    d._aim_at = time.monotonic() - 2.0         # 才追了 2 秒
    d._aim_best = 1.2
    d._aim_pose = (0.0, 0.0)
    d._aim((0.6, -0.9), percept(SECS_122, x=1.2, y=0.0), _Trace((1.2, 0.0, 0.0)))
    ok &= check("保持旧的", d.target == (2.4, 0.0), f"target={d.target}")

    print(f"\n{'通过' if ok else '不通过'}")
    return 0 if ok else 1


class _Trace:
    def __init__(self, pose):
        self.pose = pose

    def summary(self, *_a, **_kw):
        return ""


class _DriverThinker:
    """只给 `Driver` 用的最小壳。`_aim` 只读 `set_goal`。"""

    last_percept = None
    percept = None

    def set_goal(self, text, target=None):
        pass


if __name__ == "__main__":
    sys.exit(main())
