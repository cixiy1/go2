"""G 层在「没有地图」时的选向：空旷度到底占多大权重。

    .venv/bin/python test/drive_score.py

背景：用户问「他为什么总想去空旷的地方」。有地图时走向由 `memory.guide()` 的
三条硬筛决定；没地图时走 `drive._by_distance`，它原来的分数是 `gap + BIAS` ——
也就是「那个方位有多空旷」。量程 3 m 里越远越加分，于是「身后 3.0 m 空旷」比
「正前 1.5 m 能走」高 1.5 分，比方向偏好（+0.25 对 -0.90，共 1.15）还大。
结果就是正前明明能走，它也转向 45° 去追那片更空的地方。

这里断言的是**性质**，不是某个具体方位名 —— 「正前 1.6 m、左前 2.0 m，该选
哪个」没有唯一正确答案，但「正前够走时不许为了更空旷而转向」是硬要求。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.llm.drive import (                                       # noqa: E402
    BIAS,
    BLOCK_GAP,
    GAP_CAP,
    Driver,
)
from quadruped.percept.types import (                                   # noqa: E402
    Percept,
    Ray,
    SECTOR_NAMES,
    SelfState,
)

RANGE_MAX = 3.0
_HEADING = {name: center for name, center in SECTOR_NAMES}


class Thinker:
    """`Driver` 只用这两个口，外加一个 `percept`（拿 trace，这里给 None）。"""

    percept = None

    def __init__(self, p: Percept):
        self.last_percept = p
        self.goals: list[str] = []

    def set_goal(self, text: str, target=None) -> None:
        self.goals.append(text)


def percept(gaps: dict[str, float]) -> Percept:
    rays = [Ray(bearing=center, distance=min(gaps.get(name, RANGE_MAX), RANGE_MAX))
            for name, center in SECTOR_NAMES]
    return Percept(stamp=0.0, origin="dummy",
                   ego=SelfState(x=0.0, y=0.0, yaw=0.0),
                   rays=tuple(rays), range_max=RANGE_MAX)


def pick(gaps: dict[str, float]) -> str:
    t = Thinker(percept(gaps))
    g = Driver(t, verbose=False)
    g._fresh = False                  # 不是第一拍，走滞回比较
    g.tick()
    return g.direction


def old_pick(gaps: dict[str, float]) -> str:
    """改之前的口径：不封顶。复刻这几行只是为了给出对照。"""
    best, best_score = None, None
    for name, _ in SECTOR_NAMES:
        s = min(gaps.get(name, RANGE_MAX), RANGE_MAX) + BIAS.get(name, 0.0)
        if best_score is None or s > best_score:
            best, best_score = name, s
    return best


def gap_of(gaps: dict[str, float], name: str) -> float:
    return min(gaps.get(name, RANGE_MAX), RANGE_MAX)


def main() -> int:
    print(f"GAP_CAP = {GAP_CAP} m，BLOCK_GAP = {BLOCK_GAP} m\n")
    fails: list[str] = []

    # P1：正前够走时，不许因为别处更空旷而转向。
    #     这是用户看到的那一类行为：正前明明有 1.5~2 m，它还是拐 45° 追空。
    print("P1  正前够走就不许追空旷")
    for front, side in ((1.5, 3.0), (2.0, 3.0), (1.2, 3.0), (2.5, 3.0)):
        gaps = {"正前": front, "左侧": 3.0, "右前": 3.0, "左前": 3.0}
        was, now = old_pick(gaps), pick(gaps)
        ok = now == "正前"
        print(f"     正前 {front:.1f} / 侧向 {side:.1f}：旧 {was} → 新 {now}"
              f"  {'✓' if ok else '✗'}")
        if not ok:
            fails.append(f"P1 正前 {front}")

    # P2：正前被堵时必须换向，而且换到的方向得真的能走（不是把狗塞进 0.6 m）。
    print("\nP2  正前被堵时必须换到一个能走的方向")
    for front, alt in ((0.5, 2.4), (0.6, 1.8), (0.4, 1.2)):
        gaps = {"正前": front, "左侧": alt, "右侧": alt, "左前": alt, "右前": alt}
        now = pick(gaps)
        deg = abs(math.degrees(_HEADING[now]))
        ok = now != "正前" and gap_of(gaps, now) >= min(alt, 1.2) and deg <= 90.0
        print(f"     正前 {front:.1f} / 侧向 {alt:.1f}：选 {now}"
              f"（{deg:.0f}°，实际 {gap_of(gaps, now):.1f} m）"
              f"  {'✓' if ok else '✗'}")
        if not ok:
            fails.append(f"P2 正前 {front}")

    # P3：全开阔。八个方向一样空，不许产生梯度，回方向偏好（正前）。
    print("\nP3  全开阔不分高下")
    flat: dict[str, float] = {}
    was, now = old_pick(flat), pick(flat)
    print(f"     旧 {was} → 新 {now}  {'✓' if now == '正前' else '✗'}")
    if now != "正前":
        fails.append("P3 全开阔")

    # P4：四面八方都堵死时站住，不能被「哪边稍微空一点」带着顶墙。
    print("\nP4  全堵就站住")
    tight = {name: 0.5 for name, _ in SECTOR_NAMES}
    now = pick(tight)
    print(f"     八个方向 0.5 m：选 {now}  {'✓' if now == '原地' else '✗'}")
    if now != "原地":
        fails.append("P4 全堵")

    print(f"\n{'通过' if not fails else '不通过：' + '、'.join(fails)}")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
