"""G 层的目标点会不会被无声丢掉。

    .venv/bin/python test/aim_hold.py

背景（起伏地形那次实测）：0 s 时 G 给的目标点 (+1.3, −0.8) 在右前 −33°，
狗正前 1 m 是圆柱，L 层选择左绕 +60°；22 s 时目标变成 (+1.8, +0.7) 正前
+16°，它又转 −97° 追回去，再转 +60°。三次转向 23.7 秒，位置只挪 0.5 m。

那一拍里狗只挪了 0.4 m，旧点还在 1.5 m 外，按 `REACHED` 不该换点。根因是
`_by_distance()` 第一行无条件 `self.target = None`：`guide()` 只要有一拍给不出
合格候选（圆柱旁边 `depth` 硬筛很容易把所有格子筛掉），点就丢了，下一拍
`guide()` 又能给点，`_aim` 见 `target is None` 直接换一个 —— 方向翻 90°。

这里把三种情况钉住：
    1. guide 给点 → 一拍 None → guide 又给同一个点：不许换点
    2. guide 一直给不出点、距离模式必须换向：换向时作废（原有行为）
    3. guide 给不出点时四周都堵：站住并作废（原有行为）
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.llm.drive import Driver                             # noqa: E402
from quadruped.percept.types import (                              # noqa: E402
    Percept,
    Ray,
    SECTOR_NAMES,
    SelfState,
)

RANGE_MAX = 3.0


class FakeTrace:
    """按脚本吐目标点：`points` 里 None 表示这一拍 `guide()` 给不出点。"""

    def __init__(self, points, pose=(0.0, 0.0, 0.0)):
        self.points = list(points)
        self.pose = pose
        self.calls = 0

    def guide(self, **_kw):
        self.calls += 1
        return self.points.pop(0) if self.points else None

    def been(self, _name):
        return 0

    def summary(self, *_a, **_kw):
        return ""


class Thinker:
    def __init__(self, p: Percept, trace: FakeTrace):
        self.last_percept = p
        self.percept = SimpleNamespace(trace=trace)
        self.goals: list[tuple[str, object]] = []

    def set_goal(self, text: str, target=None):
        self.goals.append((text, target))


def percept(gaps: dict[str, float] | None = None) -> Percept:
    gaps = gaps or {}
    rays = [Ray(bearing=center, distance=min(gaps.get(name, RANGE_MAX), RANGE_MAX))
            for name, center in SECTOR_NAMES]
    return Percept(stamp=0.0, origin="dummy",
                   ego=SelfState(x=0.0, y=0.0, yaw=0.0),
                   rays=tuple(rays), range_max=RANGE_MAX)


def check(name: str, ok: bool, detail: str) -> bool:
    print(f"  {name}：{detail}  {'✓' if ok else '✗'}")
    return ok


def main() -> int:
    ok = True

    # 1. guide 有一拍给不出点，之后又给同一个点 —— 不许换点。
    print("丢一拍再恢复")
    trace = FakeTrace([(1.3, -0.8), None, (1.3, -0.8)])
    t = Thinker(percept(), trace)
    d = Driver(t, verbose=False)
    d.tick()
    first = d.target
    changes_after_first = d.changes
    d.tick()                       # guide 返回 None → 走 _by_distance
    kept = d.target
    d.tick()                       # guide 又给出同一个点
    ok &= check("目标点还在", kept == first, f"{first} → {kept}")
    ok &= check("没有重新下发", d.changes == changes_after_first,
                f"换向次数 {d.changes}（此前 {changes_after_first}）")

    # 2. guide 一直给不出点、且当前方向已经走不通 —— 目标点应当作废。
    print("\n距离模式真的换向")
    trace = FakeTrace([(1.3, -0.8)])
    # 只有正后是通的（3 m），当前方向（右前）只剩 0.5 m，低于 `HOPELESS_GAP`，
    # 所以滞回保不住它，必须换向。
    blocked = {n: 0.5 for n, _ in SECTOR_NAMES}
    blocked["正后"] = 3.0
    t = Thinker(percept(blocked), trace)
    d = Driver(t, verbose=False)
    d.tick()
    before = d.target
    d.tick()
    d.tick()
    ok &= check("目标点作废", d.target is None, f"{before} → {d.target}")
    ok &= check("切到了新方向", d.direction == "正后", f"方向 {d.direction}")

    # 3. 四周都堵 —— 站住，并作废。
    print("\n四周都堵")
    trace = FakeTrace([(1.3, -0.8)])
    t = Thinker(percept({n: 0.5 for n, _ in SECTOR_NAMES}), trace)
    d = Driver(t, verbose=False)
    d.tick()
    d.tick()
    ok &= check("站住", d.direction == "原地", f"方向 {d.direction}")
    ok &= check("目标点作废", d.target is None, f"{d.target}")

    print(f"\n{'通过' if ok else '不通过'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
