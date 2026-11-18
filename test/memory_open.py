"""L 层的「记忆」里到底有没有「这条路我没走过」。

    .venv/bin/python test/memory_open.py

背景：2026-11-15 迷宫实测，狗在起步不久经过一个路口没进去，走了十几米之后
在墙角停住，日志里没有任何一行能让它想起「左边那个口子我没走过」。用户原话：
「说明它根本不记得左边可以转」。

查下来确实如此：`Trace.summary()` 当时只报三样东西 —— 走了多远、哪几个方位
是刚走过的路、这一带来过几次。三样都回答不了「还有哪条路没进去」。而 L 层
每轮只看得到当下 24 条射线，一个路口离开视野就彻底没了。

这里用真实迷宫几何（`scenes.maze_boxes()`，和 2D 离线仿真、仿真器同一份）走一
遍东侧缺口，断言三件事：

    走到墙角时    必须报出「左边那条没进去的通道」
    报的方向      必须是能走过去的（BFS 沿路），不是隔着墙看见的
    报的距离      是沿路的米数，不是直线

第三点容易写错：墙背后那一格「直线 0.9 m、绕过去 3 m」，报直线会让模型以为
眼前就有个岔路，于是朝墙转。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.percept.memory import Trace                          # noqa: E402
from test.maze_plan import maze_boxes, percept_of                   # noqa: E402

# 从出生点 (0,0) 沿 row0 往东，一直走到东端墙角 —— 缺口在北墙上，狗走过去
# 却没拐进去，这是要复现的那一段。
APPROACH = [(0.0, 0.0, 0), (0.9, 0.0, 0), (1.8, 0.0, 0), (2.7, 0.0, 0),
            (3.6, 0.0, 0), (4.2, 0.2, 0)]


def build() -> Trace:
    boxes = maze_boxes()
    tr = Trace()
    for x, y, deg in APPROACH:
        tr.update(percept_of(0.0, x, y, math.radians(deg), boxes))
    return tr


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  {'✓' if ok else '✗'} {name}{'  ' + detail if detail else ''}")
    return ok


def main() -> int:
    tr = build()
    opts = tr.openings()
    names = dict(opts)

    print("站在 row0 东端墙角（缺口在左侧），记忆里报出：")
    print("  " + tr.summary())
    print()

    ok = True
    # 1. 左侧那条通道必须出现。缺口在北墙东段，狗的机头朝东，所以是「左侧」。
    ok &= check("报出了没走过的方向", bool(opts), str([n for n, _ in opts]))
    ok &= check("其中包含左侧的通道", "左侧" in names)
    # 2. 距离是沿路走的，所以不可能比直线近多少；1.5 m 是这一段的实测值。
    if "左侧" in names:
        ok &= check("距离在 1.0~2.0 m 之间（沿路）",
                    1.0 <= names["左侧"] <= 2.0, f"{names['左侧']:.2f} m")
    # 3. 走廊正前方的格子不该被算成「没走过」—— 那是 CLEAR_R 的误差带。
    right = [n for n, _ in opts if n in ("正后", "右后")]
    ok &= check("不把刚走过的方向算成没走过", not right, str(right))

    # 回头看：缺口进去之后（站在 row1 里朝北），南边那条已经走过了，不该再报。
    tr2 = build()
    for x, y, deg in [(4.2, 0.6, 45), (4.0, 1.1, 90), (3.9, 1.6, 90)]:
        tr2.update(percept_of(0.0, x, y, math.radians(deg), maze_boxes()))
    behind = dict(tr2.openings()).get("正后")
    print()
    print("拐进 row1 之后（已走过缺口）：")
    print("  " + tr2.summary())
    ok &= check("正后不再报「没走过」", behind is None,
                "" if behind is None else f"{behind:.2f} m")

    print()
    print("通过" if ok else "没通过")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
