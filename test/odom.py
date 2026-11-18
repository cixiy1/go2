"""腿部里程计的自检：几何要和控制栈那份正运动学对得上。

`quadruped/motion/odom.py` 自带一份腿长与正运动学（架构上不许 import
`control/`）。重复实现的风险是两处悄悄漂开 —— 那份是 MuJoCo 模型的真源，
一旦对不上，里程计会偏，而偏的方式很隐蔽：走直线慢慢歪。

所以这里直接拿 `Go2Controller.fk_leg` 逐腿对表。

    .venv/bin/python test/odom.py
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from quadruped.control.controller import CTRL_LEGS, Go2Controller   # noqa: E402
from quadruped.motion import odom as O                              # noqa: E402

fails: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'✓' if ok else '✗'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        fails.append(name)


print("正运动学对表（每条腿 200 组随机关节角）")
random.seed(7)
worst = 0.0
for leg in CTRL_LEGS:
    d = 0.0
    for _ in range(200):
        q = [random.uniform(-0.6, 0.6), random.uniform(0.2, 1.4),
             random.uniform(-2.4, -0.4)]
        a = Go2Controller.fk_leg(leg, q)
        b = O.foot_position(leg, q)
        d = max(d, max(abs(a[i] - b[i]) for i in range(3)))
    worst = max(worst, d)
check("四条腿的足端位置与控制栈一致", worst < 1e-9, f"最大差 {worst:.2e} m")

# 静态站姿下足端应在机身下方、接近 -站高
q0 = [0.0, 0.9, -1.8]
z = O.foot_position("FL", q0)[2]
check("站姿足端在机身下方 0.25~0.32 m", -0.32 < z < -0.25, f"z={z:+.3f} m")


# ---- 直线行走：喂一段合成的关节轨迹，看积分出来的位移 -----------------
#
# 做法：机身在世界上以 v 前进，脚踩地不动。于是「足端在机体系的位置」应当以 -v
# 后退。构造这样的轨迹喂进去，看里程计能不能还原出 v。
def straight_run(seconds=2.0, hz=50.0):
    """喂一段「脚相对机身匀速后移」的轨迹，机身应当匀速前进。

    四条腿用**同一个** q：真实步态里相位是错开的，但那样地面判定会在腿之间
    来回切，平均出来的速度带锯齿，测不出公式对不对。这一项只验方向和量级。

    thigh 角变大 = 足端往后（`foot_position("FL", [0, 0.9, -1.8])` 的 x 是
    +0.193，`[0, 1.45, -1.8]` 是 +0.055）。
    """
    dt = 1.0 / hz
    od = O.FootOdometer()
    t = 0.0
    n = int(seconds * hz)
    for i in range(n):
        s = (i % 50) / 50.0                 # 1 秒一个来回
        q = [0.0, 0.9 + 0.55 * s, -1.8] * 4
        od.update(q, 0.0, t)
        t += dt
    return od


od = straight_run()
moved = math.hypot(od.x, od.y)
# 这个合成轨迹不是真步态，只用来确认「方向对、量级对」：脚相对机身往后走，
# 积分出来的 x 必须为正（往前），且和 0.3 m/s × 2 s 同量级。
check("直线行走积分方向朝前", od.x > 0.1, f"x={od.x:+.2f} y={od.y:+.2f} m")
check("量级在合理范围（0.1~1.5 m）", 0.1 < moved < 1.5, f"位移 {moved:.2f} m")

# ---- yaw：gyro 积分 ----
od = O.FootOdometer()
t = 0.0
for _ in range(100):                       # 2 s，1 rad/s
    od.update([0.0, 0.9, -1.8] * 4, 1.0, t)
    t += 0.02
check("gyro 1 rad/s 转 2 秒 ≈ 2 rad", abs(od.yaw - 2.0) < 0.05,
      f"yaw={od.yaw:.3f} rad")

# ---- 断流：dt 过大要跳过，不能算出一个假的大位移 ----
od = O.FootOdometer()
od.update([0.0, 0.9, -1.8] * 4, 0.0, 0.0)
od.update([0.0, 0.5, -2.2] * 4, 0.0, 0.5)      # 断了 0.5 s
check("dt 超限时不当成位移", math.hypot(od.x, od.y) < 1e-9,
      f"位移 {math.hypot(od.x, od.y):.3f} m")

print()
if fails:
    print(f"失败：{'、'.join(fails)}")
    raise SystemExit(1)
print("通过（5 项）")
