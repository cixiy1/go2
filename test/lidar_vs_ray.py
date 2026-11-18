"""雷达扇区能不能顶替仿真射线：同一张迷宫，两种数据源对表。

问题来自真机：仿真里 `SimPercept` 用 MuJoCo 的 `mj_ray` 求交，一条射线一个
精确距离（24 条，每 15° 一条）。真机没有这个发布者，只有 L1 点云。`LidarPercept`
把点云切成 24 个扇区、每档取最近点，想复现同一个东西。

这个脚本不给结论，只做一件事：**在没有噪声的理想点云上，两者是不是同一个数**。
如果一致，剩下的差异就只是真机噪声和标定；如果不一致，那是算法问题，真机上
再怎么调也补不回来。

    .venv/bin/python test/lidar_vs_ray.py

三组对照：

    理想点云        每条射线在墙上放一个点        —— 只验几何等价
    带地面点        再加每档 40 个地面点          —— 验 Z_LOW 能不能滤掉
    稀疏 + 噪声     每条射线 1 点、距离加 ±2 cm   —— 模拟真实点云的稀
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np                                                   # noqa: E402

from quadruped.percept.lidar import RINGS, LidarPercept               # noqa: E402
from quadruped.sim.scenes import maze_boxes                           # noqa: E402

MOUNT_Z = 0.30          # L1 大致装高
BIN = 2 * math.pi / RINGS

ok_count = 0
bad: list[str] = []


def check(name: str, cond: bool, extra: str = "") -> None:
    global ok_count
    if cond:
        ok_count += 1
        print(f"  ✓ {name}" + (f"  （{extra}）" if extra else ""))
    else:
        bad.append(name)
        print(f"  ✗ {name}" + (f"  （{extra}）" if extra else ""))


# 射线与轴对齐盒子的最近交点。返回值是沿射线的距离，打不到就是 inf。
# 盒子格式和 `scenes.maze_boxes()` 一致：(cx, cy, hx, hy)。
def ray_boxes(ox: float, oy: float, dx: float, dy: float,
              boxes) -> float:
    best = math.inf
    for cx, cy, hx, hy in boxes:
        lo_x, hi_x = cx - hx, cx + hx
        lo_y, hi_y = cy - hy, cy + hy
        # slab 法：分别求射线在 x、y 两个板之间的参数区间，取交集
        t0, t1 = -math.inf, math.inf
        for origin, direction, lo, hi in ((ox, dx, lo_x, hi_x),
                                          (oy, dy, lo_y, hi_y)):
            if abs(direction) < 1e-12:
                if not (lo <= origin <= hi):
                    t0, t1 = 1.0, 0.0        # 平行且在外侧 → 不相交
                    break
                continue
            a = (lo - origin) / direction
            b = (hi - origin) / direction
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
        if t0 <= t1 and t1 >= 0.0:
            hit = t0 if t0 >= 0.0 else t1
            best = min(best, hit)
    return best


# 24 条射线的几何真值，方位和 `_center()` 一致（0°、15°、30°…，正为左）。
def truth(x: float, y: float, yaw: float, boxes,
          range_max: float = 3.0) -> list[float]:
    out = []
    for k in range(RINGS):
        a = -math.pi + k * BIN
        bearing = a if a < math.pi else a - 2 * math.pi
        dx = math.cos(yaw + bearing)
        dy = math.sin(yaw + bearing)
        d = ray_boxes(x, y, dx, dy, boxes)
        out.append(min(d, range_max))
    return out


# 造一份「雷达看到的东西」。每条射线上按真值距离采样；打不到的方向不放点，
# 模拟空旷（`_rays` 对这种扇区会给量程上限）。
def fake_cloud(x: float, y: float, yaw: float, boxes,
               *, per_ray: int = 3, noise: float = 0.0,
               ground: int = 0, seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    pts = []
    for k in range(RINGS):
        a = -math.pi + k * BIN
        bearing = a if a < math.pi else a - 2 * math.pi
        dx = math.cos(yaw + bearing)
        dy = math.sin(yaw + bearing)
        d = ray_boxes(x, y, dx, dy, boxes)
        if not math.isfinite(d):
            continue
        for i in range(per_ray):
            # 同一面墙上的点都落在 d 附近（±1 cm 近似「墙有厚度/点有散布」），
            # 不能按比例往近处推 —— 那是给扇区 min 人为制造偏差，测的就不是算法了。
            r = d + rng.uniform(-0.01, 0.01) + rng.normal(0.0, noise)
            # z 在雷达系：墙上取机身高度的 0.2~0.6，雷达装高 0.3 → 相对 -0.1~0.3
            z = rng.uniform(-0.1, 0.3)
            pts.append((r * dx, r * dy, z))
    for _ in range(ground):
        r = rng.uniform(0.2, 3.0)
        a = rng.uniform(-math.pi, math.pi)
        # 地面点在雷达系里 z ≈ -MOUNT_Z
        pts.append((r * math.cos(a), r * math.sin(a), -MOUNT_Z + rng.normal(0, 0.01)))
    return np.array(pts, dtype=np.float32)


# `LidarPercept._rays` 只要三个属性，不必真去订阅 DDS。
def sectors(xyz: np.ndarray) -> list[float]:
    lp = LidarPercept.__new__(LidarPercept)
    lp.range_max = 3.0
    lp.yaw_offset = 0.0
    lp.mount_z = MOUNT_Z
    return [r.distance for r in lp._rays(xyz)]


def compare(label: str, got: list[float], want: list[float],
            tol: float) -> None:
    diff = [abs(a - b) for a, b in zip(got, want)]
    worst = max(diff)
    idx = diff.index(worst)
    check(f"{label}：最大误差 {worst:.3f} m", worst <= tol,
          f"第 {idx} 档 真值 {want[idx]:.2f} 雷达 {got[idx]:.2f}")


def main() -> int:
    boxes = maze_boxes()
    # 走廊中间一处，四周都是墙，正前留一条长视线 —— 有近有远才有区分度。
    x, y, yaw = 2.1, 3.6, math.radians(1.0)
    want = truth(x, y, yaw, boxes)

    print("几何真值")
    near = [d for d in want if d < 3.0]
    check(f"有 {len(near)} 档打到墙，其余 {RINGS - len(near)} 档打空",
          0 < len(near) < RINGS, f"最近 {min(want):.2f} m")

    print("\n理想点云（每档 3 点、无噪声）")
    got = sectors(fake_cloud(x, y, yaw, boxes, per_ray=3))
    compare("24 档距离", got, want, 0.05)

    print("\n带地面点（每档 3 点 + 200 个地面点）")
    got2 = sectors(fake_cloud(x, y, yaw, boxes, per_ray=3, ground=200))
    compare("地面滤掉后", got2, want, 0.05)
    check("地面没有把最近距离压到脚下",
          min(got2) > 0.3, f"最小 {min(got2):.2f} m")

    print("\n稀疏 + 噪声（每档 1 点、±2 cm）")
    got3 = sectors(fake_cloud(x, y, yaw, boxes, per_ray=1, noise=0.02))
    compare("24 档距离", got3, want, 0.15)

    print("\n空旷场地（一个点都没有）")
    got4 = sectors(np.zeros((0, 3), dtype=np.float32))
    check(f"给 {RINGS} 档量程上限，不是空元组",
          len(got4) == RINGS and all(abs(d - 3.0) < 1e-6 for d in got4),
          f"{len(got4)} 档，{got4[0]:.1f} m")

    print(f"\n{'通过' if not bad else '失败：' + '、'.join(bad)}"
          f"（{ok_count} 项）")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
