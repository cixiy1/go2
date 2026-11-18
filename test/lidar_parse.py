"""雷达点云 → Ray / Ground 的解析回归。不需要雷达、不需要 DDS。

真机那条链路没法在这儿跑，但**解析**这一层是可以脱离硬件验的：给一组已知的
点，看它切出来的扇区距离对不对。当年仿真射线的扇区归纳就是靠这种测试定住的。
"""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from quadruped.percept.lidar import LidarPercept, _ground, _to_xyz   # noqa: E402
from quadruped.percept.types import SelfState                        # noqa: E402

ok = True


def check(name: str, cond: bool, detail: str = "") -> bool:
    global ok
    ok &= bool(cond)
    print(f"  {'✓' if cond else '✗'} {name}" + (f"  （{detail}）" if detail else ""))
    return bool(cond)


class _Field:
    def __init__(self, name, offset):
        self.name = name
        self.offset = offset


class _Cloud:
    """最小可用的 PointCloud2 替身：x/y/z + 一个没用的 intensity。"""

    def __init__(self, pts):
        self.point_step = 16
        self.width = len(pts)
        self.height = 1
        self.fields = [_Field("x", 0), _Field("y", 4), _Field("z", 8),
                       _Field("intensity", 12)]
        buf = bytearray()
        for x, y, z in pts:
            buf += struct.pack("<fff", x, y, z) + struct.pack("<f", 0.0)
        self.data = list(buf)


def _ray(pts, **kw):
    obj = LidarPercept.__new__(LidarPercept)
    obj.range_max = kw.get("range_max", 3.0)
    obj.yaw_offset = kw.get("yaw_offset", 0.0)
    obj.mount_z = kw.get("mount_z", 0.0)
    import numpy as np
    xyz = np.array(pts, dtype="float32")
    return {round(math.degrees(r.bearing)): (round(r.distance, 2), round(r.top, 2))
            for r in obj._rays(xyz)}


print("点云字段解析")
pts = [(2.0, 0.0, 0.0), (1.0, 1.0, 0.5)]
xyz = _to_xyz(_Cloud(pts))
check("形状 (n,3)", xyz.shape == (2, 3), str(xyz.shape))
check("值原样取出", abs(xyz[0][0] - 2.0) < 1e-6 and abs(xyz[1][2] - 0.5) < 1e-6,
      f"{xyz[0].tolist()} {xyz[1].tolist()}")

print("\n扇区取最近")
# 正前方 2 m 一面墙，左边 1 m 一个柜子，地面一堆点
pts = [(2.0, 0.0, 0.0), (2.0, 0.3, 0.2), (0.0, 1.0, 0.1),
       (0.3, 0.0, -0.35), (0.5, 0.2, -0.32)]      # 最后两个是地面
rays = _ray(pts)
front = [v for k, v in rays.items() if abs(k) <= 7]
left = [v for k, v in rays.items() if 83 <= k <= 97]
check("正前 2.00（不是地面的 0.3）", front and min(v[0] for v in front) == 2.0,
      str(rays.get(0)))
check("左侧 1.00", left and min(v[0] for v in left) == 1.0, str(rays.get(90)))
check("没点的方位给量程上限", rays.get(-180) == (3.0, 0.0), str(rays.get(-180)))
check("24 条", len(rays) == 24, str(len(rays)))
check("高度带上 z", rays.get(90, (0, 0))[1] == 0.1, str(rays.get(90)))

print("\n只有地面：给满量程，不给空元组")
floor_only = _ray([(0.8, 0.0, -0.35), (1.2, 0.0, -0.30)])
check("24 条都在", len(floor_only) == 24, str(len(floor_only)))
check("全是量程上限", all(v[0] == 3.0 for v in floor_only.values()),
      str(sorted({v[0] for v in floor_only.values()})))

print("\n外参：整机转 10°")
rays = _ray([(2.0, 0.0, 0.0)], yaw_offset=math.radians(10))
hit = [k for k, v in rays.items() if v[0] < 2.5]
check("墙移到 +15° 那一档（外参生效）", hit == [15], str(hit))

print("\n高度图 → Ground")
class _Map:
    resolution = 0.05
    width = 200
    height = 200
    origin = [-5.0, -5.0]
    # 前方 0.4 m 处有 0.12 m 台阶，其余是平地
    data = [0.0] * (200 * 200)

data = _Map.data
i = int((0.4 - _Map.origin[0]) / _Map.resolution)
j = int((0.0 - _Map.origin[1]) / _Map.resolution)
data[j * 200 + i] = 0.12
g = _ground(_Map(), SelfState(x=0.0, y=0.0, yaw=0.0))
check("脚下 0", g.under == 0.0, str(g.under))
check("前方 0.4 m 读到台阶", abs(g.ahead[0] - 0.12) < 1e-6, str(g.ahead))
check("step_up 认出 0.12", abs(g.step_up - 0.12) < 1e-6, str(g.step_up))

print(f"\n{'通过' if ok else '不通过'}")
sys.exit(0 if ok else 1)
