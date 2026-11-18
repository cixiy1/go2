"""V 层：把任意传感器的观测统一成 `Percept`。

    types.py     数据结构（`Percept` / `Ray` / `Sector` / `SelfState`）+ 中文渲染
    base.py      `PerceptSource` 协议与 `open_source()` 工厂
    sim.py       仿真射线，UDP 回环（几何：距离、高度）—— **只在仿真**
    lidar.py     真机雷达（rt/utlidar/cloud + height_map）—— **只在真机**
    camera.py    画面 → 三方位判读 + `Fused` 合成源；全局画面 `WorldStream`
    proprio.py   只有机身遥测，没有视野
    guard.py     安全闸：危险时直接停车，不经过 L 层
    reflex.py    R 层：20 Hz 取感知 + 查闸，并做感知源的唯一读者
    viewer.py    画面和想法放在同一个窗口里显示（cv2 + PIL）

这一层不 import mujoco。加真相机时只新增 `camera.py` 并在 `base.open_source()`
里挂一个名字，上层不知道换了传感器。
"""

from quadruped.percept.base import PerceptSource, open_source
from quadruped.percept.camera import (
    Fused,
    VideoStream,
    VisionPercept,
    WorldStream,
    open_world,
)
from quadruped.percept.guard import Guard
from quadruped.percept.reflex import Reflex
from quadruped.percept.types import (
    FRONT_SECTORS,
    SECTOR_NAMES,
    Ground,
    Percept,
    Ray,
    Sector,
    SelfState,
)
from quadruped.percept.viewer import Viewer

__all__ = [
    "FRONT_SECTORS",
    "Fused",
    "VideoStream",
    "VisionPercept",
    "WorldStream",
    "SECTOR_NAMES",
    "Ground",
    "Guard",
    "Percept",
    "PerceptSource",
    "Ray",
    "Reflex",
    "Sector",
    "SelfState",
    "Viewer",
    "open_source",
    "open_world",
]
