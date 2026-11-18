"""V 层的接入口：一个只有两个方法的协议。

换传感器就是换一个实现，上层（L 层、安全闸）不认识 mujoco、不认识 cv2。这个协议
刻意做得很窄，因为「感知」最容易变成什么都往里塞：图像、点云、语义分割、目标跟踪
最后都会有人想加进来。窄接口逼着这些处理待在实现内部，产出的仍然是 `Percept`。

测距（`rays` / `ground`）有两个实现，环境不同就换一个，**别混**：

    仿真 `sim`      → `SimPercept`：MuJoCo 射线求交，走 UDP 从仿真进程取（10 Hz）
    真机 `lidar`    → `LidarPercept`：订阅 `rt/utlidar/cloud` 和 `rt/utlidar/height_map`

其余：

    ProprioPercept  只有机身遥测，没有外部视野
    VisionPercept   画面 → 三方位判读，自己起线程（1 Hz）
    Fused           测距 + 画面，合成一份 `Percept`

`open_source()` 按名字选，`geom=False` 时把测距关掉（只剩画面 + 本体遥测）。
降级必须能从 `Percept.origin` 看出来，不能悄悄变成「看不见就当没有」。
"""

from __future__ import annotations

from typing import Protocol

from quadruped.percept.types import Percept


class PerceptSource(Protocol):
    """一次观测的来源。`read()` 必须非阻塞，拿不到就返回 None。"""

    name: str

    def read(self) -> Percept | None: ...

    def close(self) -> None: ...


# 按名字打开一个来源。`backend` 给本体相关的那几个用，别的实现可以不接。
#
# `geom` 是「要不要测距」：仿真走射线，真机走雷达 —— 同一个开关，两种实现，
# 由调用方按环境二选一（见 `runner`）。关掉之后 `rays` 是空的，安全闸不触发、
# 记忆不进格子，这两件事是**已知**的行为，不是故障。
def open_source(kind: str, backend=None, *, geom: bool = True,
                **kwargs) -> PerceptSource:
    kind = (kind or "proprio").strip().lower()

    if kind == "sim":
        from quadruped.percept.sim import SimPercept

        return SimPercept(rays=geom, **kwargs)

    # 真机的测距。名字就叫 lidar，不再复用 sim 那个别名 —— 之前 `"lidar"` 指向
    # 的是 `SimPercept`，选它等于用仿真射线，日志里却写着 radar，最坑的一处。
    if kind == "lidar":
        from quadruped.percept.lidar import LidarPercept

        if backend is None:
            raise ValueError("lidar 要 backend（位姿和倾角从它读）")
        return LidarPercept(backend, **kwargs)

    if kind == "proprio":
        from quadruped.percept.proprio import ProprioPercept

        return ProprioPercept(backend)

    if kind == "vision":
        from quadruped.percept.camera import VisionPercept

        return VisionPercept(**kwargs)

    if kind in ("sim+vision", "fused"):
        # 合成源不自己起线程，调用方要显式 `src.vision.start()`。这样「加载
        # 视觉模型」这件事发生在哪一行是看得见的，而不是在构造函数里悄悄花一秒。
        from quadruped.percept.camera import Fused, VisionPercept
        from quadruped.percept.sim import SimPercept

        sim_kw = dict(kwargs.pop("geom", {}))
        sim_kw.setdefault("rays", geom)      # 仿真射线
        return Fused(SimPercept(**sim_kw), VisionPercept(**kwargs))

    if kind in ("real+vision", "camera"):
        # 真机：前置摄像头 + 本体遥测（+ 雷达，如果开了几何）。
        #
        # `backend` 必须给：位姿、倾角、速度全从它读，没有它这条链路什么都不产出。
        from quadruped.percept.camera import CameraStream, Fused, VisionPercept
        from quadruped.percept.lidar import LidarPercept
        from quadruped.percept.proprio import ProprioPercept

        if backend is None:
            raise ValueError("real+vision 要 backend（位姿和倾角从它读）")
        camera = kwargs.pop("camera", None) or {}
        lidar_kw = kwargs.pop("lidar", None) or {}
        if geom:
            base = LidarPercept(backend, **lidar_kw)         # Go2 L1
        else:
            base = ProprioPercept(backend)                   # 只有本体
        return Fused(base,
                     VisionPercept(stream=CameraStream(**camera), **kwargs),
                     name=kind)

    raise ValueError(
        f"没有这种感知来源：{kind}（可用 sim / sim+vision / real+vision / "
        f"lidar / proprio）")
