"""底层运动服务：把「A 模型输出关节」这条路包成一层可启动/可下命令的服务。

    from quadruped.motion import BackendLink, MotionService
    link = BackendLink(backend)          # 或者任何实现了 LowLink 的东西
    svc = MotionService(link)
    svc.start()
    svc.set_velocity(vx=0.3)
    ...
    svc.stop()

频率与职责：本层 50 Hz（策略），控制器 200 Hz（力矩回路），电机侧由驱动算 PD。

`LowLink` 是控制器与外界之间唯一的接触面，见 `quadruped/motion/link.py`。它有两个实现：

    BackendLink   借道 `Go2Controller` 写 lowcmd，起站、状态、mcf 交接都由它代劳
    DirectLink    自己持有 rt/lowcmd 与 rt/lowstate，与高层运控平级

两者对 `MotionService` 没有区别。

起站不经过高层：`rise(link)` 从当前姿态余弦插值到策略的 `DEFAULT_Q`，全程只写关节目标，
没有重力前馈、没有力分配、没有平衡反馈。见 `quadruped/motion/rise.py`。

    from quadruped.motion import MotionService, rise
    link.claim("rise")
    rise(link)
    MotionService(link).start()
"""

from quadruped.motion.direct import DirectLink
from quadruped.motion.link import BackendLink, LinkBusy, LowLink
from quadruped.motion.rise import rise
from quadruped.motion.service import (
    COMMAND_TIMEOUT,
    DAMPING_KD,
    DT,
    FALL_RAD,
    RAMP_S,
    MotionService,
)

__all__ = [
    "COMMAND_TIMEOUT",
    "DAMPING_KD",
    "DT",
    "FALL_RAD",
    "BackendLink",
    "DirectLink",
    "LinkBusy",
    "LowLink",
    "MotionService",
    "RAMP_S",
    "rise",
]
