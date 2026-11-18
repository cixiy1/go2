"""推理控制器看到的那一层：九个成员，一个都不多。

推理控制器不该认识 `Go2Controller`，也不该知道步态、MPC、动作库里有什么。它只需要
「读现在的姿态」和「把关节指令写下去」这两件事。这个文件就是这两件事的边界。

## 为什么要 `claim` / `release`

`rt/lowcmd` 只能有一个写者。两个写者交替写同一个 `motor_cmd`，电机看到的目标角在抖，
看起来像「参数调不好」—— 这个坑在 `Go2Controller` 里已经踩过一次，当时是靠
`joint_target()` 顺手把 `_mode` 置 idle 压住的。那是隐式的：谁都不知道自己拿了权，
别人也看不出权在谁手上。

现在把它抬成显式的：拿权才能写，没拿就抛 `LinkBusy`。冲突会在第一帧暴露，不会
拖到「腿抖了三分钟」才被人怀疑。

## 独立到哪一步

本层只解决**接口**上的独立：推理控制器不再依赖高层控制器的实现。物理上的独立
（自带 DDS、独立进程）是另一件事，那时候换一个实现了这套协议的 `DirectLink` 就行，
推理控制器一行都不用改。
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


class LinkBusy(RuntimeError):
    """lowcmd 的写入权被别人拿着。"""


# 推理控制器的全部依赖。实现这套的东西可以是现有 `Backend`，也可以是自持 DDS 的
# 直连实现，控制器这边看不出区别。
@runtime_checkable
class LowLink(Protocol):
    # 取得 / 归还 lowcmd 的写入权。名字用于报错，能看出是谁在抢。
    def claim(self, name: str) -> None: ...

    def release(self, name: str) -> None: ...

    # 位置 PD 直通。kp / kd 由调用方给，驱动器的回路照常参与。
    def joint_target(self, q, *, kp=None, kd=None) -> None: ...

    # 力矩直通。kp / kd 置 0，力矩全走 tau。
    def joint_torque(self, tau) -> None: ...

    # 松腿进阻尼。kp = 0、kd = 给定值，保持一小段时间。
    def damping(self, kd: float) -> None: ...

    # 12 个关节角与角速度，LowCmd 序（FR, FL, RR, RL）。
    @property
    def q(self): ...

    @property
    def dq(self): ...

    # 姿态角，rad。
    @property
    def rpy(self): ...

    # 机身高度，m。注意这是运动学值，躺着也可能报 0.26，判站立要配倾角一起看。
    @property
    def height(self): ...


# 用现有 `Backend` 实现 `LowLink`。抽象层已经有 `joint_target` / `joint_torque` / `damping`
# 之外的读写能力，这里只挑推理要用的那几个，把其余能力挡在外面 —— 挡住的不是权限，
# 是误用：控制器能调 `move()` 的那天，它就会去调。
class BackendLink:
    """把一个 `Backend` 收成 `LowLink`。

    `claim` 只记名字，真正让 200 Hz 回路闭嘴的是 `Backend.joint_target()` 自己置
    idle —— 这一层还没接管仲裁。记名字仍然有用：没 claim 就写会当场抛错，而不是
    等两路指令打架。
    """

    def __init__(self, backend):
        self.b = backend
        self._owner: str | None = None

    def claim(self, name: str) -> None:
        if self._owner is not None and self._owner != name:
            raise LinkBusy(f"lowcmd 写入权在 {self._owner} 手上，{name} 拿不到")
        self._owner = name

    def release(self, name: str) -> None:
        if self._owner == name:
            self._owner = None

    def _need_claim(self, what: str) -> None:
        if self._owner is None:
            raise LinkBusy(f"没 claim 就写 {what}：先 claim('policy') 再下发")

    def joint_target(self, q, *, kp=None, kd=None) -> None:
        self._need_claim("关节目标")
        self.b.joint_target(q, kp=kp, kd=kd)

    def joint_torque(self, tau) -> None:
        self._need_claim("力矩")
        self.b.joint_torque(tau)

    def damping(self, kd: float) -> None:
        # 不进阻尼要写权：这是「松手」，主动权收回来时才需要，不设门槛反而更安全。
        self.b.joint_target(self.b.q, kp=0.0, kd=kd)

    @property
    def q(self):
        return self.b.q

    @property
    def dq(self):
        return self.b.dq

    @property
    def rpy(self):
        return self.b.rpy

    @property
    def height(self):
        return self.b.height

    @property
    def owner(self) -> str | None:
        return self._owner
