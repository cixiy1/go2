"""控制端实现：把「一个高层控制器 + 一台机器」装配成控制器能用的对象。

三个实现：

    OursBackend      Go2Robot（本体 + 200 Hz 回路）+ 自研高层，走 rt/lowcmd
    OfficialBackend  OfficialHighLevel（sport service 客户端），走 rt/api/sport/request
    PolicyBackend    推理控制器，直连 rt/lowcmd（实现在 policy.py，是适配器）

前两个对上层暴露同一套方法 —— 因为两份高层控制器本来就同名同义
（见 `control/high_level/__init__.py`）。公共部分在 `_Delegate` 里，子类只管
「怎么建连接」和「状态从哪读」。

第三个不一样：它没有高层控制器可委派，能做多少写多少，其余抛 `NotImplementedError`。

上层不该直接 import 这个文件：用 `robot.connect(endpoint, kind)`，它在需要的
时候才延迟导入。

DDS 有两个入口：前两个经 `Go2Robot`，第三个经 `quadruped/motion/direct.py`。后者
是刻意分开的 —— 推理控制器不该为了发 lowcmd 而把整个控制栈拖进来。
"""

import socket
from typing import NoReturn

import numpy as np

from quadruped.control.virtual.policy import PolicyBackend
from quadruped.robot.body import Go2Robot


# 网卡不在就早点说清楚，别等 DDS 报一句看不懂的错。
def require_iface(iface: str) -> None:
    if Go2Robot.interface_exists(iface):
        return
    avail = ", ".join(n for _, n in socket.if_nameindex())
    raise RuntimeError(f"本机没有网卡 {iface}。当前可用：{avail}")


class _Delegate:
    """把动作转给 `self.client`，转的同时不掺任何判断。

    两份高层控制器共有的部分（`Init / SetTimeout / BalanceStand / Move /
    StopMove`，外加不阻塞版的 `jump_async / crouch_async / uncrouch_async /
    stand_async / stop_async / freeze / park`）在这里一律直接转发，不按后端
    分支 —— 一旦需要分支，就说明两份的接口又漂了，该去改控制器。

    只有自研那份实现了 `forward / turn / set_gait_style`。这三条在这里声明并
    抛 `NotImplementedError`，不拿 `Move` 拼一个假的顶上：官方原生服务给的是
    `MoveToPos` 这类一次性动作，语义不对等。上层按 `NotImplementedError`
    判断「这个控制端没有该能力」，别用 `hasattr` 猜。
    """

    kind = ""
    label = ""
    target = ""
    client = None

    # SetTimeout → Init → BalanceStand，返回码非 0 = 对端没响应。
    def start(self) -> int:
        self.client.SetTimeout(3.0)
        self.client.Init()
        return self.client.BalanceStand()

    # 跳跃。排完就返回，不等落地（两份都是非阻塞的）。
    def jump(self) -> int:
        return self.client.jump_async()

    def crouch(self, depth: float = 0.08) -> int:
        return self.client.crouch_async(depth)

    # 速度指令。两份控制器同名同义，直接转发；停由 stop_motion 负责。
    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> int:
        return self.client.Move(vx, vy, vyaw)

    def uncrouch(self) -> int:
        return self.client.uncrouch_async()

    def stand(self) -> int:
        return self.client.stand_async()

    # 停车，不等收步完成 —— 20 Hz 主循环不能在这里睡着。
    def stop_motion(self) -> int:
        return self.client.stop_async()

    # 停止：中断动作并站定，仍然不阻塞。
    def freeze(self) -> int:
        return self.client.freeze()

    # 收工。自研那份要蹲下再松（仿真桥不发指令就没力矩），官方停住即可，
    # 所以这件事由各自的 `park()` 决定，这里只转发。
    def close(self) -> None:
        self.client.park()

    # -- 只有自研实现的几条 ----------------------------------------------
    # 子类覆盖；没覆盖的到这里报「没有这个能力」，别让调用方去 hasattr。
    #
    # 官方的原生运动服务不开放 `rt/lowcmd`，所以关节直通、闭环位移这些它都没有。
    # 这不是没实现，是那条链路根本不经过这一层。

    def _no_impl(self, what: str) -> NoReturn:
        raise NotImplementedError(f"{self.label}没有{what}，只有自研控制端实现了它")

    # 关节直通：12 个目标角 + PD 增益，绕过步态直接下发电机。给外部策略用。
    def joint_target(self, q, *, kp=None, kd=None, tau=None) -> int:
        self._no_impl("关节直通")

    # 力矩直通：12 个力矩直接下发，kp / kd 置 0，PD 只剩个空回路。
    def joint_torque(self, tau) -> int:
        self._no_impl("力矩直通")

    # 12 个关节角与角速度，LowCmd / LowState 顺序（FR, FL, RR, RL）。
    @property
    def q(self):
        self._no_impl("关节角读出")

    @property
    def dq(self):
        self._no_impl("关节角速度读出")

    # 走 distance 米（负值后退），按里程计闭环，会阻塞到走完。
    def forward(self, distance: float, vx: float = 0.3,
                tol: float = 0.08) -> dict:
        self._no_impl("闭环位移")

    # 原地转 degrees 度，正为逆时针，按 IMU 偏航闭环。
    def turn(self, degrees: float, yaw_rate: float = 0.5,
             tol: float = 0.1745) -> dict:
        self._no_impl("闭环转向")

    # 换运动步态风格（`mech` / `bio`）。
    def set_gait_style(self, style: str) -> str:
        self._no_impl("步态风格切换")

    @property
    def loco_gait(self):
        return self.client.gait_style_label

    def toggle_loco_gait(self):
        return self.client.toggle_gait_style()

    @property
    def jump_gait(self):
        return self.client.jump_style_label

    def toggle_jump_gait(self):
        return self.client.toggle_jump_style()

    @property
    def busy(self) -> bool:
        raise NotImplementedError

    @property
    def pos(self):
        raise NotImplementedError

    @property
    def rpy(self):
        raise NotImplementedError

    @property
    def height(self):
        raise NotImplementedError

    @property
    def loop_text(self) -> str:
        return ""


class OursBackend(_Delegate):
    """自研：Go2Robot → Go2Controller → HighLevelController。

    连接、控制回路、里程计都在 `Go2Robot` 里；这里只是在它上面取出高层那一层，
    再把状态（位置 / 姿态 / 机身高度 / 回路耗时）转给控制器看。
    """

    kind = "ours"
    label = "自研控制器 HighLevelController"

    def __init__(self, endpoint, *, kp=None, kd=None, tau=None, motion=None):
        self.endpoint = endpoint
        self.robot = Go2Robot(endpoint.preset, domain=endpoint.domain,
                              iface=endpoint.iface, kp=kp, kd=kd, tau_ff=tau,
                              motion=motion)
        c = self.robot.controller
        self.target = f"网卡 {c.iface} / domain {c.domain}"
        require_iface(c.iface)
        if not self.robot.connect():
            raise RuntimeError(
                f"{c.iface}（domain {c.domain}）3 秒内没收到 rt/lowstate。"
                "仿真先起 ./quadruped/sim/run_sim.sh；真机先确认 setup_en5.sh 过了")
        self.client = c.high_level

    # 控制回路节奏，形如「 回路5.0/6.2」。真机抖不抖先看这个：
    # 均值超过 6 ms 或 max 跳到十几毫秒，就是时序问题，不是参数问题。
    @property
    def loop_text(self) -> str:
        st = self.robot.controller.loop_stats()
        if not st.get("n"):
            return ""
        return f" 回路{st['mean']:.1f}/{st['max']:.1f}ms"

    @property
    def pos(self):
        return self.robot.pos

    @property
    def rpy(self):
        return self.robot.rpy

    @property
    def height(self):
        return self.robot.controller.body_height()

    # 腿正被跳跃 / 蹲伏占着。这时候别再发速度，等动作播完自然接着走。
    @property
    def busy(self):
        return self.robot.controller.busy

    # 下面三条自研高层本来就有，闭环保留全部语义（含收步滑行余量），
    # 这里只做转发，不在抽象层重算一遍容差。
    def forward(self, distance: float, vx: float = 0.3,
                tol: float = 0.08) -> dict:
        return self.client.forward(distance, vx=vx, tol=tol)

    def turn(self, degrees: float, yaw_rate: float = 0.5,
             tol: float = 0.1745) -> dict:
        return self.client.turn(degrees, yaw_rate=yaw_rate, tol=tol)

    def set_gait_style(self, style: str) -> str:
        return self.client.set_gait_style(style)

    # 关节直通给外部策略：控制器本身已经会用 q_des + kp/kd 发 lowcmd，
    # 这里只把它从「步态专用」变成公开入口。
    def joint_target(self, q, *, kp=None, kd=None, tau=None) -> int:
        c = self.robot.controller
        if c.q is None:
            raise RuntimeError("还没收到 rt/lowstate，不能发关节指令")
        c.joint_target(np.asarray(q, dtype=float).reshape(-1), kp=kp, kd=kd, tau=tau)
        return 0

    # 力矩直通。用途是让训练时的电机模型接管执行器，不是常规控制路径 ——
    # 没有回中的力，观测一坏就是满力矩。
    def joint_torque(self, tau) -> int:
        c = self.robot.controller
        c.joint_torque(np.asarray(tau, dtype=float).reshape(-1))
        return 0

    @property
    def q(self):
        return self.robot.controller.q

    @property
    def dq(self):
        return self.robot.controller.dq


class OfficialBackend(_Delegate):
    """官方：`OfficialHighLevel`，调真机上的原生运动服务。

    只在真机可用 —— 仿真的桥接层只收发 `rt/lowcmd` 和 `rt/sportmodestate`，
    没有 `rt/api/sport/request` 的服务端，请求会一直等到超时。
    """

    kind = "official"
    label = "官方控制器 SportClient"

    def __init__(self, endpoint, *, kp=None, kd=None, tau=None):
        # 增益是自研控制器的参数；官方那份跑在机器上，给了也用不到。
        from quadruped.control.high_level.official import OfficialHighLevel

        self.endpoint = endpoint
        require_iface(endpoint.iface)
        self.client = OfficialHighLevel(endpoint.iface, endpoint.domain)
        self.target = f"网卡 {endpoint.iface} / domain {endpoint.domain}"

    @property
    def pos(self):
        return self.client.pos

    @property
    def rpy(self):
        return self.client.rpy

    @property
    def height(self):
        return self.client.height

    # 原生服务在机器上跑，动作是同步 RPC，这边只有 progress 一个线索。
    @property
    def busy(self):
        return self.client.busy


#: 控制端名字 → 实现。`endpoint.preset` 决定能用哪些，见 robot.clients_for。
#:
#: `policy` 是推理控制器，实现在另一个文件 —— 它是适配器，把 `MotionService`
#: 翻译成这套方法，而不是推理控制器的一部分。依赖方向 control → motion。
BACKENDS = {
    "ours": OursBackend,
    "official": OfficialBackend,
    "policy": PolicyBackend,
}
