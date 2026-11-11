"""控制端实现：全项目**唯一**接触 DDS 与机器人本体的地方。

两个实现，对控制器暴露同一套方法：

    OursBackend      Go2Robot → Go2Controller → HighLevelController，发 rt/lowcmd
    OfficialBackend  unitree_sdk2py 的 SportClient，调真机上的原生运动服务

方法名沿用 SportClient 那一套（Init / SetTimeout / BalanceStand / Move /
StopMove / Damp），因为自研高层本来就是对它的模仿 —— 于是控制器写完一份，
两个环境都能跑，不用知道自己连的是哪个。

上层不该 import 这个文件：用 `robot.connect(endpoint, kind)`，它在需要的时候
才延迟导入。控制器提前 import 的唯一后果是 `main.py --list` 也要等 SDK 加载。
"""

import socket

from quadruped.robot.body import Go2Robot


def require_iface(iface: str) -> None:
    """网卡不在就早点说清楚，别等 DDS 报一句看不懂的错。"""
    if Go2Robot.interface_exists(iface):
        return
    avail = ", ".join(n for _, n in socket.if_nameindex())
    raise RuntimeError(f"本机没有网卡 {iface}。当前可用：{avail}")


class OursBackend:
    """自研：Go2Robot → Go2Controller → HighLevelController。

    高层本身就实现了 SportClient 那一套方法，所以 client 就是它本身，
    不需要再包一层。位置和姿态取自控制器在 connect 时订阅的 rt/sportmodestate。
    """

    kind = "ours"
    label = "自研控制器 HighLevelController"

    def __init__(self, endpoint, *, kp=None, kd=None, tau=None):
        self.endpoint = endpoint
        self.robot = Go2Robot(endpoint.preset, domain=endpoint.domain,
                              iface=endpoint.iface, kp=kp, kd=kd, tau_ff=tau)
        c = self.robot.controller
        self.target = f"网卡 {c.iface} / domain {c.domain}"
        require_iface(c.iface)
        if not self.robot.connect():
            raise RuntimeError(
                f"{c.iface}（domain {c.domain}）3 秒内没收到 rt/lowstate。"
                "仿真先起 ./quadruped/sim/run_sim.sh；真机先确认 setup_en5.sh 过了")
        self.client = c.high_level

    def start(self) -> int:
        """SetTimeout → Init → BalanceStand。返回码非 0 说明对端没响应。"""
        self.client.SetTimeout(3.0)
        self.client.Init()
        return self.client.BalanceStand()

    @property
    def loop_text(self) -> str:
        """控制回路节奏，形如「 回路5.0/6.2」。真机抖不抖先看这个：
        均值超过 6 ms 或 max 跳到十几毫秒，就是时序问题，不是参数问题。"""
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

    @property
    def busy(self):
        """腿正被跳跃 / 蹲伏占着。这时候别再发速度，等动作播完自然接着走。"""
        return self.robot.controller.busy

    def jump(self) -> int:
        return self.client.jump_async()          # 排完就返回，不等落地

    @property
    def loco_gait(self) -> str:
        return self.client.gait_style_label

    def toggle_loco_gait(self) -> str:
        """机械 ↔ 仿生：跑法跟着换，跳跃动作也一起换（同一档风格）。"""
        self.client.toggle_jump_style()
        return self.client.gait_style_label

    @property
    def jump_gait(self) -> str:
        return self.client.jump_style_label

    def toggle_jump_gait(self) -> str:
        """机械 ↔ 仿生：跳跃动作跟着换，跑法也一起换（同一档风格）。"""
        self.client.toggle_gait_style()
        return self.client.jump_style_label

    def crouch(self) -> int:
        return self.client.crouch_async(0.08)

    def uncrouch(self) -> int:
        return self.client.uncrouch_async()

    def stand(self) -> int:
        return self.client.stand_async()

    def stop_motion(self) -> int:
        """停车，不等收步完成 —— 20 Hz 主循环不能在这里睡着。"""
        return self.client.stop_async()

    def freeze(self) -> int:
        """停止：中断动作并站定，仍然不阻塞。"""
        return self.client.stand_async()

    def close(self):
        """停步 → 蹲下 → 阻尼。仿真桥不发指令就没有力矩，必须自己蹲好再松。"""
        self.client.park()


class OfficialBackend:
    """官方：unitree_sdk2py 的 SportClient，调真机上的原生运动服务。

    只在真机可用 —— 仿真的桥接层只收发 rt/lowcmd 和 rt/sportmodestate，
    没有 rt/api/sport/request 的服务端，请求会一直等到超时。
    """

    kind = "official"
    label = "官方控制器 SportClient"

    def __init__(self, endpoint, *, kp=None, kd=None, tau=None):
        # 增益是自研控制器的参数，官方那份跑在机器上，给了也用不到。
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize, ChannelSubscriber)
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

        self.endpoint = endpoint
        require_iface(endpoint.iface)
        ChannelFactoryInitialize(endpoint.domain, endpoint.iface)
        self._msg = None
        self.target = f"网卡 {endpoint.iface} / domain {endpoint.domain}"
        ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(
            lambda m: setattr(self, "_msg", m), 10)
        self.client = SportClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

    def start(self) -> int:
        """SetTimeout → Init → BalanceStand。官方客户端在构造时已经 Init 过，
        再 Init 一次无害（就是重设超时与内部状态）。"""
        self.client.SetTimeout(3.0)
        self.client.Init()
        return self.client.BalanceStand()

    @property
    def loop_text(self) -> str:
        return ""                            # 原生控制环在机器上，这里看不到

    @property
    def pos(self):
        return tuple(self._msg.position) if self._msg is not None else None

    @property
    def rpy(self):
        return tuple(self._msg.imu_state.rpy) if self._msg is not None else None

    @property
    def height(self):
        return None          # 原生状态里的 position 与 base_link 差一个常数，不做换算

    @property
    def busy(self):
        """原生服务在机器上跑，这些动作都是同步 RPC，这边没有「占着腿」的状态。"""
        return False

    def jump(self) -> int:
        return self.client.FrontJump()      # 原生只有前跳，没有原地跳

    @property
    def loco_gait(self) -> str | None:
        return None                         # 原生的运动步态在机器上

    def toggle_loco_gait(self) -> str | None:
        return None

    @property
    def jump_gait(self) -> str | None:
        return None                         # 原生步态在机器上，改不了

    def toggle_jump_gait(self) -> str | None:
        return None

    def crouch(self) -> int:
        """原生没有「蹲下」这个接口，用坐下代替（后腿折叠，前腿撑地）。"""
        return self.client.Sit()

    def uncrouch(self) -> int:
        return self.client.RiseSit()

    def stand(self) -> int:
        return self.client.BalanceStand()

    def stop_motion(self) -> int:
        return self.client.StopMove()

    def freeze(self) -> int:
        self.client.StopMove()
        return self.client.BalanceStand()

    def close(self):
        """原生运动服务自己有平衡环，停住就行，不需要 Damp 趴下。"""
        self.client.StopMove()


#: 控制端名字 → 实现。`endpoint.preset` 决定能用哪些，见 robot.clients_for。
BACKENDS = {
    "ours": OursBackend,
    "official": OfficialBackend,
}
