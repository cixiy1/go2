"""控制端实现：把「一个高层控制器 + 一台机器」装配成控制器能用的对象。

全项目**唯一**接触 DDS 与机器人本体的地方。两个实现：

    OursBackend      Go2Robot（本体 + 200 Hz 回路）+ 自研高层，走 rt/lowcmd
    OfficialBackend  OfficialHighLevel（sport service 客户端），走 rt/api/sport/request

两者对上层暴露同一套方法 —— 因为两份高层控制器本来就同名同义
（见 `control/high_level/__init__.py`）。公共部分在 `_Delegate` 里，子类只管
「怎么建连接」和「状态从哪读」。

上层不该直接 import 这个文件：用 `robot.connect(endpoint, kind)`，它在需要的
时候才延迟导入。
"""

import socket

from quadruped.robot.body import Go2Robot


def require_iface(iface: str) -> None:
    """网卡不在就早点说清楚，别等 DDS 报一句看不懂的错。"""
    if Go2Robot.interface_exists(iface):
        return
    avail = ", ".join(n for _, n in socket.if_nameindex())
    raise RuntimeError(f"本机没有网卡 {iface}。当前可用：{avail}")


class _Delegate:
    """把动作转给 `self.client`，转的同时不掺任何判断。

    `client` 是两份高层控制器之一，它们的方法集一致：
    `Init / SetTimeout / BalanceStand / Move / StopMove`，外加不阻塞版的
    `jump_async / crouch_async / uncrouch_async / stand_async / stop_async /
    freeze / park`。所以这里不需要按后端分支 —— 一旦需要，就说明两份的接口
    又漂了，该去改控制器而不是在这里加 if。
    """

    kind = ""
    label = ""
    target = ""
    client = None

    def start(self) -> int:
        """SetTimeout → Init → BalanceStand，返回码非 0 = 对端没响应。"""
        self.client.SetTimeout(3.0)
        self.client.Init()
        return self.client.BalanceStand()

    def jump(self) -> int:
        """跳跃。排完就返回，不等落地（两份都是非阻塞的）。"""
        return self.client.jump_async()

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
        return self.client.freeze()

    def close(self) -> None:
        """收工。自研那份要蹲下再松（仿真桥不发指令就没力矩），官方停住即可，
        所以这件事由各自的 `park()` 决定，这里只转发。"""
        self.client.park()

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

    @property
    def busy(self):
        """原生服务在机器上跑，动作是同步 RPC，这边只有 progress 一个线索。"""
        return self.client.busy


#: 控制端名字 → 实现。`endpoint.preset` 决定能用哪些，见 robot.clients_for。
BACKENDS = {
    "ours": OursBackend,
    "official": OfficialBackend,
}
