"""直连低层：自己持有 `rt/lowcmd` 与 `rt/lowstate`，与高层运控平级。

`BackendLink` 是借道 `Go2Controller._publish` 写指令 —— 那是在底层之上的封装里开窗口，
指令要经过一个 200 Hz 回路和一套 `_mode` 状态机。这个文件不一样：它自己就是 `rt/lowcmd`
的一个节点，和官方运控（mcf）在同一层，中间没有别人的状态机。

## 三条硬约束，缺一条就不要用

1. **`ChannelFactoryInitialize` 一个进程只能调一次。** 重复调会抛异常。所以本类在
   `connect()` 里吞掉「已经初始化」这一种失败 —— 但不能让 `Go2Controller` 和它同时
   活着，否则两边都在往 `rt/lowcmd` 写。
2. **lowcmd 只能有一个写者。** 本类用 `claim()` 管自己这一侧的权，管不了进程里另一个
   对象。真正的保证是：用本类的进程不要同时开高层控制器。
3. **真机上 mcf 必须先让出去。** `Connect()` 里调用运动服务释放；程序结束时交还。
   这一步不做，你的指令和官方运控会一直拔河，症状是「参数怎么调都不对」。

## 起站不管

本类只做「读姿态、写关节、进阻尼」。从趴着怎么站起来，是运控的事 —— 接上之前机器人
应当已经站着。仿真里先起 `unitree_mujoco.py` 并让它站着；真机上先让 mcf 站好再接管。
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np

from quadruped.motion.odom import FootOdometer

# 与 LowCmd / LowState 的电机顺序一致：FR, FL, RR, RL。
NUM_MOTOR = 12

# 真机 PM_Servo。仿真桥忽略这个字段，但保持一致省得两头不同。
MODE_SERVO = 0x01

# 收工阻尼发多少帧、间隔多久。与 `Go2Controller.damping` 同一组数。
DAMP_REPEATS = 10
DAMP_INTERVAL = 0.01

# 被踢走的官方运控模式记在这里。进程正常退出时用不上（`close()` 自己有
# `_mcf_prev`），它救的是 kill -9 / OOM / 拔电源之后的那一次：那时没人交还，
# 官方遥控器失效、只能重开 mcf。`deploy/release_mcf.py` 读这个文件恢复。
# 放 /tmp 是因为掉电重启后系统本来就会重开 mcf，旧记录留着反而会还原错模式。
MCF_STATE = Path(os.environ.get("GO2_MCF_STATE", "/tmp/quadruped-mcf.json"))


# 记下「我们踢掉的是谁」。写文件失败不影响控制 —— 这不是安全路径。
def _save_mcf_state(mode: str) -> None:
    try:
        MCF_STATE.parent.mkdir(parents=True, exist_ok=True)
        MCF_STATE.write_text(json.dumps({"mode": mode, "pid": os.getpid()}))
    except OSError:
        pass


# 交还成功之后删掉记录，免得下次启动按一个过期的模式去恢复。
def _clear_mcf_state() -> None:
    try:
        MCF_STATE.unlink()
    except OSError:
        pass


class DirectLink:
    """直接挂在 `rt/lowcmd` / `rt/lowstate` 上的 `LowLink` 实现。

    用法：

        link = DirectLink(domain=1, iface="lo0")
        if not link.connect():
            raise SystemExit("没收到 rt/lowstate")
        link.claim("policy")
        ...
        link.release("policy")
        link.close()
    """

    def __init__(self, domain: int = 1, iface: str | None = None,
                 release_mcf: bool = True, verbose: bool = True):
        self.domain = int(domain)
        self.iface = iface
        self.release_mcf = bool(release_mcf)
        # mcf 只踢一次。`claim()` 会被 `_rise()` 和 `motion.start()` 各调一遍，
        # 重复踢会多发两次 RPC，还会把 `_mcf_prev` 覆盖成「已经释放后的空」。
        self._mcf_taken = False
        self.verbose = verbose

        self._pub = None
        self._cmd = None
        self._crc = None
        self._owner: str | None = None
        self._mcf_prev = ""

        # 订阅回调写的这几个。初值 None，读到第一帧才算连上。
        self.q: np.ndarray | None = None
        self.dq: np.ndarray | None = None
        self.rpy: list[float] | None = None
        self._sport_pos: np.ndarray | None = None   # rt/sportmodestate 的 position
        self._sport_t = 0.0                         # 它最后一帧的时刻
        # 腿部里程计。真机被 `lowcmd` 接管后位姿话题全部停发（`rt/sportmodestate`、
        # `rt/utlidar/robot_pose`、`rt/utlidar/robot_odom`、`rt/lf/sportmodestate`
        # 一起归零，2026-11-17 实测），`pos` 会永远停在接管前那一帧 —— VLA 的
        # `memory` 只能记下起点那几格，每一拍都在「绕到墙后面那个点」。
        self.odom = FootOdometer()
        self._odom_t = 0.0
        self.frames = 0

    # 位置。**官方里程计优先，停发了就用腿算。**
    #
    # `rt/sportmodestate` 在仿真桥里一直在发，真机上被 `lowcmd` 接管后就停了 ——
    # 所以这个属性在两种环境里的来源不一样，但对上层是同一个东西。
    # 0.5 s 的新鲜度门限：DDS 偶尔抖一两帧不该让它瞬间切走。
    @property
    def pos(self):
        if self._sport_pos is not None and time.monotonic() - self._sport_t < 0.5:
            return self._sport_pos
        if self.odom.frames > 0:
            return np.array([self.odom.x, self.odom.y, 0.0])
        return self._sport_pos

    # -- 连接 ------------------------------------------------------------

    # 起 DDS、挂订阅、建发布器，等到第一帧 rt/lowstate 才返回 True。
    #
    # `ChannelFactoryInitialize` 重复调会抛；这里把「已经初始化过」当作正常情况 ——
    # 同一个进程里先跑过别的客户端是常态。
    def connect(self, timeout: float = 3.0) -> bool:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        try:
            ChannelFactoryInitialize(self.domain, self.iface)
        except Exception as err:                            # noqa: BLE001
            # 已经初始化过就继续；其它错（网卡名不对）要浮出来。
            if "already" not in str(err).lower() and "init" not in str(err).lower():
                raise

        def on_low(msg):
            # 仿真侧在第一次填数之前会先把默认消息发出来几帧，四元数是全零。
            # 那种帧不能算数：由它换算出来的姿态是 NaN，而 `q0 = link.q` 拿来
            # 做「从当前姿态平滑接管」的起点时，全零关节角会让接管从一堆不存在的
            # 姿态开始。丢掉它，等真帧。
            quat = msg.imu_state.quaternion
            if abs(quat[0]) + abs(quat[1]) + abs(quat[2]) + abs(quat[3]) < 1e-6:
                return
            self.q = np.array([msg.motor_state[i].q for i in range(NUM_MOTOR)])
            self.dq = np.array([msg.motor_state[i].dq for i in range(NUM_MOTOR)])
            self.rpy = [float(v) for v in msg.imu_state.rpy]
            self.frames += 1

            # 50 Hz 算一次就够。`lowstate` 是 500 Hz，逐帧差分等于拿关节编码器的
            # 量化噪声当速度；`FootOdometer` 内部还有一层低通，但源头先降频更省事。
            now = time.monotonic()
            if now - self._odom_t >= 0.02:
                self.odom.update(self.q, float(msg.imu_state.gyroscope[2]), now)
                self._odom_t = now

        ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)

        # 里程计单独一个话题。收不到不影响推理（策略只用本体感知），只是 `pos` 为 None。
        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            def on_sport(msg):
                self._sport_pos = np.array([float(msg.position[i])
                                            for i in range(3)])
                self._sport_t = time.monotonic()

            ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(on_sport, 10)
        except Exception as err:                            # noqa: BLE001
            self._log(f"里程计订阅不上：{type(err).__name__}")

        t0 = time.time()
        while self.frames == 0 and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.frames == 0:
            return False

        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_go_msg_dds__LowCmd_()
        self._cmd.head[0] = 0xFE
        self._cmd.head[1] = 0xEF
        for i in range(NUM_MOTOR):
            self._cmd.motor_cmd[i].mode = MODE_SERVO
        self._crc = CRC()

        # 这里**不碰**官方运控。挂上 DDS、收到 lowstate 只说明链路通，不代表
        # 这一路会真的接管。释放放在 `claim()` —— 见那里。
        self._log(f"直连 rt/lowcmd，domain {self.domain}，iface {self.iface}")
        return True

    # 把 mcf 从当前模式释放掉。失败不致命 —— 仿真里根本没有这个服务。
    #
    # 仿真是 `lo0`（`control/virtual/robot.py:313` 的约定），那边没有
    # `rt/api/motion_switcher/request` 的服务端。照旧发过去的结果是：DDS RPC
    # 等满 `SetTimeout(5.0)`，然后 SDK 内部 printf 一行
    #
    #     [ClientStub] send request error. id: 635093223554708
    #
    # 这行没有 `[direct]` 前缀，看起来像我们自己的错，实际上是「服务端不存在」
    # 的正常回执；代价还有每次启动白等 5 秒。仿真直接跳过。
    def _take_mcf(self) -> None:
        # 和 `control/controller.py:1651` 同一个判据：回环口上没有 mcf。
        # 写 `startswith("lo")` 而不是 `== "lo0"`，lo1 之类也一样是本地回环。
        if self.iface.startswith("lo"):
            self._log("仿真（lo0）没有 mcf，跳过释放")
            return
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
                import MotionSwitcherClient
            sc = MotionSwitcherClient()
            sc.SetTimeout(5.0)
            sc.Init()
            code, data = sc.CheckMode()
            cur = (data or {}).get("name", "") if code == 0 else ""
            if not cur:
                return
            # 先记下原模式再释放。顺序反了会留下一个「狗没人管」的状态：
            # `ReleaseMode()` 已经生效、但紧接着确认用的 `CheckMode()` 超时返回
            # 非 0 时，下面会 `return`，`_mcf_prev` 还是空 —— `close()` 就不还了，
            # 官方遥控器从此没反应，只能重开 mcf 才救回来。
            self._mcf_prev = cur
            _save_mcf_state(cur)
            sc.ReleaseMode()
            code, data = sc.CheckMode()
            if code != 0 or (data or {}).get("name", ""):
                self._log("mcf 没让出去，先别接管")
                return
            self._log(f"已从 {cur} 释放 lowcmd")
        except Exception as err:                            # noqa: BLE001
            self._log(f"mcf 释放不可用（仿真里正常）：{type(err).__name__}")

    # -- 写入权 ----------------------------------------------------------

    # 拿写入权。**第一个拿到的人负责把官方运控踢走**。
    #
    # 释放原来写在 `connect()` 里，而 `connect()` 是 `PolicyBackend.__init__`
    # 调的 —— 只要这个类被构造一次，mcf 就没了，哪怕那一路根本不会 `start()`。
    # 2026-11-16 用户反馈「我选了官方控制器，结果还是被切到推理」，根因就是
    # 构造函数里干了只有「真正接管」时才该干的事。
    #
    # `claim()` 才是「从这一拍起我发命令」的那一步：`_rise()` 和
    # `MotionService.start()` 都走它，而官方控制器那条路一次都不会调。
    def claim(self, name: str) -> None:
        from quadruped.motion.link import LinkBusy
        if self._owner is not None and self._owner != name:
            raise LinkBusy(f"lowcmd 写入权在 {self._owner} 手上，{name} 拿不到")
        self._owner = name
        if self.release_mcf and not self._mcf_taken:
            self._mcf_taken = True
            self._take_mcf()

    def release(self, name: str) -> None:
        if self._owner == name:
            self._owner = None

    def _need_claim(self, what: str) -> None:
        from quadruped.motion.link import LinkBusy
        if self._owner is None:
            raise LinkBusy(f"没 claim 就写 {what}：先 claim('policy') 再下发")

    # -- 写 lowcmd -------------------------------------------------------

    def joint_target(self, q, *, kp=None, kd=None) -> None:
        self._need_claim("关节目标")
        n = NUM_MOTOR
        kp = self._fill(kp, 0.0)
        kd = self._fill(kd, 0.0)
        self._write(np.asarray(q, float).reshape(n), kp, kd, np.zeros(n))

    def joint_torque(self, tau) -> None:
        self._need_claim("力矩")
        tau = np.asarray(tau, float).reshape(NUM_MOTOR)
        if not np.all(np.isfinite(tau)):
            raise ValueError("力矩里有 NaN / Inf")
        n = NUM_MOTOR
        # q 填实测角：驱动器算 q_cmd − q 时这一项为 0，万一 kp 没清零也不是满量程阶跃。
        self._write(np.asarray(self.q, float), np.zeros(n), np.zeros(n), tau)

    # 标量或 12 维都收。
    def _fill(self, value, default: float) -> np.ndarray:
        if value is None:
            return np.full(NUM_MOTOR, default)
        return np.broadcast_to(np.asarray(value, float), NUM_MOTOR)

    def _write(self, q, kp, kd, tau) -> None:
        if self._pub is None:
            raise RuntimeError("还没 connect")
        for i in range(NUM_MOTOR):
            cmd = self._cmd.motor_cmd[i]
            cmd.q = float(q[i])
            cmd.dq = 0.0
            cmd.kp = float(kp[i])
            cmd.kd = float(kd[i])
            cmd.tau = float(tau[i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    # 松腿：kp=0，kd 保持，连发几帧。停发会让驱动器攥着最后一帧，腿是僵的。
    def damping(self, kd: float = 5.0) -> None:
        if self._pub is None or self.q is None:
            return
        q = np.asarray(self.q, float)
        kp = np.zeros(NUM_MOTOR)
        kdv = np.full(NUM_MOTOR, float(kd))
        zero = np.zeros(NUM_MOTOR)
        for _ in range(DAMP_REPEATS):
            self._write(q, kp, kdv, zero)
            time.sleep(DAMP_INTERVAL)

    # -- 读 --------------------------------------------------------------

    @property
    def height(self):
        # 本层没有运动学模型，算不出机身高度。推理判摔只看倾角，这一项返回 None，
        # 调用方要容忍 —— 不要在 motion 层再拉一份连杆参数进来。
        return None

    @property
    def owner(self) -> str | None:
        return self._owner

    # -- 收工 ------------------------------------------------------------

    # 松腿、把 mcf 还回去。顺序不能反：先还的话 mcf 接手时我们还在发。
    # 关闭连接。`damp=False` 时不发阻尼：仿真桥没有 lowcmd 超时，停发后保持最后
    # 一帧的 q_des / kp，下一次可以直接接着接管。真机必须阻尼，否则腿失力。
    def close(self, damp: bool = True) -> None:
        if damp:
            self.damping()
        if self._mcf_prev:
            try:
                from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
                    import MotionSwitcherClient
                sc = MotionSwitcherClient()
                sc.SetTimeout(5.0)
                sc.Init()
                sc.SelectMode(self._mcf_prev)
                self._log(f"已交还 {self._mcf_prev}")
                _clear_mcf_state()
            except Exception as err:                        # noqa: BLE001
                self._log(f"交还失败：{err}")
            self._mcf_prev = ""
        self._pub = None
        self._log("已关闭 lowcmd")

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[direct] {msg}", flush=True)
