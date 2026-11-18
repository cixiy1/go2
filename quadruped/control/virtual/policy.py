"""控制端适配器：把推理控制器装进「控制端」这个位置。

## 两条依赖，方向相反，别搞反

    推理控制器 → 高层运动控制器      **解耦**。`quadruped/motion/` 里没有一行 import
                                    `control`，`test/arch_check.py` 规则 4 卡着。
    推理控制器 → 底层运动控制器      **耦合**。`DirectLink` 自己挂在 `rt/lowcmd` /
                                    `rt/lowstate` 上，和官方运控（mcf）同一层。

所以这个文件的位置容易看错：它在 `control/virtual/` 里面，但它是**适配器**，不是
推理控制器的一部分。它把 `Backend` 那套方法（`move` / `stop_motion` / `stand`）
翻译成 `MotionService` 的调用，让推理控制器能出现在控制端菜单里，和官方、自研并列。
反过来推理控制器对这里一无所知 —— 依赖箭头永远只有 control → motion。

## 能力是不全的，这是设计允许的

`Backend` 协议要求 `jump` / `crouch` / `forward` / `turn` / `jump` 这些，推理控制器
做不到：策略只会走，不会跳，蹲伏那个 height 命令在训练里恒为 −0.05（见
`policy/walker.py` 的说明），给的任何值都是分布外输入。

做不到的一律抛 `NotImplementedError`，不拿近似顶上。`action.py` 的 `call()` 会把它
转成 `ActionError`，上层据此换路子 —— 这条链本来就是这么设计的。
"""

from __future__ import annotations

import math
import time

import numpy as np

from quadruped.motion import DirectLink, MotionService
from quadruped.policy import gaits
from quadruped.policy.walker import (COMMAND_RANGE, DEFAULT_Q, KD, KP,
                                     POLICY_TO_SDK, tilt_rad)


class PolicyBackend:
    """推理控制器作为「控制端」。

    构造即连线（和另外两个后端一致），`start()` 才接管：拿 lowcmd 写入权、起 50 Hz
    线程。接管之前机器人应当是站着的 —— 从趴到站属于运控，这一层不管。
    """

    kind = "policy"
    label = "推理模型控制器"
    client = None               # 推理控制器没有「往下发的客户端对象」，动作直接落到自己身上

    # kp / kd 走 `MotionService`，与另外两个后端同名，启动器把同一组参数发给所有
    # 控制器时不会 TypeError。tau 是自研控制端的前馈系数，策略这一层没有对应项，
    # 收下但不使用。
    def __init__(self, endpoint, *, torque: str = "net", action_limit: float = 10.0,
                 vx_max: float | None = None, kp: float | None = None,
                 kd: float | None = None,
                 tau: float | None = None, warmup: float = 1.5, verbose: bool = True):
        del tau
        self.link = DirectLink(domain=endpoint.domain, iface=endpoint.iface,
                               verbose=verbose)
        if not self.link.connect():
            raise RuntimeError(
                f"{endpoint.iface}（domain {endpoint.domain}）3 秒内没收到 rt/lowstate。"
                "最常见的两个原因：仿真还在加载（窗口版首帧要 5~10 秒，再等一会重试）；"
                "或者窗口起不来（脚本 / 远程会话里没有图形会话，改用 "
                "--no-window）。先确认仿真进程还在：pgrep -fl unitree_mujoco.py")
        self.target = f"直连 rt/lowcmd / {endpoint.label}"
        self.svc = MotionService(self.link, torque=torque, action_limit=action_limit,
                                 kp=kp, kd=kd, verbose=verbose)
        # 上限默认取策略自己的训练范围（`COMMAND_RANGE` 的 vx，±1.0），不再在这里
        # 另写一个更小的数。原来写死 0.4，上层发 0.6 会被 `move()` 静默夹回 0.4 ——
        # 而策略是照 ±1.0 训的，那个 0.4 只是某个档位声明的巡航值，不是能力边界。
        # 要保守就由调用方显式传（键盘遥控传的是自己的 `--max-speed`）。
        self.vx_max = float(COMMAND_RANGE["vx"][1] if vx_max is None else vx_max)
        # 回环口就是仿真。它和真机在收工时的处理不一样：仿真桥没有 lowcmd
        # 超时，停发后保持最后姿态；真机没人发就失力。
        self.simulated = endpoint.iface == "lo0"
        # 接管后原地跑多久再接受速度。时钟从 0 起步，四腿相位要一两个周期才展开；
        # 演示脚本的「站 3 秒再走」就是这一段，键盘原来没有，接管 1 秒就加速，实测翻。
        self.warmup = float(warmup)
        self._running = False

    # -- 生命周期 --------------------------------------------------------

    # 接管。等价于另外两个后端的 BalanceStand —— 都是「从现在开始我发指令」。
    #
    # 接管前先把腿摆到策略的默认站姿：策略输出是相对 `DEFAULT_Q` 的偏移，从别的
    # 姿态起步等于第一帧就给一个 0.3 rad 量级的阶跃。`MotionService` 的起手渐变
    # 只有 0.6 s，压不住这个差；实测从 MuJoCo 初始姿态直接切，走 0.9 m 后翻了。
    #
    # 已经躺着就拒绝，不硬掰 —— 从 179° 拉回站姿会把腿顶飞，那是运控起站的活。
    def start(self) -> int:
        if self._running:
            return 0
        rpy = self.link.rpy
        tilt = tilt_rad(rpy)
        if tilt > math.radians(45):
            raise RuntimeError(
                f"机器人当前倾角 {math.degrees(tilt):.0f}°，已经侧翻，接管会把它硬掰。"
                "仿真里重启一次：pkill -f unitree_mujoco.py，再跑 "
                "./quadruped/sim/run_sim.sh；真机先走运控的 stand")
        self._rise()
        self.svc.start()
        # 速度命令还是 0，策略按默认命令原地踏步：让步态进入节律、姿态稳住。
        # 这段不是「等它站稳」，是等时钟跑完一两个周期，少了它起步就是斜的。
        time.sleep(self.warmup)
        self._running = True
        return 0

    # 从当前关节角线性插值到策略站姿，1.5 s，用训练增益。
    #
    # 硬切不行：`STAND_Q` 与任意起手姿态的差在髋上就有 0.16 rad、膝上 0.35 rad，
    # kp 80 时力矩到 28 N·m，左右一不对称就翻（policy_demo 里量过）。
    def _rise(self, seconds: float = 1.5) -> None:
        self.link.claim("policy")   # 先拿权再写；`svc.start()` 再 claim 同名是幂等的
        q0 = self.link.q
        if q0 is None:
            raise RuntimeError("拿不到关节角，无法摆到策略站姿")
        target = DEFAULT_Q[POLICY_TO_SDK]
        q0 = np.asarray(q0, dtype=float)
        frames = max(1, int(seconds / 0.02))
        for i in range(frames):
            s = (i + 1) / frames
            self.link.joint_target((1.0 - s) * q0 + s * target, kp=KP, kd=KD)
            time.sleep(0.02)

    # 收工：停线程 → 阻尼 → 交还 mcf。顺序在 `MotionService` 与 `DirectLink` 里各管一段。
    def close(self) -> None:
        if self._running:
            self.svc.stop(damp=not self.simulated)
            if self.simulated:
                # 策略停下的那一帧多半是行走中的摆动相，某条腿还抬着，桥照着这帧
                # 保持就会塌下去。先把腿摆回静态站姿，让桥保持这一帧。
                self.link.claim("policy")
                target = DEFAULT_Q[POLICY_TO_SDK]
                for _ in range(25):                 # 0.5 s，够腿收回来
                    self.link.joint_target(target, kp=KP, kd=KD)
                    time.sleep(0.02)
            self._running = False
        self.link.close(damp=not self.simulated)

    # -- 速度 ------------------------------------------------------------

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> dict:
        if not self._running:
            self.start()
        out = self.svc.set_velocity(vx=max(-self.vx_max, min(self.vx_max, float(vx))),
                                    vy=float(vy), vyaw=float(vyaw))
        if abs(float(vx)) > self.vx_max:
            out.setdefault("clamped", {})["vx"] = [float(vx), self.vx_max if vx > 0
                                                   else -self.vx_max]
        return out

    # 停车但保持站立。步态参数不动 —— 停住比停掉步态安全。
    def stop_motion(self) -> dict:
        return self.svc.set_velocity(0.0, 0.0, 0.0)

    # 回默认站姿命令。不重新起站：策略一直在发指令，这里只是把速度归零。
    def stand(self) -> int:
        self.svc.set_velocity(0.0, 0.0, 0.0)
        return 0

    # 策略没有「排好的一段动作」可打断，freeze 就是停车。
    def freeze(self) -> dict:
        return self.stop_motion()

    # -- 状态 ------------------------------------------------------------

    @property
    def pos(self):
        return self.link.pos

    @property
    def rpy(self):
        return self.link.rpy

    @property
    def height(self):
        # 本层没有运动学模型。策略的机身高度是关节角的几何后果，不是命令。
        return None

    # 策略从不阻塞，也没有「腿被占着」的状态。
    @property
    def busy(self) -> bool:
        return False

    @property
    def loop_text(self) -> str:
        if not self._running:
            return ""
        return f" 策略{self.svc.rate:.0f}Hz"

    # -- 做不到的能力 ----------------------------------------------------

    def _no_impl(self, what: str):
        raise NotImplementedError(f"{self.label}没有{what}：策略只会走，不会{what}")

    def jump(self) -> int:
        self._no_impl("跳")

    def crouch(self, depth: float = 0.08) -> int:
        # 蹲伏要改机身高度，而训练时 height 命令恒为 −0.05，策略没见过别的值。
        self._no_impl("蹲")

    def uncrouch(self) -> int:
        self._no_impl("蹲")

    def forward(self, distance: float, vx: float = 0.3, tol: float = 0.08) -> dict:
        # 策略能走，但没有里程计闭环的稳态精度；要做得先测出实际走多远。
        self._no_impl("闭环位移")

    def turn(self, degrees: float, yaw_rate: float = 0.5, tol: float = 0.1745) -> dict:
        self._no_impl("闭环转向")

    # 切步态。靠策略的相位三元组实现，不是高层运动模式 —— 目标端叫 "gait"，
    # 语义和官方后端的 trot/climb 不是一套，别把两边的名字混用。
    #
    # 名字与实测速度见 `quadruped/policy/gaits.py`。未知名字抛 ValueError，
    # comm 层会转成 400 并把可用列表回给调用方。
    def set_gait_style(self, style: str) -> str:
        out = self.svc.set_gait(style)
        return f"{out['gait']}（{out['label']}）"

    @property
    def loco_gait(self):
        return self.svc.gait

    # 不给名字就按预设顺序轮换。顺序 fixed 在 `gaits.PRESETS`，便于遥控器一个键循环。
    def toggle_loco_gait(self):
        names = gaits.names()
        current = self.svc.gait
        nxt = names[(names.index(current) + 1) % len(names)] if current in names else names[0]
        return self.set_gait_style(nxt)

    @property
    def jump_gait(self):
        return None

    def toggle_jump_gait(self):
        self._no_impl("切跳跃风格")

    # -- 关节直通 --------------------------------------------------------
    # 推理控制器自己就是关节指令的来源，外面再往里写会把它的观测搞乱。

    def joint_target(self, q, *, kp=None, kd=None, tau=None) -> int:
        self._no_impl("被外部写关节")

    def joint_torque(self, tau) -> int:
        self._no_impl("被外部写力矩")

    @property
    def q(self):
        return self.link.q

    @property
    def dq(self):
        return self.link.dq
