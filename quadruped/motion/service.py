"""底层运动服务：50 Hz 跑策略，把 12 个关节目标下发电机的这一层。

## 它在哪

    文本模型（1 Hz）      给目标和基调，落在 15 条命令里的速度与步态参数
    A 模型（本模块，50 Hz） 30 帧观测 → 12 个关节目标
    电机（PD 20 / 0.5）    低层回路 200 Hz 也在跑，但 idle 时不发指令

`LowLink`（`quadruped/motion/link.py`）是本模块唯一的依赖面：读 q / dq / rpy / height，
写 `joint_target` / `joint_torque`，进 `damping`，外加 `claim` / `release` 拿写入权。
它不认识 `Go2Controller`，也不认识步态、MPC、动作库 —— 推理不该被那些东西牵制。

写入权是显式的：`start()` 先 `claim("policy")`，`stop()` 最后 `release`。以前靠
`joint_target()` 顺手把控制器置 idle 压住双写，那是隐式的，谁都不知道权在谁手上；
现在没拿权就写会当场抛 `LinkBusy`，冲突在第一帧暴露。

## 三个必须做的事

1. **起手渐变**。策略的第一步 q_des 与机器人当前姿态差多少不可控（我们的站姿
   0.0/0.9/−1.8，策略的默认站姿 0.1/0.8/−1.5）。硬切等于一个阶跃，膝上 20 N·m
   的冲击。所以前 `RAMP_S` 秒从当前姿态线性过渡到策略输出。
2. **观测历史先清零**。策略训练时每个 episode 开始时历史也是全 0，它见过；
   拿第一帧铺满反而是分布外的输入。
3. **命令超时归零**。文本模型那边断了，速度不能停在最后一个值上一直走。

## 执行器：两条路

策略输出的是 `q_des`（12 个目标角），从它到力矩有两条路，`torque=` 选：

    "pd"   策略的 q_des 直接进 motor_cmd，驱动器按 kp20 / kd0.5 算力矩。默认。
    "net"  用训练时的电机模型 actuator_net 自己算 τ，只发 tau、kp=kd=0。

"net" 不是在「用 PD」和「用模型」之间二选一 —— 驱动器的 PD 回路永远在跑，`kp=kd=0`
是让它贡献为零。「用权重表达执行器」的准确含义就是这个。

值不值也得说清楚：仿真里 `net` 更差（倾角 69° / 3.5 s 倒，PD 是 48° / 8 s 没倒）。但那条
对照的对方是理想关节，而 actuator_net 拟合的是真实电机 —— 仿真里没有它要拟合的东西，
所以这个结果不能用来否掉它在真机上的价值。能判它的只有真机。

## 不会自己站起来

服务假定机器人已经站着（`connect()` 之后自己 `stand()` 并等到标称高度）。从趴着
直接进策略，观测里的关节角与「站着的狗」差太远，策略只会抖。
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np

from quadruped.motion.link import LowLink
from quadruped.policy import gaits
from quadruped.policy import walker as W
from quadruped.policy.actuator import load_actuator

# 控制周期，与训练一致（decimation 4 × 0.005 s）。
DT = 0.02

# 起手从当前姿态过渡到策略输出的时长。
RAMP_S = 0.6

# 命令多久没更新就把速度归零。文本模型 1 Hz 下指令，3 秒已经算宽。
COMMAND_TIMEOUT = 3.0

# 姿态超过这个角度就判定摔倒，直接进阻尼。
FALL_RAD = 0.8

# 回路低于这个频率就报一行。训练时策略固定 50 Hz，而时钟推进用的是写死的 `DT`：
# 回路真跑到 45 Hz 时，步态节拍还是按 3.0 × 0.02 s 推，实际步频变成 2.7 Hz，
# 30 帧历史覆盖的是 44 ms 而不是 40 ms。策略不知道这件事，观测里也没有时间项 ——
# 这是唯一一条「运控能限制推理」的路径，所以必须能从日志里看见。
RATE_WARN = 47.0

# 摔倒时用的阻尼系数，与 `lcm_position_go2.cpp` 的 damping 模式一致。
DAMPING_KD = 5.0

# 动作限幅。默认 10 就是训练的 `clip_actions`，等于不额外限制。
#
# 仿真里这两个值能把「站得住」和「走起来」分开（见 TODO.md 第七节）：
#
#     限幅 ±0.5   最大倾角 3.8°，6 s 位移 −0.01 m      站住但不动
#     限幅 ±1.0   倾角 18°，8 s 位移 +0.05 m           开始晃
#     不限幅      走 0.88 m 后翻                       能走但稳不住
#
# 所以它不是「安全阈值」，是工作点。真机上策略的输出本来就落在 ±1 附近，不需要
# 动；仿真里把限幅收窄只是把过冲掐掉，代价是步幅一起没了。
ACTION_LIMIT = 10.0
# 动作变化率限幅（单位：每帧，20 ms）。None = 不限。
#
# 比绝对值限幅更有用：它不压稳态幅度，只拦单帧跳变。仿真里 slew 0.2 能跑满 10 s
# 不翻，但高度掉到 0.18 m —— 是蹲着蹭，不是走。同样属于「把发散摁住」而不是
# 「让它走对」。
ACTION_SLEW = None

# 跟踪诊断。设 `GO2_TRACK=1` 时每秒打一行：目标角与实测角的差、按 kp/kd 估出来的
# 力矩、关节速度峰值、几个关节超了额定力矩。
#
# 这三项分别对应三种「落地很响」的原因，分开看才知道该动哪个数：
#
#   误差大（>0.3 rad）   kp 撑不住，腿是软着走的，落地先塌再撑 —— 加 kp
#   速度峰值大（>6 rad/s）kd 没吸住冲击，落地那一下是硬碰硬 —— 加 kd
#   超限多              期望力矩已经顶在电机额定值上，再调增益只会更糟 —— 减动作幅度
#
# 默认关：真机日志本来就密，这个只在排查时开。
TRACK = os.environ.get("GO2_TRACK") == "1"


class MotionService:
    """持有一个 `Backend`，按 50 Hz 把策略输出发下去。

    线程模型：`start()` 起一条循环线程，`stop()` 停下并让腿变阻尼。主线程随时可以
    `set_command()`，下一帧生效。
    """

    def __init__(self, link, walker=None, dt: float = DT, verbose: bool = True,
                 action_limit: float = ACTION_LIMIT, action_slew: float | None = ACTION_SLEW,
                 torque: str = "pd", kp: float | None = None, kd: float | None = None):
        # `link` 是 `LowLink`：只需要读姿态、写关节、进阻尼这几个能力。传 `Backend`
        # 会当场报错 —— 控制器不该有 `move()` 这种入口，有了它就会去调。
        if not isinstance(link, LowLink):
            raise TypeError("要 LowLink（见 quadruped/motion/link.py），"
                            "传 Backend 先用 BackendLink 包一层")
        self.l = link
        self.walker = walker or W.load_walker(dt=dt, action_limit=action_limit,
                                              action_slew=action_slew)
        self.dt = float(dt)
        self.verbose = verbose
        self.action_limit = float(action_limit)
        self.action_slew = None if action_slew is None else float(action_slew)

        # 关节增益。默认是真机 LowCmd 的那一对（kp 20 / kd 0.5），仿真与真机用同一组：
        # 修掉 `mlp.Mlp` 的输出层 ELU 之后，kp 20 在 MuJoCo 里就能走（抬脚 15~19 cm）。
        self.kp = W.KP if kp is None else float(kp)
        self.kd = W.KD if kd is None else float(kd)

        # 力矩由谁算：驱动器（"pd"）还是训练时的电机模型（"net"）。
        if torque not in ("pd", "net"):
            raise ValueError(f"torque 只能是 'pd' 或 'net'，拿到 {torque!r}")
        self.torque_mode = torque
        self.act = load_actuator() if torque == "net" else None

        self.commands = W.default_commands()
        # 当前步态名。默认命令 phase 0.5 / offset 0 / bound 0 就是 trot；调用方直接
        # 改相位三元组后这里会变成 "custom"，不让状态报一个已经不成立的旧名字。
        self.gait_name = gaits.match(self.commands) or "custom"
        self.command_time = time.monotonic()
        self.action = np.zeros(W.NUM_JOINTS)

        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self.frames = 0
        self.rate = 0.0
        self.fell = False
        # 最近一帧的跟踪诊断（`GO2_TRACK=1` 时才算）。status() 会带出去。
        self.track = {"err": 0.0, "tau": 0.0, "dq": 0.0, "over": 0}
        self.last_error: str | None = None
        self._start_q: np.ndarray | None = None
        self._started = time.monotonic()

    # -- 生命周期 --------------------------------------------------------

    # 起服务。返回时线程已在跑，但第一帧还没发出去。
    #
    # `q0` 是起手姿态：不传就读当前关节角。拿不到关节角就不启动 —— 没有它就算不出
    # 「从哪儿渐变到策略输出」。
    def start(self, q0=None) -> None:
        if self._thread is not None:
            return
        # 先拿写入权再读姿态：拿不到就直接抛，不要读完姿态、起好线程才发现权不在手上。
        self.l.claim("policy")
        try:
            q = self.l.q if q0 is None else np.asarray(q0, dtype=float)
            if q is None or len(q) != W.NUM_JOINTS:
                raise RuntimeError("拿不到 12 维关节角，无法启动策略")
            self._start_q = np.asarray(q, dtype=float).copy()
        except Exception:
            self.l.release("policy")
            raise

        self.walker.reset()
        if self.act is not None:
            self.act.reset()
        self.command_time = time.monotonic()
        self._started = time.monotonic()
        self.fell = False
        self.frames = 0
        self._run.set()
        self._thread = threading.Thread(target=self._loop, name="motion", daemon=True)
        self._thread.start()
        self._log(f"策略已接管，50 Hz，kp {self.kp:g} kd {self.kd:g}，力矩来自"
                  f"{'actuator_net（kp=kd=0）' if self.act is not None else '驱动器 PD'}")

    # 停服务：先停线程（它不再写 lowcmd），再让腿进阻尼，最后交还写入权。
    #
    # 顺序反了就是「线程还在发、阻尼又被下一次循环覆盖」，腿会僵在最后一个目标上。
    # 停服务。`damp=False` 用于仿真：桥没有 lowcmd 超时，停发命令后它继续用最后
    # 一帧的 q_des / kp 保持，等于「原地站住」，下一次可以直接再接管。真机必须
    # damping —— 进程退出后没人发指令，腿会直接失力。
    def stop(self, damp: bool = True) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if damp:
            try:
                self.l.damping(DAMPING_KD)
            except Exception as err:                        # noqa: BLE001
                self._log(f"进阻尼失败：{err}")
        self.l.release("policy")
        self._log("策略已退出，腿进阻尼" if damp else "策略已退出，保持站姿")

    @property
    def running(self) -> bool:
        return self._thread is not None and self._run.is_set()

    # -- 命令 ------------------------------------------------------------

    # 改命令。只认 `COMMAND_FIELDS` 里的键，值裁剪到训练范围。
    #
    # 裁剪而不是报错：调用方是模型，报错它不知道该改多少。返回实际生效的值，
    # 让上层能看出哪一项被压住了。
    def set_command(self, **fields) -> dict:
        cmd = self.commands.copy()
        clamped = {}
        for key, value in fields.items():
            if key not in W.COMMAND_FIELDS:
                raise ValueError(f"没有命令 {key!r}，可用：{', '.join(W.COMMAND_FIELDS)}")
            lo, hi = W.COMMAND_RANGE[key]
            value = float(value)
            if value < lo or value > hi:
                clamped[key] = [value, min(max(value, lo), hi)]
                value = min(max(value, lo), hi)
            cmd[W.COMMAND_FIELDS.index(key)] = value
        self.commands = cmd
        self.command_time = time.monotonic()
        # 直接改了相位三元组的调用方会绕过 `set_gait()`，名字要跟着命令走。
        if {"phase", "offset", "bound"} & set(fields):
            self.gait_name = gaits.match(cmd) or "custom"
        out = {k: float(cmd[W.COMMAND_FIELDS.index(k)]) for k in fields}
        return {"command": out, "clamped": clamped} if clamped else {"command": out}

    # 切步态。只改 phase / offset / bound 三项，速度与抬脚高度保持当前值。
    #
    # 预设的来历与实测速度见 `quadruped/policy/gaits.py`。切之前不需要先停 ——
    # 相位在下一帧的时钟里生效，步态会在一两个周期内换过来。
    def set_gait(self, name: str) -> dict:
        key = gaits.resolve(name)
        self.commands = gaits.apply(self.commands, key)
        self.gait_name = key
        self.command_time = time.monotonic()
        self._log(f"步态切换到 {key}（{gaits.LABELS[key]}）")
        return {"gait": key, "label": gaits.LABELS[key],
                "command": {k: round(float(v), 3) for k, v in
                            zip(W.COMMAND_FIELDS, self.commands)}}

    @property
    def gait(self) -> str:
        return self.gait_name

    # 只改速度，步态参数保持。文本模型绝大多数时候只需要这三条。
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> dict:
        return self.set_command(vx=vx, vy=vy, vyaw=vyaw)

    def status(self) -> dict:
        out = {"running": self.running, "frames": self.frames,
               "rate_hz": round(self.rate, 1), "fell": self.fell,
               "torque": self.torque_mode, "gait": self.gait_name}
        try:
            out["q"] = [round(float(v), 3) for v in self.l.q]
            out["rpy"] = [round(float(v), 3) for v in self.l.rpy]
            # 直连实现没有运动学模型，报 None。判站立看倾角，本来也不该靠这个。
            h = self.l.height
            out["height"] = None if h is None else round(float(h), 3)
        except Exception as err:                            # noqa: BLE001
            out["error"] = f"{type(err).__name__}: {err}"
        out["command"] = {k: round(float(v), 3) for k, v in
                          zip(W.COMMAND_FIELDS, self.commands)}
        if TRACK:
            out["track"] = {k: (round(v, 3) if isinstance(v, float) else v)
                            for k, v in self.track.items()}
        if self.last_error:
            out["last_error"] = self.last_error
        return out

    # -- 循环 ------------------------------------------------------------

    def _loop(self) -> None:
        tick = time.perf_counter()
        prev = tick
        ramp_frames = max(1, int(RAMP_S / self.dt))
        self.rate = 0.0
        try:
            while self._run.is_set():
                now = time.perf_counter()
                dt = now - prev
                prev = now

                rpy = self.l.rpy
                tilt = W.tilt_rad(rpy)
                if tilt > FALL_RAD:
                    self.fell = True
                    self._log(f"倾角 {math.degrees(tilt):.0f}° 超过阈值，停")
                    self._run.clear()
                    break

                cmds = self._live_commands()
                q = self.l.q
                dq = self.l.dq
                if q is None or dq is None:
                    raise RuntimeError("没收到 rt/lowstate")
                q_des, _ = self.walker.step(q, dq, rpy, cmds)

                # 起手渐变：前 ramp_frames 帧从当前姿态线性过渡到策略输出。
                if self.frames < ramp_frames:
                    s = (self.frames + 1) / ramp_frames
                    q_des = (1.0 - s) * self._start_q + s * q_des

                # "net" 下 q_des 只是电机模型的输入，不下发。起手渐变照样有用：
                # 误差从 0 长起来，力矩跟着从 0 长，比一步到位干净。
                if self.act is not None:
                    self.l.joint_torque(self.act.torque(q, q_des, dq))
                else:
                    self.l.joint_target(q_des, kp=self.kp, kd=self.kd)

                # 跟踪诊断（`GO2_TRACK=1`）。算的是驱动器那套 PD 会给多少力矩：
                # τ = kp·|q_des − q| + kd·|dq|，再和额定力矩比。这不是实测力矩，
                # 但足够分清「撑不住」「没吸住」「顶到上限」三种情况。
                if TRACK and self.frames >= ramp_frames:
                    err = np.abs(np.asarray(q_des) - np.asarray(q))
                    spd = np.abs(np.asarray(dq))
                    tau = self.kp * err + self.kd * spd
                    self.track = {
                        "err": float(err.max()),
                        "tau": float(tau.max()),
                        "dq": float(spd.max()),
                        "over": int((tau > W.TAU_LIMIT).sum()),
                    }
                    if self.frames % 50 == 0:
                        self._log("跟踪：误差 {:.2f} rad  力矩 {:.1f} N·m  "
                                  "关节速度峰 {:.1f} rad/s  超限 {} 个".format(
                                      self.track["err"], self.track["tau"],
                                      self.track["dq"], self.track["over"]))
                self.frames += 1
                if self.frames % 50 == 0:                   # 每秒更新一次实测频率
                    self.rate = 50.0 / max(1e-6, time.perf_counter() - self._started)
                    self._started = time.perf_counter()
                    # 每 10 秒最多报一次，别把日志刷满。
                    if self.rate < RATE_WARN and (self.frames // 50) % 10 == 0:
                        self._log(f"回路只有 {self.rate:.0f} Hz（目标 50）—— "
                                  f"步频按 {self.rate / 50:.2f} 倍走慢，"
                                  f"观测历史也按同一比例变稀")

                # 补齐到下一帧。sleep 到「上一帧开始 + dt」，避免误差累积。
                sleep = self.dt - (time.perf_counter() - tick)
                tick += self.dt
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    tick = time.perf_counter()              # 落后太多就重新对齐
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            self._log(f"循环异常退出：{self.last_error}")
        finally:
            self._run.clear()

    # 命令超时就把速度清零，步态参数保留 —— 停住比停掉步态更安全：
    # 原地踏步不会倒，突然停摆会。
    def _live_commands(self) -> np.ndarray:
        if time.monotonic() - self.command_time <= COMMAND_TIMEOUT:
            return self.commands
        cmds = self.commands.copy()
        for key in ("vx", "vy", "vyaw"):
            cmds[W.COMMAND_FIELDS.index(key)] = 0.0
        return cmds

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[motion] {msg}", flush=True)
