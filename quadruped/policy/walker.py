"""Go2 步态策略：观测组装、前向、动作换算。

这是一份 teacher-student 蒸馏出来的 student，两段前向：

    30 帧 × 70 维观测 → adaptation_module(2100→256→128→2) → 隐变量 2 维
    30 帧观测 + 隐变量 = 2102 → actor_body(512→256→128→12) → 12 个关节偏移

## 这里的每个数字都是量出来的

观测顺序、缩放、时钟推进、动作换算都以训练环境的 `compute_observations()` 为准，
参数取自训练时的 `parameters.pkl`。顺序错一位不会报错，只会乱走 —— 改之前先对照
那两处，不要凭「应该是这样」调。部署侧的参考实现有一处标量错位（站宽/站长），
不要照它。

## 关节顺序要换

策略的 12 维按 FL/FR/RL/RR 排（`POLICY_JOINTS`），而 `LowCmd` / `LowState` 是
FR/FL/RR/RL。两边都是标准 Unitree 约定，只是腿的排列不同，所以进出都要过一次
`SDK_TO_POLICY`。忘了换的表现是「左右腿反着摆」—— 前腿模仿后腿的动作。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from quadruped.policy import training as T
from quadruped.policy.mlp import Mlp

# 一帧观测的构成（70）：重力 3 + 命令 15 + 关节角 12 + 关节速度 12
#            + 当前动作 12 + 上一帧动作 12 + 时钟 4。
OBS_DIM = T.NUM_OBS

# 历史帧数，乘出来就是 actor 的 2100 维输入。
HISTORY = T.NUM_HISTORY

NUM_JOINTS = 12
# 下面这一组的来源与理由都在 `quadruped/policy/training.py`，这里只做转发 ——
# 部署侧分散着抄过三份，改一处忘两处就是在拿刚度补解析错误。
ACTION_SCALE = T.ACTION_SCALE
HIP_SCALE = T.HIP_SCALE_REDUCTION
KP = T.STIFFNESS
KD = T.DAMPING

# Go2 的关节力矩上限（N·m）：髋与大腿 23.7，小腿 45.4。按 SDK 序重复四条腿。
# 只在直接算力矩时用得到 —— 走 `joint_target` 时上限在控制器那一侧。
TAU_LIMIT = np.array(T.TAU_LIMIT)

# 观测里的缩放。dof_vel 乘 0.05 是因为关节速度能到 ±20 rad/s，不缩就把其它项压没了。
DOF_POS_SCALE = T.DOF_POS_SCALE
DOF_VEL_SCALE = T.DOF_VEL_SCALE
CLIP_ACTIONS = T.CLIP_ACTIONS

# 策略的关节顺序（训练时的 joint_names），以及对应的默认站姿。
POLICY_JOINTS = (
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
)
DEFAULT_Q = np.array([
    0.1, 0.8, -1.5,
    -0.1, 0.8, -1.5,
    0.1, 1.0, -1.5,
    -0.1, 1.0, -1.5,
], dtype=np.float64)

# LowCmd/LowState 的腿序是 FR, FL, RR, RL。这组下标把 SDK 序换成策略序。
SDK_TO_POLICY = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
POLICY_TO_SDK = np.argsort(SDK_TO_POLICY)

# 15 条命令，顺序与训练配置一致。前三条是速度，其余是步态参数。
COMMAND_FIELDS = (
    "vx", "vy", "vyaw",          # m/s, m/s, rad/s
    "height",                     # 机身高度偏移，m
    "freq", "phase", "offset", "bound", "duration",
    "footswing",                  # 抬脚高度，m
    "pitch", "roll",              # rad
    "width", "length",            # 站距、步长，m
    "aux",                        # 辅助奖励系数，部署时给 0
)

# commands_scale。以**训练侧** `legged_robot.py:1196` 的定义为准：
#
#     idx 10 body_pitch_cmd (0.3)   idx 11 body_roll_cmd (0.3)
#     idx 12 stance_width   (1.0)   idx 13 stance_length  (1.0)
#     idx 14 aux_reward     (1.0)
#
# 参考部署实现里那个数组多插了一项 `body_pitch_cmd`（源码里是注释掉的
# `# 0,` 被删了但数组没跟着改），导致 11 之后整体错位一格：宽度的缩放变成 0.3、
# 长度拿到 1.0。照部署代码抄会把站宽压到三分之一。
COMMAND_SCALE = np.array(T.COMMAND_SCALE, dtype=np.float64)

# 每条命令的允许区间，取自训练配置的 limit_*（表在 training.py）：
#
#     vx / vyaw  ±1.0      lin_vel_x、ang_vel_yaw
#     vy         ±0.6      lin_vel_y
#     height     −0.25~0.15
#     freq       2~4 Hz
#     footswing  0.03~0.35 m
#     pitch      ±0.4 rad，roll 训练时锁 0（limit_body_roll = [0,0]）
#     width      0.1~0.45 m，length 0.35~0.45 m
#
# 超出训练范围的值策略没见过，输出不可预期 —— 不是「走得更快」，是摔。
#
# 唯一的例外是 vyaw，这里放宽到 ±4.0。训练区间是 ±1.0，但实测外推仍线性：
# pronk 原地转的角速度 0.338 / 0.549 / 0.795 / 1.047 / 1.282 / 1.703 rad/s 对应
# vyaw 1.0 / 1.5 / 2.0 / 2.5 / 3.0 / 4.0，倾角只从 7.1° 涨到 10.8°，位移始终
# <0.12 m，没有摔（`test/turn_sweep.py`，4 s，抬脚 0.10）。
#
# 卡在 1.0 的代价很直接：转 90° 要 4.7 s，而 2.5 只要 1.5 s。
#
# trot 对 vyaw 的响应在高段反而更好：vyaw=1.0 时原地只有 0.13 rad/s，
# 4.0 时 0.74（不带前进）/ 0.90（带 0.15）。15 s 连续转、正反向各测一次，
# 位移 0.13~0.18 m（半径 1 cm，纯原地），倾角 7~8°，体高稳定，没有打滑。
#
# 4.0 是外推的最边缘，真机上有没有打滑、发热没验证过。真机先用 2.0 量。
COMMAND_RANGE = dict(T.COMMAND_LIMIT)


# 默认命令：站住不动，小跑步频 3 Hz、相位差半周期。表在 `training.NOMINAL_COMMAND`，
# 取值理由也在那里。这里只负责按 `COMMAND_FIELDS` 的顺序摊平成 15 维向量。
#
# height 是 −0.05 而不是 0，这个数不是猜的：
#
#     body_height_cmd = [-0.25, 0.15]，num_bins_body_height = 1
#     单 bin 的格点 = linspace(-0.25 + 0.20, 0.15 - 0.20, 1) = [-0.05]
#
# 也就是说训练全程这一项固定 −0.05，策略没见过别的值。改它不会让机身变高变矮
# （实测 −0.20 与 +0.10 的稳态高度都是 0.22 m），只会送进一个分布外输入。
# 机身高度实际由 `DEFAULT_Q` 加策略动作决定，这一项只是告诉策略「对着哪个基准」。
def default_commands() -> np.ndarray:
    cmd = np.zeros(len(COMMAND_FIELDS), dtype=np.float64)
    for name, value in T.NOMINAL_COMMAND.items():
        cmd[COMMAND_FIELDS.index(name)] = value
    return cmd


# ZYX 姿态角转旋转矩阵（机体 → 世界）。
def rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


# 重力方向在机体系里的投影：Rᵀ·[0,0,−1]。
#
# 注意取的是**第三行**取负，不是第三列。R 把机体系映到世界系，所以「世界里的重力」
# 要左乘 Rᵀ 才落到机体系，而 (Rᵀv)ᵢ = Σⱼ R[j,i]·vⱼ —— v 只有 z 分量，展开就是 −R[2,:]。
# 取第三列等于用了 R 而不是 Rᵀ，差别在纯俯仰时刚好抵消（Ry 的第二行与第二列只差一个
# 符号位置），但一旦有偏航就会把航向混进重力：机器人转个身，它以为「下」也转了。
# 参考实现的部署代码是 `np.dot(R.T, [0,0,-1])`，用它对比过，差 0.22。
def gravity_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return -rot_zyx(roll, pitch, yaw)[2, :]


# 倾角：机体 z 轴与世界 z 轴的夹角，0 = 直立。
#
# 别拿 rpy 的分量直接比阈值：欧拉角有别名，(π, 0, θ) 与 (0, 0, θ+π) 是同一个姿态，
# 机身好好站着也可能报出 roll = −3.0。MuJoCo 桥的 IMU 就会落到这个分支上，
# 按 roll 判摔会在一秒内误停一个其实没摔的机器人。
def tilt_rad(rpy) -> float:
    # `rpy` 可能是 list、tuple 或 numpy 数组 —— numpy 数组的 `if not rpy` 会抛
    # 「truth value of an array is ambiguous」，所以只判长度。
    if rpy is None or len(rpy) < 3:
        return 0.0
    return float(math.acos(max(-1.0, min(1.0, -gravity_body(*rpy)[2]))))


class Walker:
    """策略运行时：持有历史与时钟，每步给 12 维关节目标（SDK 顺序）。

    `dt` 是控制周期，与训练一致（decimation 4 × 0.005 s = 0.02 s）。
    """

    def __init__(self, body_path: str | Path, adaptation_path: str | Path,
                 dt: float = 0.02, action_limit: float = CLIP_ACTIONS,
                 action_slew: float | None = None):
        self.body = Mlp(body_path)
        self.adaptation = Mlp(adaptation_path)

        expect_body_in = OBS_DIM * HISTORY + self.adaptation.n_out
        if self.adaptation.n_in != OBS_DIM * HISTORY:
            raise ValueError(
                f"{Path(adaptation_path).name} 要 {self.adaptation.n_in} 维输入，"
                f"按 {OBS_DIM}×{HISTORY} 算应该是 {OBS_DIM * HISTORY}")
        if self.body.n_in != expect_body_in:
            raise ValueError(
                f"{Path(body_path).name} 要 {self.body.n_in} 维输入，"
                f"按 {OBS_DIM * HISTORY}+{self.adaptation.n_out} 算应该是 {expect_body_in}")
        if self.body.n_out != NUM_JOINTS:
            raise ValueError(f"{Path(body_path).name} 输出 {self.body.n_out} 维，"
                             f"应该是 {NUM_JOINTS}")

        self.dt = float(dt)
        # 动作限幅与变化率限制。默认值就是训练时的 `clip_actions`，等于不额外加限制。
        # 两个都必须**在 Walker 里**做，不能放到调用方：观测里那一项 `actions` 是
        # 「刚执行完的动作」，调用方先限幅再发、网络内部却记着限幅前的值，历史就与
        # 实际执行脱节了。
        self.action_limit = float(action_limit)
        self.action_slew = None if action_slew is None else float(action_slew)
        self.reset()

    # 回到episode 起点：历史清零、时钟归零、动作清零。
    #
    # 历史清零是照训练来的 —— 每个 episode 开始时训练环境里也是全 0，策略见过。
    # 用第一帧观测铺满反而没见过。
    def reset(self, q_sdk=None) -> None:
        del q_sdk
        self.history = np.zeros(OBS_DIM * HISTORY, dtype=np.float32)
        self.action = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.last_action = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.gait_index = 0.0
        self.clock = np.zeros(4, dtype=np.float64)

    # 走一步：组装观测 → 推历史 → 前向 → 换算成关节目标（SDK 顺序）。
    #
    # 返回 (q_des_sdk, action)。`action` 是网络原始输出，留着看量级和记日志。
    def step(self, q_sdk, dq_sdk, rpy, commands: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        obs = self._observe(q_sdk, dq_sdk, rpy, commands)
        self.history = np.concatenate((self.history[OBS_DIM:], obs))

        latent = self.adaptation(self.history)
        action = self.body(np.concatenate((self.history, latent)))

        self.last_action = self.action
        self.action = self._shape_action(action)

        q_des_policy = self.action * ACTION_SCALE
        q_des_policy[[0, 3, 6, 9]] *= HIP_SCALE     # 髋关节行程减半，训练时就乘了 0.5
        q_des_policy = q_des_policy + DEFAULT_Q

        self._advance_clock(commands)
        return q_des_policy[POLICY_TO_SDK].copy(), self.action.copy()

    # 把网络原始输出收成实际下发的动作：先按绝对值限幅，再按每帧变化率限速。
    #
    # 顺序不能反。先限速后限幅的话，第一步从 0 到 ±limit 仍是一次满幅跳变，限速
    # 等于没做；先限幅再限速，每一步的增量都在 slew 之内。
    def _shape_action(self, action: np.ndarray) -> np.ndarray:
        out = np.clip(action, -self.action_limit, self.action_limit)
        if self.action_slew is not None:
            out = np.clip(out, self.action - self.action_slew,
                          self.action + self.action_slew)
        return out

    # 当前观测。调用前必须先有上一帧的 action / last_action。
    def _observe(self, q_sdk, dq_sdk, rpy, commands: np.ndarray) -> np.ndarray:
        q = np.asarray(q_sdk, dtype=np.float64)[SDK_TO_POLICY]
        dq = np.asarray(dq_sdk, dtype=np.float64)[SDK_TO_POLICY]
        obs = np.concatenate((
            gravity_body(*rpy),
            np.asarray(commands, dtype=np.float64) * COMMAND_SCALE,
            (q - DEFAULT_Q) * DOF_POS_SCALE,
            dq * DOF_VEL_SCALE,
            np.clip(self.action, -CLIP_ACTIONS, CLIP_ACTIONS),
            np.clip(self.last_action, -CLIP_ACTIONS, CLIP_ACTIONS),
            self.clock,
        ))
        if obs.size != OBS_DIM:
            raise ValueError(f"观测拼出来是 {obs.size} 维，应该是 {OBS_DIM}")
        return obs.astype(np.float32)

    # 时钟按步频推进，四足各取一个相位 —— 策略靠它知道现在该抬哪条腿。
    #
    # 顺序 [FL, FR, RL, RR]：相位差半分周期 + 基准偏移 0 时，得到 (FL,RR) 与
    # (FR,RL) 两组对角，也就是小跑。顺序若与训练不同，腿会两两同相，变成跳。
    def _advance_clock(self, commands: np.ndarray) -> None:
        freq = float(commands[COMMAND_FIELDS.index("freq")])
        phase = float(commands[COMMAND_FIELDS.index("phase")])
        offset = float(commands[COMMAND_FIELDS.index("offset")])
        bound = float(commands[COMMAND_FIELDS.index("bound")])

        self.gait_index = (self.gait_index + self.dt * freq) % 1.0
        g = self.gait_index
        feet = (g + phase + offset + bound, g + offset, g + bound, g + phase)
        self.clock = np.sin(2 * np.pi * np.array(feet))


# 策略文件默认位置。两个 jit 跟代码放一起，见 weights/README.md。
WEIGHTS_DIR = Path(__file__).resolve().parent / "weights"


# 组装一个默认策略。文件不在就抛出带路径的错误，别让人猜。
def load_walker(dt: float = 0.02, action_limit: float = CLIP_ACTIONS,
                action_slew: float | None = None) -> Walker:
    body = WEIGHTS_DIR / "body.jit"
    adaptation = WEIGHTS_DIR / "adaptation.jit"
    for path in (body, adaptation):
        if not path.is_file():
            raise FileNotFoundError(f"缺少策略文件 {path}，见 weights/README.md")
    return Walker(body, adaptation, dt=dt,
                  action_limit=action_limit, action_slew=action_slew)
