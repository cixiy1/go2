"""训练时的控制参数，全项目唯一一份。

来源两处，都是实读出来的，不是抄文档：

    `parameters.pkl`（runs/gait-conditioned-agility/pretrain-go2）
        control.action_scale / hip_scale_reduction / stiffness / damping /
        decimation、sim.dt、env.num_observations / num_observation_history /
        clip_actions、obs_scales 与 commands_scale 的全表
    参考仓库部署侧 `go2_gym_deploy` 的 command_profile
        freq / phase / offset / bound / footswing / width / length / height 的默认值

`walker.py` 的常量与 `default_commands()` 都从这里取，`test/policy_audit.py` 拿这份表
去比对运行时实际用的值。改这里等于改「策略眼里的世界」，先跑一遍那份审计。

## 两个值看起来和部署侧不一样，是有意的

    height   训练配置 `body_height_cmd = [-0.25, 0.15]` 但 `num_bins_body_height = 1`，
             唯一格点是 −0.05 —— 训练全程这一项没变过，策略只认识它。部署侧写 0，
             那是给别的 checkpoint 留的。
    phase    训练侧四个相位参数按 bin 采样，0.5 与 0 都在分布内。部署侧默认 0
             （四条腿同相 = pronk），我们用 0.5（对角小跑），实测 tilt 更小、
             航向漂移更小，见 `gaits.py` 的表。

`actuator` 一项要单独说：训练用 `control_type = actuator_net` 的力矩模型，部署侧与
真机驱动器用 PD 20 / 0.5，两者在训练里是「同一个执行器的两种近似」。运行时默认走
PD（真机 LowCmd 就是这个语义），`--torque net` 换成模型；两条路在 MuJoCo 里的差别
见 `test/pd_audit.py` 的实测。
"""

from __future__ import annotations

# ---- 控制频率 ----------------------------------------------------------
SIM_DT = 0.005                  # 训练侧 sim.dt，200 Hz
DECIMATION = 4                  # 4 × 0.005 = 0.02 s → 策略 50 Hz
CONTROL_DT = SIM_DT * DECIMATION

# ---- 执行器 ------------------------------------------------------------
STIFFNESS = 20.0
DAMPING = 0.5
# 力矩上限（N·m）。髋与大腿 23.7，小腿 45.43，按 LowCmd 序重复四条腿。
TAU_LIMIT = (23.7, 23.7, 45.43) * 4

# ---- 动作换算 ----------------------------------------------------------
ACTION_SCALE = 0.25
HIP_SCALE_REDUCTION = 0.5

# ---- 观测 --------------------------------------------------------------
NUM_OBS = 70
NUM_HISTORY = 30
NUM_PRIVILEGED = 2
CLIP_ACTIONS = 10.0
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05

# 每条命令的缩放，顺序与 `walker.COMMAND_FIELDS` 一致。以训练侧 `legged_robot.py`
# 的 `commands_scale` 为准；参考部署实现那个数组多插了一项（源码里注释掉的 `0,`
# 被删了但数组没跟着改），11 之后整体错位一格 —— 照抄会把站宽压到三分之一。
COMMAND_SCALE = (2.0, 2.0, 0.25, 2.0,
                 1.0, 1.0, 1.0, 1.0, 1.0,
                 0.15, 0.3, 0.3, 1.0, 1.0, 1.0)

# 训练侧采样的上下界（`limit_*`）。超出这些范围的值策略没见过，输出不可预期。
COMMAND_LIMIT = {
    "vx": (-1.0, 1.0),
    "vy": (-0.6, 0.6),
    # 训练的 `ang_vel_yaw` 是 ±1.0，但实测外推到 ±4.0 仍线性且不摔（见
    # `walker.COMMAND_RANGE` 的注释），部署侧按 4.0 放。这一项是有意越界的。
    "vyaw": (-4.0, 4.0),
    "height": (-0.25, 0.15),
    "freq": (2.0, 4.0),
    "phase": (0.0, 1.0),
    "offset": (0.0, 1.0),
    "bound": (0.0, 1.0),
    "duration": (0.5, 0.5),
    "footswing": (0.03, 0.35),
    "pitch": (-0.4, 0.4),
    "roll": (0.0, 0.0),         # 训练时锁 0（limit_body_roll = [0,0]）
    "width": (0.1, 0.45),
    "length": (0.35, 0.45),
    "aux": (0.0, 0.01),
}

# 部署侧的默认命令。footswing 取训练标称 0.08 —— 它是落地冲击的唯一旋钮：
# 命令 0.19 实测触地 3.6 m/s，0.08 是 2.3 m/s（`test/pd_audit.py`）。
NOMINAL_COMMAND = {
    "vx": 0.0, "vy": 0.0, "vyaw": 0.0,
    "height": -0.05,
    "freq": 3.0,
    "phase": 0.5,               # trot。部署侧默认 0 是 pronk，理由见模块说明
    "offset": 0.0,
    "bound": 0.0,
    "duration": 0.5,
    "footswing": 0.08,
    "pitch": 0.0,
    "roll": 0.0,
    "width": 0.33,
    "length": 0.40,
    "aux": 0.0,
}
