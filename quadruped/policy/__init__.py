"""策略层：预训练运动策略的加载与前向。

目前只有一个：`walker`（Go2 步态策略，输入 30 帧本体感知，输出 12 个关节目标）。
它不依赖 torch —— 权重从 TorchScript 归档里直接读。

训练时的控制参数集中在 `training`（刚度、阻尼、动作缩放、观测缩放、默认命令），
`walker` 与 `motion/service` 都从那里取；`test/policy_audit.py` 负责核对运行时
实际用的值有没有漂。

    from quadruped.policy import load_walker
    w = load_walker()                 # dt=0.02，50 Hz
    q_des, action = w.step(q, dq, rpy, commands)
"""

from quadruped.policy import gaits, training
from quadruped.policy.actuator import ActuatorNet, load_actuator
from quadruped.policy.mlp import Mlp, NotASequentialMlp
from quadruped.policy.walker import (
    COMMAND_FIELDS,
    DEFAULT_Q,
    HISTORY,
    KP,
    KD,
    NUM_JOINTS,
    OBS_DIM,
    Walker,
    default_commands,
    gravity_body,
    load_walker,
)

__all__ = [
    "COMMAND_FIELDS",
    "DEFAULT_Q",
    "HISTORY",
    "KD",
    "KP",
    "ActuatorNet",
    "Mlp",
    "NUM_JOINTS",
    "NotASequentialMlp",
    "OBS_DIM",
    "Walker",
    "default_commands",
    "gaits",
    "gravity_body",
    "load_actuator",
    "load_walker",
    "training",
]
