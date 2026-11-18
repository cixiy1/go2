"""步态预设：用三个相位参数拼出四路时钟。

`Walker._advance_clock()` 里四足相位是这四项的和：

    feet = (g + phase + offset + bound,   # FL
            g + offset,                   # FR
            g + bound,                    # RL
            g + phase)                    # RR

`g` 是公共步进，不影响相对关系，所以两个机器人之间的步态只由 (phase, offset,
bound) 决定。三个数能张成的组合不止三种，下面这几个是解出来的整数解：

    步态      phase  offset  bound   四足相位（FL, FR, RL, RR）
    pronk       0       0       0     0, 0, 0, 0
    trot      0.5       0       0     0, 0.5, 0.5, 0
    bound     0.5       0      0.5    0, 0, 0.5, 0.5
    pace        0       0      0.5    0, 0.5, 0, 0.5
    walk        0      0.25    0.5    0, 0.5, 0.75, 0.25

## 这些是量出来的，不是推出来的

`test/gait_sweep.py` 在 MuJoCo 里跑闭环，从足端轨迹的互相关反推相位，再按上表
分类。平地、vx 0.4、kp 20、freq 3.0、footswing 0.19、20 s：

    步态    实测形态  残差   位移     速度      yaw 漂移  最大倾角  抬脚
    pronk   pronk    0.02   4.95 m   0.25 m/s   −14.0°    6.5°     11.7 cm
    trot    trot     0.01   5.81 m   0.29 m/s    +1.3°    6.3°     15.0 cm
    bound   bound    0.01   6.26 m   0.31 m/s   +31.2°   18.1°      9.0 cm
    pace    pace     0.01   6.16 m   0.31 m/s    +2.5°   12.3°     13.3 cm
    walk    pace     0.08   4.45 m   0.22 m/s    −9.5°    8.5°      6.6 cm

残差是实测相位与上表理论相位的环形距离均值，0 最好。前四种都落在 0.02 以内，
说明策略确实按命令换了步态，不是四种说法指同一个动作。

两个例外要记住：

- `walk` 没走出来。实测相位 0 / 0.48 / 0.21 / 0.61，最接近 pace，抬脚只有
  6.6 cm。它的命令值离训练分布更远（offset 0.25、bound 0.5 同时偏离），保留
  这个预设只作为「不可用」的对照，不建议拿去演示。
- `bound` 会持续偏航（20 s 31°），`walk` 之外三种里它倾角最大（18°）。前腿
  同时落地时俯仰力矩没有对冲，开环下这条航向漂移是步态本身带来的。

速度能力见 `test/gait_sweep.py --sweep-vel`：vx 命令乘以约 0.72 才是实际速度，
后退明显更弱（命令 −0.4 实测 −0.07）。这些数是在 kp 20 的 MuJoCo 闭环里量的，
真机不适用。
"""

from __future__ import annotations

import numpy as np

from quadruped.policy import walker as W

# 每个预设只写三个相位参数。其余命令（freq、footswing、width…）保持调用方当前值，
# 切步态不该顺手改速度或抬脚高度。
PRESETS: dict[str, dict[str, float]] = {
    "pronk": {"phase": 0.0, "offset": 0.0, "bound": 0.0},
    "trot": {"phase": 0.5, "offset": 0.0, "bound": 0.0},
    "bound": {"phase": 0.5, "offset": 0.0, "bound": 0.5},
    "pace": {"phase": 0.0, "offset": 0.0, "bound": 0.5},
    "walk": {"phase": 0.0, "offset": 0.25, "bound": 0.5},
}

# 中文名与一句话说明，给日志和演示用。别在别处再抄一份。
LABELS: dict[str, str] = {
    "pronk": "四条腿同相，跳",
    "trot": "对角小跑，最稳",
    "bound": "前后两拍，偏航",
    "pace": "左右两拍",
    "walk": "四拍，实测未达成",
}

_PHASE_KEYS = ("phase", "offset", "bound")


# 预设名列表，带顺序。演示按这个顺序轮。
def names() -> tuple[str, ...]:
    return tuple(PRESETS)


# 名字是否合法。大小写不敏感，允许 "trot" / "TROT"。
def resolve(name: str) -> str:
    key = str(name).strip().lower()
    if key not in PRESETS:
        raise ValueError(f"没有步态 {name!r}，可用：{', '.join(PRESETS)}")
    return key


# 把预设写进一条命令向量，返回新的向量（不改入参）。
def apply(cmd: np.ndarray, name: str) -> np.ndarray:
    key = resolve(name)
    out = np.asarray(cmd, dtype=np.float64).copy()
    for field, value in PRESETS[key].items():
        out[W.COMMAND_FIELDS.index(field)] = value
    return out


# 反查：这条命令对应哪个预设，对不上返回 None。
#
# `MotionService.set_command()` 允许直接改 phase/offset/bound，那时状态里的步态名
# 不能还留着上一个预设 —— 用它把名字改回 "custom"，别让 status 报错东西。
def match(cmd: np.ndarray) -> str | None:
    values = {key: float(cmd[W.COMMAND_FIELDS.index(key)]) for key in _PHASE_KEYS}
    for name, preset in PRESETS.items():
        if all(abs(values[key] - preset[key]) < 1e-9 for key in _PHASE_KEYS):
            return name
    return None
