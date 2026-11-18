"""训练时用的电机模型：由位置误差与角速度的历史算出力矩。

训练配置里 `control_type = "actuator_net"`，环境加载的是
`resources/actuator_nets/unitree_go1.pt`（TorchScript 归档）。结构 6 → 32 → 32 → 1，
两层 softsign，12 个关节共用同一份权重。

## 输入顺序

按 `legged_robot.py` 的 `eval_actuator_network`，6 个数依次是：

    q − q_des, (q − q_des)₋₁, (q − q_des)₋₂, dq, dq₋₁, dq₋₂

第一项是 **q − q_des**，取反的。网络自己学出了符号，别按 PD 的习惯换成
`q_des − q` —— 那样出来的是反方向力矩，机器人会立刻发力顶住地面。

## 为什么要它

策略是在这个响应特性下学出来的。换成理想 PD（τ = kp·e − kd·q̇），同样的动作
落到不同的关节轨迹上，观测跟着偏，动作继续加大。实测在我们的 MuJoCo 里，
用理想 PD 时 30% 的帧顶在动作限幅上，机身沉在 0.23 m；训练时力矩是按
这里算出来的，两者不是一回事。
"""

from __future__ import annotations

import zipfile
from pathlib import Path

import numpy as np

# 每个关节的力矩上限，取自 Go2 的额定值：髋、大腿 23.7 N·m，小腿 45.43 N·m。
# 训练侧 `torque_limits` 用的是同一组数。
TAU_LIMIT = np.array([23.7, 23.7, 45.43] * 4, dtype=np.float32)


class NotAnActuatorNet(ValueError):
    """归档的结构不是「Linear + softsign 交替、输入 6 维」。"""


def _read_layers(path: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        code = "".join(z.read(n).decode("utf-8", "replace")
                       for n in names if n.endswith(".py"))
        if "softsign" not in code:
            raise NotAnActuatorNet(f"{path.name} 里没有 softsign，不是预期的电机模型")

        roots = {n.split("/")[0] for n in names if "/data/" in n}
        if len(roots) != 1:
            raise NotAnActuatorNet(f"{path.name} 里有多个根目录：{sorted(roots)}")
        root = roots.pop()

        idx = sorted(int(n.rsplit("/", 1)[1]) for n in names
                     if n.startswith(f"{root}/data/")
                     and n.rsplit("/", 1)[1].isdigit())
        if not idx or len(idx) % 2:
            raise NotAnActuatorNet(f"{path.name} 的张量个数是 {len(idx)}，凑不成权重/偏置")

        flat = [np.frombuffer(z.read(f"{root}/data/{i}"), dtype=np.float32) for i in idx]

    layers = []
    for w, b in zip(flat[0::2], flat[1::2]):
        out = b.size
        if w.size % out:
            raise NotAnActuatorNet(f"{path.name} 里权重 {w.size} 除不尽偏置 {out}")
        layers.append((w.reshape(out, w.size // out).copy(), b.copy()))
    return layers


# 电机模型。`reset()` 只是把两帧历史清零 —— 训练侧每个 episode 开头也是这样。
class ActuatorNet:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.layers = _read_layers(self.path)
        n_in = int(self.layers[0][0].shape[1])
        if n_in != 6:
            raise NotAnActuatorNet(f"{path.name} 的输入是 {n_in} 维，预期 6 维")
        self.n_joints = 12
        self.reset()

    def reset(self) -> None:
        self._err1 = np.zeros(self.n_joints, dtype=np.float32)
        self._err2 = np.zeros(self.n_joints, dtype=np.float32)
        self._vel1 = np.zeros(self.n_joints, dtype=np.float32)
        self._vel2 = np.zeros(self.n_joints, dtype=np.float32)

    # 一次算 12 个关节。q / q_des / dq 都是 LowCmd 序（FR, FL, RR, RL）。
    #
    # 历史只在调用之后推进，所以同一组观测重复调用会得到同一个结果 —— 调用方
    # 按控制周期（训练是 50 Hz）调它，不要每个物理步都调。
    def torque(self, q, q_des, dq) -> np.ndarray:
        q = np.asarray(q, dtype=np.float32).reshape(self.n_joints)
        q_des = np.asarray(q_des, dtype=np.float32).reshape(self.n_joints)
        dq = np.asarray(dq, dtype=np.float32).reshape(self.n_joints)

        err = q - q_des
        x = np.stack([err, self._err1, self._err2, dq, self._vel1, self._vel2],
                     axis=1).astype(np.float32)

        last = len(self.layers) - 1
        with np.errstate(all="ignore"):
            y = x
            for i, (w, b) in enumerate(self.layers):
                y = y @ w.T + b
                if i < last:
                    # softsign：x / (|x| + 1)，值域 (−1, 1)。
                    y = y / (np.abs(y) + 1.0)

        self._err2, self._err1 = self._err1, err
        self._vel2, self._vel1 = self._vel1, dq
        return np.clip(y[:, 0], -TAU_LIMIT, TAU_LIMIT)

    def __repr__(self) -> str:
        dims = [int(self.layers[0][0].shape[1])] + [int(b.shape[0]) for _, b in self.layers]
        return f"ActuatorNet({' → '.join(map(str, dims))}, softsign)"


# 电机模型的默认位置，与两个 jit 放在同一目录。
WEIGHTS_DIR = Path(__file__).resolve().parent / "weights"


# 组装一个默认电机模型。文件不在就抛出带路径的错误，别让人猜。
def load_actuator() -> ActuatorNet:
    path = WEIGHTS_DIR / "actuator_net.pt"
    if not path.is_file():
        raise FileNotFoundError(f"缺少电机模型 {path}，见 weights/README.md")
    return ActuatorNet(path)
