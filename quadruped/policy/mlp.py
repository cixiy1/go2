"""从 TorchScript 归档里读权重，用 numpy 跑前向。

为什么不装 torch：这两个归档里只有一个 `Sequential(Linear, ELU, ...)`，装 torch 就是
为了四层矩阵乘。归档本身是 zip + 裸 float32，读出来直接算，50 Hz 一帧 2102×512×3
的乘加在 CPU 上是微秒级。

代价是只认这一种结构。所以 `_read_layers()` 会校验形状，并且在激活不是 ELU 时直接
报错 —— 结构一变就停下，不会静默算出错的动作。
"""

from __future__ import annotations

import zipfile
from pathlib import Path

import numpy as np


class NotASequentialMlp(ValueError):
    """归档的结构不是「Linear + ELU 交替的 Sequential」。"""


# 一层：权重 (out, in) 与偏置 (out,)。只做仿射，激活在外面加。
class Layer:
    __slots__ = ("w", "b")

    def __init__(self, w: np.ndarray, b: np.ndarray):
        self.w = w
        self.b = b

    def __call__(self, x: np.ndarray) -> np.ndarray:
        # macOS 的 Accelerate BLAS 会在内部置起浮点异常标志，numpy 事后读到就报
        # 「matmul 里除零」。结果本身是对的（与 float64 差 1e-6），但每次前向都刷
        # 一行 warning 会把这个服务的日志淹掉。
        with np.errstate(all="ignore"):
            return x @ self.w.T + self.b


# ELU(alpha=1)：y > 0 原样，y ≤ 0 取 expm1(y) ∈ (−1, 0]。
def elu(y: np.ndarray) -> np.ndarray:
    return np.where(y > 0, y, np.expm1(np.minimum(y, 0.0)))


class Mlp:
    """TorchScript 导出的 Sequential(Linear, ELU, ..., Linear)。

    `n_in` / `n_out` 从权重形状读出来，调用方据此校验观测维度和动作维度。
    """

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.layers = _read_layers(self.path)
        self.n_in = int(self.layers[0].w.shape[1])
        self.n_out = int(self.layers[-1].b.shape[0])

    # 前向。**最后一层不加激活** —— 导出的是 Sequential(Linear, ELU, ..., Linear)，
    # 输出层是裸线性的，动作可以到任意实数。
    #
    # 这里曾经对每一层都套了 ELU，包括输出层：负向动作被 expm1 压到 −1，正向原样保留，
    # 整个动作空间只剩正半边。后果是策略想弯腿（负动作）时最多给到 −1，×0.25 只有
    # −0.25 rad，足端抬不起来，必须把 kp 提到 70 才顶得动 —— 那是拿刚度补一个解析错误。
    def __call__(self, x) -> np.ndarray:
        out = np.asarray(x, dtype=np.float32).reshape(-1)
        if out.size != self.n_in:
            raise ValueError(f"{self.path.name} 要 {self.n_in} 维输入，收到 {out.size}")
        last = len(self.layers) - 1
        for i, layer in enumerate(self.layers):
            out = layer(out)
            if i < last:
                out = elu(out)
        return out

    def __repr__(self) -> str:
        shape = " → ".join(str(n) for n in
                           [self.n_in] + [int(l.b.shape[0]) for l in self.layers])
        return f"Mlp({shape}, ELU)"


# 读归档。TorchScript 把每个张量存成 `<root>/data/<序号>`，裸 float32，无头部。
#
# 序号按数值排（`data/10` 不能排在 `data/9` 前面），再按 (权重, 偏置) 配对。
def _read_layers(path: Path) -> list[Layer]:
    with zipfile.ZipFile(path) as z:
        names = z.namelist()

        code = "".join(z.read(n).decode("utf-8", "replace")
                       for n in names if n.endswith(".py"))
        if "ELU" not in code:
            raise NotASequentialMlp(
                f"{path.name} 里没有找到 ELU，这个加载器只认 Linear+ELU 的 Sequential")

        roots = {n.split("/")[0] for n in names if "/data/" in n}
        if len(roots) != 1:
            raise NotASequentialMlp(f"{path.name} 里有多个根目录：{sorted(roots)}")
        root = roots.pop()

        idx = sorted(int(n.rsplit("/", 1)[1]) for n in names
                     if n.startswith(f"{root}/data/")
                     and n.rsplit("/", 1)[1].isdigit())
        if not idx or len(idx) % 2:
            raise NotASequentialMlp(f"{path.name} 的张量个数是 {len(idx)}，凑不成权重/偏置")

        flat = [np.frombuffer(z.read(f"{root}/data/{i}"), dtype=np.float32) for i in idx]

    layers: list[Layer] = []
    for w, b in zip(flat[0::2], flat[1::2]):
        out = b.size
        if w.size % out:
            raise NotASequentialMlp(f"{path.name} 里权重 {w.size} 除不尽偏置 {out}")
        layers.append(Layer(w.reshape(out, w.size // out).copy(), b.copy()))
    return layers
