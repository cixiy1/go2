"""步态参数向量与足端轨迹。

这里定义搜索空间：8 个标量决定一套步态。向量与 `GaitParams` 一一对应，
搜出来的结果可以直接 `to_gait_params()` 交给控制器，不需要转换层。

足端轨迹与 `controller.py` 的 `foot_target()` 同构 —— 相位 → 支撑/摆动 →
机体系足端目标。区别只有一处：这里不做机身层 wrench 分配，只输出足端目标，
力矩由 `env.py` 用 PD + 重力前馈算。
"""
from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from ..control.controller import (CTRL_LEGS, LEGS, NOMINAL_HEIGHT, Go2Controller,
                                  GaitParams, _raise, _swing_poly_coeffs)

# 搜索空间的顺序即参数向量的顺序，改动这里等于改所有 checkpoints 的含义。
FIELDS = ("period", "duty", "swing_height", "stride_max",
          "stance_spread", "stride_gain", "speed_loop", "stance_sink")

# 每个维度的下界、上界。范围来自实测边界，不是随意给的：
# - period 0.30 以下步频超过 3.3 Hz，200 Hz 回路只有 60 帧一个周期，PD 跟不上
# - duty 0.75 以上双支撑变长、腾空消失，等于四脚交替挪
# - stride_max 0.13 以上 IK 会削顶（腿水平行程不够）
# - stance_spread 0.045 以上髋侧摆 q1 长期非零，电机会一直出力
BOUNDS = {
    "period":        (0.30, 0.70),
    "duty":          (0.45, 0.75),
    "swing_height":  (0.040, 0.100),
    "stride_max":    (0.050, 0.130),
    "stance_spread": (0.000, 0.045),
    "stride_gain":   (0.80, 1.60),
    "speed_loop":    (0.00, 3.00),
    "stance_sink":   (0.000, 0.010),
}

# 基准点：mech 那套实测参数，搜索从这里出发，避免从零开始瞎撞。
BASELINE = dict(period=0.5, duty=0.5, swing_height=0.06, stride_max=0.075,
                stance_spread=0.0, stride_gain=1.0, speed_loop=0.0,
                stance_sink=0.0)


@dataclass(frozen=True)
class GaitSpec:
    """一套步态参数。不可变，便于当缓存键和多进程传递。"""

    period: float = 0.5
    duty: float = 0.5
    swing_height: float = 0.06
    stride_max: float = 0.075
    stance_spread: float = 0.0
    stride_gain: float = 1.0
    speed_loop: float = 0.0
    stance_sink: float = 0.0

    @classmethod
    def from_vector(cls, x, base: "GaitSpec | None" = None,
                    bounds: dict | None = None) -> "GaitSpec":
        """把 [0,1] 的归一化向量映射回物理量，越界就夹住。

        `bounds` 可以临时收窄某一维的搜索范围，不必改模块级 `BOUNDS` ——
        多进程评估时子进程拿不到运行时打的补丁，边界必须跟着 cfg 传下去。
        """
        base = base or cls()
        b = dict(BOUNDS)
        if bounds:
            b.update(bounds)
        raw = {}
        for i, k in enumerate(FIELDS):
            lo, hi = b[k]
            raw[k] = float(np.clip(lo + float(x[i]) * (hi - lo), lo, hi))
        return cls(**raw)

    def to_vector(self) -> np.ndarray:
        """逆映射，用于把基准点喂给优化器。"""
        return np.array([(getattr(self, k) - BOUNDS[k][0])
                         / (BOUNDS[k][1] - BOUNDS[k][0]) for k in FIELDS])

    def to_gait_params(self, height: float = NOMINAL_HEIGHT) -> GaitParams:
        """转成控制器认的 GaitParams。这条路是搜索结果的唯一出口。"""
        return GaitParams(
            period=self.period, duty=self.duty,
            swing_height=self.swing_height, stride_max=self.stride_max,
            stance_spread=self.stance_spread, stride_gain=self.stride_gain,
            speed_loop=self.speed_loop, stance_sink=self.stance_sink,
            neutral=Go2Controller.stand_feet(height, self.stance_spread),
            name="search")

    def as_dict(self) -> dict:
        return asdict(self)


class FootPlanner:
    """按相位算四足端在机体系的目标位置。

    与 `controller.py:foot_target` 同一套公式，唯一的差别是这里把速度闭环
    （`speed_loop`）显式写成入参，由调用方传入实测速度，便于在仿真里算。
    """

    def __init__(self, spec: GaitSpec, height: float = NOMINAL_HEIGHT):
        self.spec = spec
        self.height = height
        self.neutral = Go2Controller.stand_feet(height, spec.stance_spread)
        self.a3, self.a4 = _swing_poly_coeffs(spec.duty)
        # 相位偏移沿用 trot：对角腿同相，两组差半个周期
        self.offsets = {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5}
        self.nz = 0.022 - height          # FOOT_RADIUS - height

    # 单腿在时刻 t 的足端目标。vx/vy 为机身期望速度，vx_meas 用于速度闭环。
    def foot(self, leg: str, t: float, vx: float, vy: float,
             vx_meas: float = 0.0) -> tuple[float, float, float]:
        s = self.spec
        nx, ny, _ = self.neutral[leg]
        ph = (t / s.period + self.offsets[leg]) % 1.0

        # 速度闭环：按实测与指令之差放大后扫速度，替代开环步幅
        v_eff = vx + s.speed_loop * (vx - vx_meas)
        v_eff = float(np.clip(v_eff, -0.6, 0.6))
        dx = 0.5 * v_eff * s.period * s.duty * s.stride_gain
        dy = 0.5 * vy * s.period * s.duty * s.stride_gain
        dx = float(np.clip(dx, -s.stride_max, s.stride_max))
        dy = float(np.clip(dy, -s.stride_max, s.stride_max))

        if ph < s.duty:
            f = 0.5 - ph / s.duty
            x, y = nx + 2.0 * dx * f, ny + 2.0 * dy * f
            z = self.nz
            if s.stance_sink:
                u = ph / s.duty
                z -= s.stance_sink * min(1.0, u / 0.15, (1.0 - u) / 0.15)
        else:
            u = (ph - s.duty) / (1.0 - s.duty)
            c = self.a3 * u ** 3 + self.a4 * u ** 4
            x = (nx - dx) + 2.0 * dx * c
            y = (ny - dy) + 2.0 * dy * c
            z = self.nz + s.swing_height * _raise(u)
        return (x, y, z)

    # 四腿足端目标 + 每腿是否处于支撑相。
    def feet(self, t: float, vx: float, vy: float,
             vx_meas: float = 0.0) -> tuple[dict, dict]:
        feet = {leg: self.foot(leg, t, vx, vy, vx_meas) for leg in LEGS}
        stance = {}
        for leg in LEGS:
            ph = (t / self.spec.period + self.offsets[leg]) % 1.0
            stance[leg] = ph < self.spec.duty
        return feet, stance
