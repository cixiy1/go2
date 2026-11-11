"""整机刚体模型：连杆参数、运动学、单腿逆动力学。

数据源是 `quadruped/sim/unitree_mujoco/unitree_robots/go2/go2.xml`（宇树官方模型）。
质量取到克，惯量取到 12 位有效数字，`verify_against_xml()` 会重新解析那份 XML
逐项核对 —— 手抄的数是可能抄错的。

**惯量别直接抄 XML 里的 diaginertia。** XML 给的是「主轴系内对角 + 一个四元数」，
主轴系跟连杆坐标系不是一个朝向。机身那三个数 (0.107, 0.098, 0.024) 绕 y 转了
90°，直接当 (Ixx, Iyy, Izz) 用就等于把偏航惯量 0.107 当成横滚惯量 —— 横滚方向
算出来的力矩会大四倍。这里是 R·diag·Rᵀ 转回连杆坐标系之后的值。

单独一个模块的理由：MPC 要整机质量、质心、机身惯量，WBC 要单腿的 M / C / g，
两者都不该依赖 DDS，也不该为了拿一个 15.2 kg 去 import controller。
`controller.py` 反过来从这里取几何常数与总质量，全项目只有一份。

坐标系与 controller 一致：机体系 x 前、y 左、z 上，足端在机体系里 z 为负。
关节顺序 (abduction, thigh, knee)，单位 rad。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np

# 重力加速度。取 9.81 而不是 9.80665：和 controller 里的 GRAVITY 一致，
# 对拍的时候不该差在这一项上。
GRAVITY = 9.81

# ---------------------------------------------------------------------------
# 连杆几何（go2.xml 的 body pos 与 default class 里的 joint range）
# ---------------------------------------------------------------------------

HIP_X = 0.1934        # 髋部原点 |x|
HIP_Y = 0.0465        # 髋部原点 |y|
THIGH_Y = 0.0955      # 髋俯仰关节相对髋部原点的侧向偏移
THIGH_LEN = 0.213     # 大腿
CALF_LEN = 0.213      # 小腿
FOOT_RADIUS = 0.022   # 足端接触球半径，足底 = foot 原点 - 0.022

#        x          y        s（左右镜像符号）
LEG_GEOM = {
    "FL": (+HIP_X, +HIP_Y, +1.0),
    "FR": (+HIP_X, -HIP_Y, -1.0),
    "RL": (-HIP_X, +HIP_Y, +1.0),
    "RR": (-HIP_X, -HIP_Y, -1.0),
}

# 关节顺序，与 LowState / LowCmd 的 12 维向量一致。**不是** LEG_GEOM 的键顺序，
# 也不是 FL/FR/RL/RR 那个「几何顺序」—— 搞错等于左右腿对调，现象是看着相位
# 整体错半周期，很容易带偏诊断方向。
MOTOR_LEGS = ("FR", "FL", "RR", "RL")


# ---------------------------------------------------------------------------
# 连杆参数
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Link:
    """一根连杆：质量、质心、绕质心的惯量张量，全部在连杆坐标系里。"""

    name: str
    mass: float
    com: tuple
    inertia: tuple

    @property
    def c(self) -> np.ndarray:
        return np.array(self.com, dtype=float)

    @property
    def I(self) -> np.ndarray:
        return np.array(self.inertia, dtype=float)


# 机身。com 相对 base_link 原点，I 绕质心。
TRUNK = Link("trunk", 6.921,
             (0.021112, 0, -0.005366),
             ((0.0244800107685, 0.000121659816362, 0.00148480886899),
              (0.000121659816362, 0.0980769911862, -3.12004728255e-05),
              (0.00148480886899, -3.12004728255e-05, 0.107000198045)))

# 四条腿各三根连杆。前后髋的质心 x 反号、左右髋的质心 y 反号，惯量的非对角
# 项跟着反号，所以四份都得写，不能共用一个再镜像 —— 镜像规则写错一次要查半天。
_LINK_TABLE = {
    "FL": (
        Link("FL_hip", 0.678,
             (-0.0054, 0.00194, -0.000105),
             ((0.000479999841865, -3.01007669626e-06, 1.10994863885e-06),
              (-3.01007669626e-06, 0.000884000491707, -1.42003378192e-06),
              (1.10994863885e-06, -1.42003378192e-06, 0.000595999666429))),
        Link("FL_thigh", 1.152,
             (-0.00374, -0.0223, -0.0327),
             ((0.00584000338109, 8.71999511792e-05, -0.000289000716584),
              (8.71999511792e-05, 0.0058000033931, 0.00080800039502),
              (-0.000289000716584, 0.00080800039502, 0.00103000022581))),
        Link("FL_calf", 0.241352,
             (0.00629595, -0.000622121, -0.141417),
             ((0.0014635374889, 2.17544990652e-07, 5.96681421675e-06),
              (2.17544990652e-07, 0.00148999288168, 1.22478687985e-05),
              (5.96681421675e-06, 1.22478687985e-05, 5.32693294162e-05))),
    ),
    "FR": (
        Link("FR_hip", 0.678,
             (-0.0054, -0.00194, -0.000105),
             ((0.000479999841865, 3.01007669626e-06, 1.10994863885e-06),
              (3.01007669626e-06, 0.000884000491707, 1.42003378192e-06),
              (1.10994863885e-06, 1.42003378192e-06, 0.000595999666429))),
        Link("FR_thigh", 1.152,
             (-0.00374, 0.0223, -0.0327),
             ((0.00584000338109, -8.71999511792e-05, -0.000289000716584),
              (-8.71999511792e-05, 0.0058000033931, -0.00080800039502),
              (-0.000289000716584, -0.00080800039502, 0.00103000022581))),
        Link("FR_calf", 0.241352,
             (0.00629595, 0.000622121, -0.141417),
             ((0.0014635374889, -2.17544990652e-07, 5.96681421675e-06),
              (-2.17544990652e-07, 0.00148999288168, -1.22478687985e-05),
              (5.96681421675e-06, -1.22478687985e-05, 5.32693294162e-05))),
    ),
    "RL": (
        Link("RL_hip", 0.678,
             (0.0054, 0.00194, -0.000105),
             ((0.000479999841865, 3.01007669626e-06, -1.10994863885e-06),
              (3.01007669626e-06, 0.000884000491707, -1.42003378192e-06),
              (-1.10994863885e-06, -1.42003378192e-06, 0.000595999666429))),
        Link("RL_thigh", 1.152,
             (-0.00374, -0.0223, -0.0327),
             ((0.00584000338109, 8.71999511792e-05, -0.000289000716584),
              (8.71999511792e-05, 0.0058000033931, 0.00080800039502),
              (-0.000289000716584, 0.00080800039502, 0.00103000022581))),
        Link("RL_calf", 0.241352,
             (0.00629595, -0.000622121, -0.141417),
             ((0.0014635374889, 2.17544990652e-07, 5.96681421675e-06),
              (2.17544990652e-07, 0.00148999288168, 1.22478687985e-05),
              (5.96681421675e-06, 1.22478687985e-05, 5.32693294162e-05))),
    ),
    "RR": (
        Link("RR_hip", 0.678,
             (0.0054, -0.00194, -0.000105),
             ((0.000479999841865, -3.01007669626e-06, -1.10994863885e-06),
              (-3.01007669626e-06, 0.000884000491707, 1.42003378192e-06),
              (-1.10994863885e-06, 1.42003378192e-06, 0.000595999666429))),
        Link("RR_thigh", 1.152,
             (-0.00374, 0.0223, -0.0327),
             ((0.00584000338109, -8.71999511792e-05, -0.000289000716584),
              (-8.71999511792e-05, 0.0058000033931, -0.00080800039502),
              (-0.000289000716584, -0.00080800039502, 0.00103000022581))),
        Link("RR_calf", 0.241352,
             (0.00629595, 0.000622121, -0.141417),
             ((0.0014635374889, -2.17544990652e-07, 5.96681421675e-06),
              (-2.17544990652e-07, 0.00148999288168, -1.22478687985e-05),
              (5.96681421675e-06, -1.22478687985e-05, 5.32693294162e-05))),
    ),
}

LEG_LINKS = _LINK_TABLE

# 整机质量 15.206408 kg，不是 15.0。之前 BODY_MASS 那个估值让重力前馈常年少给
# 1.4%，单腿支撑时差 2 N，落地那一下全靠 kp 静差补。
TOTAL_MASS = TRUNK.mass + sum(lk.mass for links in LEG_LINKS.values() for lk in links)

# 单腿三根连杆合计。判断「腿自重该不该算进承重」时用得到。
LEG_MASS = sum(lk.mass for lk in LEG_LINKS["FL"])


# ---------------------------------------------------------------------------
# 旋转小工具
# ---------------------------------------------------------------------------

def rot_x(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(q: float) -> np.ndarray:
    c, s = math.cos(q), math.sin(q)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_to_R(q) -> np.ndarray:
    # MuJoCo / unitree 的四元数顺序是 (w, x, y, z)。
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# 手写叉乘。np.cross 一次 7.7 µs，这个 0.8 µs —— 差的是它对 axis/dtype 的
# 那套通用分支。RNEA 一次要叉十几下，200 Hz 下这就是一百多微秒。
def _cross(a, b) -> np.ndarray:
    return np.array((a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]))


_AX_X = np.array([1.0, 0.0, 0.0])
_AX_Y = np.array([0.0, 1.0, 0.0])
_G_VEC = np.array([0.0, 0.0, -GRAVITY])     # 机体系 z 向上，重力朝 -z
_ZERO_I = np.zeros((3, 3))


# ---------------------------------------------------------------------------
# 单腿
# ---------------------------------------------------------------------------

class Leg:
    """一条 3 自由度腿：abduction（绕 x）、thigh（绕 y）、knee（绕 y）。

    链是「髋部原点 → hip → thigh → calf → 足端」，各级连杆坐标系与 go2.xml
    的 body 一致：thigh 在 hip 侧向偏移 0.0955 m 处，calf 在大腿末端往下
    0.213 m，足端再往下 0.213 m。

    逆动力学用递推牛顿-欧拉。基座（hip 原点）当成固定，加速度初始化成 −g，
    重力就自动进方程，不用单独一项。
    """

    __slots__ = ("name", "links", "origin", "thigh_offset", "calf_offset",
                 "foot_offset", "mass", "com_local", "inertia")

    def __init__(self, name: str):
        x, y, s = LEG_GEOM[name]
        self.name = name
        self.links = LEG_LINKS[name]
        self.origin = np.array([x, y, 0.0])          # 髋部原点在机体系
        self.thigh_offset = np.array([0.0, s * THIGH_Y, 0.0])
        self.calf_offset = np.array([0.0, 0.0, -THIGH_LEN])
        self.foot_offset = np.array([0.0, 0.0, -CALF_LEN])
        # 连杆参数提前展开成数组。dataclass 的属性每次访问都会从 tuple 新建
        # 一个 ndarray，RNEA 里每条腿要取十几次，实测能占掉一半时间 ——
        # 200 Hz 下四个字节的列表推导都算成本，这个更不算小。
        self.mass = np.array([lk.mass for lk in self.links])
        self.com_local = np.array([lk.com for lk in self.links])
        self.inertia = np.array([lk.inertia for lk in self.links])

    # --- 运动学 -----------------------------------------------------------

    # 三个关节角 → 三个关节原点的位置、三个关节轴的方向（机体系）、足端位置，
    # 以及每一级连杆坐标系相对机体系的旋转。
    def _kin(self, q) -> dict:
        q1, q2, q3 = (float(v) for v in q)
        R1 = rot_x(q1)
        o1 = self.origin
        R2 = R1 @ rot_y(q2)
        o2 = o1 + R1 @ self.thigh_offset
        R3 = R2 @ rot_y(q3)
        o3 = o2 + R2 @ self.calf_offset
        foot = o3 + R3 @ self.foot_offset
        return dict(joints=(o1, o2, o3),
                    axes=(_AX_X, R1 @ _AX_Y, R2 @ _AX_Y),
                    rots=(R1, R2, R3),
                    foot=foot)

    # 足端在机体系的三维坐标。与 controller 的 fk_leg 是同一套公式（对拍过）。
    def fk(self, q) -> np.ndarray:
        return self._kin(q)["foot"]

    # 足端坐标对三个关节角的雅可比，3×3，J[:, k] = ∂p/∂q_k。
    #
    # 转轴关节的列就是 a_k × (p_foot − p_joint_k)。解析式比数值差分稳，
    # 也不受差分步长影响；WBC 里每帧都要用，别省这点。
    def jacobian(self, q) -> np.ndarray:
        k = self._kin(q)
        J = np.zeros((3, 3))
        for i in range(3):
            J[:, i] = _cross(k["axes"][i], k["foot"] - k["joints"][i])
        return J

    # 足端位置和雅可比一起给，少算一遍 `_kin`。WBC 每帧两条都要。
    def fk_jac(self, q):
        k = self._kin(q)
        J = np.zeros((3, 3))
        for i in range(3):
            J[:, i] = _cross(k["axes"][i], k["foot"] - k["joints"][i])
        return k["foot"], J

    # 整条腿的质心（机体系）与质量。
    def com(self, q):
        k = self._kin(q)
        m = 0.0
        c = np.zeros(3)
        for i, link in enumerate(self.links):
            m += link.mass
            c += link.mass * (k["joints"][i] + k["rots"][i] @ link.c)
        return c / m, m

    # --- 逆动力学 ---------------------------------------------------------

    # RNEA：M(q)·q̈ + C(q,q̇)q̇ + g(q) = τ + Jᵀ·f_ext。
    #
    # 返回的就是电机该出的三个关节力矩，符号直接可用（不需要再乘 −1）。
    #
    # `f_ext` 是足端受的外力，机体系三个分量（典型值：地面反力 (0,0,+147)）。
    # `self_weight=False` 把连杆质量和惯量清零，只剩外力那一项 —— controller
    # 里的 gravity_tau / body_wrench_tau 本来就是只算负载，用它来对拍。
    def rnea(self, q, dq=None, ddq=None, f_ext=None,
             self_weight: bool = True) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        dq = np.zeros(3) if dq is None else np.asarray(dq, dtype=float)
        ddq = np.zeros(3) if ddq is None else np.asarray(ddq, dtype=float)
        R1 = rot_x(float(q[0]))
        R2 = rot_y(float(q[1]))
        R3 = rot_y(float(q[2]))
        stages = ((R1, self.origin, _AX_X),
                  (R2, self.thigh_offset, _AX_Y),
                  (R3, self.calf_offset, _AX_Y))

        # ---- 前向：每一级连杆的角速度、角加速度、原点线加速度 ----
        w = np.zeros(3)
        wd = np.zeros(3)
        a = -_G_VEC                     # 基座固定，把重力伪装成基座加速度
        R_abs = np.eye(3)               # 本级相对机体系的累计旋转
        state = []
        for i, (R, p, axis) in enumerate(stages):
            w_p = R.T @ w
            wd_p = R.T @ wd
            # 子系原点跟着父系做刚体运动
            a_p = R.T @ (a + _cross(wd, p) + _cross(w, _cross(w, p)))
            w_i = w_p + axis * dq[i]
            wd_i = wd_p + axis * ddq[i] + _cross(w_p, axis * dq[i])
            R_abs = R_abs @ R
            state.append((R, p, axis, w_i, wd_i, a_p, R_abs))
            w, wd, a = w_i, wd_i, a_p

        # ---- 每根连杆的惯性力与惯性力矩 ----
        F, N = [], []
        for i in range(3):
            _, _, _, w_i, wd_i, a_i, _ = state[i]
            m = self.mass[i] if self_weight else 0.0
            I = self.inertia[i] if self_weight else _ZERO_I
            c = self.com_local[i]
            a_com = a_i + _cross(wd_i, c) + _cross(w_i, _cross(w_i, c))
            F.append(m * a_com)
            N.append(I @ wd_i + _cross(w_i, I @ w_i))

        # ---- 反向：从足端往髋部推关节力/力矩 ----
        f_ext = None if f_ext is None else np.asarray(f_ext, dtype=float)
        tau = np.zeros(3)
        f_c = np.zeros(3)          # 子链传到本级的力，已转到本级坐标
        n_c = np.zeros(3)
        for i in (2, 1, 0):
            R, p, axis, _, _, _, R_abs = state[i]
            f = F[i] + f_c
            n = N[i] + _cross(self.com_local[i], F[i]) + n_c
            if i == 2 and f_ext is not None:
                # 外力是外界施加给这条腿的，进方程时取负号。力臂直接用本级
                # 坐标里的 foot_offset；外力要从机体系转进本级，用**累计**
                # 旋转 R3，不是这一级的关节旋转 —— 这里错过一次，力矩凭空
                # 多出 20%，还两边对称得看不出来。
                f_e = R_abs.T @ f_ext
                n = n - _cross(self.foot_offset, f_e)
                f = f - f_e
            tau[i] = float(axis @ n)
            if i > 0:
                # 本级结果转到父级坐标：力转一下，力矩还要加上力臂的贡献
                f_up = R @ f
                f_c = f_up
                n_c = R @ n + _cross(p, f_up)
        return tau

    # 质量矩阵：单位加速度那一列，减去重力/速度偏置。三列各跑一次 RNEA，
    # 精确、短；WBC 用它算惯性项，别用有限差分（那个误差是 O(h)）。
    def mass_matrix(self, q) -> np.ndarray:
        base = self.rnea(q, np.zeros(3), np.zeros(3))
        M = np.zeros((3, 3))
        for i in range(3):
            e = np.zeros(3)
            e[i] = 1.0
            M[:, i] = self.rnea(q, np.zeros(3), e) - base
        return 0.5 * (M + M.T)

    # 速度项 + 重力项，b = C(q,q̇)q̇ + g(q)。
    def bias(self, q, dq=None) -> np.ndarray:
        return self.rnea(q, dq, None)

    # M(q) 与 h(q, q̇) 一起给。WBC 每帧每条腿都要这一对，`mass_matrix` +
    # `bias` 是 5 次 RNEA（1.14 ms / 四腿），这里 1 次 RNEA + 一组解析式。
    #
    # 质量矩阵按动能二次型求：M[k][j] = ∂²T/∂q̇_k∂q̇_j。关节 k 转动时，
    # 它后面（远离基座那一段）每根连杆的角速度贡献就是单位轴 a_k，质心线
    # 速度贡献是 a_k × (c_i − o_k)。所以
    #
    #   M[k][j] = Σ_{i ≥ max(k,j)}  m_i·(a_k×(c_i−o_k))·(a_j×(c_i−o_j))
    #                              + a_kᵀ·I_i·a_j
    #
    # 全是机体系里的定值，不依赖 q̇，比跑三遍 RNEA 便宜一个量级，且是精确值
    # （不是差分）。h 还是要 RNEA —— 科氏项没有这种短式子。
    def mass_and_bias(self, q, dq=None):
        kin = self._kin(q)
        o = kin["joints"]
        ax = kin["axes"]
        c = [o[i] + kin["rots"][i] @ self.com_local[i] for i in range(3)]
        # 各连杆的 3×3 惯量（机体系）
        I = [kin["rots"][i] @ self.inertia[i] @ kin["rots"][i].T
             for i in range(3)]
        # Jv[i][k]：关节 k 转动时连杆 i 质心的单位线速度
        Jv = [[None] * 3 for _ in range(3)]
        for i in range(3):
            for k in range(i + 1):
                Jv[i][k] = _cross(ax[k], c[i] - o[k])
        M = np.zeros((3, 3))
        for k in range(3):
            for j in range(k + 1):
                s = 0.0
                for i in range(max(k, j), 3):
                    s += self.mass[i] * float(Jv[i][k] @ Jv[i][j])
                    s += float(ax[k] @ I[i] @ ax[j])
                M[k, j] = M[j, k] = s
        return M, self.rnea(q, dq, None)

    # 纯重力项（含连杆自重）。静止悬挂时的关节力矩就是它。
    def gravity(self, q) -> np.ndarray:
        return self.rnea(q, np.zeros(3), np.zeros(3))

    # 纯负载项 −Jᵀ·f：等价于把连杆质量清零后的 RNEA。
    def payload_tau(self, q, f_ext) -> np.ndarray:
        return self.rnea(q, np.zeros(3), np.zeros(3), f_ext=f_ext,
                         self_weight=False)


LEGS = {name: Leg(name) for name in LEG_GEOM}


def leg(name: str) -> Leg:
    return LEGS[name]


# ---------------------------------------------------------------------------
# 整机
# ---------------------------------------------------------------------------

TRUNK_COM = TRUNK.c
TRUNK_INERTIA_AT_COM = TRUNK.I      # XML 给的惯量本来就是绕质心的


def _outer(v) -> np.ndarray:
    # 平行轴定理那一项：|v|²·E − v vᵀ（再乘质量）
    v = np.asarray(v, dtype=float)
    return float(v @ v) * np.eye(3) - np.outer(v, v)


def _as_leg_map(q) -> dict:
    if isinstance(q, dict):
        return q
    q = np.asarray(q, dtype=float)
    return {name: q[3 * i:3 * i + 3] for i, name in enumerate(MOTOR_LEGS)}


# 整机质心（机体系）。q 可以按 MOTOR_LEGS 顺序给 12 维，也可以给 {腿名: q3}。
def whole_body_com(q) -> np.ndarray:
    qmap = _as_leg_map(q)
    total = TRUNK.mass * TRUNK.c
    m = TRUNK.mass
    for name, qq in qmap.items():
        c, ml = LEGS[name].com(qq)
        total = total + ml * c
        m += ml
    return total / m


# 整机绕质心的转动惯量（机体系）。腿的姿态会改变它，站姿和蹲姿差 5% 左右，
# 别当成常数写死。
def whole_body_inertia(q) -> np.ndarray:
    qmap = _as_leg_map(q)
    com_all = whole_body_com(q)
    I = TRUNK.I + TRUNK.mass * _outer(TRUNK.c - com_all)
    for name, qq in qmap.items():
        L = LEGS[name]
        k = L._kin(qq)
        for i, link in enumerate(L.links):
            R = k["rots"][i]
            c = k["joints"][i] + R @ link.c
            I = I + R @ link.I @ R.T + link.mass * _outer(c - com_all)
    return I


# ---------------------------------------------------------------------------
# 自检：把表里的数和 go2.xml 对一遍
# ---------------------------------------------------------------------------

DEFAULT_XML = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "sim", "unitree_mujoco", "unitree_robots", "go2", "go2.xml")


# 重新解析 XML 核对每一根连杆的质量、质心、惯量，返回最大相对偏差。
#
# 只用 xml.etree，不加载 MuJoCo：那份 XML 是第三方克隆、被 .gitignore 了，
# 文件不在就返回 nan 让调用方跳过，不该因此让整个模块 import 失败。
def verify_against_xml(path: str | None = None, tol: float = 1e-9) -> float:
    import xml.etree.ElementTree as ET

    path = DEFAULT_XML if path is None else path
    if not os.path.exists(path):
        return float("nan")

    root = ET.parse(path).getroot()
    body = {}
    for b in root.iter("body"):
        name = b.get("name")
        ine = b.find("inertial")
        if name is None or ine is None:
            continue
        mass = float(ine.get("mass"))
        com = np.array([float(v) for v in ine.get("pos").split()])
        R = quat_to_R([float(v) for v in ine.get("quat").split()])
        diag = np.diag([float(v) for v in ine.get("diaginertia").split()])
        body[name] = (mass, com, R @ diag @ R.T)

    worst = [0.0]

    def cmp(link: Link, ref):
        mass, com, I = ref
        pairs = [(link.mass, mass)]
        pairs += list(zip(link.c, com))
        pairs += [(link.I[i, j], I[i, j]) for i in range(3) for j in range(3)]
        for got, want in pairs:
            worst[0] = max(worst[0], abs(got - want) / max(abs(want), 1e-6))

    cmp(TRUNK, body["base_link"])
    for name, links in LEG_LINKS.items():
        for part, link in zip(("hip", "thigh", "calf"), links):
            cmp(link, body[f"{name}_{part}"])

    if worst[0] > tol:
        raise AssertionError(
            f"连杆参数与 {path} 不一致，最大相对偏差 {worst[0]:.2e}")
    return worst[0]
