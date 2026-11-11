"""单刚体 MPC（SRBD）：从「要多大速度、多高、多平」直接解出四只脚该出多大地面反力。

    x = [p(3), v(3), rpy(3), ω(3)]     世界系
    u = 每个 knot 的四足三维地面反力      世界系

动力学就是刚体的牛顿-欧拉：

    ṗ = v
    v̇ = Σf / m + (0, 0, −g)
    Θ̇ = ω
    ω̇ = I⁻¹ (Σ r_i × f_i − ω × I ω)     r_i = 足端 − 质心

决策变量只有力，状态被「仿真」掉了（condensed QP）：给定 x₀ 和力序列，状态是力
的仿射函数，代进代价就只剩力。这样 QP 的规模是 12·N（N=12 时 144 维），OSQP 十
几微秒就解完，50 Hz 完全够。

为什么要它：现在这一层是 VMC —— 按「合力 + 合力矩」现算，只闭环当前这一帧，
既不考虑下一秒要落地、也不考虑磨擦锥在整个 horizon 上的可行性。双支撑时 6 个
力分量凑 6 维 wrench 本来就不定，条件数差（就是之前 6×3n 方程秩 5、解出 100 N
假力的根因）。MPC 靠 horizon 把力摊到多个 knot 上，这个问题自然消失。

几个必须说清楚的近似，都是这一层的边界：

1. 姿态用 rpy 的小角速度近似（Θ̇ = ω）。滚转俯仰超过 ±30° 就不准了，四足正常
   走路在 ±15° 以内，够用；做后空翻要换四元数版本。
2. I 在整个 horizon 上取常值，按当前姿态转到世界系。机身姿态在一个 240 ms 的
   horizon 里变化不大，这条误差是二阶的。
3. 足端位置取「本帧实测」，horizon 内认为不动 —— 也就是不管落脚点规划。落点
   由上层步态给，MPC 只负责力。这是 SRBD-MPC 的标准做法，也是它算得动的原因。
4. 接触序列是**给定**的（来自步态相位），MPC 不决定什么时候抬脚。接触序列错
   了，MPC 会一本正经地解出一个不存在的力。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np
import osqp
import scipy.sparse as sp

from .model import GRAVITY, TOTAL_MASS, whole_body_com, whole_body_inertia

LEGS = ("FL", "FR", "RL", "RR")


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def skew(v) -> np.ndarray:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp_ = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp_ * sr - sy * cr, cy * sp_ * cr + sy * sr],
        [sy * cp, sy * sp_ * sr + cy * cr, sy * sp_ * cr - cy * sr],
        [-sp_, cp * sr, cp * cr],
    ])


# 状态向量下标，写死在这儿，免得各处 offset 对不上。
P, V, RPY, W = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12)


@dataclass
class SRBDState:
    """机身的单刚体状态。p 是**质心**位置（不是 base_link 原点）。"""

    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def as_vector(self) -> np.ndarray:
        return np.concatenate([self.p, self.v, self.rpy, self.omega])

    @staticmethod
    def from_vector(x) -> "SRBDState":
        x = np.asarray(x, dtype=float)
        return SRBDState(x[P].copy(), x[V].copy(), x[RPY].copy(), x[W].copy())

    @property
    def R(self) -> np.ndarray:
        return rpy_to_R(*self.rpy)


@dataclass
class SRBDRef:
    """参考：走多快、转多快、机身多高。姿态一律要求水平。"""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    height: float = 0.27

    def state(self, p_xy, yaw) -> np.ndarray:
        x = np.zeros(12)
        x[P] = (p_xy[0], p_xy[1], self.height)
        x[V] = (self.vx, self.vy, 0.0)
        x[RPY] = (0.0, 0.0, yaw)
        x[W] = (0.0, 0.0, self.yaw_rate)
        return x


@dataclass
class MPCSolution:
    forces: np.ndarray          # (N, 4, 3) 世界系，N 是 horizon 的 knot 数
    contacts: np.ndarray        # (N, 4) bool
    feet_rel: np.ndarray = None  # (4, 3) 质心→足端，世界系
    R: np.ndarray = None        # 机身姿态
    info: dict = field(default_factory=dict)

    # 头一个 knot 的四足力，转成机体系的合力与绕质心的合力矩。
    #
    # 控制器那一层用的是 (F, M) 机体系表达（body_wrench_tau 的入参），所以
    # 这里一并给好，别让调用方自己转 —— 转错一次就是一整轮排查。
    def wrench_body(self) -> tuple[np.ndarray, np.ndarray]:
        R = np.eye(3) if self.R is None else self.R
        f = self.forces[0]
        tau = np.zeros(3)
        for i in range(f.shape[0]):
            tau += np.cross(self.feet_rel[i], f[i])
        return R.T @ f.sum(axis=0), R.T @ tau

    @property
    def f0(self) -> np.ndarray:
        return self.forces[0]


# ---------------------------------------------------------------------------
# MPC
# ---------------------------------------------------------------------------

class SRBDMPC:
    """单刚体 MPC。构造一次，solve() 每帧调，内部热启动。"""

    def __init__(self, mass: float = TOTAL_MASS, inertia=None,
                 *, mu: float = 0.6, fz_max: float = 220.0,
                 horizon: int = 12, dt: float = 0.02,
                 w_height: float = 4e3, w_v: float = 2e2,
                 w_rpy: float = 5e2, w_omega: float = 2e1,
                 w_force: float = 5e-4,
                 com_body=None, eps_abs: float = 1e-6, eps_rel: float = 1e-6,
                 polish: bool = False):
        self.mass = float(mass)
        self.I_body = (np.diag([0.17, 0.48, 0.53]) if inertia is None
                       else np.asarray(inertia, dtype=float))
        self.mu = float(mu)
        self.fz_max = float(fz_max)
        self.N = int(horizon)
        self.dt = float(dt)
        self.com_body = (np.zeros(3) if com_body is None
                         else np.asarray(com_body, dtype=float))
        self.eps_abs = float(eps_abs)
        self.eps_rel = float(eps_rel)
        self.polish = bool(polish)

        # 代价权重，按状态分块对角。x/y 位置不给权重：走多远是步态的事，
        # MPC 管不了落点，给它位置权重只会让它用「往前多蹬一脚」去凑，
        # 结果是机身被推着跑、脚在地上滑。
        self.Q = np.diag([0.0, 0.0, w_height,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0])
        # 速度三项都要（vz 也管：高度环不该只靠 pz）。
        self.Q[3, 3] = w_v
        self.Q[4, 4] = w_v
        self.Q[5, 5] = w_v * 0.5
        self.Q[6, 6] = w_rpy          # roll
        self.Q[7, 7] = w_rpy          # pitch
        self.Q[8, 8] = w_rpy * 0.2    # yaw 只做微调，转向靠 ωz
        self.Q[9, 9] = w_omega
        self.Q[10, 10] = w_omega
        self.Q[11, 11] = w_omega * 0.3
        self.R = np.eye(12) * w_force
        # 末端代价。horizon 最后一帧没有「再往后一步」的约束，不给它加权重，
        # 求解器就会用「让机身自己在末端往下掉」来省钱 —— 实测首帧竖直力因此
        # 比 mg 高 0.18%，一直站不住。末端把速度、姿态、角速度按同比例放大，
        # 这个偷工就没那么便宜了。
        self.Q_terminal = self.Q * 20.0

        self._solver = None
        self._P = None
        # 约束矩阵只跟 horizon、腿数、μ 有关，结构整场不变 —— 建一次存着。
        # 每帧变的只有上下界（哪条腿着地），OSQP 靠 update(l, u) 吃这个。
        self._A = self._constraints()
        self.solve_times = []

    # --- 建模 -------------------------------------------------------------

    # 当前姿态下的线性化：x_{k+1} = Ad·x_k + Bd_k·f_k + bd。
    #
    # `feet` 是**质心到足端**的向量（世界系），(4, 3)。
    def _matrices(self, R, omega, feet_rel):
        n = self.N
        I = R @ self.I_body @ R.T
        Ii = np.linalg.inv(I)
        m = self.mass

        A = np.zeros((12, 12))
        A[P, V] = np.eye(3)
        A[RPY, W] = np.eye(3)
        # 陀螺项 −ω × Iω 对 ω 线性化：−skew(ω)·I·ω → ∂/∂ω = −skew(ω)·I
        A[W, W] = -Ii @ skew(omega) @ I

        Bk = np.zeros((n, 12, 12))          # 每个 knot 一块，12 力 → 12 状态
        for k in range(n):
            for i in range(4):
                Bk[k, V, 3 * i:3 * i + 3] = np.eye(3) / m
                Bk[k, W, 3 * i:3 * i + 3] = Ii @ skew(feet_rel[i])

        Ad = np.eye(12) + A * self.dt
        Bd = Bk * self.dt
        bd = np.zeros(12)
        bd[5] = -GRAVITY * self.dt
        return Ad, Bd, bd

    # --- QP 组装 ---------------------------------------------------------

    # 把「状态 = 力的仿射函数」代进代价，得到 0.5 f'Hf + g'f。
    def _condense(self, x0, x_ref, Ad, Bd, bd):
        n, dt = self.N, self.dt
        # Ad 的幂：Ad^0..Ad^n。A 只跟姿态和 ω 有关，一个 horizon 内不变。
        pw = [np.eye(12)]
        for _ in range(n):
            pw.append(pw[-1] @ Ad)

        Sx = np.zeros((12 * n, 12))
        for k in range(1, n + 1):
            Sx[12 * (k - 1):12 * k] = pw[k]

        # S_f 的块 (k, j) = Ad^(k−1−j)·Bd_j，j < k
        Sf = np.zeros((12 * n, 12 * n))
        for k in range(1, n + 1):
            for j in range(k):
                Sf[12 * (k - 1):12 * k, 12 * j:12 * (j + 1)] = pw[k - 1 - j] @ Bd[j]

        # 常数项：重力那一段也要传下去，Sx·x0 之外单独算
        bias = np.zeros(12 * n)
        for k in range(1, n + 1):
            acc = np.zeros(12)
            for j in range(k):
                acc = Ad @ acc + bd
            bias[12 * (k - 1):12 * k] = acc

        Qbar = np.kron(np.eye(n), self.Q)
        Qbar[12 * (n - 1):, 12 * (n - 1):] = self.Q_terminal
        Rbar = np.kron(np.eye(n), self.R)

        # 只算上三角再对称，省一半乘法
        #
        # 外面那层 errstate 是给 macOS 用的：numpy 2.2 的 Accelerate 后端会把
        # BLAS 内部遗留的浮点异常标志当成自己的报出来，`rand(144,144) @ rand(144)`
        # 这种全正数相乘都会「divide by zero」。真出了问题解出来是 nan，
        # solve() 末尾会拦，所以这里不必让它刷屏。
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            e = Sx @ x0 + bias - x_ref
            SfQ = Sf.T @ Qbar
            H = SfQ @ Sf + Rbar
            g = SfQ @ e
        H = 0.5 * (H + H.T)
        return H, g

    # 约束矩阵结构固定，只有上下界随接触状态变 —— OSQP 的热启动才用得上。
    #
    # 摩擦锥写成四棱锥（`±fx ≤ μ' fz`、`±fy ≤ μ' fz`）：三次线性约束就能表达，
    # 圆锥得用二阶锥，OSQP 解不了。但四棱锥的两个棱角伸到 √2·μ'·fz，比真圆锥
    # 远 41% —— 直接拿 μ 当 μ' 用，等于把摩擦系数偷偷放大 41%（实测就是解出
    # fx = fy = μ·fz = 132 N、合力 187 N 的一天）。所以按 μ/√2 内接：棱角正好
    # 落在半径 μ·fz 的圆上，四个面中间最保守的地方差 30%，而正常走路需要的
    # |fh|/fz 只有 0.1 量级（15° 斜坡也才 0.27），这 30% 拿得出来。
    def _constraints(self):
        n = self.N
        mu_p = self.mu / math.sqrt(2.0)
        rows = []
        cols = []
        vals = []
        r = 0
        for k in range(n):
            for i in range(4):
                b = 12 * k + 3 * i
                for comp, sign in ((0, 1), (0, -1), (1, 1), (1, -1)):
                    rows += [r, r]
                    cols += [b + comp, b + 2]
                    vals += [sign, -mu_p]
                    r += 1
                rows.append(r)
                cols.append(b + 2)
                vals.append(1.0)          # fz ≤ fz_max
                r += 1
                rows.append(r)
                cols.append(b + 2)
                vals.append(-1.0)         # -fz ≤ 0
                r += 1
                # 三条恒等行：摆动腿靠它们把力钉成 0
                for comp in range(3):
                    rows.append(r)
                    cols.append(b + comp)
                    vals.append(1.0)
                    r += 1
        A = sp.csc_matrix((vals, (rows, cols)), shape=(r, 12 * n))
        return A

    def _bounds(self, contacts):
        n = self.N
        touch = np.asarray(contacts, dtype=bool).reshape(n * 4)
        lo = np.empty((n * 4, 9))
        up = np.empty((n * 4, 9))
        # 每个 (knot, 腿) 占 9 行：4 条摩擦锥、fz 上限、fz 下限、3 条恒等行
        lo[:, 0:4] = -np.inf
        up[:, 0:4] = 0.0
        lo[:, 4] = -np.inf
        up[:, 4] = self.fz_max
        lo[:, 5] = -np.inf
        up[:, 5] = 0.0
        # 恒等行：着地就不限制，摆动腿钉成 0（地面不会拽着脚）
        lo[:, 6:9] = np.where(touch[:, None], -np.inf, 0.0)
        up[:, 6:9] = np.where(touch[:, None], np.inf, 0.0)
        return lo.ravel(), up.ravel()

    # --- 主接口 -----------------------------------------------------------

    # 解一次。`feet_body` 是四只脚在机体系的位置（{腿名: (3,)}），`contacts`
    # 是 (N, 4) 的布尔表，顺序按 LEGS。
    #
    # 返回的力是**世界系**：地面反力的方向不随机身转，机身在滚转时这一点尤其
    # 重要，用机体系的力去积分会把机身越滚越歪。
    def solve(self, state: SRBDState, ref: SRBDRef, feet_body: dict,
              contacts, yaw: float | None = None) -> MPCSolution:
        t0 = time.perf_counter()
        R = state.R
        yaw = float(state.rpy[2]) if yaw is None else float(yaw)

        # 足端相对质心（世界系）
        feet_rel = np.zeros((4, 3))
        for i, name in enumerate(LEGS):
            p_body = np.asarray(feet_body[name], dtype=float)
            feet_rel[i] = R @ (p_body - self.com_body)

        Ad, Bd, bd = self._matrices(R, state.omega, feet_rel)
        # 参考逐 knot 展开：yaw 按指令往前积，其余保持不变。
        x_ref = np.zeros(12 * self.N)
        for k in range(self.N):
            x_ref[12 * k:12 * (k + 1)] = ref.state(
                state.p[:2], yaw + ref.yaw_rate * (k + 1) * self.dt)
        H, g = self._condense(state.as_vector(), x_ref, Ad, Bd, bd)

        lo, up = self._bounds(contacts)

        if self._solver is None:
            self._solver = osqp.OSQP()
            self._P = sp.csc_matrix(H)
            self._solver.setup(P=self._P, q=g, A=self._A,
                               l=lo, u=up,
                               verbose=False, eps_abs=self.eps_abs,
                               eps_rel=self.eps_rel,
                               max_iter=8000, warm_start=True,
                               polish=self.polish)
        else:
            # H 每帧都在变，但结构不变（稠密），直接把数拷进预分配的 csc：
            # csc 的 data 就是列优先展开，正好等于 H 的 Fortran 序。省掉
            # 每帧 200 µs 的 稀疏化。
            self._P.data[:] = H.ravel(order="F")
            self._solver.update(P=self._P, q=g, l=lo, u=up)

        res = self._solver.solve()
        self.solve_times.append(time.perf_counter() - t0)
        if len(self.solve_times) > 200:
            self.solve_times.pop(0)

        f = np.asarray(res.x, dtype=float).reshape(self.N, 4, 3)
        if not np.isfinite(f).all():
            # 解出 nan 只有一种原因：H 或 g 里有 inf/nan（状态炸了、姿态到奇异
            # 点、或者 I 不可逆）。这时候把力交给腿上是灾难，宁可当场炸。
            raise RuntimeError(f"MPC 解出发散：{res.info.status}")
        info = dict(status=res.info.status, obj=float(res.info.obj_val),
                    iter=int(res.info.iter),
                    solve_ms=1e3 * float(res.info.run_time),
                    total_ms=1e3 * (time.perf_counter() - t0))
        return MPCSolution(forces=f, contacts=np.asarray(contacts).astype(bool),
                           feet_rel=feet_rel, R=R, info=info)

    # 求一次并给出「第一帧四足力 + 世界系合力/合力矩」，接线时直接用这个。
    def first_forces(self, state, ref, feet_body, contacts) -> tuple:
        sol = self.solve(state, ref, feet_body, contacts)
        return sol.forces[0], sol

    # 最近若干次求解的耗时统计，验收用。
    def stats(self) -> dict:
        if not self.solve_times:
            return dict(n=0, mean=0.0, p95=0.0, max=0.0)
        d = sorted(self.solve_times)
        n = len(d)
        return dict(n=n,
                    mean=1e3 * sum(d) / n,
                    p95=1e3 * d[min(n - 1, int(n * 0.95))],
                    max=1e3 * d[-1])


# ---------------------------------------------------------------------------
# 接触序列：直接取步态相位，不另做接触规划
# ---------------------------------------------------------------------------

def contact_from_gait(gait, t0: float, n: int, dt: float) -> np.ndarray:
    """按现有步态相位生成 (n, 4) 的接触表。

    `gait` 只要有 period / duty / offsets 三个属性就够（GaitParams 正好是）。
    支撑判据就是 ph < duty —— 与 `_gait_feet_publish` 里算支撑权重的相位
    同一个式子，只是那边还有预载窗口和实测闸门，这里取干净的二值。
    """
    out = np.zeros((n, 4), dtype=bool)
    for k in range(n):
        t = t0 + k * dt
        for i, name in enumerate(LEGS):
            ph = (t / gait.period + gait.offsets[name]) % 1.0
            out[k, i] = ph < gait.duty
    return out


# ---------------------------------------------------------------------------
# 状态积分：预测用，也是跟 MuJoCo 对拍的那一支
# ---------------------------------------------------------------------------

# 用给定的四足力和足端位置推一步（显式欧拉，dt 小的时候足够）。
#
# `feet_rel` 是质心到足端的世界系向量，(4,3)。返回 ẋ（12 维），
# 拿来对拍比积分值本身更干净 —— 积分器差一阶，动力学不该差。
def dynamics(state: SRBDState, forces, feet_rel, mass, inertia) -> np.ndarray:
    R = state.R
    I = R @ inertia @ R.T
    f = np.asarray(forces, dtype=float)
    tau = np.zeros(3)
    for i in range(f.shape[0]):
        tau += np.cross(feet_rel[i], f[i])
    xd = np.zeros(12)
    xd[P] = state.v
    xd[V] = f.sum(axis=0) / mass + np.array([0.0, 0.0, -GRAVITY])
    xd[RPY] = state.omega
    xd[W] = np.linalg.solve(I, tau - np.cross(state.omega, I @ state.omega))
    return xd


# 站姿的整机参数，MPC 的默认值就从这儿来。腿的姿态会改变惯量 5% 左右，
# 蹲到 0.15 m 时才需要重算。
def nominal_params(height: float = 0.27) -> dict:
    q12 = np.tile([0.0, 0.9, -1.8], 4)
    return dict(mass=TOTAL_MASS, inertia=whole_body_inertia(q12),
                com_body=whole_body_com(q12))
