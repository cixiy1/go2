"""全身控制（WBC）：给定机身 6 维 wrench 与四足轨迹，解出 12 个关节力矩。

    x = [a(12), f(12)]        a = 三个关节角加速度，f = 足端地面反力（机体系）

力矩不是决策变量，而是 (a, f) 的仿射函数：

    τ_i = M_i(q)·a_i + h_i(q, q̇) − J_iᵀ·f_i        h = C·q̇ + g

每腿三个关节、三个足端自由度，J_i 是 3×3 方阵、可逆 —— **腿内没有零空间**，
给定 a 和 f，τ 唯一。所以真正要「优化」的只有 f 的分配：四腿 12 个分量扣掉
6 维合力/合力矩，还剩 6 维内部力，那才是自由度所在。

## 和现在 VMC 那一层的区别

`body_wrench_tau` 给的是 τ = −Jᵀ·f：只有负载项，没有腿自重、没有惯性项、
没有科氏项。漏掉的量不小 —— 站姿膝力矩 4% 是腿自重，摆动腿整条腿的
惯性更是全靠外面那圈 kp 硬扛。这就是 kp 只能停在 120、再往下让就站不住
的直接原因：PD 要同时干两件事（跟踪轨迹 + 当逆动力学的近似）。

WBC 把逆动力学那一半算清楚，PD 只需补模型误差。

## 任务与优先级（加权最小二乘，权重差两个量级以上）

1. **机身 6 维 wrench**（最高，`w_wrench`）—— Σf 与 Σr×f 追 `F`、`M`。
   这是唯一真正决定「狗站不站得住」的一项，权重给到 1e5。
2. **足端轨迹**（高，`w_foot`）—— J·a = a_cmd − J̇·q̇，a_cmd 是足端 PD。
   四条腿一视同仁：支撑腿的 a_cmd 小（轨迹在地上往后扫），摆动腿的大。
3. **力分配正则**（低，`w_force`）—— 让 f 靠近给定的分配（MPC 的第一帧
   力，或 VMC 的分摊）。这一项就是 wrench 任务剩下的那 6 维内部力的
   「零空间姿态项」：同样能满足 6 维 wrench，腿往内夹还是往外撑由它定。
4. **力矩正则**（最低，`w_tau`）—— 压 ‖τ‖，抑制同一个 wrench 下的大力矩解。

## 硬约束

- τ 逐关节 ±`tau_max`。默认 20：Go2 关节额定 23.7，双支撑小跑静态就要
  17.2（实测），14 会把正常承重也削掉 —— 削了就只能靠少给力，机身塌下去。
  剩下 3.7 留给叠在上面的那圈小刚度 PD（kp 40、0.09 rad 误差约 3.6 N·m）
- 摩擦锥：按 μ/√2 内接的四棱锥（同 `mpc.py`，理由见那边的注释）
- 单边：0 ≤ fz ≤ fz_max
- 摆动腿 f ≡ 0（地面不会拽着脚）

## 支撑权重必须连续

力按 `w·f` 缩放：权重是步态给的 0→1 斜坡，于是「抬起一条腿」在方程里是
渐变的。按 0/1 二值传进来，换腿那一帧 f 从满值直接掉到 0，乘上雅可比就是
十几牛米的台阶 —— 实测帧间差峰值 17.5 N·m，噪声顺着减速器打出来。

## 边界（这一层没管的事）

- 足端任务是**机体系**的，机身自身的加速度/角速度没有进 J̇·q̇。机身怎么动
  由 wrench 任务和实际地面力决定，两边在 200 Hz 下差一阶小量。要做严格版本
  得把浮动基座 6 个自由度也放进 x，那就需要整机 18 自由度的 M（现在只有
  单腿的）。
- 不用 q̈ 的最小二乘修正，不用动力学一致性的额外约束行。四足的腿是独立
  支链，给定基座运动后各腿确实解耦，这个分解是成立的；代价是基座那一半
  只能靠 wrench 任务间接闭环。
- **腿内没有零空间**：每条腿 3 个关节、足端任务也是 3 维，J 是 3×3 方阵，
  给定足端加速度 q̈ 就唯一了。所以「姿态正则」不能挂在腿上，只能挂在力的
  分配上（任务 3）—— 这也是为什么不写 `N·τ_posture` 那一项。
- 机械步态 duty = 0.5、双支撑时间为 0，小跑任意时刻只有两条腿承重。6 个
  力分量凑 6 维 wrench，只剩 1 个自由度，Mx 和 My 由同一个竖直力差决定，
  想同时满足就得违反单边或摩擦锥。所以 `wrench` 误差不是零，是「最接近」
  （实测 p95 5.6，`body_wrench_tau` 是 14.3）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from .model import GRAVITY, MOTOR_LEGS, TOTAL_MASS, leg as leg_model
from .qp import DenseQP

# 腿序必须是**电机序**：τ 是 12 维向量，直接发到 rt/lowcmd 的 motor_cmd，
# 顺序错了就是腿互相接错。`model.py` 的 MOTOR_LEGS 与 controller 的
# CTRL_LEGS 是同一套，Step 3 接线时会在 controller 里断言这一点。
LEGS = MOTOR_LEGS

# 变量排布：x = [a0, f0, a1, f1, a2, f2, a3, f3]。一条腿的 6 个变量连在一起，
# 组装和读结果都少一层间接。
def A_OF(i: int) -> slice:
    return slice(6 * i, 6 * i + 3)


def F_OF(i: int) -> slice:
    return slice(6 * i + 3, 6 * i + 6)


NX = 6 * len(LEGS)

# 每腿的约束行数：6 条 τ 限幅 + 4 条摩擦锥 + fz 上下限 + 3 条 f≡0
ROWS_PER_LEG = 15


@dataclass
class WBCTarget:
    """一帧的期望。位置都是**机体系**（与 `Leg.fk` 一致）。"""

    feet: dict                                  # {腿名: 足端目标位置(3,)}
    feet_vel: dict = field(default_factory=dict)   # {腿名: 目标速度(3,)}
    contacts: dict = None                       # {腿名: bool}，缺省全着地
    F: np.ndarray = None                        # 期望合力（含承重，N）
    M: np.ndarray = None                        # 绕质心的合力矩（N·m）
    force: dict = None                          # {腿名: 分配目标(3,)}，力正则用
    weights: dict = None                        # {腿名: 0~1} 支撑权重

    # 支撑权重。**不要只给 0/1 的二值**：`contacts` 只在「这条腿能不能出
    # 力」这个意义上用，真正参与力分配的是权重的连续值。步态的预载窗口本来
    # 就是 0 → 1 的斜坡，按 0/1 传进来，换腿那一帧 f 从满值直接掉到 0，
    # 乘上雅可比就是十几牛米的力矩台阶 —— 实测帧间差峰值 17.5 N·m，噪声会
    # 顺着减速器打出来。
    def weight(self, name: str) -> float:
        if self.weights is not None:
            return max(0.0, min(1.0, float(self.weights.get(name, 0.0))))
        if self.contacts is None:
            return 1.0
        return 1.0 if self.contacts.get(name, True) else 0.0

    def vel(self, name: str) -> np.ndarray:
        v = self.feet_vel.get(name)
        return np.zeros(3) if v is None else np.asarray(v, dtype=float)


@dataclass
class WBCResult:
    tau: np.ndarray              # (12,) 关节力矩
    forces: np.ndarray           # (4, 3) 足端力，机体系
    accel: np.ndarray            # (4, 3) 关节角加速度
    info: dict = field(default_factory=dict)

    def force_map(self) -> dict:
        return {n: self.forces[i].copy() for i, n in enumerate(LEGS)}


# ---------------------------------------------------------------------------
# J̇·q̇
# ---------------------------------------------------------------------------

# 足端在机体系里的「科氏加速度」J̇·q̇（q̈ = 0 时足端的加速度）。
#
# 沿 q̇ 方向对雅可比做中心差分，不是对足端位置差分 —— 后者只能拿到速度。
# 摆动腿关节 5 rad/s、力臂 0.2 m 时这一项有 5 m/s²，跟足端 PD 的输出同量级，
# 漏掉它摆动腿就一直在追一个加速度偏小的目标。
def jdot_qdot(jac, q, dq, eps: float = 1e-4) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    Jp = jac(q + eps * dq)
    Jm = jac(q - eps * dq)
    return ((Jp - Jm) / (2.0 * eps)) @ dq


# ---------------------------------------------------------------------------
# WBC
# ---------------------------------------------------------------------------

class WBC:
    """构造一次，`solve()` 每帧调；约束结构固定，只换数值和上下界。"""

    def __init__(self, *, mu: float = 0.6, fz_max: float = 220.0,
                 tau_max: float = 20.0, com=None,
                 kp_foot: float = 3000.0, kd_foot: float = 120.0,
                 stance_foot: float = 1.0,
                 w_wrench: float = 1e5, w_foot: float = 1e3,
                 w_force: float = 1e-2, w_tau: float = 1e-4,
                 w_slew: float = 0.0,
                 eps_abs: float = 1e-6, eps_rel: float = 1e-6):
        self.mu = float(mu)
        self.fz_max = float(fz_max)
        self.tau_max = float(tau_max)
        self.com = np.zeros(3) if com is None else np.asarray(com, dtype=float)
        # 足端任务增益。按「关节 kp 120」折算：力臂 0.2 m 时笛卡尔刚度约
        # kp / L² = 3000 N/m，2 kg 的腿对应 6 Hz、阻尼比 0.8 时 kd ≈ 120。
        self.kp_foot = float(kp_foot)
        self.kd_foot = float(kd_foot)
        # 支撑腿保留多少足端位置任务（1 = 和摆动腿一样追，0 = 一点不追）。
        #
        # **闭环里要给 0。** 支撑脚踩在地上，机身系的目标轨迹在往后扫，机身
        # 没跟上时误差就是「机身落后了多少」，而这个误差没有任何物理界限：
        # 实测 5 cm × kp_foot 3000 = 150 m/s² 的加速度需求，QP 里没有东西能
        # 约束它（唯一的天花板是 τ ≤ 20），于是 12 个关节每帧都顶在限幅上、
        # 输出退化成 bang-bang，机身被顶得乱跳、打滑 200%。
        #
        # 支撑腿的任务只有「出力」，位置由地面和步态决定；摆动腿在空中，追
        # 轨迹才是它的活。按支撑权重线性过渡，换腿那几帧不会突变。
        #
        # 默认给 1：这一层的文档说的是「四条腿一视同仁」，离线校验也按那个
        # 语义写。要关是调用方（`controller._ensure_mpc`）显式关。
        self.stance_foot = float(stance_foot)
        self.w_wrench = float(w_wrench)
        self.w_foot = float(w_foot)
        self.w_force = float(w_force)
        self.w_tau = float(w_tau)
        self.w_slew = float(w_slew)
        self.eps_abs = float(eps_abs)
        self.eps_rel = float(eps_rel)

        self._x_prev = None

        self._qp = DenseQP(NX, ROWS_PER_LEG * 4,
                           eps_abs=eps_abs, eps_rel=eps_rel)
        self.solve_times = []

    # --- 组装 -------------------------------------------------------------

    # 四腿的 M / h / J / p 和六个任务量一起取出来。模型只在 model.py 里算，
    # 这里不重写动力学。
    def _legs(self, q12, dq12):
        out = []
        for i, name in enumerate(LEGS):
            q = q12[3 * i:3 * i + 3]
            dq = dq12[3 * i:3 * i + 3]
            lm = leg_model(name)
            Mm, h = lm.mass_and_bias(q, dq)
            p, J = lm.fk_jac(q)
            out.append(dict(name=name, q=q, dq=dq, M=Mm, h=h, J=J, p=p))
        return out

    # 代价：0.5·xᵀHx + gᵀx。
    def _cost(self, ls, wts, target, a_cmd):
        H = np.zeros((NX, NX))
        g = np.zeros(NX)

        # 与 mpc.py 同一处 macOS 问题：Accelerate 后端会把 BLAS 遗留的浮点
        # 异常标志报成自己的警告，全正数相乘也报 divide by zero。真出问题
        # 解出来是 nan，solve() 末尾会拦。
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            return self._cost_inner(H, g, ls, wts, target, a_cmd)

    def _cost_inner(self, H, g, ls, wts, target, a_cmd):
        # ---- 1) 机身 wrench：Wf·x = [Σf ; Σr×f] ----
        # 每腿的力按支撑权重缩放：f_实际 = w·f_变量。权重是步态给的连续斜坡，
        # 于是「抬起一条腿」这件事在方程里是渐变的，不会在某一帧砍出一台阶。
        Wf = np.zeros((6, NX))
        r = [l["p"] - self.com for l in ls]
        for i in range(4):
            sl = F_OF(i)
            w = wts[i]
            Wf[0:3, sl] = np.eye(3) * w
            # 合力矩 M = Σ r × f，矩阵形式就是 skew(r)·f
            rx, ry, rz = (float(v) for v in r[i])
            Wf[3:6, sl] = np.array([[0.0, -rz, ry],
                                    [rz, 0.0, -rx],
                                    [-ry, rx, 0.0]]) * w
        w6 = np.r_[np.full(3, self.w_wrench), np.full(3, self.w_wrench * 10.0)]
        H += 2.0 * (Wf.T * w6) @ Wf
        tgt = np.r_[target.F, target.M]
        g -= 2.0 * (Wf.T * w6) @ tgt

        # ---- 2) 足端轨迹：J·a = a_cmd − J̇·q̇ ----
        for i, l in enumerate(ls):
            A = np.zeros((3, NX))
            A[:, A_OF(i)] = l["J"]
            H += 2.0 * self.w_foot * A.T @ A
            g -= 2.0 * self.w_foot * A.T @ a_cmd[i]

        # ---- 3) 力分配正则：f → 给定的分配 ----
        # 只压 f，不碰 a。这一项同时兼「让 H 正定」的职责：w_tau 给 τ 提供
        # 秩 12，w_force 给 f 再提供秩 12，两个合起来才铺满 24 维。
        for i, l in enumerate(ls):
            sl = F_OF(i)
            H[sl, sl] += 2.0 * self.w_force * np.eye(3)
            if target.force is not None:
                f_des = np.asarray(target.force.get(l["name"], np.zeros(3)),
                                   dtype=float)
                g[sl] -= 2.0 * self.w_force * f_des

        # ---- 5) 时序正则：x 靠近上一帧的解 ----
        # 抑制「同样满足所有约束、只是在零空间里换了位置」的逐帧摆动。
        # 权重比 w_foot 小三个量级，跟踪精度不受影响。
        if self.w_slew > 0.0 and self._x_prev is not None:
            H += 2.0 * self.w_slew * np.eye(NX)
            g -= 2.0 * self.w_slew * self._x_prev

        # ---- 4) 力矩正则：τ = Tm·x + h → 0 ----
        Tm, hoff = self._tau_map(ls, wts)
        H += 2.0 * self.w_tau * Tm.T @ Tm
        g += 2.0 * self.w_tau * Tm.T @ hoff
        return H, g, Tm, hoff

    # τ 作为 x 的仿射函数：τ = Tm·x + hoff。力那一列同样乘支撑权重。
    def _tau_map(self, ls, wts):
        Tm = np.zeros((12, NX))
        hoff = np.zeros(12)
        for i, l in enumerate(ls):
            Tm[3 * i:3 * i + 3, A_OF(i)] = l["M"]
            Tm[3 * i:3 * i + 3, F_OF(i)] = -l["J"].T * wts[i]
            hoff[3 * i:3 * i + 3] = l["h"]
        return Tm, hoff

    # 约束矩阵。结构（哪些位置非零）整场不变，所以建一次；每帧只换数值。
    #
    # 数值上把 A 建成**全稠密**的稀疏矩阵（每个位置都有非零），这样 csc 的
    # data 就是稠密矩阵的列优先展开，每帧直接拷进去，省掉每帧一次的稀疏化。
    # `_bounds` 之外的 τ 行数值也依赖 h，所以 A 也会变 —— 不是只换 l/u。
    def solve(self, q12, dq12, target: WBCTarget) -> WBCResult:
        t0 = time.perf_counter()
        q12 = np.asarray(q12, dtype=float)
        dq12 = np.asarray(dq12, dtype=float)
        if target.F is None:
            target.F = np.array([0.0, 0.0, TOTAL_MASS * GRAVITY])
        if target.M is None:
            target.M = np.zeros(3)
        target.F = np.asarray(target.F, dtype=float)
        target.M = np.asarray(target.M, dtype=float)

        ls = self._legs(q12, dq12)
        wts = [target.weight(l["name"]) for l in ls]

        # 足端 PD → 任务加速度（机体系）。
        #
        # 增益按支撑权重折：`w = 0` 的摆动腿拿满增益去追轨迹，`w = 1` 的支撑
        # 腿拿到 `stance_foot`（默认 0）—— 见 `__init__` 里 `stance_foot` 的
        # 那段：支撑脚踩在地上，机身系位置误差就是「机身落后了多少」，没有
        # 物理上限，追它只会把 12 个关节全顶到力矩限幅。
        #
        # `J̇·q̇` 那一项不折：它是运动学量（关节速度造成的足端加速度），
        # 与「追不追位置」无关，漏掉它支撑腿的 a 就是错的。
        a_cmd = np.zeros((4, 3))
        for i, l in enumerate(ls):
            name = l["name"]
            p_des = np.asarray(target.feet[name], dtype=float)
            v_des = target.vel(name)
            p_err = p_des - l["p"]
            v_err = v_des - l["J"] @ l["dq"]
            g_foot = 1.0 - (1.0 - self.stance_foot) * wts[i]
            a_cmd[i] = (g_foot * (self.kp_foot * p_err + self.kd_foot * v_err)
                        - jdot_qdot(leg_model(name).jacobian, l["q"], l["dq"]))

        H, g, Tm, hoff = self._cost(ls, wts, target, a_cmd)
        H = 0.5 * (H + H.T) + 1e-9 * np.eye(NX)

        A = np.zeros((ROWS_PER_LEG * 4, NX))
        lo = np.full(ROWS_PER_LEG * 4, -np.inf)
        up = np.full(ROWS_PER_LEG * 4, np.inf)
        mu_p = self.mu / math.sqrt(2.0)
        for i, l in enumerate(ls):
            b = ROWS_PER_LEG * i
            fs = F_OF(i)
            # τ 限幅：Tm·x + hoff ∈ [−τmax, τmax]
            A[b:b + 3, :] = Tm[3 * i:3 * i + 3, :]
            A[b + 3:b + 6, :] = -Tm[3 * i:3 * i + 3, :]
            up[b:b + 3] = self.tau_max - hoff[3 * i:3 * i + 3]
            lo[b:b + 3] = -np.inf
            up[b + 3:b + 6] = self.tau_max + hoff[3 * i:3 * i + 3]
            lo[b + 3:b + 6] = -np.inf
            # 摩擦锥：±fx ≤ μ'fz, ±fy ≤ μ'fz（行内不含常数项）
            for k, (comp, sign) in enumerate(((0, 1), (0, -1), (1, 1), (1, -1))):
                A[b + 6 + k, fs.start + comp] = sign
                A[b + 6 + k, fs.start + 2] = -mu_p
                up[b + 6 + k] = 0.0
            # 单边：0 ≤ fz ≤ fz_max
            A[b + 10, fs.start + 2] = 1.0
            up[b + 10] = self.fz_max
            A[b + 11, fs.start + 2] = -1.0
            up[b + 11] = 0.0
            # 权重为 0 的腿（纯摆动）把力变量钉成 0。注意这不是「切换」而是
            # 补丁：真正让过渡连续的是上面的 w·f 缩放，这里只是把一个不起作用
            # 的变量定住，省得求解器在零空间里乱选。
            for k in range(3):
                A[b + 12 + k, fs.start + k] = 1.0
                if wts[i] <= 0.0:
                    lo[b + 12 + k] = up[b + 12 + k] = 0.0
                else:
                    lo[b + 12 + k] = -np.inf
                    up[b + 12 + k] = np.inf

        x, info = self._qp.solve(H, g, A, lo, up)
        self.solve_times.append(time.perf_counter() - t0)
        if len(self.solve_times) > 200:
            self.solve_times.pop(0)

        if not np.isfinite(x).all():
            raise RuntimeError(f"WBC 解出发散：{info['status']}")
        self._x_prev = x
        tau = Tm @ x + hoff
        forces = np.array([x[F_OF(i)] for i in range(4)])
        accel = np.array([x[A_OF(i)] for i in range(4)])
        info["total_ms"] = 1e3 * (time.perf_counter() - t0)
        return WBCResult(tau=tau, forces=forces, accel=accel, info=info)

    # 最近若干次求解的耗时统计。
    def stats(self) -> dict:
        if not self.solve_times:
            return dict(n=0, mean=0.0, p95=0.0, max=0.0)
        d = sorted(self.solve_times)
        n = len(d)
        return dict(n=n, mean=1e3 * sum(d) / n,
                    p95=1e3 * d[min(n - 1, int(n * 0.95))],
                    max=1e3 * d[-1])
