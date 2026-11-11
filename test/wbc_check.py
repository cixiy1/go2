"""WBC 自检：离线跑，不接机器人。

    .venv/bin/python test/wbc_check.py

六条，对应 TODO 里 B 段的验收：

1. 站姿静平衡：Σfz = mg、M ≈ 0，且**力矩等于逆动力学的结果** —— 和
   `gravity_tau` 的差额应当正好是腿自重那一份（4%），这是 WBC 存在的理由
2. 硬约束：τ 限幅、fz ∈ [0, fz_max]、|fh| ≤ μ·fz、摆动腿 f ≡ 0
3. 足端任务：给偏移目标，J·a 追上 a_cmd − J̇·q̇
4. 平滑性 A/B：同一条步态轨迹下，WBC 与 `body_wrench_tau` 的力矩帧间差 p95
5. 站姿下解出来的 q̈ ≈ 0 —— PD 不需要出力，kp 才有资格往下让
6. 耗时
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadruped.control import model as M                          # noqa: E402
from quadruped.control import wbc as WB                           # noqa: E402
from quadruped.control.controller import Go2Controller as C       # noqa: E402

FAIL = []
LEGS = WB.LEGS
STAND_Q = np.array([0.0, 0.9, -1.8])
MG = M.TOTAL_MASS * M.GRAVITY
COM = M.whole_body_com(np.tile(STAND_Q, 4))


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<40}{detail}")
    if not ok:
        FAIL.append(name)


def stand_feet(q12=None):
    if q12 is None:
        q12 = np.tile(STAND_Q, 4)
    return {n: M.leg(n).fk(q12[3 * i:3 * i + 3]) for i, n in enumerate(LEGS)}


def make(**kw):
    return WB.WBC(com=COM, **kw)


def _q_from_feet(feet):
    q = np.zeros(12)
    for i, n in enumerate(LEGS):
        q[3 * i:3 * i + 3] = C.ik_leg(n, feet[n]).q
    return q


# ---------------------------------------------------------------------------

def check_qp():
    """回归：`DenseQP` 每帧换 H/A 之后必须真的换掉。

    这条守的是一个静默 bug：把 `solver._P.data` 原地改完再把同一个对象传回
    `update(P=...)`，osqp-python 会认为「和缓存的一样」直接跳过，求解器
    一直用第一帧的矩阵，不报错。WBC 上表现为力矩限幅被违反 7 N·m。
    """
    print("\n[0] DenseQP 的数值更新")
    from quadruped.control.qp import DenseQP
    q = DenseQP(2, 2)
    A = np.eye(2)
    lo, up = np.full(2, -50.0), np.full(2, 50.0)
    g = np.array([1.0, 1.0])
    H1 = np.eye(2)
    x1, _ = q.solve(H1, g, A, lo, up)
    H2 = np.array([[4.0, 1.0], [1.0, 4.0]])
    x2, _ = q.solve(H2, g, A, lo, up)
    want1 = -g
    want2 = -np.linalg.solve(H2, g)
    e1 = float(np.abs(x1 - want1).max())
    e2 = float(np.abs(x2 - want2).max())
    check("换 H 之后解跟着变", e2 < 1e-6 and e1 < 1e-6,
          f"差 {e1:.1e} / {e2:.1e}")

    # A 也要真的换：把约束从 |x|≤50 收紧到 x0 ≥ 20
    A2 = np.array([[1.0, 0.0], [0.0, 1.0]])
    lo2 = np.array([20.0, -50.0])
    x3, _ = q.solve(H2, g, A2, lo2, up)
    check("换 A 之后约束跟着变", x3[0] > 19.9, f"x0 = {x3[0]:.4f}（要 ≥ 20）")


def check_stand():
    print("\n[1] 站姿静平衡")
    q12 = np.tile(STAND_Q, 4)
    dq12 = np.zeros(12)
    feet = stand_feet(q12)
    w = make()
    # 力给均分，只是正则目标；wrench 任务才是真约束
    feed = {n: np.array([0.0, 0.0, MG / 4]) for n in LEGS}
    sol = w.solve(q12, dq12, WB.WBCTarget(feet=feet, F=np.array([0.0, 0.0, MG]),
                                          force=feed))
    fz = sol.forces[:, 2].sum()
    err = 100.0 * (fz / MG - 1.0)
    check("Σfz = mg", abs(err) < 0.1, f"Σfz {fz:.3f} N，mg {MG:.3f} N，"
                                      f"误差 {err:+.4f}%")
    mom = np.zeros(3)
    for i, n in enumerate(LEGS):
        mom += np.cross(feet[n] - COM, sol.forces[i])
    check("合力矩 ≈ 0", np.abs(mom).max() < 0.5,
          f"M = ({mom[0]:+.3f}, {mom[1]:+.3f}, {mom[2]:+.3f}) N·m")
    # WBC 的 τ 应该等于 rnea(q, 0, a, f)：两边同一套动力学，差得动就说明
    # τ = Tm·x + hoff 那一步组装写错了
    worst = 0.0
    for i, n in enumerate(LEGS):
        lm = M.leg(n)
        q = q12[3 * i:3 * i + 3]
        ref = lm.rnea(q, np.zeros(3), sol.accel[i], sol.forces[i])
        worst = max(worst, float(np.abs(ref - sol.tau[3 * i:3 * i + 3]).max()))
    check("τ == rnea(q, 0, a, f)", worst < 1e-8, f"最大差 {worst:.1e} N·m")

    # 和 VMC 的重力前馈比：差额应当≈腿自重（gravity_tau 从来没算过这一项）
    vmc = C.gravity_tau(q12, {n: 1.0 for n in LEGS})
    diff = sol.tau - vmc
    knee = M.leg("FR").gravity(STAND_Q)[2]
    print(f"        VMC 重力前馈膝力矩 {vmc[[2, 5, 8, 11]].mean():+.2f} N·m，"
          f"WBC {sol.tau[[2, 5, 8, 11]].mean():+.2f} N·m，"
          f"差 {diff[[2, 5, 8, 11]].mean():+.2f}（腿自重项 {knee:+.2f}）")
    check("比 VMC 多出腿自重那一份", 0.9 < diff[[2, 5, 8, 11]].mean() / knee < 1.1,
          f"比值 {diff[[2, 5, 8, 11]].mean() / knee:.2f}")


def check_bounds():
    print("\n[2] 硬约束")
    q12 = np.tile(STAND_Q, 4)
    feet = stand_feet(q12)
    w = make()
    # 离谱的指令：往上要 3 倍体重、外加 40 N·m 的滚转力矩
    sol = w.solve(q12, np.zeros(12),
                  WB.WBCTarget(feet=feet, F=np.array([0.0, 0.0, 3 * MG]),
                               M=np.array([40.0, 0.0, 0.0]),
                               contacts={n: n in ("FR", "RL") for n in LEGS}))
    check("|τ| ≤ tau_max", np.abs(sol.tau).max() <= w.tau_max + 1e-4,
          f"峰 {np.abs(sol.tau).max():.3f} N·m，限幅 {w.tau_max}")
    check("fz ∈ [0, fz_max]",
          sol.forces[:, 2].min() >= -1e-3 and sol.forces[:, 2].max() <= w.fz_max + 1e-3,
          f"fz {sol.forces[:, 2].min():+.3f} ~ {sol.forces[:, 2].max():.3f} N")
    fh = np.hypot(sol.forces[:, 0], sol.forces[:, 1])
    over = float((fh - w.mu * sol.forces[:, 2]).max())
    check("|fh| ≤ μ·fz", over <= 1e-3, f"最大越界 {1e3 * over:+.2e} mN")
    swing = [i for i, n in enumerate(LEGS) if n not in ("FR", "RL")]
    check("摆动腿 f ≡ 0", np.abs(sol.forces[swing]).max() < 1e-6,
          f"最大 {np.abs(sol.forces[swing]).max():.1e} N")


def check_task():
    print("\n[3] 足端任务跟踪")
    q12 = np.tile(STAND_Q, 4)
    dq12 = np.zeros(12)
    feet = stand_feet(q12)
    w = make()
    # 把一条腿的目标往下压 2 cm，另外三条不动
    tgt = dict(feet)
    tgt["FR"] = feet["FR"] + np.array([0.0, 0.0, -0.02])
    sol = w.solve(q12, dq12, WB.WBCTarget(feet=tgt, F=np.array([0.0, 0.0, MG]),
                                          contacts={n: n != "FR" for n in LEGS}))
    i = LEGS.index("FR")
    lm = M.leg("FR")
    a_cmd = w.kp_foot * (tgt["FR"] - lm.fk(q12[3 * i:3 * i + 3]))
    got = lm.jacobian(q12[3 * i:3 * i + 3]) @ sol.accel[i]
    err = float(np.abs(got - a_cmd).max())
    check("J·a 追上 a_cmd", err < 0.05 * np.abs(a_cmd).max(),
          f"要 {np.linalg.norm(a_cmd):.2f} m/s²，得 {np.linalg.norm(got):.2f}，"
          f"差 {err:.2e}")


# ---- 步态轨迹：直接用控制器自己的 foot_target ----
#
# 不要自己搭梯形轨迹做这个对比：支撑/摆动切换处速度会跳，kd 乘上去就是
# 一个 90 m/s² 的假指令，两种算法都会被这一帧拖出 15 N·m 的尖峰，掩盖掉
# 真正要看的东西。控制器里的摆动相是四次多项式，两端速度与支撑相衔接。

def gait_frame(ctl, t, vx=0.3, h=1e-4):
    feet, vel, touch = {}, {}, {}
    gp = ctl.gait
    for n in LEGS:
        feet[n] = np.array(ctl.foot_target(n, t, vx, 0.0, 0.0))
        p0 = np.array(ctl.foot_target(n, t - h, vx, 0.0, 0.0))
        p1 = np.array(ctl.foot_target(n, t + h, vx, 0.0, 0.0))
        vel[n] = (p1 - p0) / (2.0 * h)
        ph = (t / gp.period + gp.offsets[n]) % 1.0
        touch[n] = ph < gp.duty
    return feet, vel, touch


def ramp_weight(ctl, t, name):
    """步态的支撑权重：支撑相两端各留 20% 线性过渡（模拟预载窗口）。"""
    gp = ctl.gait
    ph = (t / gp.period + gp.offsets[name]) % 1.0
    if ph < 0.2 * gp.duty:
        return ph / (0.2 * gp.duty)
    if ph < gp.duty:
        return 1.0
    return max(0.0, 1.0 - (ph - gp.duty) / (0.2 * gp.duty))


def _wrench(k, dt):
    """姿态环在 2~4 Hz 上追一个抖动的机身 —— 真实行走里 F/M 从来不是常数。"""
    t = k * dt
    Mx = -160.0 * math.radians(1.5) * math.sin(2 * math.pi * 2.3 * t)
    My = -160.0 * math.radians(1.2) * math.sin(2 * math.pi * 3.1 * t + 0.7)
    Fz = MG + 1500.0 * 0.006 * math.sin(2 * math.pi * 2.7 * t)
    return np.array([0.0, 0.0, Fz]), np.array([Mx, My, 0.0])


def check_smooth():
    print("\n[4] 保真性 / 平滑性 A/B（控制器自己的小跑步态 0.3 m/s）")
    ctl = C(real=False)
    w = make()
    dt = 1.0 / 200.0
    n = 400                                 # 2 s，四个步态周期
    q_prev = None
    tau_w, tau_v, err_w, err_v, mx_w, mx_v = [], [], [], [], [], []
    for k in range(n):
        feet, vel, touch = gait_frame(ctl, k * dt)
        q12 = _q_from_feet(feet)
        dq12 = np.zeros(12) if q_prev is None else (q12 - q_prev) / dt
        q_prev = q12
        F, Mv = _wrench(k, dt)
        wt = {n_: ramp_weight(ctl, k * dt, n_) for n_ in LEGS}
        sw = sum(wt.values())
        feed = {n_: np.array([0.0, 0.0, MG * wt[n_] / max(sw, 1e-3)]) for n_ in LEGS}

        sol = w.solve(q12, dq12, WB.WBCTarget(
            feet=feet, feet_vel=vel, contacts={n_: wt[n_] > 0 for n_ in LEGS},
            F=F, M=Mv, force=feed, weights=wt))
        tau_w.append(sol.tau)
        mx_w.append(float(np.abs(sol.tau).max()))
        # 权重乘回去才是真正作用在身上的力
        fw = sol.forces * np.array([[wt[n_]] * 3 for n_ in LEGS])
        err_w.append(_wrench_err(feet, fw, F, Mv))

        tv = C.body_wrench_tau(q12, wt, F, Mv)
        tau_v.append(tv)
        mx_v.append(float(np.abs(tv).max()))
        err_v.append(_vmc_wrench_err(feet, q12, wt, F, Mv))

    tau_w = np.array(tau_w)
    tau_v = np.array(tau_v)
    d1w = np.abs(np.diff(tau_w, axis=0))
    d1v = np.abs(np.diff(tau_v, axis=0))
    d2w = np.abs(np.diff(tau_w, 2, axis=0))
    d2v = np.abs(np.diff(tau_v, 2, axis=0))
    pw = float(np.percentile(d1w, 95))
    pv = float(np.percentile(d1v, 95))
    print(f"        力矩帧间差 p95：WBC {pw:.3f}  VMC {pv:.3f} N·m")
    print(f"        力矩帧间差峰 ：WBC {d1w.max():.2f}  VMC {d1v.max():.2f} N·m")
    print(f"        力矩二阶差分 p95：WBC {np.percentile(d2w, 95):.3f}  "
          f"VMC {np.percentile(d2v, 95):.3f} N·m")
    # 注意这里**不可能**做到零误差，两种算法都一样：机械步态 duty = 0.5，
    # 双支撑时间为 0（见 TODO「原有遗留」），任意时刻只有两条腿承重。6 个力
    # 分量刚好凑 6 维 wrench，只剩 1 个自由度，Mx 和 My 由同一个竖直力差
    # 决定 —— 想独立满足就得违反单边或摩擦锥。所以比的是「谁更接近」。
    print(f"        wrench 误差 p95：WBC {np.percentile(err_w, 95):.3f}  "
          f"VMC {np.percentile(err_v, 95):.3f}")
    print(f"        力矩峰值：WBC {max(mx_w):.2f}  "
          f"VMC {max(mx_v):.2f} N·m（限幅 {w.tau_max}）")
    print(f"        超限幅的帧：WBC {100 * np.mean(np.array(mx_w) > w.tau_max + 1e-3):.0f}%"
          f"  VMC {100 * np.mean(np.array(mx_v) > 14.01):.0f}% (按 14)")

    check("力矩不超限幅", max(mx_w) <= w.tau_max + 1e-3,
          f"峰 {max(mx_w):.2f} N·m")
    ew, ev = float(np.percentile(err_w, 95)), float(np.percentile(err_v, 95))
    check("wrench 误差不到 body_wrench_tau 的一半", ew < 0.5 * ev,
          f"{ew:.2f} vs {ev:.2f}")
    check("帧间差不高于 body_wrench_tau", pw <= 1.2 * pv,
          f"{pw:.3f} vs {pv:.3f} N·m")
    check("帧间差峰不高于 1.2 倍", d1w.max() <= 1.2 * d1v.max(),
          f"{d1w.max():.2f} vs {d1v.max():.2f} N·m")


def _wrench_err(feet, fw, F, Mv):
    """给定足端力，实际作用在机身上的合力与合力矩和指令的差（取各分量最大者）。"""
    mom = np.zeros(3)
    for i, n in enumerate(LEGS):
        mom += np.cross(feet[n] - COM, fw[i])
    return float(max(np.abs(fw.sum(axis=0) - F).max(),
                     np.abs(mom - Mv).max() * 2.0))


def _vmc_wrench_err(feet, q12, wt, F, Mv):
    """VMC 那条路要给多少力矩，用 −Jᵀ 反推回足端力再算 wrench 误差。

    `body_wrench_tau` 只返回力矩，没有中间的力，所以反推：
    τ_i = −J_iᵀ f_i → f_i = −(J_iᵀ)⁻¹ τ_i。反推出来的力就是它实际在用的力。
    """
    tau = C.body_wrench_tau(q12, wt, F, Mv)
    fw = np.zeros((4, 3))
    for i, n in enumerate(LEGS):
        J = M.leg(n).jacobian(q12[3 * i:3 * i + 3])
        fw[i] = np.linalg.solve(-J.T, tau[3 * i:3 * i + 3])
    return _wrench_err(feet, fw, F, Mv)


def check_pd_free():
    print("\n[5] 站姿下 q̈ ≈ 0（PD 不该出力）")
    q12 = np.tile(STAND_Q, 4)
    feet = stand_feet(q12)
    w = make()
    feed = {n: np.array([0.0, 0.0, MG / 4]) for n in LEGS}
    sol = w.solve(q12, np.zeros(12),
                  WB.WBCTarget(feet=feet, F=np.array([0.0, 0.0, MG]), force=feed))
    worst = float(np.abs(sol.accel).max())
    check("解出的关节加速度 ≈ 0", worst < 0.05, f"峰 {worst:.3f} rad/s²")


def check_speed():
    print("\n[6] 耗时（200 Hz 的预算是 5 ms）")
    w = make()
    q12 = np.tile(STAND_Q, 4)
    feet = stand_feet(q12)
    tgt = WB.WBCTarget(feet=feet, F=np.array([0.0, 0.0, MG]))
    w.solve(q12, np.zeros(12), tgt)
    for _ in range(60):
        w.solve(q12, np.zeros(12), tgt)
    st = w.stats()
    print(f"        mean {st['mean']:.2f} ms  p95 {st['p95']:.2f} ms  "
          f"max {st['max']:.2f} ms")
    check("p95 < 2.5 ms", st["p95"] < 2.5, f"p95 {st['p95']:.2f} ms")


def main():
    print("=" * 78)
    print("WBC 自检")
    print("=" * 78)
    check_qp()
    check_stand()
    check_bounds()
    check_task()
    check_smooth()
    check_pd_free()
    check_speed()
    print()
    if FAIL:
        print(f"{len(FAIL)} 项不过：" + "、".join(FAIL))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
