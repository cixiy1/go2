"""MPC 自检：离线跑，不接机器人。

    .venv/bin/python test/mpc_check.py

四条验收（TODO 里写的）+ 一条闭环：

1. 静态悬停：四脚着地、零速度指令，首帧 Σfz = m·g，误差 < 0.1%
2. 单边与摩擦锥：随机状态、随机接触序列下 fz ≥ 0、|fx|,|fy| ≤ μ·fz
3. 耗时：稳态 p95 < 3 ms（50 Hz 的预算是 20 ms，这里按 200 Hz 的一半算）
4. 与 MuJoCo 单刚体对拍：同样的四足力，加速度 1e-9 量级一致
5. 闭环：用 SRBD 自身积分跑 2 秒，指令 0.3 m/s，看速度能不能跟上、机身稳不稳
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadruped.control import mpc as MP                            # noqa: E402

FAIL = []
LEGS = MP.LEGS

# 力的判据容差。OSQP 收在 eps_abs = 1e-6，落到 12·N 维的力向量上是 1e-5~1e-4 N
# 量级；判据卡得比求解器还紧，只会测出求解器自己的残差。
TOL_F = 1e-3

# 站立时的足端位置（机体系，足端 body 原点）。
STAND_FEET = {"FL": (0.1934, 0.142, -0.248),
              "FR": (0.1934, -0.142, -0.248),
              "RL": (-0.1934, 0.142, -0.248),
              "RR": (-0.1934, -0.142, -0.248)}


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<44}{detail}")
    if not ok:
        FAIL.append(name)


def make(horizon=12, **kw):
    p = MP.nominal_params()
    return MP.SRBDMPC(mass=p["mass"], inertia=p["inertia"],
                      com_body=p["com_body"], horizon=horizon, **kw)


def hover_state(h=0.27):
    return MP.SRBDState(p=np.array([0.0, 0.0, h]))


# ---------------------------------------------------------------------------

def check_hover():
    print("\n[1] 静态悬停")
    mpc = make()
    m = mpc.mass
    sol = mpc.solve(hover_state(), MP.SRBDRef(height=0.27), STAND_FEET,
                    np.ones((mpc.N, 4), bool))
    fz = float(sol.forces[0][:, 2].sum())
    err = 100.0 * (fz / (m * MP.GRAVITY) - 1.0)
    check("首帧 Σfz = mg", abs(err) < 0.1,
          f"Σfz {fz:.3f} N，mg {m * MP.GRAVITY:.3f} N，误差 {err:+.4f}%")

    # 质心偏后 2 mm，后腿该多扛一点
    f = sol.forces[0][:, 2]
    check("前后腿分担合理（质心偏后）",
          f[2] + f[3] > f[0] + f[1],
          f"前 {f[0] + f[1]:.2f} N 后 {f[2] + f[3]:.2f} N")
    Mx, My = sol.wrench_body()[1][:2]
    check("首帧力矩接近 0", max(abs(Mx), abs(My)) < 0.05,
          f"M = ({Mx:+.3f}, {My:+.3f}) N·m")


def check_cone():
    print("\n[2] 单边约束与摩擦锥")
    rng = np.random.default_rng(7)
    mpc = make()
    worst_neg = 0.0
    worst_over = -1.0                 # 超过 μ·fz 的量（N），负数表示全程没越界
    worst_cone = -1.0                 # 只在 fz > 1 N 的样本里看比值
    n_bad = 0
    for _ in range(40):
        st = MP.SRBDState(p=np.array([0.0, 0.0, rng.uniform(0.22, 0.32)]),
                          v=rng.uniform(-0.4, 0.4, 3),
                          rpy=rng.uniform(-0.2, 0.2, 3),
                          omega=rng.uniform(-1.0, 1.0, 3))
        contacts = rng.random((mpc.N, 4)) > 0.5
        if not contacts[0].any():
            contacts[0, 0] = True
        ref = MP.SRBDRef(vx=0.3, vy=0.0, height=0.27)
        sol = mpc.solve(st, ref, STAND_FEET, contacts)
        f = sol.forces
        worst_neg = min(worst_neg, float(f[:, :, 2].min()))
        # 摩擦锥用**绝对量**比：|fh| − μ·fz ≤ 0，μ 是物理摩擦系数（QP 里那个
        # 是 μ/√2 的内接棱锥，比这更严）。比值形式在 fz → 0 时会被放大成几十，
        # 看着像越界，其实是 0/0 —— 那点水平力只有几毫牛，落地瞬间的四舍五入。
        fh = np.hypot(f[:, :, 0], f[:, :, 1])
        worst_over = max(worst_over, float((fh - mpc.mu * f[:, :, 2]).max()))
        on = f[:, :, 2] > 1.0
        if on.any():
            worst_cone = max(worst_cone, float((fh[on] / f[:, :, 2][on]).max()))
        # 摆动腿必须严格是 0
        swing = ~sol.contacts
        if swing.any() and np.abs(f[swing]).max() > 1e-6:
            n_bad += 1
    # OSQP 是内点法，约束残差按 eps_abs 落在 1e-5~1e-4 N 量级。1 mN 对
    # 15 kg 机身是 7 ppm，没有物理意义，判据不能比求解器本身更紧。
    check("fz ≥ 0", worst_neg > -TOL_F, f"最小 fz {1e3 * worst_neg:+.3f} mN")
    check("水平力在 μ fz 之内", worst_over <= TOL_F,
          f"最大越界 {1e3 * worst_over:+.2e} mN；"
          f"fz>1 N 时 |fh|/fz 峰 {worst_cone:.3f}，μ {mpc.mu}")
    check("摆动腿力为 0", n_bad == 0, f"{n_bad} 次违反")


def check_speed():
    print("\n[3] 耗时（50 Hz 的预算是 20 ms）")
    mpc = make()
    ref = MP.SRBDRef(height=0.27)
    contacts = np.ones((mpc.N, 4), bool)
    st = hover_state()

    mpc.solve(st, ref, STAND_FEET, contacts)     # 冷启动
    for _ in range(50):
        mpc.solve(st, ref, STAND_FEET, contacts)
    d = sorted(mpc.solve_times[5:])
    p95 = 1e3 * d[min(len(d) - 1, int(len(d) * 0.95))]
    print(f"        稳态 mean {1e3 * sum(d) / len(d):.2f} ms  "
          f"p95 {p95:.2f} ms  max {1e3 * d[-1]:.2f} ms")
    check("稳态 p95 < 3 ms", p95 < 3.0, f"p95 {p95:.2f} ms")


def _single_body_model(p):
    """一个只有机身的 MJCF，四只脚用 site 标出来，用来做动力学对拍。

    用 site 而不是 `mj_applyFT`：`mj_applyFT` 的 `point` 是**相对质心**的世界
    点，语义容易记错（踩过一次，差 4.9 m/s² 就是它）。站点雅可比没有这个歧义，
    `Σ Jᵢᵀfᵢ` 就是四足力对 6 维广义力的贡献，且走的是 MuJoCo 自己那条链。

    机身原点和质心重合（`inertial pos="0 0 0"`），所以 qpos 前三位是质心位置，
    site 的 pos 就是「质心 → 足端」。

    惯量必须用 `fullinertia`：整机的 I_xz = −0.0165，是横滚惯量的 10%，只写
    `diaginertia` 等于换了一个张量，角加速度会差 4 m/s² 量级。坐标也别图省事
    写 9 位小数 —— 150 N × 1e-9 m = 1.5e-7 N·m，正好盖住要验的误差。
    """
    import mujoco
    I = np.asarray(p["inertia"], dtype=float)
    full = (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])
    sites = "\n".join(
        "      <site name=%r pos=\"%.17g %.17g %.17g\"/>" % (
            n, *(np.asarray(STAND_FEET[n], float) - p["com_body"]))
        for n in LEGS)
    xml = """
<mujoco>
  <option gravity="0 0 -%r"/>
  <worldbody>
    <body name="trunk" pos="0 0 1">
      <freejoint/>
      <inertial pos="0 0 0" mass="%.17g"
        fullinertia="%.17g %.17g %.17g %.17g %.17g %.17g"/>
%s
    </body>
  </worldbody>
</mujoco>""" % (MP.GRAVITY, p["mass"], *full, sites)
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def check_mujoco():
    print("\n[4] 与 MuJoCo 单刚体对拍")
    try:
        import mujoco
    except ImportError:
        print("  SKIP  没装 mujoco")
        return
    p = MP.nominal_params()
    m, d = _single_body_model(p)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk")
    sid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n) for n in LEGS]
    nv = m.nv
    jacp = np.zeros((3, nv))
    jacr = np.zeros((3, nv))
    M = np.zeros((nv, nv))
    rng = np.random.default_rng(11)

    worst_f = worst_a = 0.0
    for _ in range(12):
        rpy = rng.uniform(-0.3, 0.3, 3)
        R = MP.rpy_to_R(*rpy)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R.ravel())
        w_body = rng.uniform(-2.0, 2.0, 3)
        d.qpos[:] = [0.0, 0.0, 1.0, *quat]
        d.qvel[:] = [0.0, 0.0, 0.0, *w_body]
        d.qacc[:] = 0.0
        d.xfrc_applied[:] = 0.0
        mujoco.mj_forward(m, d)
        # mj_rne 读的是 forward 阶段算好的量，不先过一次读出来全是 0

        forces = rng.uniform(-15, 15, (4, 3))
        forces[:, 2] = rng.uniform(10, 120, 4)
        feet_body = np.array([STAND_FEET[n] for n in LEGS]) - p["com_body"]
        feet_rel = (R @ feet_body.T).T

        # MuJoCo 侧：站点雅可比把四足力累加成 6 维广义力
        qfrc = np.zeros(nv)
        for i in range(4):
            mujoco.mj_jacSite(m, d, jacp, jacr, sid[i])
            qfrc += jacp.T @ forces[i]

        st = MP.SRBDState(p=np.array([0.0, 0.0, 1.0]), rpy=rpy,
                          omega=R @ w_body)
        xd = MP.dynamics(st, forces, feet_rel, p["mass"], p["inertia"])
        # 我们自己算的广义力（绕质心的合力与合力矩，世界系）
        tau_w = np.zeros(3)
        for i in range(4):
            tau_w += np.cross(feet_rel[i], forces[i])
        mine = np.concatenate([forces.sum(axis=0), tau_w])
        # 自由关节的角自由度在机体系，转过去再比
        worst_f = max(worst_f, float(np.abs(qfrc - np.concatenate(
            [mine[:3], R.T @ mine[3:]])).max()))

        mujoco.mj_fullM(m, d, M)
        bias = np.zeros(nv)
        mujoco.mj_rne(m, d, 0, bias)
        qacc_mj = np.linalg.solve(M, qfrc - bias)
        want = np.concatenate([xd[MP.V], R.T @ xd[MP.W]])
        worst_a = max(worst_a, float(np.abs(qacc_mj - want).max()))

    check("四足力 → 6 维广义力一致", worst_f < 1e-9,
          f"最大差 {worst_f:.1e}")
    check("接上质量矩阵后加速度一致", worst_a < 1e-8,
          f"最大差 {worst_a:.1e} m/s²")


def check_closed_loop():
    print("\n[5] 闭环（SRBD 自身积分 2 秒，指令 0.3 m/s）")
    p = MP.nominal_params()
    mpc = make()
    dt = 1.0 / 50.0                      # MPC 50 Hz
    sub = 4                              # 内部 200 Hz 积分
    steps = int(2.0 / dt)
    st = hover_state()
    feet = np.array([STAND_FEET[n] for n in LEGS])

    # 小跑相位：对角腿一组
    offsets = {"FL": 0.5, "FR": 0.0, "RL": 0.0, "RR": 0.5}

    class G:
        period = 0.5
        duty = 0.5

    g = G()
    g.offsets = offsets

    worst_roll = worst_pitch = 0.0
    for k in range(steps):
        contacts = MP.contact_from_gait(g, k * dt, mpc.N, dt)
        if not contacts[0].any():       # 保证至少一条腿着地
            contacts[0, 0] = True
        sol = mpc.solve(st, MP.SRBDRef(vx=0.3, height=0.27), dict(
            zip(LEGS, feet - p["com_body"])), contacts)
        # 用第一帧的力在子步里推，足端跟着机身平移（不模拟落脚点）
        for _ in range(sub):
            R = st.R
            rel = (R @ (feet - p["com_body"]).T).T
            xd = MP.dynamics(st, sol.forces[0], rel, p["mass"], p["inertia"])
            st.p = st.p + xd[MP.P] * (dt / sub)
            st.v = st.v + xd[MP.V] * (dt / sub)
            st.rpy = st.rpy + xd[MP.RPY] * (dt / sub)
            st.omega = st.omega + xd[MP.W] * (dt / sub)
        worst_roll = max(worst_roll, abs(st.rpy[0]))
        worst_pitch = max(worst_pitch, abs(st.rpy[1]))

    v = st.v[0]
    print(f"        末速度 vx {v:+.3f} m/s  高度 {st.p[2]:.3f} m  "
          f"roll 峰 {np.degrees(worst_roll):.1f}° pitch 峰 {np.degrees(worst_pitch):.1f}°")
    check("前进速度跟上 0.3", 0.22 < v < 0.38, f"vx {v:+.3f} m/s")
    check("高度没漂", abs(st.p[2] - 0.27) < 0.03,
          f"{st.p[2]:.3f} m（−0.03 ~ +0.03）")
    check("姿态没翻", max(worst_roll, worst_pitch) < np.radians(12.0),
          f"峰 {np.degrees(max(worst_roll, worst_pitch)):.1f}°（< 12°）")


def main():
    print("=" * 78)
    print("SRBD-MPC 自检")
    print("=" * 78)
    check_hover()
    check_cone()
    check_speed()
    check_mujoco()
    check_closed_loop()
    print()
    if FAIL:
        print(f"{len(FAIL)} 项不过：" + "、".join(FAIL))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
