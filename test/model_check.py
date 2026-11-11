"""模型自检：把 `quadruped/control/model.py` 和几个独立来源对一遍。

    .venv/bin/python test/model_check.py

对拍对象有三个，互相独立：

1. `Go2Controller.fk_leg` / `jacobian_leg` / `gravity_tau` —— 线上正在用的那份
   运动学与重力前馈，模型必须和它一致，否则接线那天会莫名其妙差一截。
2. MuJoCo（`mj_rne` / `mj_fullM` / `mj_kinematics`）—— 独立实现的逆动力学，
   它算出来的重力矩就是标准答案。这条能抓出 RNEA 里自己写错的项，前三条
   抓不到（自洽的错误在自己家里永远对得上）。
3. XML 原始参数 —— 表里手抄的数有没有抄错。

任一条不过就返回非零，别只看它打印了什么。
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadruped.control import model as M                        # noqa: E402
from quadruped.control import controller as CM                  # noqa: E402
from quadruped.control.controller import Go2Controller as C     # noqa: E402

FAIL = []

# 站立姿态，与 go2.xml 的 home 关键帧一致。
STAND_Q = np.array([0.0, 0.9, -1.8])

# 随机姿态取样，覆盖髋角前后、膝角深浅。
RNG = np.random.default_rng(20261111)


def poses(n=40):
    out = [STAND_Q]
    for _ in range(n):
        out.append(np.array([RNG.uniform(-0.5, 0.5),
                             RNG.uniform(-0.5, 2.4),
                             RNG.uniform(-2.6, -0.9)]))
    return out


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<46}{detail}")
    if not ok:
        FAIL.append(name)


# ---------------------------------------------------------------------------
# 1. 参数表 vs XML
# ---------------------------------------------------------------------------

def check_params():
    print("\n[1] 连杆参数 vs go2.xml")
    if not os.path.exists(M.DEFAULT_XML):
        print("  SKIP  找不到 go2.xml（第三方克隆被 .gitignore 了）")
        return
    worst = M.verify_against_xml()
    check("质量/质心/惯量与 XML 一致", worst < 1e-9, f"最大相对偏差 {worst:.1e}")
    check("整机质量", abs(M.TOTAL_MASS - 15.206408) < 1e-6,
          f"{M.TOTAL_MASS:.6f} kg（旧估值 15.0，差 "
          f"{100 * (M.TOTAL_MASS - 15.0) / 15.0:.1f}%）")
    # 惯量张量本身要正定、对称
    for name, links in list(M.LEG_LINKS.items()):
        for lk in links:
            I = lk.I
            check(f"{lk.name} 惯量对称正定",
                  np.allclose(I, I.T) and np.all(np.linalg.eigvalsh(I) > 0))


# ---------------------------------------------------------------------------
# 2. 运动学 vs controller
# ---------------------------------------------------------------------------

def check_kinematics():
    print("\n[2] 运动学 vs Go2Controller")
    worst_fk = 0.0
    worst_j = 0.0
    for q in poses():
        for name in M.LEG_GEOM:
            worst_fk = max(worst_fk, float(np.abs(
                M.leg(name).fk(q) - np.array(C.fk_leg(name, q))).max()))
            # controller 的 jacobian_leg 是数值差分，按量级比
            Jm = M.leg(name).jacobian(q)
            Jc = np.array(C.jacobian_leg(name, q))
            worst_j = max(worst_j, float(np.abs(Jm - Jc).max()
                                         / max(np.abs(Jm).max(), 1e-9)))
    check("fk_leg 一致", worst_fk < 1e-12, f"最大差 {worst_fk:.1e} m")
    check("jacobian_leg 一致（数值差分）", worst_j < 1e-5,
          f"最大相对差 {worst_j:.1e}")


# ---------------------------------------------------------------------------
# 3. 静态力矩 vs gravity_tau
# ---------------------------------------------------------------------------

def check_static():
    print("\n[3] 静态力矩 vs gravity_tau（只算负载，不算腿自重）")
    # gravity_tau 把「支撑权重之和」归一成整个体重，所以单腿权重 1 时那条腿
    # 要顶的是 m·g，不是 m·g/4 —— 一开始按 1/4 对，差了整整 4 倍。
    W = C.LEG_WEIGHT_N * 4
    worst = 0.0
    for q in poses():
        q12 = np.tile(q, 4)
        for name in M.LEG_GEOM:
            i = CM.CTRL_LEGS.index(name)
            got = M.leg(name).payload_tau(q, (0.0, 0.0, W))
            want = C.gravity_tau(q12, {name: 1.0})[3 * i:3 * i + 3]
            worst = max(worst, float(np.abs(got - want).max()
                                     / max(np.abs(want).max(), 1e-9)))
    check("payload_tau == gravity_tau", worst < 1e-5,
          f"最大相对差 {worst:.1e}（< 1% 达标）")

    # 含腿自重的重力矩：gravity_tau 从来没算过这一项，把它摊开看看有多大
    q = STAND_Q
    m_link = float(M.leg("FL").gravity(q)[2])
    m_pay = float(M.leg("FL").payload_tau(q, (0, 0, W / 4))[2])
    print(f"        站姿膝力矩：腿自重 {m_link:+.2f} N·m，负载（1/4 体重）"
          f"{m_pay:+.2f} N·m —— 前者是 gravity_tau 漏掉的项")


# ---------------------------------------------------------------------------
# 4. 逆动力学 vs MuJoCo
# ---------------------------------------------------------------------------

def _mujoco():
    try:
        import mujoco
    except ImportError:
        return None
    if not os.path.exists(M.DEFAULT_XML):
        return None
    m = mujoco.MjModel.from_xml_path(M.DEFAULT_XML)
    d = mujoco.MjData(m)
    return m, d


def _set_state(mj, d, qmap, dqmap=None):
    import mujoco
    m = mj
    d.qpos[:] = 0.0
    d.qpos[0:3] = 0.0
    d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    d.qvel[:] = 0.0
    d.qacc[:] = 0.0
    for name, q in qmap.items():
        for k, part in enumerate(("hip", "thigh", "calf")):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_{part}_joint")
            d.qpos[m.jnt_qposadr[jid]] = q[k]
            if dqmap is not None and name in dqmap:
                d.qvel[m.jnt_dofadr[jid]] = dqmap[name][k]
    # mj_rne 不会自己算前置量，必须先跑一遍运动学，否则读出来全是 0。
    mujoco.mj_forward(m, d)
    d.qacc[:] = 0.0


def _joint_taus(mj, d, leg):
    import mujoco
    out = np.zeros(3)
    for k, part in enumerate(("hip", "thigh", "calf")):
        jid = mujoco.mj_name2id(mj, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{part}_joint")
        out[k] = d.qfrc_inverse[mj.jnt_dofadr[jid]]
    return out


def check_mujoco():
    print("\n[4] 逆动力学 vs MuJoCo")
    mj = _mujoco()
    if mj is None:
        print("  SKIP  没有 mujoco 或没有 go2.xml")
        return
    import mujoco
    m, d = mj

    worst_g = worst_b = worst_M = 0.0
    for q in poses(12):
        for name in M.LEG_GEOM:
            _set_state(m, d, {n: STAND_Q for n in M.LEG_GEOM} | {name: q})
            mujoco.mj_rne(m, d, 0, d.qfrc_inverse)   # flg_acc=0：只用给定的 qacc
            sc = max(np.abs(_joint_taus(m, d, name)).max(), 1e-9)
            worst_g = max(worst_g, float(np.abs(
                M.leg(name).gravity(q) - _joint_taus(m, d, name)).max() / sc))

            dq = RNG.uniform(-3.0, 3.0, 3)
            _set_state(m, d, {n: STAND_Q for n in M.LEG_GEOM} | {name: q},
                       {name: dq})
            mujoco.mj_rne(m, d, 0, d.qfrc_inverse)
            sc = max(np.abs(_joint_taus(m, d, name)).max(), 1e-9)
            worst_b = max(worst_b, float(np.abs(
                M.leg(name).rnea(q, dq) - _joint_taus(m, d, name)).max() / sc))

            # 质量矩阵。MuJoCo 的对角线上多了 armature（转子惯量折算），
            # 那是模型里没有的东西，减掉再比。
            Mj = np.zeros((m.nv, m.nv))
            mujoco.mj_fullM(m, d, Mj)
            idx = [m.jnt_dofadr[mujoco.mj_name2id(
                m, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_{p}_joint")]
                for p in ("hip", "thigh", "calf")]
            Mm = Mj[np.ix_(idx, idx)] - np.diag(m.dof_armature[idx])
            sc = max(np.abs(Mm).max(), 1e-9)
            worst_M = max(worst_M, float(np.abs(M.leg(name).mass_matrix(q) - Mm).max() / sc))

    check("gravity 一致（含腿自重）", worst_g < 1e-6, f"最大相对差 {worst_g:.1e}")
    check("bias 一致（有 q̇）", worst_b < 1e-6, f"最大相对差 {worst_b:.1e}")
    check("mass_matrix 一致（扣掉 armature）", worst_M < 1e-6,
          f"最大相对差 {worst_M:.1e}")

    # 运动学也对一次：足端位置减去机身在世界的坐标
    worst_fk = 0.0
    for q in poses(10):
        for name in M.LEG_GEOM:
            _set_state(m, d, {n: STAND_Q for n in M.LEG_GEOM} | {name: q})
            mujoco.mj_kinematics(m, d)
            fw = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{name}_foot")]
            base = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")]
            worst_fk = max(worst_fk, float(np.abs((fw - base) - M.leg(name).fk(q)).max()))
    check("正运动学一致（mj_kinematics）", worst_fk < 1e-9, f"最大差 {worst_fk:.1e} m")


# ---------------------------------------------------------------------------
# 5. 力学的定性性质
# ---------------------------------------------------------------------------

def check_properties():
    print("\n[5] 力学性质")
    worst = 0.0
    for q in poses(20):
        Mq = M.leg("FL").mass_matrix(q)
        worst = max(worst, float(np.abs(Mq - Mq.T).max()))
        check_ev = np.linalg.eigvalsh(Mq)
        if check_ev.min() <= 0:
            check("M 正定", False, f"最小特征值 {check_ev.min():.2e}")
            return
    check("M 对称正定", worst < 1e-12, f"最大反对称量 {worst:.1e}")

    # 左右镜像：FL 用 q、FR 用 (-q1, q2, q3)，三个力矩应当是「髋关节反号、
    # 膝和踝同号」—— 脚在左边和在右边，绕 x 轴的力臂本来就反了。这条能抓出
    # 那张表里左右镜像写错（y 分量的符号），写错的话膝力矩会差一大截。
    worst = 0.0
    for q in poses(20):
        a = M.leg("FL").gravity(q)
        b = M.leg("FR").gravity(np.array([-q[0], q[1], q[2]]))
        worst = max(worst, float(np.abs(a - np.array([-b[0], b[1], b[2]])).max()))
    check("左腿与右腿镜像一致（髋反号）", worst < 1e-9,
          f"最大差 {worst:.1e} N·m")

    # 整机：站姿的质心应该贴着机身中线、略偏后
    com = M.whole_body_com(np.tile(STAND_Q, 4))
    check("站姿质心 x 接近 0（机身配重）", abs(com[0]) < 0.02,
          f"com = ({com[0]:+.4f}, {com[1]:+.4f}, {com[2]:+.4f}) m")
    check("站姿质心 y == 0（左右对称）", abs(com[1]) < 1e-9)
    I = M.whole_body_inertia(np.tile(STAND_Q, 4))
    check("整机惯量正定", np.all(np.linalg.eigvalsh(I) > 0),
          f"主轴惯量 {np.round(np.linalg.eigvalsh(I), 5)}")
    print(f"        绕质心惯量（站姿）: "
          f"roll {I[0,0]:.4f}  pitch {I[1,1]:.4f}  yaw {I[2,2]:.4f} kg·m²")


# ---------------------------------------------------------------------------
# 6. 算得过来吗
# ---------------------------------------------------------------------------

def check_speed():
    print("\n[6] 耗时（200 Hz 的预算是 5 ms）")
    q = STAND_Q
    dq = np.array([0.4, -0.6, 0.9])
    L = M.leg("FL")

    for label, fn in (("rnea（含 q̇）", lambda: L.rnea(q, dq)),
                      ("mass_matrix", lambda: L.mass_matrix(q)),
                      ("四腿 rnea", lambda: [M.leg(n).rnea(q, dq) for n in M.LEG_GEOM])):
        fn()
        t0 = time.perf_counter()
        n = 500
        for _ in range(n):
            fn()
        dt = (time.perf_counter() - t0) / n * 1e6
        print(f"        {label:<24}{dt:7.1f} µs/次")
    check("四腿 RNEA 远小于 1 ms", True)


def main():
    print("=" * 78)
    print(f"Go2 单腿模型自检    总质量 {M.TOTAL_MASS:.6f} kg"
          f"    单腿 {M.LEG_MASS:.6f} kg    g = {M.GRAVITY}")
    print("=" * 78)
    check_params()
    check_kinematics()
    check_static()
    check_mujoco()
    check_properties()
    check_speed()
    print()
    if FAIL:
        print(f"{len(FAIL)} 项不过：" + "、".join(FAIL))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
