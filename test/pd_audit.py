"""量 kp / kd 在 MuJoCo 里到底被什么拖住，以及落地的冲击有多大。

    # 静态：把腿按 DEFAULT_Q 定住，看 kp 20 撑体重需要多少角度差
    .venv/bin/python test/pd_audit.py hold --kp 20 --kd 0.5

    # 闭环：走 12 s，报位移、跟踪误差、抬脚高度、落地速度
    .venv/bin/python test/pd_audit.py walk --kp 20 --kd 0.5 --vx 0.4

    # 原地转，抬脚高度对转向速度的影响
    .venv/bin/python test/pd_audit.py walk --vx 0 --vyaw 4 --footswing 0.08

`policy_sim.py` 只回答「站不站得住」。站得住之后剩下的问题是两个数：关节跟不跟得上
目标（`q_des − q`），以及脚落到地面那一瞬的竖直速度。后者就是真机上听到的音量 ——
落地冲量 ∝ 触地速度，与 kp / kd 只在「触地后那几毫秒吸不吸得住」上有关。

实测数据记在 `TODO.md` 第十二节（训练参数集中与 kp 的实测边界），改参数之前先看那张表。

## 三个指标各自的含义

    撑重误差   定住站姿时 q_des − q。等于「关节力矩 / kp」，与 kp 成反比。
               它决定摆动相起点是不是已经偏了 —— 观测里的 dpos 带着这个偏置。
    跟踪误差   走路时 q_des − q 的均值与 95 分位。小腿最大：它是唯一承担体重的
               单关节，误差直接变成机身下沉。
    触地速度   脚从空中落到地面时的 |dz/dt|。真机上这是撞击声的来源，20 cm 落差
               的落地速度约 2 m/s，8 cm 约 1.2 m/s，差一倍声压级。

## 用法上的两条约束

物理步长固定 200 Hz（训练侧同值），策略 50 Hz —— 与 `policy_sim.py` 一致，
换 `--sim-hz` 会同时改 PD 频率，那就不再是「训练时的执行器」了。
`--pd-hz 50` 是把 PD 降到控制拍、步与步之间保持力矩的写法，用来证明「PD 必须按
物理步算」；真机驱动器和训练侧都是后者。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test"))

import policy_sim as PS                                                 # noqa: E402
from quadruped.policy import default_commands, load_actuator, load_walker   # noqa: E402
from quadruped.policy.walker import (                                   # noqa: E402
    COMMAND_FIELDS,
    DEFAULT_Q,
    KD,
    KP,
    POLICY_TO_SDK,
    TAU_LIMIT,
    tilt_rad,
)
from quadruped.sim.scenes import flat_scene                             # noqa: E402

LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
SIM_HZ = 200.0                  # 训练侧 dt 0.005
DECIMATION = 4                  # 4 × 0.005 = 0.02 s → 50 Hz


# 建模型并按 --joint-* / --foot-friction 覆盖参数。覆盖项用来把 go2.xml 的真机化参数
# （damping 0.3 / frictionloss 0.3 / armature 0.015、足底摩擦 0.9）退回到训练时的样子。
def build(joint_damp=None, joint_friction=None, joint_armature=None,
          foot_friction=None):
    model = mujoco.MjModel.from_xml_path(str(flat_scene()))
    model.opt.timestep = 1.0 / SIM_HZ
    if joint_damp is not None:
        model.dof_damping[:] = joint_damp
    if joint_friction is not None:
        model.dof_frictionloss[:] = joint_friction
    if joint_armature is not None:
        model.dof_armature[:] = joint_armature
    if foot_friction is not None:
        # 足底球挂在 `*_calf` 上（不是 `*_foot`），几何体名就是腿名 FL/FR/RL/RR。
        # 按名字选，别按类型猜 —— base_link 上还有一串 capsule，改错一处就是整机变滑。
        for i in range(model.ngeom):
            if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) in LEGS:
                model.geom_friction[i, 0] = foot_friction
    data = mujoco.MjData(model)
    return model, data


# 起手姿态：机身 z 抬高一点，四腿摆成 DEFAULT_Q。关节按名字写进 qpos ——
# MJCF 里关节序是 FL/FR/RL/RR（正好是策略序），不是 SDK 的 FR/FL/RR/RL。
def reset(model, data, z=0.30):
    data.qpos[:] = 0.0
    data.qpos[2] = z
    data.qpos[3] = 1.0
    for i, leg in enumerate(LEGS):
        for j, part in enumerate(PARTS):
            adr = int(model.joint(f"{leg}_{part}_joint").qposadr[0])
            data.qpos[adr] = DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)


# 足端位置（四只脚 × xyz）。用 body 位置而不是几何体 —— 足端球心就是 body 原点。
def foot_pos(model, data):
    return np.array([data.body(f"{leg}_foot").xpos for leg in LEGS])


def label(i: int) -> str:
    return f"{LEGS[i // 3]}_{PARTS[i % 3]}"


# ---- 静态撑重 ----------------------------------------------------------

# 把 12 个关节定在 DEFAULT_Q，跑 --seconds 秒。稳态时的 q_des − q 就是
# 「该关节要出多少力矩 / kp」—— 与 15 kg 体重在此姿态下的分布一一对应。
def run_hold(a) -> int:
    model, data = build(a.joint_damp, a.joint_friction, a.joint_armature,
                        a.foot_friction)
    reset(model, data, a.init_z)
    q_des = DEFAULT_Q[POLICY_TO_SDK].copy()
    steps = int(a.seconds * SIM_HZ)
    for _ in range(steps):
        q, dq, _ = PS.sensor_view(model, data)
        tau = np.clip(a.kp * (q_des - q) - a.kd * dq, -TAU_LIMIT, TAU_LIMIT)
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)

    q, dq, quat = PS.sensor_view(model, data)
    err = q_des - q
    print(f"kp {a.kp:g}  kd {a.kd:g}  定住 {a.seconds:g} s 后："
          f"机身 {data.qpos[2]:.3f} m  倾角 {math.degrees(tilt_rad(PS.rpy_from_quat(quat))):.1f}°")
    print(f"  {'关节':<14}{'q_des−q':>9}{'力矩 N·m':>10}")
    for i in range(12):
        print(f"  {label(i):<12}{err[i]:+9.3f}{a.kp * err[i]:10.1f}")
    print(f"\n  三组均值  hip {err[[0, 3, 6, 9]].mean():+.3f}  "
          f"thigh {err[[1, 4, 7, 10]].mean():+.3f}  "
          f"calf {err[[2, 5, 8, 11]].mean():+.3f}  （弧度）")
    print(f"  小腿要顶住体重，误差 = 力矩 / kp = {abs(a.kp * err[[2, 5, 8, 11]].mean()) / a.kp:.2f} rad"
          f"（kp {a.kp:g} 时）")
    return 0


# ---- 闭环 --------------------------------------------------------------

# 走一段，返回这次跑的指标。除位移/倾角外，记录：
#
#   calf 误差的均值与 95 分位      —— 小腿跟不上目标的量
#   每只脚的抬脚高度（最高 − 最低） —— 与命令里的 footswing 对表
#   触地速度                      —— 脚第一次下探到「最低点 + 1 cm」时的 |dz/dt|
#   打滑                          —— 脚贴地那几帧的水平速度，判「推力被摩擦吃掉」
#
# 触地判据用「上一帧在阈值上方、这一帧在下方」，与真实的地面接触不同步，但两者
# 只差一两帧，量级不受影响。
def run_walk_once(a, cmd, seed_z):
    model, data = build(a.joint_damp, a.joint_friction, a.joint_armature,
                        a.foot_friction)
    reset(model, data, seed_z)

    dt = 1.0 / SIM_HZ
    walker = load_walker(dt=dt * DECIMATION, action_limit=a.limit)
    act = load_actuator() if a.actuator else None
    pd_every = max(1, int(round(SIM_HZ / a.pd_hz)))

    q_des = DEFAULT_Q[POLICY_TO_SDK].copy()
    held = np.zeros(12)
    warmup = int(a.warmup * SIM_HZ)
    total = warmup + int(a.seconds * SIM_HZ)

    calf_err, all_err = [], []
    pos_trace = []
    tau_peak = 0.0
    worst = 0.0
    x0 = None
    yaw = 0.0
    yaw_last = 0.0
    for step in range(total):
        q, dq, quat = PS.sensor_view(model, data)
        rpy = PS.rpy_from_quat(quat)
        if step % DECIMATION == 0:
            q_des, _ = walker.step(q, dq, rpy, cmd)

        if act is not None:
            # 训练用的电机模型：每个物理步按当拍 q/dq 算一次，与训练侧
            # `_compute_torques()` 同频。它自带两帧历史，不能按控制拍保持。
            held = np.clip(act.torque(q, q_des, dq), -TAU_LIMIT, TAU_LIMIT)
        elif step % pd_every == 0:
            held = np.clip(a.kp * (q_des - q) - a.kd * dq, -TAU_LIMIT, TAU_LIMIT)
        data.ctrl[:] = held
        mujoco.mj_step(model, data)

        pos_trace.append(foot_pos(model, data))
        if step >= warmup:
            err = q_des - q
            all_err.append(np.abs(err))
            calf_err.append(np.abs(err[[2, 5, 8, 11]]))
            tau_peak = max(tau_peak, float(np.abs(held).max()))
            worst = max(worst, tilt_rad(rpy))
            if x0 is None:
                x0 = data.qpos[0]
                yaw_last = rpy[2]
            # 偏航累计：欧拉角在 ±π 处跳变，按增量累加。
            d = (rpy[2] - yaw_last + math.pi) % (2 * math.pi) - math.pi
            yaw += d
            yaw_last = rpy[2]
            if tilt_rad(rpy) > 1.2:
                break

    moved_s = (step - warmup) * (1.0 / SIM_HZ)

    pos = np.array(pos_trace)                    # (帧, 4 脚, xyz)
    z = pos[:, :, 2]
    v = np.diff(pos, axis=0) / dt
    land = []
    slip = []
    for leg in range(4):
        lo = z[:, leg].min()
        thresh = lo + 0.01
        for k in range(1, len(z)):
            if z[k, leg] < thresh <= z[k - 1, leg]:
                land.append(abs(float(v[k - 1, leg, 2])))
        # 打滑：脚已经贴到地面高度时，脚在水平方向的速度。真空转的腿在空中也慢，
        # 所以只取贴地那几帧，否则会被摆动相稀释。`v` 比 `z` 少一帧（差分）。
        touch = z[:-1, leg] < thresh
        if touch.any():
            slip.append(float(np.hypot(v[touch, leg, 0], v[touch, leg, 1]).mean()))
    lift = z.max(axis=0) - z.min(axis=0)
    calf = np.array(calf_err)
    alls = np.array(all_err)
    return dict(
        moved=float(data.qpos[0] - x0) if x0 is not None else 0.0,
        tilt=math.degrees(worst),
        h=float(data.qpos[2]),
        calf_mean=float(calf.mean()), calf_p95=float(np.percentile(calf, 95)),
        err_mean=float(alls.mean()), err_p95=float(np.percentile(alls, 95)),
        lift=lift, land=float(np.mean(land)) if land else 0.0,
        land_max=float(np.max(land)) if land else 0.0,
        slip=float(np.mean(slip)) if slip else 0.0,
        tau=tau_peak, yaw=math.degrees(yaw),
        yaw_rate=math.degrees(yaw) / max(1e-6, moved_s),
    )


def run_walk(a) -> int:
    cmd = default_commands()
    cmd[COMMAND_FIELDS.index("vx")] = a.vx
    cmd[COMMAND_FIELDS.index("vyaw")] = a.vyaw
    for name in ("footswing", "width", "phase", "height", "freq"):
        value = getattr(a, name)
        if value is not None:
            cmd[COMMAND_FIELDS.index(name)] = value

    runs = [run_walk_once(a, cmd, a.init_z + dz) for dz in a.seed_z[:a.repeats]]
    moved = [r["moved"] for r in runs]
    tilt = [r["tilt"] for r in runs]
    print(f"kp {a.kp:g}  kd {a.kd:g}  PD {a.pd_hz:g} Hz  vx {a.vx:g}  "
          f"footswing {cmd[COMMAND_FIELDS.index('footswing')]:g}  起手 z {a.init_z:g}")
    print(f"位移 {'/'.join(f'{m:+.2f}' for m in moved)} m   "
          f"最大倾角 {max(tilt):.0f}°   末态高度 {runs[-1]['h']:.3f} m")
    r = runs[-1]
    print(f"跟踪误差  均值 {r['err_mean']:.3f}  p95 {r['err_p95']:.3f} rad   "
          f"小腿 均值 {r['calf_mean']:.3f}  p95 {r['calf_p95']:.3f}")
    print(f"抬脚高度  " + "  ".join(f"{LEGS[i]} {r['lift'][i]:.3f}" for i in range(4))
          + f" m   命令 {cmd[COMMAND_FIELDS.index('footswing')]:.2f}")
    print(f"触地速度  均值 {r['land']:.2f}  最大 {r['land_max']:.2f} m/s   "
          f"力矩峰 {r['tau']:.1f} N·m")
    print(f"打滑      足端贴地时水平速度 {r['slip']:.2f} m/s")
    if a.vyaw:
        print(f"转向      {r['yaw']:+.0f}°   {r['yaw_rate']:.0f}°/s"
              f"（{math.radians(r['yaw_rate']):.2f} rad/s）")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="kp/kd 与 MuJoCo 关节模型的一致性审计")
    ap.add_argument("mode", choices=("hold", "walk"))
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--vx", type=float, default=0.4)
    ap.add_argument("--vyaw", type=float, default=0.0, help="转向命令，rad/s")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--warmup", type=float, default=0.6)
    ap.add_argument("--limit", type=float, default=10.0, help="动作限幅，10 = 训练值")
    ap.add_argument("--actuator", action="store_true", help="力矩改用 actuator_net")
    ap.add_argument("--pd-hz", type=float, default=SIM_HZ,
                    help="PD 计算频率。50 = 每控制拍算一次再保持（错的做法）")
    ap.add_argument("--init-z", type=float, default=0.31)
    ap.add_argument("--repeats", type=int, default=3, help="换起手高度重复几次")
    ap.add_argument("--joint-damp", type=float, default=None)
    ap.add_argument("--joint-friction", type=float, default=None)
    ap.add_argument("--joint-armature", type=float, default=None)
    ap.add_argument("--foot-friction", type=float, default=None,
                    help="足底摩擦系数。go2.xml 是 0.9，真机橡胶脚垫实测 0.6~0.8。"
                         "注意 MuJoCo 会把它和地面（floor 1.0）合成，取到 1.0 以上"
                         "就没有区别了")
    for name in ("footswing", "width", "phase", "height", "freq"):
        ap.add_argument(f"--{name}", type=float, default=None)
    a = ap.parse_args()
    a.seed_z = (0.0, 0.005, -0.005, 0.01)
    return run_hold(a) if a.mode == "hold" else run_walk(a)


if __name__ == "__main__":
    raise SystemExit(main())
