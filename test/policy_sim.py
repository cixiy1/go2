"""把步态策略直接接到 MuJoCo 的 ctrl 上，绕开 DDS、控制器和抽象层。

    .venv/bin/python test/policy_sim.py --seconds 8

存在的意义是分锅：这份权重在真机上跑得通，在我们自己的仿真里跑不通。中间隔了
DDS、`Go2Controller` 的 idle 模式、`joint_target` 的 50 Hz 发布、桥的力矩低通
四层。哪一层的问题，套着整条链路看不出来。

这里只保留两件事：MuJoCo 的物理，和策略自己。物理按 500 Hz 步进、策略按 50 Hz
更新，PD 每步都算 —— 与训练时的 decimation 一致。

`--actuator` 把力矩换成训练时的电机模型（`actuator_net`），`--realism` 把仿真桥那三样
真机特性（5 ms 延迟 / 150 Hz 带宽 / 编码器噪声）搬进来。两个都是证伪用的开关。

2026-11-18 实测（`--flat --seconds 6 --vx 0.4`，本仓库当前权重）：

    配置                                  位移       最大倾角   足端 z 峰峰
    PD kp20 kd0.5（500 Hz）              +1.456 m    10.7°     7.8 cm
    + 真机特性（--realism）              +1.561 m    10.2°     8.8 cm
    + actuator_net（--actuator）         +1.173 m    11.2°     —
    PD 250 Hz（--pd-every 2）            +0.200 m    11.3°     0.8 cm
    PD 166 Hz（--pd-every 3）            -0.039 m     7.9°     0.6 cm
    PD 125 Hz（--pd-every 4）            -0.018 m     6.5°     0.8 cm

结论：**执行器模型不是瓶颈**（换 actuator_net、补真机特性都还走得动）；真正的开关是
**力矩更新率**。力矩被保持 4 ms（250 Hz）以上时足端峰峰掉到 1 cm 量级、机身沉到
0.15 m，看上去就是「kp20 脚不离地、走不动」—— 现场与 `--pd-every 2` 及以上一致。

足端几乎不离地不是摩擦问题：地面 μ 从 0.4 扫到 1.0、足端 μ 从 0.2 扫到 0.9，
位移变化都在 ±2% 以内（见 `--floor-friction` / `--foot-friction`）。

判据只有一条：机身倾角。`height` 不能用，我们的运动学高度在机器人仰躺时照样
报 0.26 m。
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.policy import default_commands, load_actuator, load_walker   # noqa: E402
from quadruped.sim.scenes import SCENE, flat_scene                        # noqa: E402
from quadruped.policy.walker import (                              # noqa: E402
    COMMAND_FIELDS,
    DEFAULT_Q,
    KP,
    KD,
    POLICY_TO_SDK,
    TAU_LIMIT,
    tilt_rad,
)

ROOT = Path(__file__).resolve().parent.parent
DT = 0.005
DECIMATION = 4                      # 4 × 0.005 = 0.02 s，策略 50 Hz


# 从传感器读 12 个关节角/角速度与机身四元数，全按 SDK 序（FR, FL, RR, RL）。
def sensor_view(model, data):
    names = ("FR", "FL", "RR", "RL")
    def adr(name):
        return int(model.sensor(name).adr[0])

    q = np.array([data.sensordata[adr(f"{n}_{j}_pos")]
                  for n in names for j in ("hip", "thigh", "calf")])
    dq = np.array([data.sensordata[adr(f"{n}_{j}_vel")]
                   for n in names for j in ("hip", "thigh", "calf")])
    quat = data.sensordata[adr("imu_quat"):adr("imu_quat") + 4]
    return q, dq, quat


# 从四元数算 rpy，与桥的换算同一套公式 —— 目的是让这里的姿态和走 DDS 时读到的一致。
def rpy_from_quat(quat):
    w, x, y, z = (float(v) for v in quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


def main() -> int:
    ap = argparse.ArgumentParser(description="策略直连 MuJoCo，不经通讯层")
    ap.add_argument("--seconds", type=float, default=8.0, help="接管后跑多久")
    ap.add_argument("--warmup", type=float, default=0.6, help="接管前先站住多久")
    ap.add_argument("--vx", type=float, default=0.0, help="前进速度命令")
    ap.add_argument("--kp", type=float, default=KP,
                    help="关节刚度，默认 20 = 真机 LowCmd 的值")
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--clamp", type=float, default=1.0,
                    help="力矩上限的倍率，1.0 = 用 Go2 的额定值")
    ap.add_argument("--limit", type=float, default=10.0,
                    help="动作限幅，10 = 训练的 clip_actions（等于不额外限制）。"
                         "0.5 能站住但不动，见 TODO.md 第七节")
    ap.add_argument("--slew", type=float, default=None,
                    help="每帧动作变化上限，默认不限")
    ap.add_argument("--actuator", action="store_true",
                    help="力矩改用训练时的电机模型算（actuator_net），不用 PD")
    ap.add_argument("--realism", action="store_true",
                    help="按仿真桥的方向加真机特性：指令延迟、力控带宽、编码器噪声。"
                         "不加就是理想 PD：当帧生效、传感器无噪")
    ap.add_argument("--latency", type=float, default=0.005, help="指令延迟，s")
    ap.add_argument("--bw", type=float, default=150.0, help="力控一阶带宽，Hz")
    ap.add_argument("--q-noise", type=float, default=0.0015, help="关节角噪声，rad")
    ap.add_argument("--dq-noise", type=float, default=0.15, help="关节速度噪声，rad/s")
    ap.add_argument("--flat", action="store_true",
                    help="换成没有台阶的平地。默认 scene.xml 在 x=1.2 m 有台阶，"
                         "走不到 1.2 m 就撞住，位移量的是撞停距离")
    ap.add_argument("--height", type=float, default=None,
                    help="身高命令，默认值见 default_commands（−0.05）；"
                         "参考仓库部署值是 0")
    ap.add_argument("--phase", type=float, default=None,
                    help="四路时钟相位差。0.5 = 对角小跑，0 = 四条腿同相（跳）")
    ap.add_argument("--footswing", type=float, default=None, help="抬脚高度，m")
    # 步频单独给：它决定「步幅 = vx / (2×freq)」，也就是走起来像迈步还是像原地抖。
    ap.add_argument("--freq", type=float, default=None, help="步频，Hz（默认 3.0）")
    ap.add_argument("--width", type=float, default=None, help="站宽，m")
    ap.add_argument("--offset", type=float, default=None,
                    help="相位参数之一。预设与实测步态见 quadruped/policy/gaits.py")
    ap.add_argument("--bound", type=float, default=None,
                    help="相位参数之一。0.5 与 phase 0.5 组合是 bound")
    ap.add_argument("--pd-every", type=int, default=1,
                    help="力矩每几个物理步算一次、中间保持（零阶保持）。1 = 每个物理步"
                         "重算，与真机驱动器（kHz 级电流环）和仿真桥的 "
                         "UpdateTorque（每个物理步一次）一致。>1 是诊断用的劣化开关："
                         "2 就已经足端不离地、位移归零，不要拿它当「仿真桥的样子」")
    ap.add_argument("--lag", action="store_true",
                    help="控制用上一拍的策略输出（人为加 20 ms 延迟）。诊断用")
    ap.add_argument("--solref", type=float, nargs=2, default=None,
                    metavar=("TIME", "DAMP"),
                    help="覆盖所有 geom 的接触 solref。go2.xml 里足端是 0.01 1、"
                         "其余 0.02 1；数值越小接触越硬。PhysX 那边相当于 4 次"
                         "位置迭代的硬接触")
    ap.add_argument("--joint-damp", type=float, default=None,
                    help="覆盖关节阻尼。go2.xml 是 0.3；训练用的 URDF 没有这一项")
    ap.add_argument("--joint-friction", type=float, default=None,
                    help="覆盖关节干摩擦。go2.xml 是 0.3；训练侧的 frictionloss 是 0")
    ap.add_argument("--joint-armature", type=float, default=None,
                    help="覆盖电枢惯量。go2.xml 是 0.015；训练侧 URDF 没有这一项")
    ap.add_argument("--sim-hz", type=float, default=200.0,
                    help="物理步频率。训练侧是 200 Hz（dt 0.005），真机驱动器内部"
                         "位置环在 kHz 级。策略恒 50 Hz，decimation 自动取 sim_hz/50")
    ap.add_argument("--init-z", type=float, default=0.31,
                    help="起始机身高度。0.31 略高于站高，落地会更硬；参考链用 0.30")
    ap.add_argument("--floor-friction", type=float, default=None,
                    help="覆盖地面滑动摩擦系数（go2.xml 是 0.4）。训练域随机化是 "
                         "0.4~1.2，0.4 是下界不是笔误；这一项只用来量它对位移的影响")
    ap.add_argument("--foot-friction", type=float, default=None,
                    help="覆盖足端滑动摩擦（go2.xml 是 0.9）。MuJoCo 的接触摩擦取"
                         "两个 geom 的逐分量最大值，所以只有这一项真正决定抓地力")
    ap.add_argument("--foot-report", action="store_true",
                    help="报足端行为：抬脚高度、着地时足端水平速度（打滑量）、"
                         "落地次数。位移数字只能说明机体动了，滑也能让 x 变大")
    ap.add_argument("--cold-warm", action="store_true",
                    help="预热期不调用策略（动作历史全 0、时钟不动），与部署侧"
                         "「先按住 0.6 s 再接管」一致")
    a = ap.parse_args()

    # 物理步长与策略周期分开算：策略恒 50 Hz，物理步按 --sim-hz。
    #
    # 200 Hz 是训练侧的值（dt 0.005），也是我们一直用的。真机驱动器内部位置环
    # 在 kHz 级，200 Hz 差了 5 倍 —— 这一项直接影响跟踪相位滞后，而 kp 越小越敏感。
    dt = 1.0 / a.sim_hz
    dec = max(1, int(round(a.sim_hz / 50.0)))
    model = mujoco.MjModel.from_xml_path(str(flat_scene() if a.flat else SCENE))
    model.opt.timestep = dt
    if a.floor_friction is not None:
        gid = int(model.geom("floor").id)
        model.geom_friction[gid, 0] = a.floor_friction
    if a.foot_friction is not None:
        for name in ("FL", "FR", "RL", "RR"):
            model.geom_friction[int(model.geom(name).id), 0] = a.foot_friction
    if a.solref is not None:
        model.geom_solref[:, :] = a.solref
    if a.joint_damp is not None:
        model.dof_damping[:] = a.joint_damp
    if a.joint_friction is not None:
        model.dof_frictionloss[:] = a.joint_friction
    if a.joint_armature is not None:
        model.dof_armature[:] = a.joint_armature
    data = mujoco.MjData(model)

    # 初态：机身 0.31 m 直立，四肢按策略默认站姿摆好。
    #
    # 关节角必须按名字写进 qpos：MJCF 里关节在 qpos 的排列是 FL/FR/RL/RR，
    # 不是 SDK 的 FR/FL/RR/RL。按 SDK 序填进这 12 个槽，起手姿态就是乱的，
    # 策略一上手就得先把它掰回来 —— 那点偏差足够把一个本来能站住的策略测成摔。
    data.qpos[:] = 0.0
    data.qpos[2] = a.init_z
    data.qpos[3] = 1.0
    for i, leg in enumerate(("FL", "FR", "RL", "RR")):
        for j, part in enumerate(("hip", "thigh", "calf")):
            adr_q = int(model.joint(f"{leg}_{part}_joint").qposadr[0])
            data.qpos[adr_q] = DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    walker = load_walker(dt=dt * dec, action_limit=a.limit,
                         action_slew=a.slew)
    # 电机模型只在控制节拍上算一次，力矩保持到下一拍 —— 训练里也是这个节奏。
    act = load_actuator() if a.actuator else None
    cmd = default_commands()
    cmd[0] = a.vx
    # 部署侧（go2_gym_deploy 的 command_profile）与我们的 default_commands 有四项不一致：
    # height / phase / footswing / width。命令行能逐项换，用来判断"走不快"是命令还是观测。
    for name, value in (("height", a.height), ("phase", a.phase),
                        ("offset", a.offset), ("bound", a.bound),
                        ("footswing", a.footswing), ("width", a.width),
                        ("freq", a.freq)):
        if value is not None:
            cmd[COMMAND_FIELDS.index(name)] = value

    hold = DEFAULT_Q[POLICY_TO_SDK].copy()
    q_des = hold.copy()
    warmup_steps = int(a.warmup / dt)
    total_steps = warmup_steps + int(a.seconds / dt)

    # 真机特性，三项与仿真桥同参数、同一套做法（见 unitree_sdk2py_bridge.py）：
    #
    #   延迟   发出去的力矩晚到几毫秒才生效。按物理步长向上取整，不足一帧也给一帧。
    #   带宽   电流环跟不上瞬时跳变，是一阶滞后。
    #   噪声   编码器读数带噪；速度是差分出来的，噪声大一个量级。注意噪声只进
    #          **控制器的观测**，不进物理，也不进电机自己的 PD —— 真机上驱动器用的
    #          是它自己那路编码器，不是经过 DDS 传输、被我们再读一次的那份。
    lat_frames = max(1, int(round(a.latency / dt)))
    bw_alpha = 1.0 - math.exp(-2.0 * math.pi * a.bw * dt)
    rng = np.random.default_rng(20261110)                # 固定种子，结果可复现
    cmd_q: deque = deque()
    ctrl_f = np.zeros(len(q_des))

    # 足端统计。位移是机体的，不区分「走」和「滑」；这里量足端自己：
    #   z 峰峰  = 抬脚高度（<2 cm 就是在地上蹭）
    #   着地滑速 = 该脚处于接触状态时的水平速度（脚踩住了就该接近 0）
    feet = [int(model.geom(n).id) for n in ("FL", "FR", "RL", "RR")]
    fz = {g: [9.9, -9.9] for g in feet}
    slip = {g: [0.0, 0] for g in feet}        # [累计滑速×时间, 采样数]
    last_pos = {g: None for g in feet}
    air = {g: False for g in feet}
    steps_n = {g: 0 for g in feet}

    worst = 0.0
    moved = None
    history_head = []
    held = np.zeros(len(q_des))
    pending = None
    for step in range(total_steps):
        q, dq, quat = sensor_view(model, data)
        rpy = rpy_from_quat(quat)
        # 控制器看到的那一份带噪；电机 PD 用真实值算，与仿真桥一致。
        q_obs, dq_obs = q, dq
        if a.realism:
            q_obs = q + rng.normal(0.0, a.q_noise, len(q))
            dq_obs = dq + rng.normal(0.0, a.dq_noise, len(dq))

        # 策略 50 Hz 更新；预热期间只推历史、不采纳输出，让起手姿态稳下来。
        # `--cold-warm` 改成预热期根本不调用策略：动作历史全 0、时钟不动，与部署侧
        # 「先按住 DEFAULT_Q 0.6 s 再接管」一致。两种做法的初始条件不同，后续轨迹就不同。
        if step % dec == 0 and (not a.cold_warm or step >= warmup_steps):
            q_out, action = walker.step(q_obs, dq_obs, rpy, cmd)
            if step >= warmup_steps:
                # `--lag`：控制用上一拍的策略输出，人为加 20 ms 延迟。诊断用 ——
                # 这个策略对时序极其敏感，一拍延迟能把 8 s 位移从 1.2 m 抬到 2.4 m，
                # 也能把参考链从 1.9 m 打到 0。
                if a.lag:
                    if pending is None:
                        pending = q_out
                    q_des, pending = pending, q_out
                else:
                    q_des = q_out
            if step == warmup_steps:
                print(f"接管：倾角 {math.degrees(tilt_rad(rpy)):5.1f}°  "
                      f"高度 {data.qpos[2]:.3f} m  动作 {np.round(action[[0, 4, 8]], 2)}")
            if step % 100 == 0 and step >= warmup_steps:
                history_head.append(np.round(action[[0, 4, 8]], 2))

        limit = TAU_LIMIT * a.clamp
        if act is not None:
            # 电机模型每个物理步算一次，不是每控制拍算一次再保持。
            #
            # 训练侧 `_compute_torques()` 在每个物理子步被调用，输入是当拍的
            # `dof_pos / dof_vel`，还带前两帧误差与速度的历史 —— 它本身就是个
            # 带时序的模型。按 50 Hz 算、再保持 4 步，等于额外加了一层零阶保持。
            raw = np.clip(act.torque(q, q_des, dq), -limit, limit)
        else:
            if step % a.pd_every == 0:
                tau = a.kp * (q_des - q) - a.kd * dq
                raw = np.clip(tau, -limit, limit)
            else:
                raw = held

        if a.realism:
            cmd_q.append(raw)
            out = cmd_q.popleft() if len(cmd_q) > lat_frames else cmd_q[0]
            ctrl_f += bw_alpha * (out - ctrl_f)
            data.ctrl[:] = ctrl_f
        else:
            data.ctrl[:] = raw
        mujoco.mj_step(model, data)

        if a.foot_report and step >= warmup_steps:
            con = {c.geom1 for c in data.contact[:data.ncon]} | \
                  {c.geom2 for c in data.contact[:data.ncon]}
            for g in feet:
                z = float(data.geom_xpos[g][2])
                fz[g][0] = min(fz[g][0], z)
                fz[g][1] = max(fz[g][1], z)
                pos = data.geom_xpos[g][:2].copy()
                if last_pos[g] is not None and g in con:
                    v = float(np.linalg.norm(pos - last_pos[g])) / dt
                    slip[g][0] += v
                    slip[g][1] += 1
                last_pos[g] = pos
                # 落地次数：从「离地」到「接触」
                touching = g in con
                if touching and air[g]:
                    steps_n[g] += 1
                air[g] = not touching

        if step >= warmup_steps:
            tilt = tilt_rad(rpy)
            worst = max(worst, tilt)
            if moved is None:
                moved = data.qpos[0]
            if step % int(0.5 / dt) == 0:
                print(f"  t={((step - warmup_steps) * dt):.1f}s  "
                      f"倾角 {math.degrees(tilt):5.1f}°  高度 {data.qpos[2]:.3f}  "
                      f"x {data.qpos[0]:+.3f}")
            if tilt > 1.2:
                print(f"  倒了：t={((step - warmup_steps) * dt):.2f}s  "
                      f"倾角 {math.degrees(tilt):.0f}°")
                break

    print()
    # 朝向一并打出来：只在「直线走」时 x 位移才等于走了多远。斜着跑或者画圆，
    # 同一个 x 位移对应的是更长的路径。
    yaw = math.degrees(rpy_from_quat(sensor_view(model, data)[2])[2])
    print(f"最大倾角 {math.degrees(worst):.1f}°   末态高度 {data.qpos[2]:.3f} m   "
          f"位移 {(data.qpos[0] - moved) if moved is not None else 0:+.3f} m   "
          f"末态朝向 {yaw:+.0f}°")
    if a.foot_report:
        print()
        for g in feet:
            name = model.geom(g).name
            pp = (fz[g][1] - fz[g][0]) * 100
            v = (slip[g][0] / slip[g][1]) if slip[g][1] else float("nan")
            print(f"  足 {name}: z 峰峰 {pp:4.1f} cm   着地滑速 {v:5.3f} m/s"
                  f"   落地 {steps_n[g]} 次")
    src = "actuator_net" if act is not None else f"PD kp{a.kp:g} kd{a.kd:g}"
    src += " + 真机特性" if a.realism else " 理想执行器"
    print(f"结论：{'站住' if worst < 0.6 else '没站住'}"
          f"（{src}  limit={a.limit} slew={a.slew}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
