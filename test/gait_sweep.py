"""量步态：给一组命令，跑闭环，从足端轨迹反过来判它走成什么样。

    .venv/bin/python test/gait_sweep.py            # 扫预设
    .venv/bin/python test/gait_sweep.py --modes trot bound --vx 0.3

存在的意义：`phase/offset/bound` 三个数在 `_advance_clock()` 里拼成四路时钟，
公式能推出「应该」是哪种步态，但推不出**策略能不能走**。策略见过哪些相位、
在哪个组合下不摔、走多快，只能从物理里量。

判据分两层：

    形态   四足抬脚信号的互相关相位差 → 归类成 pronk / trot / bound / pace / walk
    可用   位移、速度跟踪、最大倾角、抬脚高度

形态对了但原地打转，说明这个相位组合策略接不住；形态不对但走得快，说明它
用自己的方式完成了同一个命令 —— 两种都记下来，不按公式改数据。

物理与 `policy_sim.py` 同一套：MuJoCo 500 Hz（--sim-hz 可改）、策略 50 Hz、
PD 每个物理步重算。区别只在这里不打印每 0.5 s 的流水账，跑完给一行。
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.policy import default_commands, load_walker              # noqa: E402
from quadruped.sim.scenes import flat_scene                             # noqa: E402
from quadruped.policy.walker import (                                  # noqa: E402
    COMMAND_FIELDS,
    DEFAULT_Q,
    KP,
    KD,
    POLICY_TO_SDK,
    TAU_LIMIT,
    tilt_rad,
)

ROOT = Path(__file__).resolve().parent.parent

# 四足在 MJCF / SDK 里都叫 FR, FL, RR, RL；统计时按 FL, FR, RL, RR 排，
# 与 `walker.POLICY_JOINTS` 一致，看表时不用再换算。
LEGS = ("FL", "FR", "RL", "RR")

# 预设的相位三元组。公式：feet = (g+phase+offset+bound, g+offset, g+bound, g+phase)。
#
# walk 那个解不是猜的，四个方程解出来：offset+bound=0.75、offset=phase+0.25、
# bound=offset+0.25 → (0, 0.25, 0.5)，四腿相位 0 / 0.5 / 0.75 / 0.25。
PRESETS: dict[str, dict[str, float]] = {
    "pronk": dict(phase=0.0, offset=0.0, bound=0.0),
    "trot": dict(phase=0.5, offset=0.0, bound=0.0),
    "bound": dict(phase=0.5, offset=0.0, bound=0.5),
    "pace": dict(phase=0.0, offset=0.0, bound=0.5),
    "walk": dict(phase=0.0, offset=0.25, bound=0.5),
}

# 分类用的理论相位（以 FL 为 0）。环形差取 min(|d|, 1-|d|)。
PATTERNS: dict[str, np.ndarray] = {
    "pronk": np.array([0.0, 0.0, 0.0, 0.0]),
    "trot": np.array([0.0, 0.5, 0.5, 0.0]),
    "bound": np.array([0.0, 0.0, 0.5, 0.5]),
    "pace": np.array([0.0, 0.5, 0.0, 0.5]),
    "walk": np.array([0.0, 0.5, 0.75, 0.25]),
}


# 从四元数算 rpy，与仿真桥同一套公式。
def rpy_from_quat(quat) -> list[float]:
    w, x, y, z = (float(v) for v in quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


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


# 用互相关找 a 相对 b 的滞后，单位：周期。
#
# 只搜 ±半周期，因为步态相位只有相对意义，滞后 0.7 与 −0.3 是同一个关系；
# 搜全程会因为周期信号有多个峰而跳到隔壁周期上。没抬脚（方差为 0）时返回 0，
# 免得把噪声的相关当成相位。
def lag_cycles(a: np.ndarray, b: np.ndarray, fs: float, freq: float) -> tuple[float, float]:
    a = a - a.mean()
    b = b - b.mean()
    if a.std() < 1e-6 or b.std() < 1e-6:
        return 0.0, 0.0
    c = np.correlate(a, b, mode="full")
    lags = np.arange(-len(a) + 1, len(a))
    half = max(1, int(round(0.5 / freq * fs)))
    keep = np.abs(lags) <= half
    k = int(np.argmax(c[keep]))
    lag = float(lags[keep][k]) / fs * freq
    # 归一化相关系数，看关联到底强不强。
    strength = float(c[keep][k] / (np.linalg.norm(a) * np.linalg.norm(b)))
    return lag, strength


# 把实测的足端相位归类。返回 (名字, 最小残差)。残差按环形距离，0 最好。
def classify(phases: np.ndarray) -> tuple[str, float]:
    best = ("?", 1.0)
    for name, ref in PATTERNS.items():
        d = np.abs((phases - ref + 0.5) % 1.0 - 0.5)
        err = float(np.mean(d))
        if err < best[1]:
            best = (name, err)
    return best


@dataclass
class Result:
    name: str
    disp_x: float
    disp_y: float
    yaw: float
    speed: float
    cmd_vx: float
    max_tilt: float
    body_h: float
    foot_lift: float
    gait: str
    residual: float
    phases: np.ndarray
    corr: float
    fell: bool


def run(name: str, cmd_over: dict[str, float], *, seconds: float, warmup: float,
        vx: float, vy: float, vyaw: float, kp: float, kd: float,
        sim_hz: float, footswing: float, width: float) -> Result:
    dt = 1.0 / sim_hz
    dec = max(1, int(round(sim_hz / 50.0)))
    model = mujoco.MjModel.from_xml_path(str(flat_scene()))
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    data.qpos[:] = 0.0
    data.qpos[2] = 0.31
    data.qpos[3] = 1.0
    for i, leg in enumerate(("FL", "FR", "RL", "RR")):
        for j, part in enumerate(("hip", "thigh", "calf")):
            data.qpos[int(model.joint(f"{leg}_{part}_joint").qposadr[0])] = DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    walker = load_walker(dt=dt * dec)
    cmd = default_commands()
    cmd[COMMAND_FIELDS.index("vx")] = vx
    cmd[COMMAND_FIELDS.index("vy")] = vy
    cmd[COMMAND_FIELDS.index("vyaw")] = vyaw
    cmd[COMMAND_FIELDS.index("footswing")] = footswing
    cmd[COMMAND_FIELDS.index("width")] = width
    for key, value in cmd_over.items():
        cmd[COMMAND_FIELDS.index(key)] = value
    freq = float(cmd[COMMAND_FIELDS.index("freq")])

    geom_ids = [int(model.geom(leg).id) for leg in LEGS]
    fs = 100.0                       # 足端轨迹按 100 Hz 记，够分辨 4 Hz 的步频
    sample_every = max(1, int(round(1.0 / (fs * dt))))
    feet_hist: list[np.ndarray] = []

    hold = DEFAULT_Q[POLICY_TO_SDK].copy()
    q_des = hold.copy()
    warm_steps = int(warmup / dt)
    total = warm_steps + int(seconds / dt)
    x0 = y0 = None
    yaw0 = 0.0
    worst = 0.0
    fell = False

    for step in range(total):
        q, dq, quat = sensor_view(model, data)
        rpy = rpy_from_quat(quat)
        if step % dec == 0:
            q_des, _ = walker.step(q, dq, rpy, cmd)
        tau = kp * (q_des - q) - kd * dq
        data.ctrl[:] = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        mujoco.mj_step(model, data)

        if step >= warm_steps:
            if x0 is None:
                x0, y0 = float(data.qpos[0]), float(data.qpos[1])
                yaw0 = rpy[2]
            worst = max(worst, tilt_rad(rpy))
            if tilt_rad(rpy) > 1.2:
                fell = True
                break
            if step % sample_every == 0:
                feet_hist.append(np.array([data.geom_xpos[g][2] for g in geom_ids]))

    elapsed = max(1e-6, (total - warm_steps) * dt)
    if feet_hist:
        ft = np.asarray(feet_hist).T                     # (4, T)
        # 减去每条腿自己的最低点：抬脚信号只看“离地多高”，机身整体起伏对四腿相同，
        # 用中位数差别不大；用 10% 分位更接近“脚尖贴地”的基准。
        base = np.percentile(ft, 10, axis=1, keepdims=True)
        lift = ft - base
        phases = np.zeros(4)
        strength = []
        for i in range(4):
            lag, s = lag_cycles(lift[i], lift[0], fs, freq)
            phases[i] = (lag if i else 0.0) % 1.0
            strength.append(s)
        gait, residual = classify(phases)
        corr = float(np.mean(strength[1:]))
        # lift 已经是“离该腿最低点多高”，不用再减基准。
        foot_lift = float(np.mean(np.percentile(lift, 98, axis=1)))
    else:
        phases = np.zeros(4)
        gait, residual, corr, foot_lift = "?", 1.0, 0.0, 0.0

    disp_x = float(data.qpos[0] - (x0 or 0.0))
    disp_y = float(data.qpos[1] - (y0 or 0.0))
    yaw = float((rpy_from_quat(sensor_view(model, data)[2])[2] - yaw0 + math.pi)
                % (2 * math.pi) - math.pi)
    return Result(name, disp_x, disp_y, yaw, disp_x / elapsed, vx,
                  math.degrees(worst), float(data.qpos[2]),
                  foot_lift * 100.0, gait, residual, phases, corr, fell)


def main() -> int:
    ap = argparse.ArgumentParser(description="扫命令，量出来的步态")
    ap.add_argument("--modes", nargs="*", default=list(PRESETS),
                    help=f"预设名，可选 {', '.join(PRESETS)}")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--warmup", type=float, default=0.6)
    ap.add_argument("--vx", type=float, default=0.4)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--vyaw", type=float, default=0.0)
    ap.add_argument("--freq", type=float, default=3.0)
    ap.add_argument("--footswing", type=float, default=0.19)
    ap.add_argument("--width", type=float, default=0.3)
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--sim-hz", type=float, default=200.0)
    ap.add_argument("--sweep-plant", action="store_true",
                    help="vx=0 下扫 vy / vyaw，量原地横移与原地转向")
    ap.add_argument("--sweep-vel", action="store_true",
                    help="在 trot 上扫 vx / vy / vyaw，量遥控维度跟不跟得上")
    ap.add_argument("--footswing-grid", action="store_true",
                    help="在四种步态上扫 footswing 与 freq，看哪个组合真走起来")
    a = ap.parse_args()


    def one(name, over, **kw):
        base = dict(seconds=a.seconds, warmup=a.warmup, vx=a.vx, vy=a.vy,
                    vyaw=a.vyaw, kp=a.kp, kd=a.kd, sim_hz=a.sim_hz,
                    footswing=a.footswing, width=a.width)
        base.update(kw)
        over = dict(over)
        over.setdefault("freq", a.freq)
        r = run(name, over, **base)
        print(f"{r.name:<14} {r.gait:<6} 残差 {r.residual:.2f}  "
              f"x {r.disp_x:+6.2f} m  y {r.disp_y:+5.2f}  "
              f"yaw {math.degrees(r.yaw):+6.1f}°  v {r.speed:5.2f}  "
              f"倾 {r.max_tilt:4.1f}°  高 {r.body_h:.3f}  "
              f"抬 {r.foot_lift:4.1f} cm  相 "
              f"{''.join(f'{p:.2f} ' for p in r.phases)} r={r.corr:.2f}"
              f"{'  摔' if r.fell else ''}")
        return r

    print(f"命令 vx {a.vx}  freq {a.freq}  footswing {a.footswing}  "
          f"kp {a.kp}  {a.seconds:.0f}s\n")
    print(f"{'预设':<14} {'形态':<6} {'残差':>8}  {'位移':>10}  {'横移':>6}  "
          f"{'yaw':>8}  {'速度':>5}  {'倾角':>5}  {'高度':>5}  {'抬脚':>6}  {'相关':>5}")

    if a.sweep_plant:
        short = dict(seconds=8.0, vx=0.0)
        for vy in (-0.3, -0.15, 0.15, 0.3):
            one(f"原地 vy {vy:+.2f}", PRESETS["trot"], vy=vy, **short)
        for wz in (-0.6, -0.3, 0.3, 0.6):
            one(f"原地 yaw {wz:+.1f}", PRESETS["trot"], vyaw=wz, **short)
        return 0

    if a.sweep_vel:
        short = dict(seconds=8.0)
        for vx in (-0.4, -0.2, 0.0, 0.2, 0.4, 0.6):
            one(f"vx {vx:+.1f}", PRESETS["trot"], vx=vx, **short)
        for vy in (-0.3, -0.15, 0.15, 0.3):
            one(f"vy {vy:+.2f}", PRESETS["trot"], vy=vy, **short)
        for wz in (-0.6, -0.3, 0.3, 0.6):
            one(f"yaw {wz:+.1f}", PRESETS["trot"], vyaw=wz, **short)
        return 0

    if a.footswing_grid:
        rows = []
        for mode in a.modes:
            for fs in (0.19, 0.25, 0.30):
                for fq in (2.5, 3.0, 3.5):
                    rows.append(one(f"{mode}/fs{fs}/f{fq}",
                                    dict(PRESETS[mode], footswing=fs, freq=fq)))
        print()
        ok = [r for r in rows if not r.fell and abs(r.speed) > 0.1]
        ok.sort(key=lambda r: -abs(r.speed))
        for r in ok[:6]:
            print(f"  走得动：{r.name:<22} {r.gait:<6} {r.speed:5.2f} m/s  "
                  f"倾 {r.max_tilt:4.1f}°")
        return 0

    for mode in a.modes:
        one(mode, PRESETS[mode])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
