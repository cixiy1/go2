"""转 90° 之后停得住吗：拿 `think.turn_command` 在 MuJoCo 上闭环，量收尾的过冲。

    .venv/bin/python test/turn_overshoot.py

为什么要单独量：修掉 vyaw 残留、提到 20 Hz 之后，迷宫日志里还剩下一种漂移 ——
转向结束在 +68°，两秒后 +82°，稳在 +83°；另一段结束在 +151°，稳在 +178°。
平地直行 20 s 只漂 0.9°（`heading_hold.py`），所以这 15~30° 不是走路漂的，
是转向收尾带出来的角动量：判定「到位」那一刻机体还在转，切断命令后靠摩擦慢
慢停，又多转二十几度。模型看见偏了就补一个转向，转完再漂。

判据是两条：`loop` 是控制环节判定到位时的角度误差，`rest` 是之后 6 秒不再
发 vyaw 时又转了多少。后者才是模型看到的东西。
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

from quadruped.llm.think import (                                       # noqa: E402
    TURN_FLOOR,
    TURN_GAIN,
    TURN_TICK,
    TURN_TOL,
    turn_command,
)
from quadruped.policy import default_commands, load_walker              # noqa: E402
from quadruped.policy.walker import (                                  # noqa: E402
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
# 和 `think.TROT` 一样：转向全程 trot，不切 pronk。
TROT = dict(phase=0.5, offset=0.0, bound=0.0)

# 和 `think.py` 发出去的一样：转向时带一点前进（`TURN_VX`），转完按 trot 走。
TURN_VX = 0.15
VX_WALK = 0.30
FREQ = 3.0
FOOTSWING = 0.19
TICK = TURN_TICK       # 转向闭环周期，和 `think.py` 用同一个值


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


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Result:
    name: str
    loop: float           # 控制环判定到位时的误差，度
    rest: float           # 之后 6 s 又转了多少，度（带符号，过冲为正）
    seconds: float        # 控制环用了多久
    jitter: float         # 收尾阶段误差的峰峰值，度（来回摆的幅度）
    max_tilt: float
    fell: bool


def run(name: str, *, target_deg: float = 90.0, old: bool = False,
        rest_seconds: float = 6.0, sim_hz: float = 500.0) -> Result:
    dt = 1.0 / sim_hz
    dec = max(1, int(round(sim_hz / 50.0)))
    tick_steps = int(round(TICK / dt))
    model = mujoco.MjModel.from_xml_path(str(flat_scene()))
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    data.qpos[:] = 0.0
    data.qpos[2] = 0.31
    data.qpos[3] = 1.0
    for i, leg in enumerate(LEGS):
        for j, part in enumerate(("hip", "thigh", "calf")):
            data.qpos[int(model.joint(f"{leg}_{part}_joint").qposadr[0])] = \
                DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    walker = load_walker(dt=dt * dec)
    cmd = default_commands()
    cmd[COMMAND_FIELDS.index("vy")] = 0.0
    cmd[COMMAND_FIELDS.index("freq")] = FREQ
    cmd[COMMAND_FIELDS.index("footswing")] = FOOTSWING
    for key, value in TROT.items():
        cmd[COMMAND_FIELDS.index(key)] = value

    q_des = DEFAULT_Q[POLICY_TO_SDK].copy()
    warm = int(1.0 / dt)
    yaw0 = 0.0
    target = 0.0
    handed = False
    speed = 0.0
    loop_deg = 0.0
    loop_time = 0.0
    step = 0

    def advance() -> list[float]:
        nonlocal q_des, step
        q, dq, quat = sensor_view(model, data)
        rpy = rpy_from_quat(quat)
        if step % dec == 0:
            q_des, _ = walker.step(q, dq, rpy, cmd)
        tau = KP * (q_des - q) - KD * dq
        data.ctrl[:] = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        mujoco.mj_step(model, data)
        step += 1
        return rpy_from_quat(sensor_view(model, data)[2])

    for _ in range(warm):
        advance()
    rpy = rpy_from_quat(sensor_view(model, data)[2])
    yaw0 = rpy[2]
    target = yaw0 + math.radians(target_deg)
    cmd[COMMAND_FIELDS.index("vx")] = TURN_VX

    # ---- 控制环：每 50 ms 更新一次 vyaw ----
    worst_tilt = 0.0
    fell = False
    while step * dt < warm * dt + 20.0:
        rpy = rpy_from_quat(sensor_view(model, data)[2])
        err = wrap(target - rpy[2])
        wz = float(data.sensordata[int(model.sensor("imu_gyro").adr[0]) + 2])
        if old:
            # 基线：只按角度给，进容差就停（这是改之前的行为）。
            if abs(err) <= TURN_TOL:
                speed = 0.0
                break
            speed = max(-1.0, min(1.0, err * TURN_GAIN))
            if abs(speed) < TURN_FLOOR:
                speed = math.copysign(TURN_FLOOR, err)
        else:
            got = turn_command(err, wz)
            if got is None:
                speed = 0.0
                break
            speed = got
        cmd[COMMAND_FIELDS.index("vyaw")] = speed
        for _ in range(tick_steps):
            advance()
        worst_tilt = max(worst_tilt, tilt_rad(rpy_from_quat(
            sensor_view(model, data)[2])))
        if worst_tilt > 1.2:
            fell = True
            break

    rpy = rpy_from_quat(sensor_view(model, data)[2])
    loop_deg = math.degrees(wrap(rpy[2] - target))
    loop_time = step * dt - 1.0

    # ---- 收尾：不再发 vyaw、改回走路速度，看它自己还转多少 ----
    # 步态一直是 trot，这里不用切（生产里 `_restore_gait` 也只兜底别处改过的情况）。
    cmd[COMMAND_FIELDS.index("vx")] = VX_WALK
    cmd[COMMAND_FIELDS.index("vyaw")] = 0.0

    start = rpy[2]
    devs = []
    for _ in range(int(rest_seconds / dt)):
        advance()
        rpy = rpy_from_quat(sensor_view(model, data)[2])
        devs.append(math.degrees(wrap(rpy[2] - start)))
        worst_tilt = max(worst_tilt, tilt_rad(rpy))
        if worst_tilt > 1.2:
            fell = True
            break

    rest = devs[-1] if devs else 0.0
    jitter = (max(devs) - min(devs)) if devs else 0.0
    return Result(name=name, loop=loop_deg, rest=rest, seconds=loop_time,
                  jitter=jitter, max_tilt=math.degrees(worst_tilt), fell=fell)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=90.0)
    ap.add_argument("--rest", type=float, default=6.0)
    args = ap.parse_args()

    rows = [run("旧（只看角度）", target_deg=args.angle, old=True,
                rest_seconds=args.rest),
            run("新（含角速度）", target_deg=args.angle, rest_seconds=args.rest)]

    print(f"\n转 {args.angle:.0f}°\n")
    print(f"{'方案':<14}{'到位时误差':>11}{'收尾又转':>10}{'来回幅度':>10}"
          f"{'用时':>7}{'倾角':>8}")
    for r in rows:
        print(f"{r.name:<14}{r.loop:>+10.1f}°{r.rest:>+9.1f}°"
              f"{r.jitter:>9.1f}°{r.seconds:>6.1f}s{r.max_tilt:>7.1f}°"
              + ("  摔了" if r.fell else ""))

    old, new = rows
    ok = (not new.fell
          and not any(r.fell for r in rows)
          and abs(new.rest) < max(4.0, abs(old.rest) * 0.5))
    print(f"\n{'通过' if ok else '不通过'}：收尾过冲 {abs(old.rest):.0f}° → "
          f"{abs(new.rest):.0f}°，来回幅度 {new.jitter:.0f}°")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
