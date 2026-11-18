"""量直行时的航向漂移：trot 走 20 s，机头偏多少；加一点闭环修正能不能压住。

    .venv/bin/python test/heading_hold.py

为什么要单独量：迷宫里机头会在走路时自己走偏（日志里 +151° 走到 +178°，
15 s 漂 27°），模型看见偏了就补一个转向，转完接着漂 —— 用户说的「总是转过
头来回转」在修掉 vyaw 残留之后还剩这一层。

漂移来自策略本身：trot 对角小跑没有偏航项，两只前腿的落地差一点就会累积成
转向。它不是「转向算错了」，所以不能改转角的算法，只能在走路时把机头当成
一个要维持的量去做闭环。

判据三条，都必须同时看：
    drift   20 s 的航向漂移（度，绝对值）
    hold    加上修正之后的残余漂移，要明显小于 drift
    lateral 侧向偏移（m）。只压航向不够 —— 已经攒出来的侧移要靠角速度慢慢
            扭回来，如果扭回来时把身体拖到贴墙，那这个修正就不能用
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

# 走路的命令。和 `think.py` 发出去的一样：vx=0.3、trot、步频 3 Hz。
VX = 0.30
FREQ = 3.0
FOOTSWING = 0.19
GAIT = dict(phase=0.5, offset=0.0, bound=0.0)

# 修正增益与上限。策略的 vyaw 上限是 1.0（训练区间），这里远低于它：只是抵消
# 漂移，不能变成「边走边转」—— 那个动作实测会画大圈，走廊里等于撞墙。
HOLD_GAIN = 1.5
HOLD_LIMIT = 0.25


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
    drift: float          # 最大航向偏差，度（带符号，取绝对值最大的那一次）
    final: float          # 结束时的航向偏差，度
    lateral: float        # 离起点直线的横向偏移，m
    forward: float        # 前进距离，m
    worst_vyaw: float     # 用到的最大修正量
    max_tilt: float
    fell: bool


def run(name: str, *, hold: bool, seconds: float = 20.0,
        warmup: float = 1.0, sim_hz: float = 500.0) -> Result:
    dt = 1.0 / sim_hz
    dec = max(1, int(round(sim_hz / 50.0)))
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
    cmd[COMMAND_FIELDS.index("vx")] = VX
    cmd[COMMAND_FIELDS.index("vy")] = 0.0
    cmd[COMMAND_FIELDS.index("vyaw")] = 0.0
    cmd[COMMAND_FIELDS.index("freq")] = FREQ
    cmd[COMMAND_FIELDS.index("footswing")] = FOOTSWING
    for key, value in GAIT.items():
        cmd[COMMAND_FIELDS.index(key)] = value

    q_des = DEFAULT_Q[POLICY_TO_SDK].copy()
    warm_steps = int(warmup / dt)
    total = warm_steps + int(seconds / dt)

    x0 = y0 = 0.0
    base_yaw = 0.0
    worst = 0.0
    worst_signed = 0.0
    final = 0.0
    worst_tilt = 0.0
    used = 0.0
    fell = False

    for step in range(total):
        q, dq, quat = sensor_view(model, data)
        rpy = rpy_from_quat(quat)
        if step % dec == 0:
            if hold:
                err = wrap(base_yaw - rpy[2])
                v = max(-HOLD_LIMIT, min(HOLD_LIMIT, HOLD_GAIN * err))
                cmd[COMMAND_FIELDS.index("vyaw")] = v
                used = max(used, abs(v))
            q_des, _ = walker.step(q, dq, rpy, cmd)
        tau = KP * (q_des - q) - KD * dq
        data.ctrl[:] = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        mujoco.mj_step(model, data)

        if step < warm_steps:
            continue
        if step == warm_steps:
            x0, y0, base_yaw = (float(data.qpos[0]), float(data.qpos[1]),
                                rpy[2])
            continue

        dev = math.degrees(wrap(rpy[2] - base_yaw))
        if abs(dev) > abs(worst_signed):
            worst_signed = dev
        worst = max(worst, abs(dev))
        final = dev
        worst_tilt = max(worst_tilt, tilt_rad(rpy))
        if worst_tilt > 1.2:
            fell = True
            break

    dx = float(data.qpos[0]) - x0
    dy = float(data.qpos[1]) - y0
    # 侧向偏移：相对起点的初始机头方向（base_yaw）的垂直分量。
    lateral = abs(-dx * math.sin(base_yaw) + dy * math.cos(base_yaw))
    forward = dx * math.cos(base_yaw) + dy * math.sin(base_yaw)
    return Result(name=name, drift=worst_signed, final=final,
                  lateral=lateral, forward=forward, worst_vyaw=used,
                  max_tilt=math.degrees(worst_tilt), fell=fell)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    rows = [run("不修正", hold=False, seconds=args.seconds),
            run("航向闭环", hold=True, seconds=args.seconds)]

    print(f"\n{'方案':<10}{'最大偏差':>10}{'结束时':>9}{'侧移':>8}"
          f"{'前进':>8}{'vyaw上限':>10}{'倾角':>8}")
    for r in rows:
        print(f"{r.name:<10}{r.drift:>+9.1f}°{r.final:>+8.1f}°"
              f"{r.lateral:>7.2f}m{r.forward:>7.2f}m"
              f"{r.worst_vyaw:>10.2f}{r.max_tilt:>7.1f}°"
              + ("  摔了" if r.fell else ""))

    plain, held = rows
    ok = (not held.fell
          and abs(held.drift) < abs(plain.drift) * 0.5
          and held.lateral < 0.35)
    print(f"\n{'通过' if ok else '不通过'}：漂移 {abs(plain.drift):.0f}° → "
          f"{abs(held.drift):.0f}°，侧移 {held.lateral:.2f} m")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
