"""量转向：同样的 vyaw 下，换步态 / 步频 / 抬脚 / 带不带前进，转得多快。

    .venv/bin/python test/turn_sweep.py
    .venv/bin/python test/turn_sweep.py --seconds 6 --vyaw 1.0

为什么单独量：迷宫卡住的地方全是「原地转 90°」，而 `vyaw` 的上限被训练区间钉死
在 1.0（`walker.COMMAND_RANGE`），再大策略没见过，输出不是转得更快而是摔。能动的
只剩三个：

    gait        四足的相位关系，决定哪几条腿同时在蹬地
    freq        步频，2~4 Hz，同一段时间里摆几次腿
    footswing   抬脚高度，腿不抬起来就扭不动

指标是 `yaw_rate`（rad/s），但**不能只看它**：边走边转的角速度天生比原地转高，
代价是画大圈，在 1.8 m 宽的走廊里等于撞墙。所以同时记 `disp`（走了多远）和
`max_tilt`，用来判断这个角速度是不是白给的。

物理与 `gait_sweep.py` 同一套：MuJoCo 500 Hz、策略 50 Hz、PD 每步重算。
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

PRESETS: dict[str, dict[str, float]] = {
    "pronk": dict(phase=0.0, offset=0.0, bound=0.0),
    "trot": dict(phase=0.5, offset=0.0, bound=0.0),
    "bound": dict(phase=0.5, offset=0.0, bound=0.5),
    "pace": dict(phase=0.0, offset=0.0, bound=0.5),
}


def rpy_from_quat(quat) -> list[float]:
    w, x, y, z = (float(v) for v in quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


# 关节顺序用 sensor 名取：(FR, FL, RR, RL) 恰好是 `POLICY_TO_SDK` 的顺序。
# 和 `gait_sweep.sensor_view` 同一份实现，两边量的数才能直接比。
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


@dataclass
class TurnResult:
    name: str
    yaw_rate: float          # rad/s，正为逆时针
    disp: float              # 水平位移，m
    max_tilt: float          # 最大倾角，度
    body_h: float            # 末段机身高度，m
    fell: bool


def run(name: str, *, gait: str, vyaw: float, vx: float, freq: float,
        footswing: float, vy: float = 0.0, seconds: float = 4.0, warmup: float = 1.0,
        kp: float = KP, kd: float = KD, sim_hz: float = 500.0) -> TurnResult:
    dt = 1.0 / sim_hz
    dec = max(1, int(round(sim_hz / 50.0)))
    model = mujoco.MjModel.from_xml_path(str(flat_scene()))
    model.opt.timestep = dt
    data = mujoco.MjData(model)

    data.qpos[:] = 0.0
    data.qpos[2] = 0.31
    data.qpos[3] = 1.0                    # 四元数 w，机体正立
    for i, leg in enumerate(LEGS):
        for j, part in enumerate(("hip", "thigh", "calf")):
            data.qpos[int(model.joint(f"{leg}_{part}_joint").qposadr[0])] = \
                DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    walker = load_walker(dt=dt * dec)
    cmd = default_commands()
    cmd[COMMAND_FIELDS.index("vx")] = vx
    cmd[COMMAND_FIELDS.index("vy")] = vy
    cmd[COMMAND_FIELDS.index("vyaw")] = vyaw
    cmd[COMMAND_FIELDS.index("freq")] = freq
    cmd[COMMAND_FIELDS.index("footswing")] = footswing
    for key, value in PRESETS[gait].items():
        cmd[COMMAND_FIELDS.index(key)] = value

    q_des = DEFAULT_Q[POLICY_TO_SDK].copy()
    warm_steps = int(warmup / dt)
    total = warm_steps + int(seconds / dt)

    x0 = y0 = 0.0
    worst = 0.0
    fell = False
    yaw_total = 0.0
    last_yaw = 0.0
    h_tail: list[float] = []

    for step in range(total):
        q, dq, quat = sensor_view(model, data)
        rpy = rpy_from_quat(quat)
        if step % dec == 0:
            q_des, _ = walker.step(q, dq, rpy, cmd)
        tau = kp * (q_des - q) - kd * dq
        data.ctrl[:] = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        mujoco.mj_step(model, data)

        if step < warm_steps:
            continue
        if step == warm_steps:
            x0, y0, last_yaw = float(data.qpos[0]), float(data.qpos[1]), rpy[2]

        # yaw 绕 ±pi，累加增量而不是首尾相减。
        d = (rpy[2] - last_yaw + math.pi) % (2 * math.pi) - math.pi
        yaw_total += d
        last_yaw = rpy[2]

        t = tilt_rad(rpy)
        worst = max(worst, t)
        if step >= total - int(1.0 / dt):
            h_tail.append(float(data.qpos[2]))
        if t > 1.2:
            fell = True
            break

    elapsed = max(1e-6, (total - warm_steps) * dt)
    return TurnResult(
        name=name,
        yaw_rate=yaw_total / elapsed,
        disp=math.hypot(float(data.qpos[0]) - x0, float(data.qpos[1]) - y0),
        max_tilt=math.degrees(worst),
        body_h=float(np.median(h_tail)) if h_tail else float(data.qpos[2]),
        fell=fell,
    )


# 第一行是基线：迷宫现在跑的就是这一组，实测 0.32 rad/s。
CASES: list[tuple[str, dict]] = [
    ("pronk 基线",         dict(gait="pronk", vx=0.0, freq=3.0, footswing=0.19)),
    ("pronk freq4",        dict(gait="pronk", vx=0.0, freq=4.0, footswing=0.19)),
    ("pronk 高抬脚",       dict(gait="pronk", vx=0.0, freq=3.0, footswing=0.30)),
    ("pronk freq4+高抬脚", dict(gait="pronk", vx=0.0, freq=4.0, footswing=0.30)),
    ("pronk 带 0.15 前进", dict(gait="pronk", vx=0.15, freq=3.0, footswing=0.25)),
    ("pronk freq4 带 0.15", dict(gait="pronk", vx=0.15, freq=4.0, footswing=0.30)),
    ("pronk 带 0.30 前进", dict(gait="pronk", vx=0.30, freq=3.0, footswing=0.19)),
    ("trot 带 0.30 前进",  dict(gait="trot", vx=0.30, freq=3.0, footswing=0.19)),
    ("pace 基线",          dict(gait="pace", vx=0.0, freq=3.0, footswing=0.19)),
    ("bound 基线",         dict(gait="bound", vx=0.0, freq=3.0, footswing=0.19)),
]


def main() -> int:
    ap = argparse.ArgumentParser(description="量不同参数下的转向速度")
    ap.add_argument("--vyaw", type=float, default=1.0,
                    help="转向命令幅度，范围 1.0 是训练上限（默认 1.0）")
    ap.add_argument("--seconds", type=float, default=4.0, help="每组跑多久")
    ap.add_argument("--only", nargs="*", default=None,
                    help="只跑名字里含这些词的组合")
    args = ap.parse_args()

    cases = CASES
    if args.only:
        cases = [c for c in CASES if any(k in c[0] for k in args.only)]
    if not cases:
        print("没有匹配的组合")
        return 2

    print(f"vyaw={args.vyaw:+.1f}，每组 {args.seconds:.0f} s"
          f"（另加 1 s 起立），按 yaw 角速度排序\n")
    rows = []
    for name, kw in cases:
        r = run(name, vyaw=args.vyaw, seconds=args.seconds, **kw)
        rows.append(r)
        flag = "摔" if r.fell else ""
        print(f"  {name:<20} {r.yaw_rate:+6.2f} rad/s  "
              f"位移 {r.disp:4.2f} m  倾角 {r.max_tilt:4.1f}°  "
              f"体高 {r.body_h:.2f} m  {flag}", flush=True)

    print("\n按角速度排序：")
    for r in sorted(rows, key=lambda x: -x.yaw_rate):
        print(f"  {r.yaw_rate:+6.2f} rad/s  {r.name}"
              + ("  ← 摔了，不可用" if r.fell else ""))

    print("\n参考：原地转 90° 需要的时间")
    for r in sorted(rows, key=lambda x: -abs(x.yaw_rate))[:3]:
        if r.fell or abs(r.yaw_rate) < 1e-3:
            continue
        print(f"  {r.name:<20} {math.pi / 2 / abs(r.yaw_rate):5.1f} s"
              f"（绕行 {r.disp * math.pi / 2 / max(abs(r.yaw_rate), 1e-6) / 4:.2f} m）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
