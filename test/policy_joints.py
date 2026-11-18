"""看策略到底输出了哪 12 个关节角：形状、顺序、幅度、左右对称。

    # 单帧静态：当前姿态假定是 DEFAULT_Q，不跑物理
    .venv/bin/python test/policy_joints.py --dry

    # 闭环：MuJoCo 里真跑，每 0.5 s 打一行，最后汇总每个关节的幅度
    .venv/bin/python test/policy_joints.py --seconds 6

`policy_sim.py` 判的是「站不站得住」，这个脚本判的是**输出本身**：12 维、有限、
按 LowCmd 序、左右腿镜像。站不住是另一个问题 —— 一个站不住但输出合法的策略，
和一个输出接反了导致站不住的策略，修法完全不同。

## 这里不查关节限位

限位数在模型里（`go2.xml` 的 `jnt_range`）和真机的挡块上，命令层不做这件事：训练侧
发的是位置目标，超限的目标照样进 PD，多出来的力矩由 PhysX 的关节约束顶住；真机
驱动器同理。所以脚本也不拿限位做判据 —— 关节角跑到行程之外是这个策略的行为，
不是脚本该替它修的东西。

要查某个角是不是超出机械行程，去 `go2.xml` 查 `jnt_range`，别在这里再造一份抄录。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test"))

import mujoco                                                          # noqa: E402

import policy_sim as PS                                                 # noqa: E402
from quadruped.policy.walker import (                                   # noqa: E402
    DEFAULT_Q, POLICY_TO_SDK, TAU_LIMIT, default_commands, load_walker, tilt_rad,
)

# LowCmd / LowState 的电机顺序。
LEGS = ("FR", "FL", "RR", "RL")
PARTS = ("hip", "thigh", "calf")

# 与 SDK 序对齐的默认站姿，用来判断「左右腿是否镜像」。
DEFAULT_SDK = DEFAULT_Q[POLICY_TO_SDK]


def label(i: int) -> str:
    return f"{LEGS[i // 3]}_{PARTS[i % 3]}"


def table(q_des, action=None, q_now=None) -> str:
    head = f"  {'关节':<14}{'q_des':>9}"
    head += f"{'action':>9}" if action is not None else ""
    head += f"{'当前 q':>9}" if q_now is not None else ""
    lines = [head]
    for i in range(12):
        row = f"  {label(i):<14}{q_des[i]:>+9.3f}"
        if action is not None:
            row += f"{action[i]:>+9.3f}"
        if q_now is not None:
            row += f"{q_now[i]:>+9.3f}"
        lines.append(row)
    return "\n".join(lines)


def summary(samples: np.ndarray) -> str:
    """samples: (帧数, 12)，按 SDK 序。"""
    if not len(samples):
        return "  没采到样本"
    lo = samples.min(axis=0)
    hi = samples.max(axis=0)
    lines = [f"  每关节 q_des 的幅度（共 {len(samples)} 拍）：",
             f"  {'关节':<14}{'最小':>9}{'最大':>9}{'跨度':>9}"]
    for i in range(12):
        lines.append(f"  {label(i):<14}{lo[i]:>+9.3f}{hi[i]:>+9.3f}"
                     f"{(hi[i] - lo[i]):>9.3f}")

    # 左右镜像：hip 是反向安装，两腿的 hip 偏离应当反号；thigh / calf 同号。
    # 对不上说明 POLICY_TO_SDK 换错了，或者网络输入的关节顺序接反 —— 两者的表现
    # 都是「前腿模仿后腿的动作」，肉眼看只觉得走得怪。
    d = samples - DEFAULT_SDK
    # 列下标按 SDK 序：FR=0..2, FL=3..5, RR=6..8, RL=9..11
    asym = [
        ("hip   FR/FL", np.abs(d[:, 0] + d[:, 3])),
        ("thigh FR/FL", np.abs(d[:, 1] - d[:, 4])),
        ("calf  FR/FL", np.abs(d[:, 2] - d[:, 5])),
        ("hip   RR/RL", np.abs(d[:, 6] + d[:, 9])),
        ("thigh RR/RL", np.abs(d[:, 7] - d[:, 10])),
        ("calf  RR/RL", np.abs(d[:, 8] - d[:, 11])),
    ]
    lines.append("  左右对称性（相对 DEFAULT_Q 的偏离，0 = 完全镜像）：")
    for name, diff in asym:
        lines.append(f"    {name}  最大偏差 {diff.max():.3f} rad")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="看策略输出的 12 个关节角")
    ap.add_argument("--dry", action="store_true", help="不跑物理，单帧静态")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--warmup", type=float, default=0.6)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--every", type=float, default=0.5, help="每几秒打一行")
    a = ap.parse_args()

    walker = load_walker(dt=PS.DT * PS.DECIMATION)
    cmd = default_commands()
    cmd[0] = a.vx
    hold = DEFAULT_SDK.copy()

    if a.dry:
        # 观测里的关节偏差为 0，历史全 0 —— 这是「刚 reset 的第一帧」，训练里也见过。
        q_des, action = walker.step(hold, np.zeros(12), [0.0, 0.0, 0.0], cmd)
        print("单帧输出（当前姿态 = DEFAULT_Q，历史全 0）：")
        print(table(q_des, action, hold))
        print(f"\n  形状 {q_des.shape}  有限 {np.all(np.isfinite(q_des))}  "
              f"最大 |action| {np.abs(action).max():.2f}")
        return 0

    model = mujoco.MjModel.from_xml_path(str(PS.SCENE))
    model.opt.timestep = PS.DT
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[2] = 0.31
    data.qpos[3] = 1.0
    for i, leg in enumerate(("FL", "FR", "RL", "RR")):
        for j, part in enumerate(PARTS):
            data.qpos[int(model.joint(f"{leg}_{part}_joint").qposadr[0])] = \
                DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    warm = int(a.warmup / PS.DT)
    total = warm + int(a.seconds / PS.DT)
    every = max(1, int(a.every / PS.DT))
    samples = []
    first = True
    q_des = hold

    for step in range(total):
        q, dq, quat = PS.sensor_view(model, data)
        rpy = PS.rpy_from_quat(quat)
        if step % PS.DECIMATION == 0:
            q_des, action = walker.step(q, dq, rpy, cmd)
            if step >= warm:
                samples.append(q_des.copy())
                if first:
                    print(f"接管后第一帧（倾角 "
                          f"{math.degrees(tilt_rad(rpy)):.1f}°）：")
                    print(table(q_des, action, q))
                    first = False
                elif step % every == 0:
                    print(f"\n  t={(step - warm) * PS.DT:4.1f}s  "
                          f"倾角 {math.degrees(tilt_rad(rpy)):5.1f}°")
                    print(table(q_des, action, q))
        # 力矩走 PD，与 policy_sim 一致；这一步只为把 q_des 放进真实闭环里看它怎么被用。
        hold_q = q_des if step >= warm else hold
        data.ctrl[:] = np.clip(20.0 * (hold_q - q) - 0.5 * dq, -TAU_LIMIT, TAU_LIMIT)
        mujoco.mj_step(model, data)
        if step >= warm and tilt_rad(rpy) > 1.2:
            print(f"\n  倒了：t={(step - warm) * PS.DT:.2f}s")
            break

    print()
    print(summary(np.array(samples)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
