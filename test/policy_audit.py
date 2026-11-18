"""训练参数与运行时实际取值的一致性检查。

    .venv/bin/python test/policy_audit.py

训练侧的真值在 `quadruped/policy/training.py`（来源与推导写在那份文件顶部）。
这个脚本不重复那些数，只做一件事：把运行时**真正传下去**的值读出来比对。
任何一项对不上就返回 1，并打印两边。

## 为什么要它

这些数分散在五个地方都用过：`walker.py` 的常量、`motion/service.py` 的控制周期与
动作限幅、`virtual/policy.py` 的起手插值、`motion/rise.py` 的站起增益、`rl/env.py`
的搜索环境。历史上真出过两次错，两次都不是「数写错了」，而是**只有一处跟着改**：

- `mlp.py` 的输出层多套了一层 ELU，动作空间只剩正半边，最后靠把 kp 提到 70 顶上去
- `commands_scale` 照参考部署实现抄，多了一项，站宽被压到三分之一

两者都不会报错，只会「走得像但不对」。所以判据不放在注释里，放在这个脚本里。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from quadruped.policy import gaits                                   # noqa: E402
from quadruped.policy import training as T
from quadruped.policy import walker as W
from quadruped import limits as L
from quadruped.motion import service as S

# (名字, 训练值, 运行时值)。运行时值一律现读对象属性，不在脚本里重抄一遍。
def rows() -> list[tuple[str, object, object]]:
    cmd = W.default_commands()
    nominal = dict(zip(W.COMMAND_FIELDS, (float(v) for v in cmd)))
    out = [
        ("关节刚度 kp", T.STIFFNESS, W.KP),
        ("关节阻尼 kd", T.DAMPING, W.KD),
        ("动作缩放 action_scale", T.ACTION_SCALE, W.ACTION_SCALE),
        ("髋关节减半 hip_scale", T.HIP_SCALE_REDUCTION, W.HIP_SCALE),
        ("动作限幅 clip_actions", T.CLIP_ACTIONS, W.CLIP_ACTIONS),
        ("观测维度", T.NUM_OBS, W.OBS_DIM),
        ("历史帧数", T.NUM_HISTORY, W.HISTORY),
        ("观测 dof_pos 缩放", T.DOF_POS_SCALE, W.DOF_POS_SCALE),
        ("观测 dof_vel 缩放", T.DOF_VEL_SCALE, W.DOF_VEL_SCALE),
        ("控制周期 s", T.CONTROL_DT, S.DT),
        ("力矩上限 小腿", T.TAU_LIMIT[2], float(W.TAU_LIMIT[2])),
    ]
    for i, name in enumerate(W.COMMAND_FIELDS):
        out.append((f"命令缩放 {name}", float(T.COMMAND_SCALE[i]),
                    float(W.COMMAND_SCALE[i])))
    for name, value in T.NOMINAL_COMMAND.items():
        out.append((f"默认命令 {name}", float(value), nominal[name]))
    out.append(("服务里动作限幅", T.CLIP_ACTIONS, S.ACTION_LIMIT))
    out.append(("服务里动作 slew", None, S.ACTION_SLEW))
    return out


# 命令区间必须涵盖默认值与所有步态预设，否则切个步态就被静默夹住。
def range_checks() -> list[str]:
    bad = []
    cmd = W.default_commands()
    for name, (lo, hi) in W.COMMAND_RANGE.items():
        value = float(cmd[W.COMMAND_FIELDS.index(name)])
        if not lo <= value <= hi:
            bad.append(f"默认命令 {name}={value:g} 不在 [{lo:g}, {hi:g}] 里")
    for gait, preset in gaits.PRESETS.items():
        for name, value in preset.items():
            lo, hi = W.COMMAND_RANGE[name]
            if not lo <= value <= hi:
                bad.append(f"步态 {gait} 的 {name}={value:g} 不在 [{lo:g}, {hi:g}] 里")
    if W.COMMAND_RANGE.keys() != set(W.COMMAND_FIELDS):
        bad.append("COMMAND_RANGE 的键与 COMMAND_FIELDS 不一致")
    # `limits.py` 不许 import 项目模块（arch_check 规则 3），那两个上限是字面量，
    # 靠这里跟策略范围对齐。
    for field, value in (("vx", L.VX_MAX), ("vyaw", L.VYAW_MAX)):
        hi = W.COMMAND_RANGE[field][1]
        if abs(value - hi) > 1e-9:
            bad.append(f"limits.{field.upper()}_MAX={value:g} 与策略上限 {hi:g} 不一致")
    return bad


def fmt(value) -> str:
    if value is None:
        return "不限"
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def main() -> int:
    bad = 0
    width = max(len(name) for name, _, _ in rows()) + 2
    print(f"{'参数':<{width}}{'训练值':>12}{'运行时':>12}   判定")
    for name, want, have in rows():
        same = (want is None and have is None) or (
            want is not None and have is not None and abs(float(want) - float(have)) < 1e-9)
        if not same:
            bad += 1
        print(f"{name:<{width}}{fmt(want):>12}{fmt(have):>12}   {'一致' if same else '不一致'}")

    problems = range_checks()
    for line in problems:
        print(f"\n区间问题：{line}")
    bad += len(problems)

    # 力矩来源不在「参数」这一栏：训练用 actuator_net，真机 LowCmd 走 PD，
    # 两者是同一执行器的两种近似，选择由 `--torque` 给，这里只报当前默认。
    print(f"\n力矩来源：默认 PD（kp {W.KP:g} / kd {W.KD:g}）；`--torque net` 换成"
          f"训练时的电机模型。两条路的实测见 `test/pd_audit.py`。")
    print(f"结论：{'全部一致' if bad == 0 else f'{bad} 项不一致'}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
