"""纯位置伺服起站：只往低层写 12 个目标角，中间没有任何平衡层。

**这条现在是失败的，别默认用。** 实测从趴着的折叠姿态（大腿 1.31 / 小腿 −2.73）插值到
`DEFAULT_Q`（0.8 / −1.5），四条腿同时把脚往地里顶，每条腿的力矩都撞在 23.7 N·m 的限幅
上，竖直分量合起来远超体重，机身被顶翻：倾角在 1.2 s 内从 4° 涨到 173°，仿真里就是
「甩飞」。要修得换成足端轨迹（脚踩住不动、机身高度按轨迹抬）+ 力矩限幅，不是关节插值。
代码留着是因为 `rise()` 的骨架（claim / 插值 / 收尾压策略增益 / 残余量回报）还能用。

推理控制器要接管时，机器人往往还趴着。把「从趴到站」这件事交给高层运控，等于让
MPC + WBC 先决定一个站姿，再由策略从这个站姿出发 —— 而策略的 `DEFAULT_Q` 与高层
的站姿差到 0.4 rad（小腿），交接瞬间就是一记硬拉。

这里换成另一种做法：目标角直接取 `DEFAULT_Q`，从当前姿态按余弦插值过去，全程只用
`link.joint_target()`。没有重力前馈、没有足端力分配、没有平衡反馈 —— 谁在写关节，
谁就是唯一的控制器。姿态插完之后再压一段策略自己的增益（kp 20 / kd 0.5），让交接点
落在策略训练时的工作点上。

    link.claim("rise")
    rise(link)                       # 站起来，停在 DEFAULT_Q
    svc = MotionService(link)
    svc.start()                      # 接管

## 为什么不用高层的 `stand()`

除了站姿不同，还有一条：高层的 `stand()` 会把 200 Hz 回路切进 `hold`，那段回路每帧
都在写 lowcmd。交接时它必须停下（`_mode` 切 idle），而「停下」这个动作本身就是一次
时序事件 —— 起站和接管之间多一次状态切换，就多一次「谁在写」的空窗。这里从头到尾
只有一个写者。

## 限幅

`kp * (q_des - q)` 在刚起步的第一帧可能很大：趴着的时候关节角与目标差到 2 rad，
kp=80 就是 160 N·m。所以起站用的增益比站立低（默认 80），且每帧的角速度由插值曲线
本身限定 —— 一帧最多走 `seconds` 对应的那一小段。
"""

from __future__ import annotations

import math
import time

import numpy as np

from quadruped.motion.link import LinkBusy
from quadruped.policy.walker import DEFAULT_Q, KD, KP, POLICY_TO_SDK, tilt_rad

#: 起站的目标姿态就是策略的默认站姿，不是高层运控那一套。
TARGET = DEFAULT_Q[POLICY_TO_SDK].copy()


# 从当前姿态余弦插值到目标，最后一段换成策略增益。
#
# `kp` 用 80：够把机身从地上顶起来，又不至于第一帧就把力矩顶到限幅。`seconds`
# 0.1 量级会甩腿，1.0 以上看不出动作；1.2 s 是从趴到站能从画面上读出来的最短时间。
#
# 返回值里 `q_err` 是收尾时目标与实测的最大差 —— 它等于「交接时策略要补多少」，
# 超过 0.3 rad 说明这次起站没到位，别接着跑策略。
def rise(link, target=None, *, kp: float = 80.0, kd: float = 2.0,
         seconds: float = 1.2, settle: float = 0.6, rate: float = 50.0,
         kp_settle: float = KP, kd_settle: float = KD,
         fall_rad: float = 1.2, verbose: bool = True) -> dict:
    q_goal = TARGET.copy() if target is None else np.asarray(target, float).reshape(-1)
    q0 = link.q
    if q0 is None or len(q0) != 12:
        raise RuntimeError("拿不到 12 维关节角，无法起站")
    q0 = np.asarray(q0, float).copy()

    try:
        link.claim("rise")
    except LinkBusy as err:
        raise RuntimeError(f"起站拿不到写入权：{err}") from None

    dt = 1.0 / rate
    n = max(1, int(seconds * rate))
    n_settle = max(0, int(settle * rate))
    worst = 0.0
    try:
        for i in range(1, n + 1):
            # 余弦插值：两端速度为 0，起步和到顶都不甩。
            s = 0.5 - 0.5 * math.cos(math.pi * i / n)
            link.joint_target(q0 + (q_goal - q0) * s, kp=kp, kd=kd)
            worst = _tilt(link, worst)
            time.sleep(dt)
        for _ in range(n_settle):
            link.joint_target(q_goal, kp=kp_settle, kd=kd_settle)
            worst = _tilt(link, worst)
            time.sleep(dt)
    finally:
        link.release("rise")

    q_end = link.q
    q_err = (float(np.abs(np.asarray(q_end, float) - q_goal).max())
             if q_end is not None else float("nan"))
    out = {"q_err": round(q_err, 3), "tilt": round(math.degrees(worst), 1)}
    if verbose:
        print(f"[rise] 起站完成：残余 {q_err:.3f} rad  最大倾角 "
              f"{out['tilt']:.1f}°  kp {kp:g}→{kp_settle:g}")
    return out


def _tilt(link, worst: float) -> float:
    rpy = link.rpy
    return worst if rpy is None else max(worst, tilt_rad(rpy))
