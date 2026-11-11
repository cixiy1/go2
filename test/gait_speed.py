#!/usr/bin/env python3
"""测步态实际跑多快：给定指令速度和一组步态参数，跑固定时长，报速度/姿态/漂移。

    # 先起仿真器：./quadruped/sim/run_sim.sh
    uv run python test/gait_speed.py --vx 0.9 --dur 8
    uv run python test/gait_speed.py --vx 0.9 --dur 8 --wrench 0
    uv run python test/gait_speed.py --vx 0.9 --dur 8 --stride-max 0.12

为什么单独写一个：`control_robot.py trot` 只打中间过程的前向读数，那个读数
来自 sportmodestate 的 position，机身转起来之后它在世界系里绕圈，看起来忽大
忽小，不能直接当速度用。这里按机身**实际位移**算平均速度，同时统计 yaw 漂移
和姿态峰峰 —— 判断「跑得快」还是「原地打转」要看这三个一起。

每次测完脚本会给腿下发阻尼。**再测下一组要重启仿真器**，否则从瘫倒姿态开始
插值，结果不可比（这条踩过）。
"""

import argparse
import math
import os
import sys
import threading
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadruped.robot.body import Go2Robot


def main() -> int:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--iface", default=None, help="lo0 = 仿真，其它 = 真机")
    ap.add_argument("--vx", type=float, default=0.9, help="前进指令 m/s")
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--dur", type=float, default=8.0, help="步态持续秒数")
    ap.add_argument("--style", default="mech", choices=("mech", "bio", "search"))
    ap.add_argument("--wrench", type=float, default=None,
                    help="覆盖机身层权重：0 = 老路子（纯位置伺服 + 重力前馈），1 = VMC")
    ap.add_argument("--stride-max", type=float, default=None)
    ap.add_argument("--duty", type=float, default=None)
    ap.add_argument("--period", type=float, default=None)
    ap.add_argument("--swing", type=float, default=None)
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--height", type=float, default=None)
    ap.add_argument("--ideal", action="store_true",
                    help="走理想模型旁路（力矩当帧到位、无噪声摩擦），用来对照早期仿真")
    a = ap.parse_args()

    real = bool(a.iface) and not a.iface.startswith("lo")
    kw = {k: v for k, v in (("height", a.height), ("kp", a.kp), ("kd", a.kd))
          if v is not None}
    robot = Go2Robot(preset=("real" if real else "sim"), iface=a.iface,
                     real_params=(False if a.ideal else None), **kw)
    if not robot.connect():
        print("连不上，先起仿真器或查网卡", flush=True)
        return 1
    c = robot.controller
    c.set_gait_style(a.style)
    g = c.gait
    for key, val in (("wrench", a.wrench), ("stride_max", a.stride_max),
                     ("duty", a.duty), ("period", a.period),
                     ("swing_height", a.swing)):
        if val is not None:
            setattr(g, key, val)
    if a.period is not None or a.duty is not None:
        from quadruped.control.controller import _swing_poly_coeffs
        g.a3, g.a4 = _swing_poly_coeffs(g.duty)

    print(f"\n参数：{a.style} period={g.period} duty={g.duty} "
          f"swing={g.swing_height * 1000:.0f}mm smax={g.stride_max * 1000:.0f}mm "
          f"wrench={g.wrench} kp={c.kp[0]:.0f} kd={c.kd[0]:.1f}", flush=True)

    samples: list[tuple[float, tuple, tuple]] = []
    stop = threading.Event()

    def run():
        c.trot(vx=a.vx, vy=a.vy, yaw=a.yaw, duration=a.dur)
        stop.set()

    th = threading.Thread(target=run, daemon=True)
    th.start()
    t0 = time.time()
    while not stop.is_set() and time.time() - t0 < a.dur + 20:
        if c.pos is not None and c.rpy:
            samples.append((time.time() - t0, tuple(c.pos), tuple(c.rpy)))
        time.sleep(0.02)
    th.join(timeout=5.0)

    if len(samples) < 10:
        print("采样太少，没跑起来", flush=True)
        robot.shutdown()
        return 1

    p0, p1 = samples[0][1], samples[-1][1]
    t_run = samples[-1][0] - samples[0][0]
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dist = math.hypot(dx, dy)
    yaw0 = samples[0][2][2]
    yaw = [math.atan2(math.sin(s[2][2] - yaw0), math.cos(s[2][2] - yaw0))
           for s in samples]
    pit = [s[2][1] for s in samples]
    rol = [s[2][0] for s in samples]
    stats = c.loop_stats()

    print(f"\n位移 {dx:+.3f} / {dy:+.3f} m   路程 {dist:.3f} m   "
          f"用时 {t_run:.1f} s   平均速度 {dist / t_run:.3f} m/s "
          f"（指令 {a.vx:.2f}，达成 {100 * dist / t_run / max(a.vx, 1e-6):.0f}%）",
          flush=True)
    print(f"yaw 漂移 {yaw[-1]:+.2f} rad（{math.degrees(yaw[-1]):+.0f}°）  "
          f"pitch 峰峰 {1000 * (max(pit) - min(pit)):.1f} mrad  "
          f"roll 峰峰 {1000 * (max(rol) - min(rol)):.1f} mrad", flush=True)
    print(f"回路 均值 {stats['mean']:.2f} ms  p95 {stats['p95']:.2f} ms  "
          f"max {stats['max']:.2f} ms  超时 {stats['over']} 帧", flush=True)

    robot.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
