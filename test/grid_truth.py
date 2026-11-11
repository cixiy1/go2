"""在真链路上扫步态参数网格，拿真值表。

`rl/env.py` 是简化被控对象：平地、左右对称、没有摩擦各向异性。它的最优解稳定
落在「高 duty + 低抬脚」，而这两项在真实接触上正是打转和蹭地的来源 —— duty 0.738
时真链路 yaw 漂移 −137°、速度反向 142%。所以在补齐环境之前，参数可不可用只能
在真链路上量。

这条链路和带窗口的仿真器完全一致：`headless.py` 就是去掉 viewer 的
`unitree_mujoco.py`，物理推进和 DDS 收发都在 `UnitreeSdk2Bridge` 里。

    # 终端 1（或后台）
    cd quadruped/sim/unitree_mujoco/simulate_python
    python headless.py --seconds 400

    # 终端 2
    GO2_MOTION=vmc python test/grid_truth.py --vx 0.25 --dur 6

运控用 vmc：`mpc` 模式下 50 Hz 的 MPC 线程会把 200 Hz 主回路的 p95 顶到 10~16 ms，
那是另一件事，会污染步态的测量。

每个点报四个数：yaw 漂移、达成率、roll 峰峰、回路 p95。判据是 yaw 漂移 ——
超过 20° 这套参数就不能用，速度再高也没意义。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from quadruped.robot.body import Go2Robot


# 跑一个参数点：设参数 → 跑 trot → 采样 → 算指标。
def run_point(c, duty: float, swing: float, stride: float,
              vx: float, dur: float) -> dict:
    from quadruped.control.controller import _swing_poly_coeffs

    g = c.gait
    g.duty = float(duty)
    g.swing_height = float(swing)
    g.stride_max = float(stride)
    g.a3, g.a4 = _swing_poly_coeffs(g.duty)

    samples: list[tuple] = []
    stop = threading.Event()

    def drive():
        c.trot(vx=vx, vy=0.0, yaw=0.0, duration=dur)
        stop.set()

    th = threading.Thread(target=drive, daemon=True)
    th.start()
    t0 = time.time()
    while not stop.is_set() and time.time() - t0 < dur + 15:
        if c.pos is not None and c.rpy:
            samples.append((time.time() - t0, tuple(c.pos), tuple(c.rpy)))
        time.sleep(0.02)
    th.join(timeout=5.0)
    stats = c.loop_stats()

    if len(samples) < 10:
        return dict(duty=duty, swing=swing, stride=stride, ok=False,
                    yaw=9.9, ratio=0.0, roll_pp=9.9, p95=stats["p95"])

    p0, p1 = samples[0][1], samples[-1][1]
    t_run = max(samples[-1][0] - samples[0][0], 1e-6)
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    yaw0 = samples[0][2][2]
    # 偏航按最短弧展开，避免 ±π 处跳变被当成「漂移 6 rad」
    ys = [s[2][2] for s in samples]
    yaw = 0.0
    for a, b in zip(ys, ys[1:]):
        yaw += math.atan2(math.sin(b - a), math.cos(b - a))
    rol = [s[2][0] for s in samples]
    pit = [s[2][1] for s in samples]
    return dict(duty=duty, swing=swing, stride=stride, ok=True,
                yaw=yaw, ratio=dist / t_run / max(vx, 1e-6),
                roll_pp=max(rol) - min(rol), pitch_pp=max(pit) - min(pit),
                p95=stats["p95"], speed=dist / t_run)


# 点位之间回站稳：上一点如果打转了，直接接着跑下一点会带着残余角速度和偏航。
def settle(c, seconds: float = 1.2) -> None:
    try:
        c.stop_motion()
    except Exception:
        pass
    time.sleep(seconds)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="真链路步态网格")
    ap.add_argument("--vx", type=float, default=0.25, help="指令速度 m/s")
    ap.add_argument("--dur", type=float, default=6.0, help="每点时长")
    ap.add_argument("--style", default="search")
    ap.add_argument("--kd", type=float, default=3.0, help="与训练时一致")
    ap.add_argument("--iface", default=None)
    ap.add_argument("--duty-list", default="0.50,0.55,0.60,0.66,0.74")
    ap.add_argument("--swing-list", default="0.040,0.060,0.080",
                    help="抬脚高度，米")
    ap.add_argument("--stride", type=float, default=0.072, help="固定步幅上限")
    a = ap.parse_args(argv)

    duties = [float(v) for v in a.duty_list.split(",")]
    swings = [float(v) for v in a.swing_list.split(",")]

    # real_params=None：用真机那套执行器参数（与部署一致），不是理想模型。
    robot = Go2Robot(preset="sim", iface=a.iface,
                     real_params=None, kd=a.kd)
    if not robot.connect():
        print("连不上，先起 headless.py 或 run_sim.sh", flush=True)
        return 1
    c = robot.controller
    c.set_gait_style(a.style)
    print(f"档位 {a.style}，kp={c.kp[0]:.0f} kd={c.kd[0]:.1f}，"
          f"指令 {a.vx:.2f} m/s，每点 {a.dur:.0f} s", flush=True)
    print(f"网格：duty {duties} × swing {swings}，共 {len(duties) * len(swings)} 点\n",
          flush=True)

    rows = []
    for duty in duties:
        for swing in swings:
            r = run_point(c, duty, swing, a.stride, a.vx, a.dur)
            rows.append(r)
            if r["ok"]:
                print(f"  duty {duty:.2f}  swing {swing * 1000:4.0f}mm  "
                      f"yaw {math.degrees(r['yaw']):+7.1f}°  "
                      f"达成 {r['ratio'] * 100:5.0f}%  "
                      f"roll {math.degrees(r['roll_pp']):5.1f}°  "
                      f"p95 {r['p95']:5.2f}ms", flush=True)
            else:
                print(f"  duty {duty:.2f}  swing {swing * 1000:4.0f}mm  "
                      f"采样不足（没跑起来或摔了）", flush=True)
            settle(c)

    # 汇总：按 duty 分组，看「到哪一档开始打转」
    print("\n=== 汇总结论 ===")
    print(f"{'duty':>6} {'swing':>7} {'yaw°':>8} {'达成':>6} {'可用':>5}")
    for r in rows:
        if not r["ok"]:
            continue
        usable = abs(math.degrees(r["yaw"])) < 20 and r["ratio"] > 0.5
        print(f"{r['duty']:6.2f} {r['swing'] * 1000:6.0f}mm "
              f"{math.degrees(r['yaw']):+8.1f} {r['ratio'] * 100:5.0f}% "
              f"{'✓' if usable else '✗':>5}")

    best = [r for r in rows if r["ok"]]
    if best:
        good = [r for r in best
                if abs(math.degrees(r["yaw"])) < 20 and r["ratio"] > 0.5]
        print(f"\n可用点 {len(good)} / {len(best)}")
        for r in sorted(good, key=lambda x: -x["ratio"])[:3]:
            print(f"  最优候选 duty {r['duty']:.2f} swing {r['swing'] * 1000:.0f}mm "
                  f"→ 达成 {r['ratio'] * 100:.0f}%，yaw {math.degrees(r['yaw']):+.0f}°")

    robot.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
