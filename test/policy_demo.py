"""在带窗口的仿真里跑步态策略，给人看的。

    # 另一个终端先起窗口
    cd quadruped/sim/unitree_mujoco/simulate_python && python unitree_mujoco.py

    # 再跑这个
    .venv/bin/python test/policy_demo.py --stand 4 --walk 10

两段：先站住，再给 `vx` 让它走。两段的动作限幅分开设 —— 站立段收到 0.5 能压到
倾角 4°，走的那段放开到 1.0 才可能起步（代价是晃）。见 TODO.md 第七节。

这里走的是完整链路：`connect()` → `Backend` → `MotionService` → `joint_target`
→ `lowcmd` → 仿真桥。想绕开这些单独看策略，用 `test/policy_sim.py`。

## 一次跑完想再跑，先重启仿真

收尾的 `close()` 会 park：蹲下、松阻尼。仿真里那就是机器人趴在地上滚一下，
下一次 `connect()` 接手时倾角可能已经 160°。所以同一个仿真进程只演示一次，
要重来就 `pkill -f unitree_mujoco.py` 再起。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.control.controller import NOMINAL_HEIGHT                # noqa: E402
from quadruped.control.virtual.robot import choose_client, connect, resolve_endpoint  # noqa: E402
from quadruped.motion import BackendLink, MotionService                    # noqa: E402
from quadruped.policy.walker import (                                 # noqa: E402
    DEFAULT_Q,
    KD,
    KP,
    POLICY_TO_SDK,
    tilt_rad,
)


# 先让机器人站住，再把关节切到策略的默认站姿。
#
# 高度不由 `stand()` 决定，也不由命令里那条 `height` 决定（训练时它恒为 −0.05，
# 策略没学过别的值）。策略输出的是相对 `DEFAULT_Q` 的偏移，机身高度是这组角加
# 动作的几何后果，约 0.28 m。controller 站到的是另一套姿态（0.27 m 且后腿蜷），
# 直接交接等于让策略从分布外起步。
#
# 切换用训练增益硬切，不做高 kp 渐变：偏差只有 0.16 / 0.35 rad，kp=80 时力矩到
# 28 N·m，接近上限，左右一旦不对称就翻（实测 1 s 内 154°）。
def rise(b, kp: float = KP, seconds: float = 1.5, timeout: float = 8.0) -> bool:
    b.stand()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # 高度是运动学值，躺着也报 0.26 m，所以要和倾角一起看。
        if (float(b.height) >= NOMINAL_HEIGHT * 0.85
                and math.degrees(tilt_rad(b.rpy)) < 20):
            break
        time.sleep(0.1)
    else:
        return False

    target = DEFAULT_Q[POLICY_TO_SDK]
    for _ in range(int(seconds / 0.02)):
        b.joint_target(target, kp=kp, kd=KD)
        time.sleep(0.02)
    return True


# 保持控制但不再改命令，给人看够。`close()` 的 park 会让机器人趴下，
# 所以演示完到收工之间的这段时间要单独留出来。
def linger(svc, seconds: float) -> None:
    if seconds <= 0 or svc.fell:
        return
    print(f"\n[保持 {seconds:.0f}s] 控制不停，收工后才会趴下")
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        if svc.fell:
            print("  摔了")
            return
        s = svc.status()
        print(f"  {time.monotonic() - t0:4.1f}s  倾角 "
              f"{math.degrees(tilt_rad(s['rpy'])):5.1f}°  高度 {s['height']:.3f}")
        time.sleep(1.0)


def main() -> int:
    ap = argparse.ArgumentParser(description="带窗口跑策略")
    ap.add_argument("--preset", default="sim", help="端点预设")
    ap.add_argument("--backend", default="ours", help="ours / official")
    ap.add_argument("--stand", type=float, default=4.0, help="站住多少秒")
    ap.add_argument("--walk", type=float, default=10.0, help="走多少秒")
    ap.add_argument("--stand-limit", type=float, default=1.0, help="站立段的动作限幅")
    ap.add_argument("--limit", type=float, default=10.0,
                    help="行走段的动作限幅。10 = 训练的 clip_actions")
    ap.add_argument("--vx", type=float, default=0.2,
                    help="前进速度 m/s。策略跟不上命令，0.2 大约走出 0.4")
    ap.add_argument("--gait", default="trot",
                    help="步态：pronk / trot / bound / pace / walk。定义与实测速度"
                         "见 quadruped/policy/gaits.py")
    ap.add_argument("--sequence", default="",
                    help="逗号分隔的步态序列，按 --segment 秒轮换，例如 "
                         "trot,bound,pace,pronk。给了它就不用 --gait")
    ap.add_argument("--segment", type=float, default=4.0,
                    help="序列里每个步态演多少秒")
    ap.add_argument("--freq", type=float, default=3.0, help="步频 Hz")
    ap.add_argument("--footswing", type=float, default=0.19,
                    help="抬脚高度 m，0.19 = 训练时的唯一取值")
    ap.add_argument("--linger", type=float, default=0.0,
                    help="演示完再保持控制多少秒。给人看的时候设大一点，"
                         "否则收工的 park 会让机器人当场趴下")
    ap.add_argument("--torque", default="pd", choices=("pd", "net"),
                    help="力矩由谁算：pd = 驱动器按 kp20/kd0.5（默认），"
                         "net = 训练时的电机模型，只发 tau、kp=kd=0")
    ap.add_argument("--kp", type=float, default=KP,
                    help="关节刚度，默认 20 = 真机 LowCmd 的值")
    ap.add_argument("--motion", default="vmc", choices=("vmc", "mpc"),
                    help="自研控制端的力路：vmc = 关掉 MPC+WBC（默认，策略接管时"
                         "不需要那层），mpc = 开着，用于对照")
    a = ap.parse_args()

    ep = resolve_endpoint(a.preset)
    kind = choose_client(ep, a.backend)
    # `motion` 只有自研控制端认；官方和推理控制器都没有那层。
    gains = {"motion": a.motion} if kind == "ours" else {}
    b = connect(ep, kind, **gains)
    # 策略只通过 link 看外界：读姿态、写关节、进阻尼，看不到 move() 那些入口。
    link = BackendLink(b)
    svc = None
    try:
        if not rise(b, kp=a.kp):
            print(f"起站失败：高度只有 {float(b.height):.3f} m，倾角 "
                  f"{math.degrees(tilt_rad(b.rpy)):.0f}°")
            return 1
        print(f"起站完成：高度 {float(b.height):.3f} m，倾角 "
              f"{math.degrees(tilt_rad(b.rpy)):.0f}°")

        # -- 第一段：站住 --------------------------------------------------
        print(f"\n[站立 {a.stand:.0f}s] 限幅 ±{a.stand_limit}  力矩 {a.torque}")
        svc = MotionService(link, action_limit=a.stand_limit, verbose=True,
                            torque=a.torque, kp=a.kp, kd=KD)
        svc.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.stand:
            if svc.fell:
                print("  摔了，停止演示")
                return 0
            s = svc.status()
            print(f"  {time.monotonic() - t0:4.1f}s  倾角 {math.degrees(tilt_rad(s['rpy'])):5.1f}°"
                  f"  高度 {s['height']:.3f}  {s['rate_hz']:.0f} Hz")
            time.sleep(1.0)

        # -- 第二段：走 ----------------------------------------------------
        if a.walk <= 0:
            linger(svc, a.linger)
            return 0
        print(f"\n[行走 {a.walk:.0f}s] vx={a.vx} 限幅 ±{a.limit}  力矩 {a.torque}")
        svc.stop()
        svc = MotionService(link, action_limit=a.limit, verbose=True,
                            torque=a.torque, kp=a.kp, kd=KD)
        seq = [g.strip() for g in a.sequence.split(",") if g.strip()] or [a.gait]
        svc.set_gait(seq[0])
        svc.set_command(vx=a.vx, freq=a.freq, footswing=a.footswing)
        x0 = float(b.pos[0])
        svc.start()
        t0 = time.monotonic()
        shown = 0          # 序列第一个已经由上面的 set_gait 设好
        while time.monotonic() - t0 < a.walk:
            # 每秒续期一次。`MotionService` 有 3 s 的命令超时（外部进程掉线时把速度
            # 归零），只发一次的话 3 s 后 vx 就变 0，剩下的时间都在原地踏步 ——
            # 表现是速度不随命令变、永远只有 0.07 m/s。
            #
            # 只续期速度与步频：相位由 `set_gait()` 管，这里再发一遍 phase 会把
            # 当前步态覆盖掉。
            svc.set_command(vx=a.vx, freq=a.freq, footswing=a.footswing)
            # 序列轮换。按下标切而不是按「到点就 next」，时间偶尔超一帧也不会连跳两个。
            k = int((time.monotonic() - t0) // max(1e-3, a.segment)) % len(seq)
            if k != shown:
                shown = k
                svc.set_gait(seq[k])
            if svc.fell:
                print("  摔了，停止演示")
                break
            s = svc.status()
            print(f"  {time.monotonic() - t0:4.1f}s  {s['gait']:<5} "
                  f"倾角 {math.degrees(tilt_rad(s['rpy'])):5.1f}°"
                  f"  高度 {s['height']:.3f}  x {float(b.pos[0]) - x0:+.3f} m"
                  f"  {s['rate_hz']:.0f} Hz")
            time.sleep(1.0)
        print(f"\n位移 {float(b.pos[0]) - x0:+.3f} m（命令 {a.vx} m/s × {a.walk:.0f}s）")

        linger(svc, a.linger)
        return 0
    finally:
        if svc is not None:
            svc.stop()
        b.close()
        print("已收工")


if __name__ == "__main__":
    raise SystemExit(main())
