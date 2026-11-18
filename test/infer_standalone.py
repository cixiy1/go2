"""独立跑推理：只挂在 rt/lowcmd / rt/lowstate 上，不经过任何高层控制器。

    # 仿真（先起 unitree_mujoco.py）
    .venv/bin/python test/infer_standalone.py --preset sim --seconds 30 --vx 0.3 --vx-at 12

    # 真机（会先把 mcf 从当前模式释放）
    .venv/bin/python test/infer_standalone.py --preset real --seconds 30

和 `policy_demo.py` 的区别只有一处，但很关键：那边是先连 `Backend` 再借
`Go2Controller._publish` 写指令，起站、状态、mcf 交接都由它代劳；这边自己就是
`rt/lowcmd` 的一个节点，跟高层运控平级，中间没有别人的 200 Hz 回路和 `_mode` 状态机。

## 起站

默认假定机器人已经站着（仿真里先用 `policy_demo.py --motion vmc` 站一次）。`--rise`
试着自己起站，但那条路现在会失败：从趴着的折叠姿态直接插值到 `DEFAULT_Q`，四条腿同时
顶地、力矩打满，实测把机身顶翻（倾角 173°）。要修得改成足端轨迹 + 限力矩，不是插值。

## 先看住前一秒

接管的第一帧力矩不该大。`DEFAULT_Q` 与当前姿态差多少不可控，所以 `rise()` 先插值、
`MotionService` 再叠 0.6 s 的起手渐变（`RAMP_S`）。如果一开始就超过 10 N·m，
先停下查姿态，别硬跑。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.motion import DirectLink, MotionService, rise                # noqa: E402
from quadruped.policy.walker import tilt_rad                              # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="独立推理节点：直连 rt/lowcmd")
    ap.add_argument("--domain", type=int, default=1)
    ap.add_argument("--iface", default="lo0")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vx-at", type=float, default=0.0,
                    help="第几秒开始给 --vx。0 = 一接管就走")
    ap.add_argument("--torque", default="net", choices=("pd", "net"))
    ap.add_argument("--limit", type=float, default=10.0)
    ap.add_argument("--rise", action="store_true",
                    help="先自己起站（实验，见 quadruped/motion/rise.py：从趴着插值"
                         "到 DEFAULT_Q 会把机身顶飞，默认不用）")
    ap.add_argument("--no-mcf", action="store_true",
                    help="不碰 mcf。已经在别的进程里释放过时用")
    a = ap.parse_args()

    link = DirectLink(domain=a.domain, iface=a.iface, release_mcf=not a.no_mcf)
    if not link.connect():
        print(f"3 秒内没收到 rt/lowstate（domain {a.domain} / iface {a.iface}）。"
              "仿真没起，或者网卡名不对")
        return 1

    print(f"已连上：q = {[round(float(v), 2) for v in link.q[:3]]}...  "
          f"rpy = {[round(float(v), 2) for v in link.rpy]}")
    # 直连这层没有运动学模型，高度不可知（`DirectLink.height` 恒为 None）。
    h = link.height
    print(f"倾角 {math.degrees(tilt_rad(link.rpy)):.1f}°  "
          f"高度 {f'{h:.3f} m' if h is not None else '不可知'}")

    if a.rise:
        st = rise(link)
        if st["q_err"] > 0.35:
            print(f"起站没到位（残余 {st['q_err']} rad），先别接管")
            link.close()
            return 1

    svc = MotionService(link, torque=a.torque, action_limit=a.limit, verbose=True)
    try:
        svc.start()
        t0 = time.monotonic()
        walked = False
        while time.monotonic() - t0 < a.seconds:
            el = time.monotonic() - t0
            if a.vx and not walked and el >= a.vx_at:
                svc.set_velocity(vx=a.vx)
                walked = True
                print(f"  给速度 vx={a.vx}")
            if svc.fell:
                print("摔了，停止")
                break
            s = svc.status()
            hh = s.get("height")
            print(f"  {el:4.1f}s  倾角 "
                  f"{math.degrees(tilt_rad(s['rpy'])):5.1f}°  "
                  f"高度 {f'{hh:.3f}' if hh is not None else '—'}  "
                  f"{s['rate_hz']:4.0f} Hz  力矩 {s['torque']}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        svc.stop()
        link.close()
        print("已收工")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
