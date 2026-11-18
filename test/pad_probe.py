"""读手柄的轴和按键，用来定映射表。只打印，不发任何指令。

    # 扩展坞 / 机器人上（Linux，先把手柄配对好）
    SDL_VIDEODRIVER=dummy .venv/bin/python test/pad_probe.py

    # macOS 本地
    .venv/bin/python test/pad_probe.py

北通这类手柄有两种呈现方式：XInput（Xbox 兼容，轴 0/1 是左摇杆、4/5 是扳机）和
DInput（轴序完全不同，扳机常在 2）。**不猜**，插上按一遍，看这一行：

    动左摇杆 → 哪两个轴在跳；按 A/B/X/Y → 哪几个按钮号

有了这张表再去写 `quadruped/control/physical/pad.py` 的映射，否则「往前推变成
右移」这种错误在狗身上试，一次就是一次撞墙。

Linux 上读不到设备时先查权限：

    ls -l /dev/input/js*             # 一般属于 input 组
    sudo usermod -aG input $USER     # 加完要重新登录
    cat /proc/bus/input/devices | grep -A4 -i beitong

蓝牙手柄会断连重连。重连之后设备号可能变（js0 → js1），所以控制层不能把
`js0` 写死，每次断开都要重新扫描。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# 没有显示器的会话里 pygame 起不来；joystick 本身不需要窗口，给个假视频驱动。
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame                                                       # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="手柄轴/按键探针")
    ap.add_argument("--index", type=int, default=0, help="第几个手柄（默认 0）")
    ap.add_argument("--seconds", type=float, default=30.0, help="跑多久")
    ap.add_argument("--all", action="store_true",
                    help="每秒打印全部轴的值，不只是变化")
    a = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    if n == 0:
        print("没找到手柄。检查：蓝牙是否配对（bluetoothctl devices）、"
              "手柄是否开机、/dev/input/js* 是否存在")
        return 2
    print(f"找到 {n} 个：")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  {i}) {js.get_name()}  轴 {js.get_numaxes()}  "
              f"按钮 {js.get_numbuttons()}  帽 {js.get_numhats()}")
    if a.index >= n:
        print(f"--index {a.index} 超了")
        return 2

    js = pygame.joystick.Joystick(a.index)
    js.init()
    print(f"\n读 {a.index}：动一下左摇杆，再逐个按按钮。idle 时轴值不会等于 0 是正常的"
          f"（摇杆校准），死区按实测的 idle 值取。\n")

    last = [None] * js.get_numaxes()
    last_b = [0] * js.get_numbuttons()
    t0 = time.time()
    t_print = t0
    try:
        while time.time() - t0 < a.seconds:
            pygame.event.pump()
            axes = [js.get_axis(i) for i in range(js.get_numaxes())]
            for i, v in enumerate(axes):
                if last[i] is None or abs(v - last[i]) > 0.08:
                    print(f"  轴 {i:2d} = {v:+.2f}"
                          + (f"   （上一帧 {last[i]:+.2f}）" if last[i] is not None else ""))
                    last[i] = v
            for i in range(js.get_numbuttons()):
                b = js.get_button(i)
                if b != last_b[i]:
                    print(f"  按钮 {i:2d} {'按下' if b else '松开'}")
                    last_b[i] = b
            for i in range(js.get_numhats()):
                hat = js.get_hat(i)
                if hat != (0, 0):
                    print(f"  帽 {i} = {hat}")
            if a.all and time.time() - t_print >= 1.0:
                t_print = time.time()
                print("  全轴 " + " ".join(f"{v:+.2f}" for v in axes), flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n停")
    finally:
        pygame.joystick.quit()
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
