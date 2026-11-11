"""Move 的安全版本：全交互，不依赖命令行参数。

    .venv/bin/python test/move_demo.py
"""

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


def ask(text: str, default: str = "") -> str:
    v = input(text).strip()
    return v if v else default


def main():
    # 1. 网口：回车则让 CycloneDDS 自动选（多网卡机器上可能选错）
    iface = ask("网口名（如 en5，回车=自动选择）: ") or None
    print(f"  -> 使用网口: {iface or '自动'}")

    ChannelFactoryInitialize(0, iface)

    client = SportClient()          # 构造函数里建好两条 RPC 通道，固定等 0.5s
    client.SetTimeout(3.0)          # 失败时最多等 3 秒，官方示例的 10 秒太磨人
    client.Init()                   # 注册 39 个 apiId，不发包

    # 2. 连通性探针：这一步失败就不要往下走
    code = client.BalanceStand()
    print(f"BalanceStand -> {code}")
    if code != 0:
        print("  对端没响应。先 ping 192.168.123.161，通了再回来。")
        return

    # 3. 参数与二次确认
    vx = float(ask("前进速度 m/s（回车=0.2）: ", "0.2"))
    dur = float(ask("持续秒数（回车=1.5）: ", "1.5"))

    print(f"\n即将执行 Move({vx}, 0, 0)，持续 {dur}s，约 {vx * dur * 100:.0f}cm")
    print("确认狗周围至少 1 米空旷。")
    if input("输入 go 执行，其他任意键取消: ").strip() != "go":
        print("已取消，未发送运动指令。")
        return

    # 4. 执行，无论中途怎样都兜一次 StopMove
    try:
        code = client.Move(vx, 0, 0)
        print(f"Move({vx}, 0, 0) -> {code}")
        if code != 0:
            print("  指令没发出去（3102 = 没有订阅者）")
            return
        time.sleep(dur)
    finally:
        code = client.StopMove()
        print(f"StopMove -> {code}")


if __name__ == "__main__":
    main()
