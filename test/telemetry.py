"""只读连接验证：订阅 rt/lowstate，打印遥测。不发任何控制指令，机器狗不会动。

用法：
    .venv/bin/python test/telemetry.py           # 默认网卡 en5，等 8 秒
    .venv/bin/python test/telemetry.py en0 5     # 指定网卡与等待秒数

网线直连时本机应为 192.168.123.222/24，机器人固定 192.168.123.161。
先用 ping 确认链路，再跑这个确认 DDS 通。
"""

import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


def main() -> int:
    iface = sys.argv[1] if len(sys.argv) > 1 else "en5"
    wait = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0

    ChannelFactoryInitialize(0, iface)
    print(f"DDS 初始化完成，网卡 {iface}，等 {wait:.0f} 秒遥测…", flush=True)

    stats = {"n": 0, "last": None}

    def on_state(msg):
        stats["n"] += 1
        stats["last"] = msg

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state, 10)
    time.sleep(wait)

    if stats["n"] == 0:
        print("失败：没收到 rt/lowstate，DDS 未通")
        return 1

    msg = stats["last"]
    print(f"成功：{stats['n']} 帧 / {wait:.0f}s ≈ {stats['n'] / wait:.0f} Hz")
    print(f"  电池 {msg.power_v:.2f} V  {msg.power_a:.2f} A")
    print(f"  姿态 rpy {[round(v, 3) for v in msg.imu_state.rpy]}")
    print(f"  温度 ntc1={msg.temperature_ntc1} ntc2={msg.temperature_ntc2}")
    print(f"  前 6 关节 q {[round(m.q, 3) for m in msg.motor_state[:6]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
