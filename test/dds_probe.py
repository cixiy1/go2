"""单播 DDS 探测：绕过组播，直接向指定 IP 发发现包，看对方是不是 DDS 节点。

用法：
    .venv/bin/python test/dds_probe.py 10.216.47.253          # 指定单个 IP
    .venv/bin/python test/dds_probe.py sweep                  # 扫整个 /24
    .venv/bin/python test/dds_probe.py sweep 128 255           # 只扫某段区间

原理：SDK 内置配置模板只设网卡、靠组播发现。校园网/企业 AP 常把组播丢掉，
于是「能 ping 通但 DDS 全哑」。这里关掉 AllowMulticast、改用 Peers 单播，
就能绕开组播限制，逐个 IP 试探谁在跑 DDS。
"""

import ipaddress
import subprocess
import sys
import time

from unitree_sdk2py.core import channel as ch
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

IFACE = "en0"


def self_ip() -> str:
    return subprocess.check_output(["ipconfig", "getifaddr", IFACE], text=True).strip()


def build_config(peers: list[str]) -> str:
    peer_xml = "".join(f'        <Peer address="{p}"/>\n' for p in peers)
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="{IFACE}" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
{peer_xml}      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>'''


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    me = self_ip()
    net = ipaddress.ip_network(f"{me}/24", strict=False)

    if sys.argv[1] == "sweep":
        lo = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        hi = int(sys.argv[3]) if len(sys.argv) > 3 else 254
        peers = [str(net.network_address + i) for i in range(lo, hi + 1) if str(net.network_address + i) != me]
        wait = 12
    else:
        peers = [sys.argv[1]]
        wait = 6

    print(f"本机 {me}，探测 {len(peers)} 个地址，等 {wait} 秒...")

    ch.ChannelConfigHasInterface = build_config(peers)
    ch.ChannelFactoryInitialize(0, IFACE)

    stats = {"count": 0, "last": None}

    def on_state(msg):
        stats["count"] += 1
        stats["last"] = msg

    sub = ch.ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state, 10)
    time.sleep(wait)

    if stats["count"] == 0:
        print("没收到 rt/lowstate — 这段地址里没有 DDS 节点")
        return 1

    msg = stats["last"]
    print(f"收到 {stats['count']} 帧 / {wait} 秒（约 {stats['count'] / wait:.0f} Hz）")
    print(f"  电压 {msg.power_v:.2f} V   姿态 {[round(v, 3) for v in msg.imu_state.rpy]}")
    print(f"  q[:6] = {[round(m.q, 3) for m in msg.motor_state[:6]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
