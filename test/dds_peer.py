"""单播 Peers 探测：指定域号 + 指定对端 IP，绕开被热点丢弃的组播。

用法：
    .venv/bin/python test/dds_peer.py <domain> <ip>[,<ip>...] [seconds]

例：
    .venv/bin/python test/dds_peer.py 0 10.216.47.253
    .venv/bin/python test/dds_peer.py 0 10.216.47.253 10

为什么不用 CYCLONEDDS_URI：
    unitree_sdk2py 的 ChannelFactory.Init() 把自带的 XML 直接传给
    dds_create_domain()，优先级高于环境变量，所以 CYCLONEDDS_URI 会被
    完全忽略（日志里 AllowMulticast 仍是 default 就是证据）。
    唯一有效的注入方式是替换 channel 模块里的模板全局变量。
"""

import sys
import time

IFACE = "en0"


def build_config(peers: list[str], iface: str) -> str:
    peer_xml = "".join(f'            <Peer address="{p}"/>\n' for p in peers)
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="{iface}" priority="default" multicast="default"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
{peer_xml}      </Peers>
    </Discovery>
    <Tracing>
      <Verbosity>config</Verbosity>
      <OutputFile>/tmp/cdds_peer.LOG</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>'''


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2

    domain = int(sys.argv[1])
    peers = [p.strip() for p in sys.argv[2].split(",") if p.strip()]
    wait = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0

    from unitree_sdk2py.core import channel as ch
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

    # 必须在 ChannelFactoryInitialize 之前替换模板
    ch.ChannelConfigHasInterface = build_config(peers, IFACE)
    ch.ChannelFactoryInitialize(domain, IFACE)

    stats = {"n": 0, "last": None}

    def on_state(msg):
        stats["n"] += 1
        stats["last"] = msg

    sub = ch.ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state, 10)
    time.sleep(wait)

    tag = f"域 {domain:2d}  对端 {', '.join(peers)}"
    if stats["n"] == 0:
        print(f"[x] {tag}  ->  {wait:.0f}s 内 0 帧")
        return 1

    msg = stats["last"]
    print(f"[OK] {tag}  ->  {stats['n']} 帧 / {wait:.0f}s（约 {stats['n'] / wait:.0f} Hz）")
    print(f"       电压 {msg.power_v:.2f} V   姿态 {[round(v, 3) for v in msg.imu_state.rpy]}")
    print(f"       q[:6] = {[round(m.q, 3) for m in msg.motor_state[:6]]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
