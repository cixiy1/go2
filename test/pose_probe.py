"""扫一遍接管后还在发的话题，找里程计的替代来源。

背景：真机被 `lowcmd` 接管后，`rt/sportmodestate` 立刻停发（2026-11-17 实测
298 Hz → 0 Hz），而 `DirectLink.pos` 正是取自它。于是 `PolicyBackend.pos`
一直是 None —— 仿真里永远有桥在发，这个问题只有在真机上才暴露。

`rt/utlidar/cloud` 在接管后照发（15.3 Hz），说明 utlidar 服务还活着。那么它
配套的位姿话题大概率也还在。这一项就是把候选一个个订阅上，看谁有帧。

    .venv/bin/python test/pose_probe.py --iface en5 --dur 3
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="en5")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--dur", type=float, default=3.0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelSubscriber,
    )

    ChannelFactoryInitialize(a.domain, a.iface)
    print(f"已起 DDS（domain {a.domain}，iface {a.iface}）", flush=True)

    seen: dict[str, int] = {}
    first: dict[str, str] = {}

    def watch(topic, typ, describe):
        seen[topic] = 0

        def cb(msg):
            seen[topic] += 1
            if topic not in first:
                try:
                    first[topic] = describe(msg)
                except Exception as err:                    # noqa: BLE001
                    first[topic] = f"<描述失败 {type(err).__name__}>"

        try:
            ChannelSubscriber(topic, typ).Init(cb, 10)
        except Exception as err:                            # noqa: BLE001
            seen[topic] = -1
            first[topic] = f"订阅失败 {type(err).__name__}: {err}"

    # 位姿：SLAM / 里程计
    from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

    watch("rt/utlidar/robot_pose", PoseStamped_,
          lambda m: f"({m.pose.position.x:+.3f}, {m.pose.position.y:+.3f}, "
                    f"{m.pose.position.z:+.3f})")
    watch("rt/utlidar/robot_odom", Odometry_,
          lambda m: f"({m.pose.pose.position.x:+.3f}, "
                    f"{m.pose.pose.position.y:+.3f})")
    watch("rt/sportmodestate", SportModeState_,
          lambda m: f"({m.position[0]:+.3f}, {m.position[1]:+.3f})")
    watch("rt/lf/sportmodestate", SportModeState_,
          lambda m: f"({m.position[0]:+.3f}, {m.position[1]:+.3f})")

    # 按秒分段打。整段平均看不出接管那一刻哪些停了 —— 而那正是要的答案。
    t0 = time.time()
    prev = dict(seen)
    while time.time() - t0 < a.dur:
        time.sleep(1.0)
        dt = time.time() - t0
        cells = []
        for topic, n in seen.items():
            if n < 0:
                continue
            cells.append(f"{topic.replace('rt/', ''):24s} {n - prev[topic]:5d}")
            prev[topic] = n
        print(f"{dt:5.1f}s  " + "  ".join(cells), flush=True)

    print(f"\n收 {a.dur:.0f} s：", flush=True)
    for topic, n in seen.items():
        if n < 0:
            print(f"  {topic:26s} {first.get(topic, '')}", flush=True)
            continue
        hz = n / a.dur
        mark = "  <-- 可用" if hz >= 1.0 else ""
        print(f"  {topic:26s} {hz:6.1f} Hz   {first.get(topic, '<无帧>')}{mark}",
              flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
