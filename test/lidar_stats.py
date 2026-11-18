"""真机点云分布：距离、高度各是什么样。

`test/lidar_probe.py` 只报「有没有帧」。这一项回答「帧里的点在哪」——
2026-11-17 真机 VLA 八个方位全读成 0.10 m（正好是 `lidar.R_MIN`），
空旷场地不可能，说明近处堆着一批点：要么是机身自身，要么是单位/原点不对。

    .venv/bin/python test/lidar_stats.py --iface en5
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="en5")
    ap.add_argument("--domain", type=int, default=0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

    ChannelFactoryInitialize(a.domain, a.iface)

    latest = [None]

    def on_cloud(msg):
        # 按 fields 里的 offset 取值，不假定布局。unitree 的 PointCloud2_ 和
        # ROS 的不完全一样（没有 frame_id），不能照着 ROS 的写法解。
        step = int(msg.point_step)
        n = int(msg.width) * int(msg.height)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * step]
        buf = buf.reshape(n, step)
        off = {f.name: int(f.offset) for f in msg.fields}
        get = lambda name: buf[:, off[name]:off[name] + 4].copy().view(np.float32).ravel()
        latest[0] = np.stack([get("x"), get("y"), get("z")], axis=1).astype(np.float64)

    ChannelSubscriber("rt/utlidar/cloud", PointCloud2_).Init(on_cloud, 10)

    print(f"连上 {a.iface}，收点云 …", flush=True)
    t0 = time.time()
    while latest[0] is None and time.time() - t0 < 5.0:
        time.sleep(0.1)
    if latest[0] is None:
        print("5 秒内没有点云", flush=True)
        return 1

    xyz = latest[0]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    horiz = np.hypot(x, y)

    def pct(v, name, unit="m"):
        q = np.percentile(v, [0, 1, 5, 50, 95, 99, 100])
        print(f"  {name:6s} {'  '.join(f'{t:+.3f}' for t in q)}   {unit}",
              flush=True)

    print(f"\n点数 {len(xyz)}", flush=True)
    print("  分位   0%      1%      5%     50%     95%     99%    100%", flush=True)
    pct(x, "x")
    pct(y, "y")
    pct(z, "z")
    pct(horiz, "水平距")

    print("\n水平距离直方图（m）", flush=True)
    edges = [0, 0.1, 0.2, 0.3, 0.5, 0.8, 1.2, 2.0, 3.0, 5.0, 100]
    for lo, hi in zip(edges[:-1], edges[1:]):
        n = int(((horiz >= lo) & (horiz < hi)).sum())
        bar = "#" * int(60 * n / max(1, len(horiz)))
        print(f"  {lo:5.1f}~{hi:5.1f}  {n:6d}  {bar}", flush=True)

    print("\n离原点最近的 12 个点（m）", flush=True)
    order = np.argsort(horiz)[:12]
    for i in order:
        print(f"  ({x[i]:+.3f}, {y[i]:+.3f}, {z[i]:+.3f})  水平 "
              f"{horiz[i]:.3f}", flush=True)

    print("\n高度直方图（m，雷达系）", flush=True)
    zedges = [-2.0, -1.0, -0.6, -0.3, -0.15, 0.0, 0.15, 0.3, 0.6, 1.2, 5.0]
    for lo, hi in zip(zedges[:-1], zedges[1:]):
        n = int(((z >= lo) & (z < hi)).sum())
        bar = "#" * int(60 * n / max(1, len(z)))
        print(f"  {lo:+5.2f}~{hi:+5.2f}  {n:6d}  {bar}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
