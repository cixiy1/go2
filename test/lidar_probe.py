"""探一下真机的雷达话题有没有数据，以及数据长什么样。

    用法（en5 插好、有 192.168.123.x 地址之后）：
        .venv/bin/python test/lidar_probe.py            # 默认 en5
        .venv/bin/python test/lidar_probe.py en0        # 换网卡

只订阅、不启动任何控制，也不需要运控在跑 —— 雷达是机器人自己发的。

看三件事：
  1. 三个话题有没有帧，多少 Hz
  2. 点云的字段布局（point_step / 每个点几字节）、点数
  3. 高度图的分辨率、覆盖范围、z 的范围

这三样决定 `percept/lidar.py` 里扇区怎么切、外参怎么定。
"""

from __future__ import annotations

import sys
import time


def main() -> int:
    iface = sys.argv[1] if len(sys.argv) > 1 else "en5"

    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelSubscriber,
    )

    ChannelFactoryInitialize(0, iface)
    print(f"已起 DDS（domain 0，iface {iface}）")

    seen = {"cloud": 0, "height": 0, "state": None}

    # 点云：Go2 的 L1 发的是 sensor_msgs/PointCloud2，点按 point_step 步进。
    try:
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

        def on_cloud(msg):
            if seen["cloud"] == 0:
                step = int(msg.point_step)
                n = int(msg.width) * int(msg.height)
                fields = [(f.name, int(f.offset), int(f.datatype)) for f in msg.fields]
                # `PointCloud2_` 是 unitree 自己生成的 IDL，字段比 ROS 的少 ——
                # 没有 `frame_id`，直接取会 AttributeError 把回调线程打死
                # （2026-11-17 真机第一次跑就撞上，结果「0 帧」分不清是雷达
                # 没发还是我们自己炸了）。用 getattr 兜底。
                fid = getattr(msg, "frame_id", "<无此字段>")
                print(f"\n点云首帧：{msg.width}x{msg.height} 共 {n} 点，"
                      f"每点 {step} 字节，frame_id={fid}")
                print(f"  字段：{fields}")
                print(f"  data 字节数：{len(msg.data)}")
            seen["cloud"] += 1

        ChannelSubscriber("rt/utlidar/cloud", PointCloud2_).Init(on_cloud, 10)
    except Exception as err:                                # noqa: BLE001
        print(f"点云话题订阅失败：{type(err).__name__}: {err}")

    # 高度图：一张以机身为中心的 2D 网格，值是该格的离地高度。
    # 仿真里那 9 条向下射线的替代品，直接给台阶和坑。
    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

        def on_height(msg):
            if seen["height"] == 0:
                res = float(msg.resolution)
                w, h = int(msg.width), int(msg.height)
                data = list(msg.data[:w * h])
                lo = min(data) if data else 0.0
                hi = max(data) if data else 0.0
                print(f"\n高度图首帧：{w}x{h}，分辨率 {res:.3f} m/格，"
                      f"覆盖 {w * res:.1f} x {h * res:.1f} m，"
                      f"原点 ({msg.origin[0]:+.2f}, {msg.origin[1]:+.2f})")
                print(f"  高度范围 {lo:+.3f} .. {hi:+.3f} m，"
                      f"网格数 {len(msg.data)}")
            seen["height"] += 1

        ChannelSubscriber("rt/utlidar/height_map", HeightMap_).Init(on_height, 10)
    except Exception as err:                                # noqa: BLE001
        print(f"高度图话题订阅失败：{type(err).__name__}: {err}")

    # 雷达状态：转速、健康度。没有也不影响。
    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_

        def on_state(msg):
            if seen["state"] is None:
                seen["state"] = f"转速 {getattr(msg, 'rotate_speed', 0):.1f} " \
                                f"健康 {getattr(msg, 'health_error_code', 0)}"
            seen["state"] = seen["state"]

        ChannelSubscriber("rt/utlidar/lidar_state", LidarState_).Init(on_state, 10)
    except Exception as err:                                # noqa: BLE001
        print(f"雷达状态订阅失败：{type(err).__name__}: {err}")

    print("\n收 6 秒，看频率……")
    time.sleep(6.0)
    print(f"\n点云 {seen['cloud'] / 6.0:.1f} Hz（{seen['cloud']} 帧）")
    print(f"高度图 {seen['height'] / 6.0:.1f} Hz（{seen['height']} 帧）")
    print(f"状态 {seen['state'] or '没收到'}")

    if seen["cloud"] == 0 and seen["height"] == 0:
        print("\n两个都是 0：这台机器的雷达没开机，或者这个网卡收不到广播域。")
        print("先确认 ifconfig 有 192.168.123.x，再看运控板 192.168.123.161 通不通。")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
