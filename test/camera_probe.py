"""探真机前置摄像头：能不能拿到帧、画面长什么样。

videohub 是 RPC 服务（api_id 1001），只认一个订阅者 —— 手机 App 连着的时候
这边 `code != 0`，那不是故障但要能看出来。

    .venv/bin/python test/camera_probe.py --iface en5
    .venv/bin/python test/camera_probe.py --iface en5 --save /tmp/cam.jpg
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
    ap.add_argument("--dur", type=float, default=6.0)
    ap.add_argument("--save", default=None, help="把拿到的一帧存成 JPEG/PNG")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(a.domain, a.iface)
    print(f"已起 DDS（domain {a.domain}，iface {a.iface}）", flush=True)

    from quadruped.percept.camera import CameraStream

    cam = CameraStream(hz=5.0, timeout=1.5, backoff=1.0)
    cam.start()

    t0 = time.time()
    frame = None
    while time.time() - t0 < a.dur:
        f = cam.read()
        if f is not None and frame is None:
            frame = f
            print(f"{time.time() - t0:4.1f}s 拿到首帧 {f.shape} dtype={f.dtype}",
                  flush=True)
        time.sleep(0.05)
    cam.close()

    print(f"\n收 {a.dur:.0f} s：成功 {cam.frames} 帧 "
          f"({cam.frames / a.dur:.1f} Hz)，丢弃 {cam.dropped}，"
          f"最后 code={cam.last_code}", flush=True)
    if cam.last_error:
        print(f"错误：{cam.last_error}", flush=True)
    if frame is None:
        print("\n一帧都没拿到。先看是不是手机 App 连着（videohub 只认一个订阅者），"
              "再看相机服务有没有开。", flush=True)
        return 1

    if a.save:
        import cv2
        # 存之前翻回 BGR：`CameraStream` 交出来的是 RGB（VLM 和缩略图都按 RGB 写的）。
        cv2.imwrite(a.save, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        print(f"已存 {a.save}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
