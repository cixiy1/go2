"""量 `Viewer.pump()` 单帧到底花多少时间 —— 分开算「渲染+编码」和「imshow」。

    .venv/bin/python test/viewer_fps.py            # 只渲染（网页那一路）
    .venv/bin/python test/viewer_fps.py --show     # 带 cv2 窗口（会弹窗）

为什么要分开量：网页那条路 `display=False`，本机那条路 `display=True`。
如果差别很大，说明瓶颈在 `cv2.imshow`/`waitKey`，不在渲染；反之在渲染本身。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from quadruped.percept.viewer import Viewer                          # noqa: E402

N = 60
CADENCE = 1.0 / 20.0        # runner.VIEW_HZ


def eye(k: int) -> np.ndarray:
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[..., 0] = 40 + (k * 3) % 180
    img[..., 1] = 60
    return img


def world(k: int) -> np.ndarray:
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[..., 1] = 90 + (k * 2) % 120
    return img


def run(display: bool) -> None:
    out = Path("/tmp/go2_fps.jpg")
    v = Viewer(dual=True, display=display, stream=str(out))
    spans: list[float] = []
    tick = time.perf_counter()
    start = tick
    for k in range(N):
        t0 = time.perf_counter()
        v.pump(rgb=eye(k), world=world(k), scene="正前两米空地，左侧一堵墙",
               thought="右侧贴墙，往左走", command={"vx": 0.3, "vy": 0.0},
               goal="左前", sectors=[("正前", 1.8)] * 8,
               history=[(1.0, "测试")], foot=["vx +0.30"])
        spans.append((time.perf_counter() - t0) * 1000)
        # 照 runner 的节奏补睡眠，把「渲染本身」和「等下一拍」分开
        tick += CADENCE
        rest = tick - time.perf_counter()
        if rest > 0:
            time.sleep(rest)
    total = time.perf_counter() - start
    v.close()

    spans_sorted = sorted(spans)
    mid = spans_sorted[len(spans_sorted) // 2]
    print(f"display={display}  {N} 帧 / {total:.2f} s = {N / total:.1f} fps")
    print(f"  单帧 ms: 中位 {mid:.1f}  最小 {spans[0]:.1f}  "
          f"最大 {max(spans):.1f}  中位以上均值 "
          f"{sum(s for s in spans if s >= mid) / max(1, sum(1 for s in spans if s >= mid)):.1f}")
    print(f"  导出 {v.streamed} 帧（STREAM_HZ 上限下应为 20 左右）")


if __name__ == "__main__":
    run(display="--show" in sys.argv)
