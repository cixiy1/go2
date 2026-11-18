"""画面导出给网页：文件写得出来、限频生效、读到的永远是完整 JPEG。

    .venv/bin/python test/viewer_stream.py

`--stream` 是网页那一路的入口：真机上没有图形会话（`--no-watch`），人看不到窗口，
只能看网页。所以这里要证明三件事：

    1. `display=False` 时照样渲染并导出 —— 否则真机上网页永远是空白
    2. 5 Hz 限频真的在限 —— 每一帧都编码的话，扩展坞上要多花半个核
    3. 导出是原子替换 —— 网页随时可能在读，不能让它拿到半张图
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2                                                          # noqa: E402

from quadruped.percept.viewer import STREAM_HZ, Viewer              # noqa: E402

ok = 0
bad: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    global ok
    if cond:
        ok += 1
        print(f"  ok   {name}")
    else:
        bad.append(name)
        print(f"  FAIL {name}  {detail}")


def eye(color: int) -> np.ndarray:
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[..., 0] = color
    return img


def main() -> int:
    out = Path("/tmp/go2_frame_test.jpg")
    out.unlink(missing_ok=True)

    print("\n导出")
    v = Viewer(dual=False, display=False, stream=str(out))
    check("display=False 也能建", v.display is False)
    v.pump(rgb=eye(120), scene="正前两米空地", thought="往前走",
           sectors=[("正前", 2.0)] * 8, foot=["vx +0.30"])
    check("第一帧就写出文件", out.exists(), str(out))
    size = out.stat().st_size if out.exists() else 0
    check("文件不是空的", size > 5000, f"{size} 字节")
    img = cv2.imread(str(out))
    check("读回来是完整 JPEG", img is not None and img.shape[0] > 100,
          "None" if img is None else str(img.shape))
    check("没有留下 .tmp", not Path(str(out) + ".tmp").exists())

    print("\n限频")
    # 上限的意义是「别让编码风暴拖垮主循环」，不是「帧率」。所以断言写成
    # 「导出次数 ≈ 时长 × 上限」，而不是「连刷 N 帧只写一次」—— 后者在
    # 上限高于渲染速度时本来就不该成立（那是设计意图）。
    start, before = time.perf_counter(), v.streamed
    n = 120
    for i in range(n):
        v.pump(rgb=eye(60 + i % 180), thought=f"第 {i} 帧")
    span = time.perf_counter() - start
    made = v.streamed - before
    cap = span * STREAM_HZ + 2
    check(f"{n} 帧用了 {span:.2f} s，导出 {made} 帧（上限 {cap:.0f}）",
          1 <= made <= cap, f"made={made} cap={cap:.1f}")
    check("确实在丢帧，不是每帧都导", made < n, f"made={made} n={n}")
    check("最新一张仍然是完整 JPEG",
          cv2.imread(str(out)) is not None and out.stat().st_size > 5000)

    print("\n没有 stream 时")
    quiet = Viewer(dual=False, display=False)
    quiet.pump(rgb=eye(10))
    check("不导出也不抛", True)

    v.close()
    quiet.close()
    out.unlink(missing_ok=True)
    print(f"\n{ok} 条通过" + (f"，{len(bad)} 条失败：{'、'.join(bad)}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
