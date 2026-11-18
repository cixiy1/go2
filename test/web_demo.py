"""网页的假 VLA：只为了看界面，不碰机器人。

    # 一边起网页一边挂这个
    python -m quadruped.web.app --port 8080 --token demo \
        --cmd ".venv/bin/python test/web_demo.py --stream logs/frame.jpg --interactive"

它做三件和真 VLA 一样的事，数值全是编的：

    1. 每 0.2 s 渲染一帧给网页（`Viewer` 导出，5 Hz 限频在 Viewer 里）
    2. 每 2 s 打一行状态，格式与 `runner.py` 一致 —— 网页的指标格靠它
    3. 读 stdin，收到话就回一句 —— 网页的「它说」靠日志里那行

存在的理由：改页面样式的时候不想起 MuJoCo、不想连真机、更不想让它开口调模型。
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from quadruped.percept.viewer import Viewer                     # noqa: E402

# 假场景里「墙」的位置：画面中一条竖带，随进度左右移动，看起来像在走廊里走。
WALL_W = 60

REPLIES = (
    "正前两米是空的，可以往前走",
    "左边贴着墙，我往中间靠一点",
    "前面有台阶，慢点",
    "这一段没走过，我直着走",
)


def fake_eye(t: float) -> np.ndarray:
    """一张会动的假机载画面：地面渐变 + 一堵移动的墙 + 地面纹理。"""
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    # 天空 → 地面的渐变
    ramp = np.linspace(105, 45, 240, dtype=np.uint8)[:, None]
    img[..., 0] = ramp
    img[..., 1] = (ramp.astype(np.int16) + 6).clip(0, 255).astype(np.uint8)
    img[..., 2] = (ramp.astype(np.int16) - 10).clip(0, 255).astype(np.uint8)
    # 地平线
    img[110:114, :, :] = (70, 80, 90)
    # 一堵会左右移动的墙
    x = int(160 + 110 * math.sin(t * 0.35))
    img[80:, x:x + WALL_W, :] = (150, 140, 128)
    # 地面格子，给出「在往前走」的感觉
    for i in range(6):
        y = 120 + int((i * 26 + (t * 40) % 26))
        if y < 239:
            img[y:y + 2, 20:300, :] = (96, 100, 104)
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description="网页用的假 VLA")
    ap.add_argument("--stream", default="logs/frame.jpg")
    ap.add_argument("--interactive", action="store_true")
    a = ap.parse_args()

    out = Path(a.stream)
    out.parent.mkdir(parents=True, exist_ok=True)

    viewer = Viewer(dual=False, display=False, stream=str(out))
    stop = threading.Event()

    def bye(*_):
        stop.set()

    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)

    print("接管策略 …（假数据，不连机器人）", flush=True)
    print(f"画面导出到 {out}", flush=True)

    said: list[str] = []

    def reader() -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            said.append(line)
            reply = REPLIES[len(said) % len(REPLIES)]
            print(f"[L] 收到：{line}", flush=True)
            print(f"[L] 触发：新指令（距上次 1.0 s）", flush=True)
            print(f"    想法：听到「{line}」，{reply}", flush=True)
            print(f"[L] 回话：{reply}", flush=True)
        stop.set()

    if a.interactive:
        threading.Thread(target=reader, daemon=True).start()

    t0 = time.monotonic()
    prints = 0
    next_at = t0
    while not stop.is_set():
        t = time.monotonic() - t0
        thought = (f"听到「{said[-1]}」，{REPLIES[len(said) % len(REPLIES)]}"
                   if said else "正前两米是空地，先往前走")
        viewer.pump(
            rgb=fake_eye(t),
            scene="左：墙，不能，很近；中：空地，能，远；右：空地，能，中",
            scene_age=0.4,
            thought=thought, why="接着想",
            command={"vx": 0.4, "freq": 3.0},
            goal="往东走，找出口",
            sectors=[("正前", 2.4), ("左前", 0.8), ("左侧", 0.5), ("左后", 1.2),
                     ("正后", 2.0), ("右后", 1.8), ("右侧", 1.4), ("右前", 2.2)],
            history=[(t - 6.0, "前面开阔，加速"), (t - 3.0, "左边有墙，往中间靠")],
            foot=["vx +0.40", "策略 50 Hz", "记忆 128 格 / 21.4 m",
                  "闸 0 次", "C 实测 50 Hz"],
            alert=False,
        )
        if t - prints >= 2.0:
            prints = t
            print(f"  {t:4.1f}s  vx=+0.40  策略 50 Hz  前方 2.40  "
                  f"推理 {int(t) + 3} 次 / 1.03s  位置 ({t * 0.4:+.1f}, +0.3) m  "
                  f"朝向 +45°", flush=True)
        # 主循环 20 Hz，和 `runner.py` 的 `VIEW_HZ` 一致。按绝对时刻推进而不是
        # `sleep(0.05)`：后者会把每帧的渲染耗时（约 7 ms）累加进去，20 Hz 变成 15 Hz。
        next_at += 0.05
        slack = next_at - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            next_at = time.monotonic()          # 落后太多就不追帧

    viewer.close()
    print("[L] 蹲下…", flush=True)
    print("已交还 ai", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
