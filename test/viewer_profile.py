"""`Viewer.pump()` 的 43 ms 花在哪 —— cProfile 一次，按累计时间排序。

    .venv/bin/python test/viewer_profile.py

用真实长度的文本（想法 30 字、foot 7 项、history 3 条）跑，模拟迷宫那次的版式：
`test/viewer_fps.py` 里文字都很短，测出 12 ms；真实跑是 43 ms，差在排版。
"""

from __future__ import annotations

import cProfile
import io
import pstats
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from quadruped.percept.viewer import Viewer                          # noqa: E402

SCENE = "正前：空地，能，中(1到3米) | 中：墙壁，不能，很近(1米内) | 右：墙壁，不能，很近(1米内)"
THOUGHT = "正前1.62米有墙，右侧0.39米太窄，左侧1.34米也偏窄，左前2.49米最开阔，继续左转绕开"
FOOT = ["(+4.3, -0.3) m", "推理 37 次 / 1.39s", "转 13 次", "记忆 167 格 / 19.7 m",
        "闸 8 次", "视频 312 帧 / 丢 0", "C 实测 48 Hz", "21.5/20/8.3 Hz  画面 12.5 fps"]


def eye(k: int) -> np.ndarray:
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[..., 0] = 40 + (k * 3) % 180
    return img


def main() -> int:
    v = Viewer(dual=True, display=False, stream="/tmp/go2_prof.jpg")
    args = dict(rgb=eye(1), world=eye(2), scene=SCENE, scene_age=1.4,
                thought=THOUGHT, why="视觉打断：画面变了，用新描述重看",
                command={"vx": 0.27, "vy": 0.0}, goal="右前",
                sectors=[("正前", 0.47), ("左前", 0.41), ("左侧", 0.39),
                         ("左后", 0.35), ("正后", 0.65), ("右后", 3.0),
                         ("右侧", 2.47), ("右前", 1.68)],
                history=[(63.4, "正前0.47米贴墙，左侧0.39米太窄"),
                         (65.4, "转到位，继续右转"), (67.5, "安全闸停车")],
                foot=FOOT)

    for _ in range(3):                      # 预热字体缓存
        v.pump(**args)

    pr = cProfile.Profile()
    pr.enable()
    for k in range(20):
        v.pump(**{**args, "rgb": eye(k)})
    pr.disable()
    v.close()

    buf = io.StringIO()
    pstats.Stats(pr, stream=buf).sort_stats("tottime").print_stats(14)
    text = buf.getvalue()
    print(text[text.find("ncalls"):])


if __name__ == "__main__":
    raise SystemExit(main())
