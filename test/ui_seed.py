"""联调用的假后端：假机器狗 + 假 VLA + 合成画面，只为了把 App 填满数据。

    .venv/bin/python test/ui_seed.py --port 8780 --token demo

然后 App（或浏览器）连 `ws://127.0.0.1:8780/?t=demo`。

## 这东西不属于产品路径

它不驱动任何真实硬件，也不模拟控制律。存在的理由只有一个：**界面上的每个格子
都得有真实形状的数据才看得出布局对不对**。没有它，调 VLA 页就得开着 MuJoCo、
加载视觉模型、等模型生成 —— 一次十几秒，改一个 padding 也要等。

## 数据是编的，但形状是真的

字段名、类型、取值范围全部照抄 `runner.py` 的 `sink.publish(...)`。所以它验的是
「界面能不能吃下真实数据」，不是「假数据好不好看」。真要改字段，改 `runner.py`
的时候顺手改这里 —— 两边对不上，界面会安静地显示旧值。

    --frames 0   不生成画面（只看数字）
    --still      画面不变（测 Image.memory 的 gaplessPlayback）
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test"))

from gateway_check import FakeRobot, FakeVla          # noqa: E402
from quadruped.comm.gateway import Gateway           # noqa: E402

THOUGHTS = [
    "正前方 1.4 米有台阶边缘，左边空着 2.9 米。往左偏 20 度能绕过去。",
    "左边是墙，右边有一条通道。先直走再看。",
    "前方 0.8 米有矮障碍，退一点再转向，别贴着走。",
    "目标在东北方向，机头偏了 35 度，先转过去。",
]
REPLIES = [
    "我往左边绕，前面有台阶。",
    "看到你了，正前 1.6 米，我这就过去。",
    "我停下了，正前 0.78 米有东西挡着。",
    "转了 60 度，还没看到你，看到就过去。",
]


class FakeCamFeed:
    """假相机馈送：只提供「在跑 + 有个文件」，内容由 Seed 写。"""

    def __init__(self, path: Path):
        self.stream_path = path
        self.running = True

    def start(self):
        self.running = True
        return {"ok": True, "pid": 3}

    def stop(self):
        self.running = False

    def status(self):
        return ({"running": True, "pid": 3} if self.running
                else {"running": False})


class DriftingRobot(FakeRobot):
    """状态随时间缓慢变化，界面上的数字才会跳。"""

    def _reply(self, rid, cmd: str, args: dict) -> dict:
        t = time.monotonic() - self.t0
        data = super()._reply(rid, cmd, args)
        if cmd == "status":
            d = data["data"]
            d["pos"] = [round(0.28 * t, 3), round(0.12 * math.sin(t / 3), 3)]
            d["rpy"] = [round(0.02 * math.sin(t), 3), round(0.03 * math.cos(t / 2), 3),
                        round(0.4 * math.sin(t / 8), 3)]
            d["height"] = round(0.27 + 0.004 * math.sin(t * 3), 4)
            d["moving"] = True
            d["loop"] = "策略 50 Hz · 回路 49.7 Hz · 闸 0"
        return data


class Seed:
    def __init__(self, gateway: Gateway, fps: float, still: bool, port: int):
        self.gw = gateway
        self.fps = fps
        self.still = still
        self.t0 = time.monotonic()
        self.seq = 0
        self.reply_seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 文件名带上端口：同时起两个假后端时，两条流水线会抢同一个 .tmp，
        # 后 `os.replace` 的那个报 ENOENT，画面一帧都出不来。
        self.img = Path(tempfile.gettempdir()) / f"go2-vla-stream-{port}.jpg"
        self.cam_img = Path(tempfile.gettempdir()) / f"go2-cam-{port}.jpg"

    # -- 画面 ------------------------------------------------------------

    def draw(self, t: float) -> None:
        from PIL import Image, ImageDraw

        w, h = 640, 480
        im = Image.new("RGB", (w, h), (18, 22, 26))
        d = ImageDraw.Draw(im)

        # 地面：一点透视的横线，越远越密。
        for i in range(14):
            y = h * 0.55 + (h * 0.45) * (i / 14) ** 1.8
            d.line([(0, y), (w, y)], fill=(30 + i * 2, 38 + i * 2, 44 + i * 2), width=1)
        for i in range(-6, 7):
            d.line([(w / 2 + i * 40, h), (w / 2 + i * 9, h * 0.55)],
                   fill=(28, 34, 40), width=1)

        # 两个障碍物。左前那个就是「安全闸」在报的东西。
        d.rectangle([70, 250, 210, 360], fill=(52, 60, 68), outline=(86, 96, 106))
        d.rectangle([430, 210, 560, 330], fill=(46, 54, 62), outline=(78, 88, 98))

        # 移动的人形标记：证明画面在刷新，而不是一帧卡住。
        x = w / 2 + 120 * math.sin(t / 4)
        d.ellipse([x - 9, 205, x + 9, 223], fill=(200, 120, 70))
        d.line([(x, 223), (x, 290)], fill=(200, 120, 70), width=6)

        d.text((14, h - 26), f"SIM · go2.xml · {t:05.1f}s", fill=(120, 200, 170))
        d.text((w - 96, 14), "640×480", fill=(90, 100, 110))

        for path in (self.img, self.cam_img):
            tmp = path.with_suffix(path.suffix + ".tmp")
            im.save(tmp, "JPEG", quality=78)
            os.replace(tmp, path)

    # -- 事件 ------------------------------------------------------------

    def event(self, t: float) -> dict:
        front = 1.35 + 0.55 * math.sin(t / 7)
        front = max(0.55, front)
        names = ["正前", "左前", "左侧", "左后", "正后", "右后", "右侧", "右前"]
        sectors = []
        for i, name in enumerate(names):
            base = front if i == 0 else 1.6 + 1.2 * abs(math.sin(t / 9 + i))
            sectors.append({"name": name, "gap": round(min(3.0, base), 2),
                            "top": 0.0})
        if int(t) % 11 == 10:
            self.reply_seq += 1
        self.seq += 1
        return {
            "kind": "vla",
            "seq": self.seq,
            "t": round(t, 2),
            "scene": "flat",
            "source": "sim+vision",
            "auto": True,
            "cmd": {"vx": round(0.18 + 0.06 * math.sin(t / 3), 3),
                    "vy": 0.0,
                    "vyaw": round(0.12 * math.sin(t / 5), 3)},
            "gait": "trot",
            "front_gap": round(front, 2),
            "sectors": sectors,
            "scene_text": "画面里有一条走廊，左前有一级台阶，右侧2米处有墙。"
                          "地面平整，没有看到人。" if int(t) % 9 else "画面里能看到一个人，正前方约1.6米。",
            "scene_age": round(0.4 + (t % 3), 1),
            "thought": THOUGHTS[int(t / 8) % len(THOUGHTS)],
            "reply": REPLIES[self.reply_seq % len(REPLIES)],
            "reply_seq": self.reply_seq,
            "emit_thought": True,
            "goal": "东北 · (2.4, 1.8)",
            "turns": int(t / 12),
            "turn_left_deg": None if int(t) % 12 > 3 else round(28 - (t % 12) * 2),
            "thoughts": int(t / 1.2) + 1,
            "gen_s": round(1.1 + 0.4 * abs(math.sin(t / 6)), 2),
            "think_rate": 0.62,
            "last_error": None,
            "guard": {"trips": int(t / 25),
                      "last": "正前 0.42 m，压到 0.18" if t > 25 else None},
            "memory": {"cells": 12 + int(t / 10), "travelled": round(1.2 * t / 10, 1)},
            "pos": [round(0.8 + 0.05 * t, 2), round(0.4 * math.sin(t / 8), 2)],
            "yaw_deg": round(20 * math.sin(t / 10), 1),
            "tilt_deg": round(4.2 + 1.5 * abs(math.sin(t / 2)), 1),
            "policy_hz": 49.7,
            "reflex_hz": 18.4,
            "view_fps": 20.0,
            "vision": {"frames": int(t * 5), "dropped": int(t / 20),
                       "views": int(t / 3), "gen_s": 1.2},
            "driver": {"direction": "东北", "changes": 3, "blocked": 1},
            "alert": front < 0.7,
        }

    def run(self) -> None:
        self.gw.vla.stream_path = self.img
        while True:
            t = time.monotonic() - self.t0
            if self.fps > 0 and (self.still is False or self.seq < 6):
                try:
                    self.draw(t)
                except Exception as err:                   # noqa: BLE001
                    print(f"画不出来：{err}", flush=True)
            payload = self.event(t)
            self.sock.sendto(json.dumps(payload, ensure_ascii=False).encode(),
                             ("127.0.0.1", self.gw.event_port))
            time.sleep(1 / 5.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8780)
    ap.add_argument("--token", default="demo")
    ap.add_argument("--frames", type=float, default=2.0, help="画面帧率，0=不出图")
    ap.add_argument("--still", action="store_true")
    ap.add_argument("--events-port", type=int, default=0,
                    help="VLA 事件端口，默认由内核分配（同时跑两个实例时别写死）")
    ap.add_argument("--mode", choices=("vla", "manual"), default="vla",
                    help="vla=两页都有数据（默认）；manual=只有遥控页有数据")
    args = ap.parse_args()

    tmpdir = tempfile.mkdtemp(prefix="go2-seed-")
    robot = DriftingRobot(os.path.join(tmpdir, "robot.sock"))
    robot.t0 = time.monotonic()
    robot.start()

    gw = Gateway(host="127.0.0.1", port=args.port, token=args.token,
                 socket_path=robot.path, preset="sim", robot_manage="attach",
                 host_flag=None, events_port=args.events_port, verbose=True)
    fake = FakeVla()
    gw.vla = fake
    # 先把机器人链路打开（遥控页要有 pos/rpy/height 才看得出布局），再假装切到
    # VLA（VLA 页要有事件）。真实环境里这两件事互斥，这里为了让两页同时有数据
    # 故意错开了 —— 截图好看，不代表可以这么跑。
    gw._switch_manual()
    # 手动模式下网关会自己拉 `percept.feed`；假环境里没有相机，先把那个子进程
    # 收掉（否则它会一直空转并往日志里刷「收图 0 帧」），再换成我们的假文件。
    gw.cam.stop()
    gw.cam = FakeCamFeed(Path(tempfile.gettempdir()) / f"go2-cam-{args.port}.jpg")
    if args.mode == "vla":
        fake.running = True
        gw.mode = "vla"
    gw.mode_note = ""

    seed = Seed(gw, fps=args.frames, still=args.still, port=args.port)
    threading.Thread(target=seed.run, daemon=True, name="seed").start()

    print(f"假后端就绪：ws://127.0.0.1:{args.port}/?t={args.token}"
          f"   事件端口 {gw.event_port}", flush=True)
    print("（Ctrl+C 退出）", flush=True)
    try:
        return gw.serve()
    except KeyboardInterrupt:
        return 0
    finally:
        robot.close()


if __name__ == "__main__":
    sys.exit(main())
