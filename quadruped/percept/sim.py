"""仿真感知源：从仿真进程收一帧「激光扫描」，转换成 `Percept`。

数据走 UDP 回环，不走 DDS。原因是 Unitree 的 IDL 里没有感知消息，加一个自定义
topic 要改两边 IDL、重新生成绑定，而这一步对整个 V 层没有任何价值 —— 上真机时
这一层会换成相机（`camera.py`），DDS 那套也跟着作废。UDP + JSON 是谁都能替换的
临时约定，接口窄到只传数字。

帧格式（`simulate_python/percept_pub.py` 发）：

    {"t": 12.3, "origin": "sim-lidar",
     "ego": [x, y, z, yaw, pitch, roll, vx, vy, wz],
     "rays": [[bearing_deg, distance, top], ...],
     "ground": {"under": 0.0, "ahead": [0.0, 0.08]},
     "range_max": 3.0}

`rays` 用列表不用对象：每帧 33 条，字段名占的字节比数据还多，而这个格式只会被
本文件解析，可读性交给这里。
"""

from __future__ import annotations

import json
import math
import os
import socket
import time
from dataclasses import replace

from quadruped.percept.types import Ground, Percept, Ray, SelfState

DEFAULT_PORT = 42300


# 端口可以用环境变量改，两个进程要一致。仿真侧读同一个变量。
def default_port() -> int:
    try:
        return int(os.environ.get("PERCEPT_PORT", DEFAULT_PORT))
    except ValueError:
        return DEFAULT_PORT


class SimPercept:
    name = "sim-lidar"

    def __init__(self, port: int | None = None, host: str = "127.0.0.1",
                 stale_after: float = 1.0, rays: bool = False):
        self.port = default_port() if port is None else int(port)
        self.stale_after = float(stale_after)
        # 要不要把射线交给上层。**默认不要**：射线是 MuJoCo 的 `mj_ray` 求交，
        # 零噪声、24 条均布、量程正好 3.0 m —— 这三样真机一样都没有，真机上只有
        # 一个相机。带着它跑出来的「能走迷宫」在真机上不成立，所以默认丢掉，
        # 让仿真和真机拿到同一份输入：一张图 + 机身遥测。
        #
        # 留着这个开关是为了对照实验（同样的 L 层、只多一路射线，差多少），
        # 不是为了默认跑。上层的开关是 `--geom`，见 `percept/base.open_source`。
        self.rays = bool(rays)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 允许立刻重开：控制器崩了重启时，上个进程的 socket 可能还在 TIME_WAIT。
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 调大接收缓冲。上层如果几秒没读（比如在等视觉模型加载），10 Hz 的
        # 射线帧会把默认的 42 KB 挤满，而队列满时丢的是**新**包 —— 之后读到的
        # 全是旧的，表现是「感知已停 4 秒」，明明仿真一直在发。
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        self._sock.bind((host, self.port))
        self._sock.setblocking(False)

        self.frames = 0
        self.bad_frames = 0
        self.last_stamp = 0.0
        self._last: Percept | None = None

    def close(self) -> None:
        self._sock.close()

    # 把这一拍攒下的帧全部收掉，只留最新的。
    #
    # 不排队：感知是「现在是什么样」，不是「曾经发生过什么」。仿真跑 10 Hz、
    # 控制循环 50 Hz，两者不同步，攒着会导致看到的永远是过去的场景。
    def _drain(self) -> dict | None:
        latest = None
        while True:
            try:
                raw, _ = self._sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                latest = json.loads(raw)
            except (json.JSONDecodeError, UnicodeDecodeError):
                self.bad_frames += 1
        return latest

    def read(self) -> Percept | None:
        frame = self._drain()
        if frame is not None:
            try:
                self._last = self._parse(frame)
                self.frames += 1
                self.last_stamp = time.monotonic()
                return self._last
            except (KeyError, TypeError, ValueError, IndexError):
                # 仿真侧改字段、或者半包，都落到这里。丢掉这一帧，保留上一帧：
                # 让上层看到「一秒前的场景」比看到 None 更好用，因为上层能自己
                # 判断过期时间（`age`）。
                self.bad_frames += 1

        if self._last is None or not self.stale:
            return self._last
        # 没有新帧、旧帧又过期了。这时候必须标出来：一份「看着正常、其实是
        # 三秒前」的观测，比没有观测更危险 —— 模型会照着三秒前的地形做决定。
        return replace(self._last, notes=self._last.notes + (
            f"感知已停 {self.age:.1f} s，画面是旧的",))

    # 距上一次成功解析过去了多久。L 层用它决定还能不能信这份观测。
    @property
    def age(self) -> float:
        return float("inf") if not self.last_stamp else time.monotonic() - self.last_stamp

    @property
    def stale(self) -> bool:
        return self.age > self.stale_after

    def _parse(self, frame: dict) -> Percept:
        ego = frame["ego"]
        if len(ego) != 9:
            raise ValueError(f"ego 应该是 9 个数，收到 {len(ego)}")
        state = SelfState(*(float(v) for v in ego))

        rays = tuple(
            Ray(math.radians(float(b)), float(d), float(h))
            for b, d, h in frame.get("rays", ())
        ) if self.rays else ()

        g = frame.get("ground") or {}
        ground = Ground(
            under=float(g.get("under", 0.0)),
            ahead=tuple(float(v) for v in g.get("ahead", ())),
        )

        notes = list(frame.get("notes") or ())
        return Percept(
            stamp=float(frame.get("t", 0.0)),
            origin=str(frame.get("origin", self.name)),
            ego=state,
            rays=rays,
            ground=ground,
            range_max=float(frame.get("range_max", 3.0)),
            notes=tuple(notes),
        )
