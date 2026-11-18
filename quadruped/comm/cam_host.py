"""相机馈送子进程：手动遥控时那一路画面。

## 它拥有什么

    * `quadruped.percept.feed` 这个进程（只出图，不写 lowcmd）
    * 它写的那个 JPEG 文件

## 它不拥有什么

不拥有相机本身。真机的 videohub 只认一个订阅者，仿真的视频流是 UDP 广播 ——
「谁能取到帧」由 `percept` 层决定，这里只决定「什么时候去取」。所以：

    VLA 在跑   → 不启动（VLA 自己就在取，两路同时开只会互相抢）
    手动模式   → 启动
    其它       → 停掉

## 为什么单独一个进程

`ChannelFactoryInitialize` 一进程只调一次。相机和控制链路挤在同一个进程里，
谁先初始化，另一条就被绑在那套 domain/网卡上，而且不报错（详见
`comm/__init__.py`）。分进程是最便宜的分界线。
"""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path

from quadruped.comm.child import ROOT, Child, interpreter

#: 停止等待。这个进程没有收尾动作（不碰腿），5 秒足够。
STOP_GRACE = 5.0


class CamHost:
    """相机馈送进程的托管者。"""

    def __init__(self, *, preset: str, hz: float = 8.0, echo: bool = True,
                 iface: str | None = None):
        self.preset = preset
        self.hz = hz
        self.echo = echo
        self.iface = iface
        self.child: Child | None = None
        # 文件名带 pid：同时跑两个网关时不会互相覆盖同一个文件。
        self.stream_path = Path(tempfile.gettempdir()) / f"go2-cam-{os.getpid()}.jpg"
        # 雷达和相机同一个进程。两者都要 DDS，而 `ChannelFactoryInitialize`
        # 一进程只能一次 —— 分成两个进程就等于要初始化两遍，第二遍直接抛。
        self.radar_path = Path(tempfile.gettempdir()) / f"go2-radar-{os.getpid()}.json"

    def _log(self, msg: str) -> None:
        if self.echo:
            print(f"[cam] {msg}", flush=True)

    @property
    def running(self) -> bool:
        return self.child is not None and self.child.running

    def start(self) -> dict:
        if self.running:
            return {"ok": True, "already": True}
        self.stream_path.unlink(missing_ok=True)
        self.radar_path.unlink(missing_ok=True)
        args = [interpreter(), "-m", "quadruped.percept.feed",
                "--preset", self.preset, "--out", str(self.stream_path),
                "--hz", f"{self.hz:g}", "--radar-out", str(self.radar_path)]
        if self.iface:
            args += ["--iface", self.iface]
        self._log(f"起馈送：{' '.join(args[3:])}")
        self.child = Child(name="cam", args=args, cwd=ROOT,
                           echo=self.echo).start()
        return {"ok": True, "pid": self.child.pid}

    def stop(self) -> None:
        child = self.child
        if child is None:
            return
        child.stop(grace=STOP_GRACE)
        self.child = None
        self.stream_path.unlink(missing_ok=True)
        self.radar_path.unlink(missing_ok=True)

    def status(self) -> dict:
        if not self.running:
            child = self.child
            note = ""
            if child is not None and child.lines:
                # 最后一行就是它死掉的原因（端口占用、没有相机……）。
                note = child.lines[-1]
            return {"running": False,
                    "exit": None if child is None else child.returncode,
                    "note": note}
        return {"running": True, "pid": self.child.pid if self.child else None,
                "path": str(self.stream_path),
                "radar": str(self.radar_path),
                "uptime": round(time.time() - (self.child.started_at or time.time()), 1)}
