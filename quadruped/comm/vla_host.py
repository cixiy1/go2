"""VLA 子进程：起停、日志、人说的一句话。

## 它拥有什么

    * `quadruped.control.embodied.vla` 那个进程的生死
    * 那个进程的 stdin（`--interactive` 模式下，一行就是 `thinker.say()`）
    * 它渲染的窗口文件（`--stream`），网关按 mtime 变化转发给 App

## 它不拥有什么

**不拥有 `rt/lowcmd`。** VLA 里的 `PolicyBackend` 自己写 lowcmd，这一层只负责
把它叫起来、等它体面地退。所以切到 VLA 之前，`robot_link` 必须先 park 掉服务端
（否则两个写者），这个顺序由 `gateway._switch_vla` 保证，不在这里兜。

## 停止 = SIGINT，不是 SIGKILL

真机上 `runner.run()` 的 finally 里写着「蹲下 → 阻尼 → 交还 mcf」。那条路只在
`KeyboardInterrupt` 上走。SIGKILL 等于腿失力：狗从 27 cm 直接掉下来，官方遥控器
也要等下次重启才回得来。所以 `stop()` 先 SIGINT 等 25 秒，等不到才强杀，并把
「强杀了」这件事写进日志 —— 那意味着这次收尾没走完。
"""

from __future__ import annotations

import tempfile
import time
from pathlib import Path

from quadruped.comm.child import ROOT, Child, interpreter

#: 收尾等待上限。`runner` 的 finally 要停推理线程（≤3 s）、写 mp4、关 DDS，
#: 真机上十几秒是正常的；25 秒是「明显不正常」的界线。
STOP_GRACE = 25.0


class VlaHost:
    """VLA 子进程的托管者。"""

    def __init__(self, *, preset: str, event_port: int, echo: bool = True):
        self.preset = preset
        self.event_port = event_port
        self.echo = echo
        self.child: Child | None = None
        self.stream_path = Path(tempfile.gettempdir()) / "go2-vla-stream.jpg"
        self.log_path = (ROOT / "logs" /
                         f"vla-{time.strftime('%Y%m%d-%H%M%S')}.log")

    def _log(self, msg: str) -> None:
        if self.echo:
            print(f"[vla] {msg}", flush=True)

    @property
    def running(self) -> bool:
        return self.child is not None and self.child.running

    @property
    def log(self) -> list[str]:
        return list(self.child.lines) if self.child is not None else []

    # 起 VLA。`--no-watch` + `--stream`：真机上没有图形会话，画面走文件给网关；
    # `--interactive` 让 stdin 上的每一行变成 `thinker.say()`。
    def start(self, goal: str | None = None, scene: str | None = None) -> dict:
        if self.running:
            return {"ok": True, "already": True}
        self.stream_path.unlink(missing_ok=True)
        args = [interpreter(), "-m", "quadruped.control.embodied.vla",
                "--preset", self.preset, "--no-watch",
                "--stream", str(self.stream_path),
                "--events", f"127.0.0.1:{self.event_port}",
                "--interactive", "--log", str(self.log_path)]
        if goal:
            args += ["--goal", goal]
        env = {"GO2_SCENE": scene} if scene else None
        self.child = Child(name="vla", args=args, cwd=ROOT, env=env,
                           stdin=True, echo=self.echo).start()
        self._log(f"已启动 pid={self.child.pid}，画面 → {self.stream_path}")
        return {"ok": True, "pid": self.child.pid}

    # 停止。返回退出码；`killed=True` 表示等超时被强杀（收尾没走完）。
    def stop(self) -> dict:
        child = self.child
        if child is None:
            return {"ok": True, "already": True}
        code = child.stop(grace=STOP_GRACE)
        killed = code is not None and code < 0
        if killed:
            self._log(f"{STOP_GRACE:.0f}s 没退，强杀（腿可能没阻尼）")
        self.child = None
        return {"ok": True, "exit": code, "killed": killed}

    # 人说一句话。stdin 那一头是 `_stdin_loop`，一行一句。
    def say(self, text: str) -> bool:
        return self.child is not None and self.child.write(text)

    def status(self) -> dict:
        child = self.child
        return {
            "running": self.running,
            "pid": None if child is None else child.pid,
            "uptime": (None if child is None or not self.running
                       else round(time.time() - (child.started_at or time.time()), 1)),
            "log": str(self.log_path),
            "stream": str(self.stream_path),
        }
