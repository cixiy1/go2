"""子进程的统一外壳：起、收日志、按信号停。

网关要管三个子进程，它们的收尾方式不同：

    comm.server    先发 `park`（停步 → 蹲下 → 阻尼），等它自己退
    vla            SIGINT，等它蹲下 → 阻尼 → 交还 mcf
    percept.feed   SIGINT，几秒内退

共同点只有三件：独立会话（要能 `killpg` 一次收掉它拉起的线程）、输出要进日志、
先 SIGINT 再考虑 SIGKILL。这一层只做这三件，不碰任何业务。

## 进程归属只有这里

`subprocess.Popen` 在整个 `quadruped/comm/` 里只出现在这个文件。谁起谁停的规定
写在 `docs/control-ownership.md`，由 `test/arch_check.py` 规则 6 检查。

这样做不是为了少写几行，是为了让「谁杀了谁」只有一个答案。之前三份几乎一样的
spawn/stop 代码里，VLA 那份等 25 秒、相机那份等 5 秒、机器人那份先 park 再等 ——
差异都是对的，但散在三处，改一个就会和另外两个不一致。
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

#: 仓库根目录。子进程的 cwd 都设在这里，`python -m quadruped...` 才找得到包。
ROOT = Path(__file__).resolve().parents[2]


class Child:
    """一个受管子进程。构造后调 `start()`，用完调 `stop()`。

    `on_line` 收到每一行输出（已经去掉行尾），用于给界面提供日志环形缓冲；
    `echo` 控制要不要打到网关终端。
    """

    def __init__(self, *, name: str, args: list[str], cwd: Path | None = None,
                 env: dict | None = None, stdin: bool = False,
                 echo: bool = True, keep: int = 400,
                 on_line: Callable[[str], None] | None = None):
        self.name = name
        self.args = list(args)
        self.cwd = Path(cwd or ROOT)
        self.env = env
        self.echo = echo
        self.stdin_pipe = stdin
        self.on_line = on_line
        self.lines: deque[str] = deque(maxlen=keep)
        self.proc: subprocess.Popen | None = None
        self.started_at: float | None = None

    # -- 生命周期 --------------------------------------------------------

    @property
    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    @property
    def pid(self) -> int | None:
        return None if self.proc is None else self.proc.pid

    @property
    def returncode(self) -> int | None:
        return None if self.proc is None else self.proc.returncode

    def start(self) -> Child:
        if self.running:
            return self
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        if self.env:
            env.update(self.env)
        self.proc = subprocess.Popen(
            self.args, cwd=str(self.cwd), env=env,
            stdin=subprocess.PIPE if self.stdin_pipe else subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1,
            # 独立会话：停止时 `killpg` 能一次收掉它自己拉起的线程和子进程。
            start_new_session=True)
        self.started_at = time.time()
        threading.Thread(target=self._pump, daemon=True,
                         name=f"{self.name}-log").start()
        return self

    def _pump(self) -> None:
        proc = self.proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            text = line.rstrip()
            self.lines.append(text)
            if self.echo:
                print(f"[{self.name}] {text}", flush=True)
            if self.on_line is not None:
                try:
                    self.on_line(text)
                except Exception:                     # noqa: BLE001
                    # 日志回调不该把读线程打死。
                    pass

    # -- 交互 ------------------------------------------------------------

    # 往子进程的 stdin 写一行。没有 stdin 或者进程已退就返回 False。
    def write(self, line: str) -> bool:
        proc = self.proc
        if proc is None or proc.poll() is not None or proc.stdin is None:
            return False
        try:
            proc.stdin.write(line.replace("\n", " ").strip() + "\n")
            proc.stdin.flush()
            return True
        except (BrokenPipeError, ValueError):
            return False

    # -- 收尾 ------------------------------------------------------------

    # 等它自己退。超时返回 None（不抛异常）—— 调用方按「还没退」处理，
    # 决定是再等、发信号还是强杀。
    def wait(self, timeout: float) -> int | None:
        proc = self.proc
        if proc is None:
            return None
        try:
            return proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    # 先 SIGINT，超时再 SIGKILL。返回退出码（拿不到就是 None）。
    #
    # SIGINT 不是 SIGTERM：这几条链路的收尾（蹲下 → 阻尼 → 交还）都挂在
    # `KeyboardInterrupt` 那条路上，SIGTERM 在部分进程里没有处理器。
    def stop(self, grace: float = 10.0) -> int | None:
        proc = self.proc
        if proc is None:
            return None
        if proc.poll() is None:
            self._signal(signal.SIGINT)
            if self.wait(grace) is None:
                self._signal(signal.SIGKILL)
                self.wait(5.0)
        code = proc.returncode
        self.proc = None
        return code

    def kill(self) -> None:
        if self.running:
            self._signal(signal.SIGKILL)
            self.wait(3.0)

    def _signal(self, sig: int) -> None:
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            # 已经退了，或者不是本用户的进程（不该发生）。都不该中断收尾。
            pass

    # 给自己看的一行状态，日志和状态帧共用。
    def describe(self) -> str:
        if self.proc is None:
            return "未启动"
        if self.running:
            age = time.time() - (self.started_at or time.time())
            return f"运行中 pid={self.proc.pid} {age:.0f}s"
        return f"已退出 code={self.proc.returncode}"


#: 起子进程用的解释器。测试里可以换成别的，生产就是当前解释器。
def interpreter() -> str:
    return sys.executable
