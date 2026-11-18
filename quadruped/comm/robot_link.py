"""机器人链路：网关到 `comm.server` 的那一条连接，以及那个服务端的生命周期。

## 它拥有什么

    * 唯一一条到 `comm.server` 的 TCP/Unix 连接（`comm` 的会话是一问一答，
      多条线程同时写会把响应配错请求，所以 `call` 全程持锁）
    * 如果服务端是网关自己拉起来的，还拥有那个子进程
    * `rt/lowcmd` 的间接使用权：只有这个类的 `call("move"/"stop"/...)` 能让狗动

**不拥有**：VLA（在 `vla_host.py`）、相机（在 `cam_host.py`）、WebSocket 会话
（在 `gateway.py`）。三者互不 import，只由 `gateway.py` 编排。

## 交接的硬约束

`rt/lowcmd` 同一时刻只能有一个发送者。所以 `shutdown()` 不是「断开连接」，
是「让服务端放手」：先发 `park` 让它停步 → 蹲下 → 阻尼 → 自己退出，再关连接。
只 close 的话那个进程还握着 lowcmd，VLA 起来就是两个写者 —— 表现是腿一抽一抽，
而不是报错。
"""

from __future__ import annotations

import os
import socket
import threading
import time

from quadruped.comm.child import ROOT, Child, interpreter
from quadruped.comm.client import RobotClient, RobotError
from quadruped.comm.protocol import DEFAULT_PORT, DEFAULT_SOCKET

#: 状态轮询频率。和网关的状态帧同频（10 Hz）—— 再快就是自己在套接字上造队列。
POLL_HZ = 10.0


class RobotLink:
    """到 `comm.server` 的一条连接。可能连的是别人的服务端，也可能是子进程。"""

    def __init__(self, *, socket_path: str = DEFAULT_SOCKET, host: str | None = None,
                 port: int = DEFAULT_PORT, preset: str = "sim",
                 manage: str = "auto", poll_hz: float = POLL_HZ,
                 echo: bool = True):
        self.socket_path = socket_path
        self.host = host
        self.port = int(port)
        self.preset = preset
        self.manage = manage
        self.poll_hz = poll_hz
        self.echo = echo
        self.client: RobotClient | None = None
        self.child: Child | None = None
        self.error: str | None = None
        self.status: dict = {}
        self.rtt_ms: float | None = None
        self.status_ts = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _log(self, msg: str) -> None:
        if self.echo:
            print(f"[robot] {msg}", flush=True)

    @property
    def owns_server(self) -> bool:
        """服务端是网关拉起来的吗。是才允许强杀，别人起的不归我们管。"""
        return self.child is not None

    # -- 连接 ------------------------------------------------------------

    def _listening(self) -> bool:
        if self.host:
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            probe.settimeout(0.3)
            try:
                probe.connect((self.host, self.port))
                return True
            except OSError:
                return False
            finally:
                probe.close()
        return os.path.exists(self.socket_path)

    # 起一个 comm.server 子进程。只在没有现成服务端时走这条路。
    #
    # 起之前先看套接字在不在：有就说明已经有人在管机器人（可能是用户自己起的），
    # 再去起第二个就是两个写者抢 `rt/lowcmd`。
    def _spawn(self) -> None:
        args = [interpreter(), "-m", "quadruped.comm.server",
                "--preset", self.preset, "--quiet"]
        if self.host:
            args += ["--host", self.host, "--port", str(self.port)]
        else:
            args += ["--socket", self.socket_path]
        self._log(f"拉起服务端：{' '.join(args[1:])}")
        self.child = Child(name="robot", args=args, cwd=ROOT,
                           echo=self.echo).start()

    # 打开连接。已经有服务端就连上去，没有就按 `manage` 决定要不要起。
    def open(self) -> None:
        with self._lock:
            if self.client is not None:
                return
        if not self._listening():
            if self.manage == "attach":
                raise ConnectionError(
                    f"没有可连的服务端（{self.socket_path}）。先起 `python -m "
                    "quadruped.comm.server`，或者把 --robot 改成 auto")
            self._spawn()
            deadline = time.monotonic() + 40.0
            while time.monotonic() < deadline:
                if self._listening():
                    break
                if self.child is not None and not self.child.running:
                    raise ConnectionError(
                        f"服务端起不来（退出码 {self.child.returncode}）")
                time.sleep(0.25)
            else:
                raise ConnectionError("服务端 40 秒内没开门")
        client = RobotClient(target=self.socket_path, host=self.host,
                             port=self.port)
        client.connect()
        with self._lock:
            self.client = client
        self.error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._poll, daemon=True,
                                        name="robot-poll")
        self._thread.start()
        self._log(f"已连上（{self.host or self.socket_path}）")

    # 状态轮询。必须走 `call`（带锁）：轮询和下发速度同时往同一个套接字写，
    # 响应会配错请求 —— 表现是偶尔拿到别人的数据，最难查。
    def _poll(self) -> None:
        while not self._stop.wait(1.0 / self.poll_hz):
            t0 = time.monotonic()
            try:
                data = self.call("status", timeout=2.0)
            except (OSError, RobotError, TimeoutError) as err:
                self.error = f"{type(err).__name__}: {err}"
                continue
            self.rtt_ms = round((time.monotonic() - t0) * 1000.0, 1)
            self.status = data
            self.status_ts = time.monotonic()
            self.error = None

    # 发一条命令。串行化：`comm` 的会话是「一问一答」。
    def call(self, cmd: str, timeout: float = 6.0, **args) -> dict:
        client = self.client
        if client is None or self._stop.is_set():
            raise ConnectionError("还没有连上机器人")
        with self._lock:
            return client.call(cmd, timeout=timeout, **args)

    # 收工：先让服务端自己 park（停步 → 蹲下 → 阻尼 → 退出），再关连接。
    #
    # 等不到也不无限等 —— 「服务端还开着监听」和「它还没放手」不是一回事：
    # 等待期间操作权会因为没有心跳被收回，用户看到的是「切个模式，遥控权没了」。
    def shutdown(self, grace: float = 10.0) -> None:
        self._stop.set()
        client = self.client
        if client is not None:
            try:
                client.call("park", timeout=grace)
            except (OSError, RobotError, TimeoutError, ConnectionError):
                pass
            client.close()
        with self._lock:
            self.client = None

        child = self.child
        if child is not None:
            if child.wait(grace) is None:
                self._log("服务端没自己退，送 SIGINT")
                child.stop(grace=10.0)
            self.child = None
        else:
            # 别人起的服务端：park 之后它会自己退。等 3 秒，等不到也只能算了 ——
            # 连接已经断开，那一侧会 `release()` 停车。
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and self._listening():
                time.sleep(0.2)

    def describe(self) -> dict:
        return {
            "target": self.host or self.socket_path,
            "preset": self.preset,
            "owns_server": self.owns_server,
            "server": None if self.child is None else self.child.describe(),
        }
