"""通讯客户端：外部程序（Yuki、脚本、网页后端）用的是这一份。

    from quadruped.comm.client import RobotClient

    with RobotClient() as dog:          # 默认 /tmp/go2_robot.sock
        dog.move(vx=0.2)
        print(dog.status()["pos"])
        dog.stop()

命令行：

    python -m quadruped.comm.client status
    python -m quadruped.comm.client move --vx 0.2
    python -m quadruped.comm.client forward 1.0
    python -m quadruped.comm.client stop

## 这一层不认识机器人

不 import `virtual`、不 import DDS、不知道 lo0 还是 en5 —— 它只知道一个套接字。
网口、域号、控制端都在服务端那边定了，客户端换台机器也只是换地址。

## 连接失败是什么意思

`RobotError` 的 code 是机器人返回码（4xxx）或本层的 4xx/5xx；
套接字打不开抛 `ConnectionError`，消息里会说「服务端没起」还是「路径不对」。
"""

from __future__ import annotations

import json
import os
import socket
import sys
import time

from quadruped.comm.protocol import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_SOCKET,
    MAX_LINE,
    encode,
)


class RobotError(RuntimeError):
    """服务端回了 ok=false。code 是机器人返回码或本层错误码。"""

    def __init__(self, code: int, msg: str):
        super().__init__(f"[{code}] {msg}")
        self.code = code
        self.msg = msg


class RobotClient:
    """一条到服务端的连接。用完调 close()，或者用 with。"""

    def __init__(self, target: str | None = None, host: str | None = None,
                 port: int = DEFAULT_PORT, timeout: float = 30.0):
        self.target = target or os.environ.get("GO2_COMM_SOCKET") or DEFAULT_SOCKET
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._sock: socket.socket | None = None
        self._buf = b""
        self._next_id = 1

    # -- 连接 ------------------------------------------------------------

    def connect(self) -> RobotClient:
        if self._sock is not None:
            return self
        if self.host:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            try:
                sock.connect((self.host, self.port))
            except OSError as err:
                sock.close()
                raise ConnectionError(
                    f"连不上 tcp://{self.host}:{self.port}：{err}。"
                    "服务端起了吗？`python -m quadruped.comm.server --host "
                    f"{self.host}`") from None
        else:
            if not os.path.exists(self.target):
                raise ConnectionError(
                    f"套接字不存在：{self.target}。先起服务端："
                    "`python -m quadruped.comm.server`（真机加 `--preset real`）")
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            try:
                sock.connect(self.target)
            except OSError as err:
                sock.close()
                raise ConnectionError(f"连不上 {self.target}：{err}") from None
        self._sock = sock
        return self

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._buf = b""

    def __enter__(self) -> RobotClient:
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- 调用 ------------------------------------------------------------

    # 发一条命令，返回响应里的 data。ok=false 抛 RobotError。timeout 覆盖本次等待。
    def call(self, cmd: str, timeout: float | None = None, **args) -> dict:
        self.connect()
        assert self._sock is not None
        req_id = self._next_id
        self._next_id += 1
        if timeout is not None:
            self._sock.settimeout(float(timeout))
        try:
            self._sock.sendall(encode({"id": req_id, "cmd": cmd, "args": args}))
            line = self._read_line()
        except TimeoutError:
            # 超时后必须把连接扔掉。这条会话是「一问一答」：迟到的那个响应还
            # 躺在套接字里，下一次请求的 `_read_line` 会把它当成自己的 —— 之后
            # 每一条命令都错开一格，返回的是上一条的数据，或者干脆不是合法 JSON。
            # 现象是「某一次动作超时之后，狗就再也不听指令了」，而日志里只有一条
            # 超时。断掉重连是唯一可靠的恢复方式：下一次 `call` 会重新 `connect`。
            self.close()
            raise RobotError(504, f"{cmd} 等待响应超时（{timeout or self.timeout:.0f}s）"
                                  "；长动作要给更长的 timeout") from None
        finally:
            # 超时分支已经把连接关掉了（见上面），`_sock` 会是 None。
            if timeout is not None and self._sock is not None:
                self._sock.settimeout(self.timeout)

        try:
            resp = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as err:
            raise RobotError(500, f"响应不是合法 JSON：{err}（{line[:120]!r}）") from None
        if not resp.get("ok"):
            raise RobotError(int(resp.get("code", 500)), str(resp.get("msg", "")))
        return resp.get("data") or {}

    def _read_line(self) -> bytes:
        assert self._sock is not None
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("服务端关闭了连接")
            self._buf += chunk
            if len(self._buf) > MAX_LINE and b"\n" not in self._buf:
                raise RobotError(500, "响应超过单行上限")
        line, self._buf = self._buf.split(b"\n", 1)
        return line

    # -- 快捷方法 --------------------------------------------------------

    def ping(self) -> dict:
        return self.call("ping")

    def status(self) -> dict:
        return self.call("status")

    def move(self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0) -> dict:
        return self.call("move", vx=vx, vy=vy, vyaw=vyaw)

    def stop(self) -> dict:
        return self.call("stop")

    def freeze(self) -> dict:
        return self.call("freeze")

    def stand(self) -> dict:
        return self.call("stand")

    def crouch(self, depth: float = 0.08) -> dict:
        return self.call("crouch", depth=depth)

    def uncrouch(self) -> dict:
        return self.call("uncrouch")

    def jump(self) -> dict:
        return self.call("jump")

    def gait(self, style: str | None = None) -> dict:
        return self.call("gait", style=style) if style else self.call("gait")

    # 闭环位移，阻塞到走够距离或到时。2 m @0.3 m/s 约 9 s，超时按距离放宽。
    def forward(self, distance: float, vx: float = 0.3) -> dict:
        budget = abs(distance) / max(vx, 0.05) + 6.0
        return self.call("forward", timeout=budget, distance=distance, vx=vx)

    def turn(self, degrees: float, yaw_rate: float = 0.5) -> dict:
        budget = abs(degrees) / 57.3 / max(yaw_rate, 0.05) + 6.0
        return self.call("turn", timeout=budget, degrees=degrees, yaw_rate=yaw_rate)

    def park(self) -> dict:
        return self.call("park")


# ---- 命令行 ------------------------------------------------------------------


def build_parser():
    import argparse

    ap = argparse.ArgumentParser(description="机器狗通讯客户端")
    ap.add_argument("--socket", default=os.environ.get("GO2_COMM_SOCKET", DEFAULT_SOCKET),
                    help="Unix 域套接字路径")
    ap.add_argument("--host", default=None, help=f"走 TCP 时填地址（如 {DEFAULT_HOST}）")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP 端口")
    ap.add_argument("--timeout", type=float, default=30.0, help="单次等待上限（秒）")

    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping", help="确认服务端活着")
    sub.add_parser("status", help="位置、姿态、高度、回路、步态")
    sub.add_parser("stand", help="站定")
    sub.add_parser("uncrouch", help="从蹲姿起来")
    p = sub.add_parser("crouch", help="蹲下")
    p.add_argument("--depth", type=float, default=0.08, help="下降高度 m")
    sub.add_parser("stop", help="停车并站定")
    sub.add_parser("freeze", help="中断当前动作并站定")
    sub.add_parser("jump", help="跳一下")
    sub.add_parser("park", help="收工：停步 → 蹲下 → 阻尼（服务端随后退出）")

    p = sub.add_parser("move", help="持续速度指令，发一次就一直走")
    p.add_argument("--vx", type=float, default=0.0, help="前进 m/s")
    p.add_argument("--vy", type=float, default=0.0, help="左移 m/s")
    p.add_argument("--vyaw", type=float, default=0.0, help="偏航 rad/s")
    p.add_argument("--seconds", type=float, default=0.0,
                   help="走多少秒后自动停；0=一直走，Ctrl+C 停")

    p = sub.add_parser("forward", help="闭环走一段距离")
    p.add_argument("distance", type=float, help="米，负数为后退")
    p.add_argument("--vx", type=float, default=0.3, help="速度 m/s")

    p = sub.add_parser("turn", help="闭环转一个角度")
    p.add_argument("degrees", type=float, help="度，正为逆时针")
    p.add_argument("--yaw-rate", type=float, default=0.5, help="角速度 rad/s")

    p = sub.add_parser("gait", help="切换步态；不给 style 就轮换")
    p.add_argument("style", nargs="?", default=None, help="mech / bio / search")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        with RobotClient(args.socket, host=args.host, port=args.port,
                         timeout=args.timeout) as dog:
            if args.cmd == "move":
                data = dog.move(args.vx, args.vy, args.vyaw)
                print(json.dumps(data, ensure_ascii=False), flush=True)
                _hold(dog, args.seconds)
                return 0
            if args.cmd == "forward":
                data = dog.forward(args.distance, args.vx)
            elif args.cmd == "turn":
                data = dog.turn(args.degrees, args.yaw_rate)
            elif args.cmd == "crouch":
                data = dog.crouch(args.depth)
            elif args.cmd == "gait":
                data = dog.gait(args.style)
            else:
                data = getattr(dog, args.cmd)()
    except ConnectionError as err:
        print(f"连接失败：{err}", file=sys.stderr)
        return 2
    except RobotError as err:
        print(f"命令失败：{err}", file=sys.stderr)
        return 1
    print(json.dumps(data, ensure_ascii=False, indent=2))
    return 0


# 持续速度模式下的等待循环。每秒发一次 status：既是给人看的进度，也是心跳 ——
# 服务端的空闲停车靠请求刷新计时，不发就会被当成「客户端掉了」而停下。
def _hold(dog: RobotClient, seconds: float) -> None:
    deadline = None if seconds <= 0 else time.monotonic() + seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(1.0)
            st = dog.status()
            pos = st.get("pos") or [0.0, 0.0, 0.0]
            print(f"  x={pos[0]:+.2f} y={pos[1]:+.2f} 高度 {st.get('height')} m"
                  f" 步态 {st.get('gait')}", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        print("\n  中断", file=sys.stderr)
    finally:
        dog.stop()


if __name__ == "__main__":
    sys.exit(main())
