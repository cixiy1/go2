"""VLA 事件通道：把这一层的运行状态按行 JSON 打到一个 UDP 地址。

## 为什么是 UDP，不是套接字/TCP

发布方是 VLA 子进程，订阅方是网关。三方里**只有 VLA 是必须活下去的那个** ——
UI 崩了、网关被关掉，狗还在自主走，不能因为「没人看」就卡在 `send` 上。

UDP 正好是这个语义：发出去就不管，没人收也不报错，不会给控制回路加任何
背压。代价是丢包和乱序，所以每条事件都带 `seq` 和 `t`，收方按 `seq` 丢旧帧。

## 只发「人能看见」的量

这个通道不是日志。日志是文本、给人读、可以慢慢刷；这里是 UI 的数据源，
字段名和 Flutter 那边一一对应，改一侧就要改另一侧（`docs/flutter-client.md`
有一张对照表）。所以宁少不多：策略频率、命令、8 方位、视觉描述、想法、回话、
安全闸、位置朝向，够了。加字段之前先问：界面上哪个像素要用它。

## 这不是控制链路

VLA 的命令只走 `PolicyBackend` 直连 `rt/lowcmd`。事件通道单向、只读，
`socket.sendto` 失败静默吞掉 —— 它挂了不影响狗走路。
"""

from __future__ import annotations

import json
import socket


# 解析 "127.0.0.1:8790"。端口没给就用 8790，写死一个默认值是为了
# `--events on` 这种偷懒写法也能用。
def parse_target(text: str) -> tuple[str, int]:
    text = (text or "").strip()
    if not text or text == "on":
        return "127.0.0.1", 8790
    host, _, port = text.rpartition(":")
    if not host:
        return text, 8790
    return host, int(port)


class EventSink:
    """一条 UDP 出口。`publish` 不抛异常、不阻塞。"""

    def __init__(self, target: str = "127.0.0.1:8790"):
        self.host, self.port = parse_target(target)
        self.seq = 0
        self.sent = 0
        self.failed = 0
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 设成非阻塞：对端不在时 sendto 会立刻返回 ENOBUFS 或 EAGAIN，
            # 而不是在控制回路里等。UDP 本来也不会真的阻塞，但这是保险。
            self._sock.setblocking(False)
        except OSError:
            self._sock = None

    @property
    def target(self) -> str:
        return f"{self.host}:{self.port}"

    # 发一条事件。`obj` 必须是可 JSON 序列化的 dict。
    #
    # 序列化失败只记数不抛：一个字段是 numpy 标量就让整条 VLA 崩掉，不划算。
    # UI 少一帧没人看得出来，狗停下来所有人都看得出来。
    def publish(self, obj: dict) -> None:
        if self._sock is None:
            return
        self.seq += 1
        obj = dict(obj)
        obj["seq"] = self.seq
        try:
            line = json.dumps(obj, ensure_ascii=False, default=float)
        except (TypeError, ValueError):
            self.failed += 1
            return
        try:
            self._sock.sendto(line.encode("utf-8"), (self.host, self.port))
            self.sent += 1
        except OSError:
            self.failed += 1

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class NullSink:
    """`--events` 没给时用它。省掉每处 `if sink is not None`。"""

    seq = 0
    sent = 0
    failed = 0
    target = ""

    def publish(self, obj: dict) -> None:
        pass

    def close(self) -> None:
        pass
