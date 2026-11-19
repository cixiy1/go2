"""最小 WebSocket 服务端：只够网关用。

## 为什么自己写

`quadruped/comm/` 的模块级 import 只允许标准库（`test/arch_check.py` 规则 2），
而手机和浏览器都只能走 WebSocket —— 裸 TCP 和 Unix 套接字在 Flutter web 上
不存在。装 `websockets` 或 `aiohttp` 会让「装不装 DDS 都能 import 客户端」这条
性质失效，那是一家之言换来的便利，不值。

只做网关真正用到的部分：

    握手         Upgrade: websocket，校验 Sec-WebSocket-Key
    数据帧       文本、二进制，服务端不分片；客户端来的分片帧拼起来
    控制帧       只处理 ping / pong / close
    长度         支持 7 位、16 位、64 位三种

**不做**：扩展协商（permessage-deflate）、子协议、客户端侧掩码（服务端不发掩码）。
少做这些的代价写在 `_read_frame` 的注释里：遇到不认识的 opcode 直接断连接，
而不是猜。

## 线程模型

一条连接 = 一个 socket。`recv` 带超时，调用方用它来定期回头看停止标志；
`send_*` 内部有锁，因为广播线程和命令应答会在同一时刻写同一个 socket ——
两段字节交错写进去，对端会看到一帧被劈开，表现是「画面花屏、状态乱跳」。
"""

from __future__ import annotations

import base64
import hashlib
import select
import socket
import struct
import threading

# 握手用的固定串，RFC 6455 §1.3。
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

# 握手请求头上限。正常不到 1 KB，8 KB 只可能来自非 WebSocket 的东西连上来。
MAX_HEAD = 8 * 1024

# 单帧上限。摄像头 JPEG 一帧 20~60 KB，给到 4 MB 是为了留余量；超过就断。
MAX_FRAME = 4 * 1024 * 1024

# socket 的兜底超时。读的等待由 `select` 控制，这个值管的是「半帧卡住」和
# 「对端不读导致 sendall 堵死」—— 手机在弱网里两样都会发生。
IO_TIMEOUT = 5.0

OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


class HandshakeError(RuntimeError):
    """握手不合法。消息回给对端后断开。"""


# 从 socket 读满 n 字节。对端关连接返回 None（不是抛异常 —— 断开是正常事件）。
def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


# 读 HTTP 请求头，直到空行。返回值不做解析，交给 `_accept_key`。
def _read_head(sock: socket.socket) -> bytes:
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(1024)
        if not chunk:
            return b""
        buf += chunk
        if len(buf) > MAX_HEAD:
            raise HandshakeError(f"请求头超过 {MAX_HEAD} 字节")
    return buf.split(b"\r\n\r\n", 1)[0]


# 从请求头里取 Sec-WebSocket-Key，算回应值。缺 key 说明不是 WebSocket 请求。
def _accept_key(head: bytes) -> str:
    key = None
    for line in head.split(b"\r\n")[1:]:
        name, _, value = line.partition(b":")
        if name.strip().lower() == b"sec-websocket-key":
            key = value.strip()
            break
    if not key:
        raise HandshakeError("缺少 Sec-WebSocket-Key")
    digest = hashlib.sha1(key + GUID.encode()).digest()
    return base64.b64encode(digest).decode()


# 解析请求行里的路径，网关用它取 `?t=token`。
def _request_path(head: bytes) -> str:
    line = head.split(b"\r\n", 1)[0]
    parts = line.split(b" ")
    if len(parts) < 2:
        return "/"
    return parts[1].decode("latin-1")


class Conn:
    """一条已握手的连接。`recv` 超时返回 None，断线也返回 None。"""

    def __init__(self, sock: socket.socket, path: str = "/"):
        self.sock = sock
        self.path = path
        self._lock = threading.Lock()
        self._buf = b""
        self.closed = False
        self.frag_op = None
        self.frag_buf = b""
        # 断开原因，给人看的。对端发 CLOSE 和网线被拔都表现为「recv 返回 None」，
        # 日志里只写「离开」分不清是 App 主动退、还是连接被掐 —— 这两种要查的
        # 方向完全不同（前者看客户端，后者看网络）。
        self.bye = ""

    # -- 握手 ------------------------------------------------------------

    # 收升级请求并回 101。失败抛 HandshakeError，调用方负责关 socket。
    @classmethod
    def accept(cls, sock: socket.socket) -> Conn:
        sock.settimeout(10.0)
        head = _read_head(sock)
        if not head:
            raise HandshakeError("对端没有发请求头")
        first = head.split(b"\r\n", 1)[0]
        if b"upgrade" not in head.lower():
            raise HandshakeError(f"不是 WebSocket 升级请求：{first!r}")
        accept = _accept_key(head)
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        )
        sock.sendall(resp.encode())
        sock.settimeout(IO_TIMEOUT)
        return cls(sock, _request_path(head))

    # -- 发送 ------------------------------------------------------------

    # 打一帧。FIN=1，不分片；长度按三种编码挑最小的那种。
    def _send_frame(self, opcode: int, payload: bytes) -> None:
        if self.closed:
            return
        head = bytearray()
        head.append(0x80 | opcode)
        n = len(payload)
        if n < 126:
            head.append(n)
        elif n < 65536:
            head.append(126)
            head += struct.pack("!H", n)
        else:
            head.append(127)
            head += struct.pack("!Q", n)
        with self._lock:
            try:
                self.sock.sendall(bytes(head) + payload)
            except OSError:
                self.closed = True

    def send_text(self, text: str) -> None:
        self._send_frame(OP_TEXT, text.encode("utf-8"))

    def send_binary(self, data: bytes) -> None:
        self._send_frame(OP_BINARY, data)

    # 主动关：先发 close 帧，再关 socket。对端已经走了也不报错。
    def close(self, code: int = 1000, reason: str = "") -> None:
        if not self.closed:
            body = struct.pack("!H", code) + reason.encode("utf-8")
            self._send_frame(OP_CLOSE, body)
        self.closed = True
        try:
            self.sock.close()
        except OSError:
            pass

    # -- 接收 ------------------------------------------------------------

    # 读一帧，返回 (fin, opcode, payload)。超时抛 TimeoutError，断线返回 None。
    #
    # 用 `select` 等数据、而不是靠 `settimeout`：写这一头也在同一个 socket 上，
    # 读线程把超时设成 0.5 秒之后，广播线程发一帧 60 KB 的 JPEG 就只有 0.5 秒
    # 预算，弱网下必然失败 —— 症状是「画面隔几秒黑一次」，很难归因到这里。
    def _read_frame(self, timeout: float | None):
        if timeout is not None:
            ready = select.select([self.sock], [], [], timeout)[0]
            if not ready:
                raise TimeoutError
        head = _recv_exact(self.sock, 2)
        if head is None:
            self.bye = self.bye or "对端关闭连接（FIN/EOF）"
            return None

        b0, b1 = head[0], head[1]
        fin = bool(b0 & 0x80)
        opcode = b0 & 0x0F
        masked = bool(b1 & 0x80)
        n = b1 & 0x7F
        if n == 126:
            ext = _recv_exact(self.sock, 2)
            if ext is None:
                return None
            n = struct.unpack("!H", ext)[0]
        elif n == 127:
            ext = _recv_exact(self.sock, 8)
            if ext is None:
                return None
            n = struct.unpack("!Q", ext)[0]
        if n > MAX_FRAME:
            self.bye = f"帧过大（{n} > {MAX_FRAME}）"
            return None
        mask = _recv_exact(self.sock, 4) if masked else None
        if masked and mask is None:
            return None
        payload = _recv_exact(self.sock, n) if n else b""
        if payload is None:
            self.bye = self.bye or "帧读到一半对端断了"
            return None
        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return fin, opcode, payload

    # 读一条完整消息（把分片拼起来），返回 (opcode, payload)。
    #
    # 超时抛 TimeoutError（这一帧还没来，连接是好的），断线返回 None（对端走了）。
    # 两者混成一个返回值，调用方就没法区分「暂时没消息」和「客户端掉线」——
    # 前者要 `continue`，后者要停车并释放操作权，弄错就是狗一直往前走。
    def recv(self, timeout: float | None = None):
        while True:
            frame = self._read_frame(timeout)
            if frame is None:
                self.closed = True
                return None
            fin, opcode, payload = frame
            if opcode == OP_PING:
                self._send_frame(OP_PONG, payload)
                continue
            if opcode == OP_PONG:
                continue
            if opcode == OP_CLOSE:
                code, why = _parse_close(payload)
                self.bye = f"客户端关闭 {code}{(' ' + why) if why else ''}"
                self.close(code, why)
                return None
            if opcode not in (OP_TEXT, OP_BINARY):
                self.close(1003, "unsupported opcode")
                return None
            if not fin:
                self.frag_op = opcode
                self.frag_buf = payload
                continue
            if self.frag_op is not None:
                payload = self.frag_buf + payload
                opcode = self.frag_op
                self.frag_op = None
                self.frag_buf = b""
            return opcode, payload


# close 帧的 body 是 2 字节状态码 + UTF-8 原因。空 body 是合法的。
def _parse_close(payload: bytes) -> tuple[int, str]:
    if len(payload) < 2:
        return 1000, ""
    code = struct.unpack("!H", payload[:2])[0]
    return code, payload[2:].decode("utf-8", "replace")
