"""网关自检：手写 WebSocket 服务端 × 真实客户端，加上一条假机器狗。

    .venv/bin/python test/gateway_check.py

## 为什么用真客户端

`ws.py` 是自己写的帧编解码。用自己写的客户端去测自己写的服务端，两边对同一个
错误理解一致，测试全绿而浏览器连不上 —— 这种「自证」在协议层没有任何价值。
所以这里用 `websockets`（第三方、独立实现）当客户端，它握手成功、帧收得对，
才说明网关能被 Dart / 浏览器接上。

## 假机器狗

`FakeRobot` 是一个只说 `comm/protocol.py` 的服务端：记下收到的每条命令，
按命令表回一个像样的响应。用它才能断言「网关到底把什么发给了机器人」——
这一层的 bug 全在「发了什么」上，而不是「有没有发」。

## 断言的是边界，不是功能

租约互斥、模式闸门、设定值过期、断线停车、非法命令拒绝。这些任意一条失效，
真机上表现为「狗不听使唤」或「两台手机抢着开」，而不是异常。
"""

from __future__ import annotations

import json
import os
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from websockets.sync.client import connect          # noqa: E402

from quadruped import limits                        # noqa: E402
from quadruped.comm import ws                       # noqa: E402
from quadruped.comm.gateway import (                # noqa: E402
    FRAME_CAM,
    FRAME_VLA,
    Gateway,
)
from quadruped.comm.protocol import encode, parse_request  # noqa: E402

FAILS: list[str] = []
CHECKS = [0]


def check(ok: bool, what: str) -> None:
    CHECKS[0] += 1
    if not ok:
        FAILS.append(what)
        print(f"  ✗ {what}")


def wait_for(pred, what: str, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    check(False, f"{what}（{timeout:.1f}s 内没等到）")
    return False


# ---- 假机器狗 ----------------------------------------------------------------


class FakeRobot:
    """说 `comm` 协议的服务端。命令记进 `calls`，`park` 之后自己关连接。"""

    def __init__(self, path: str):
        self.path = path
        self.calls: list[tuple[str, dict]] = []
        self.conns = 0
        self.parked = threading.Event()
        self._stop = threading.Event()
        self._sock: socket.socket | None = None

    def start(self) -> None:
        if os.path.exists(self.path):
            os.unlink(self.path)
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(self.path)
        sock.listen(2)
        sock.settimeout(0.2)
        self._sock = sock
        threading.Thread(target=self._accept, daemon=True, name="fake-robot").start()

    def _accept(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            self.conns += 1
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        conn.settimeout(0.2)
        buf = b""
        with conn:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(65536)
                except TimeoutError:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        rid, cmd, args = parse_request(line)
                    except Exception:                   # noqa: BLE001
                        conn.sendall(encode({"id": None, "ok": False, "code": 400,
                                             "msg": "bad", "data": {}}))
                        continue
                    self.calls.append((cmd, args))
                    conn.sendall(encode(self._reply(rid, cmd, args)))
                    if cmd == "park":
                        self.parked.set()
                        return

    def _reply(self, rid, cmd: str, args: dict) -> dict:
        data: dict = {}
        if cmd == "status":
            data = {"target": "fake", "pos": [0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
                    "height": 0.27, "busy": False, "gait": "trot",
                    "moving": False, "loop": "fake 200 Hz"}
        elif cmd == "move":
            data = {"moving": any(abs(args.get(k, 0)) > 0 for k in ("vx", "vy", "vyaw"))}
        elif cmd == "stop":
            data = {"moving": False}
        elif cmd == "gait":
            data = {"gait": args.get("style") or "bound"}
        return {"id": rid, "ok": True, "code": 0, "msg": "", "data": data}

    # 最近一条指定命令的参数；没有就是 None。
    def last(self, cmd: str):
        for name, args in reversed(self.calls):
            if name == cmd:
                return args
        return None

    def count(self, cmd: str) -> int:
        return sum(1 for name, _ in self.calls if name == cmd)

    def clear(self) -> None:
        self.calls.clear()

    def close(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        if os.path.exists(self.path):
            os.unlink(self.path)


# ---- 假 VLA ------------------------------------------------------------------


class FakeCam:
    """只当一个「有文件、在跑」的相机馈送，用来验网关的转发与来源标记。"""

    def __init__(self, path: Path):
        self.stream_path = path
        self.running = True
        self.starts = 0
        self.stops = 0

    def start(self):
        self.starts += 1
        self.running = True
        return {"ok": True, "pid": 2}

    def stop(self):
        self.stops += 1
        self.running = False

    def status(self) -> dict:
        return {"running": self.running, "pid": 2 if self.running else None}


class FakeVla:
    """不真起 VLA：这里要验的是「切换时谁被关掉了」，不是模型会不会走路。"""

    def __init__(self):
        self.running = False
        self.said: list[str] = []
        self.starts = 0
        self.stops = 0
        self.stream_path = Path(tempfile.gettempdir()) / "gateway-check-stream.jpg"
        self.log: list[str] = []

    def start(self, goal=None, scene=None):
        self.starts += 1
        self.running = True
        return {"ok": True, "pid": 1}

    def stop(self):
        self.stops += 1
        self.running = False
        return {"ok": True, "exit": 0}

    def say(self, text: str) -> bool:
        self.said.append(text)
        return self.running

    def status(self) -> dict:
        return {"running": self.running, "pid": 1 if self.running else None,
                "uptime": 0.0, "log": "fake", "stream": "fake"}


# ---- 客户端 ------------------------------------------------------------------


class Peer:
    """一条 App 连接。后台线程收消息，测试按需等某一条。"""

    def __init__(self, url: str, name: str = "peer"):
        self.name = name
        self.ws = connect(url, open_timeout=5, max_size=None)
        self.msgs: list[dict] = []
        self.binary: list[bytes] = []
        self.closed = threading.Event()
        self._lock = threading.Lock()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        try:
            while True:
                raw = self.ws.recv(timeout=30)
                with self._lock:
                    if isinstance(raw, bytes):
                        self.binary.append(raw)
                    else:
                        self.msgs.append(json.loads(raw))
        except Exception:                               # noqa: BLE001
            self.closed.set()

    def wait(self, pred, what: str, timeout: float = 3.0) -> dict | None:
        found: list[dict] = []

        def probe() -> bool:
            with self._lock:
                # 从新往旧找：同一类消息会反复出现（状态帧 10 Hz、vla 事件
                # 5 Hz），要找的是「最新那条满足条件的」。
                for m in reversed(self.msgs):
                    if pred(m):
                        found.append(m)
                        return True
            return False

        if not wait_for(probe, what, timeout):
            return None
        return found[0]

    def ack(self, rid: int, timeout: float = 3.0) -> dict | None:
        return self.wait(lambda m: m.get("op") == "ack" and m.get("id") == rid,
                         f"{self.name} 收到 ack {rid}", timeout)

    def state(self, timeout: float = 3.0) -> dict | None:
        return self.wait(lambda m: m.get("op") == "state",
                         f"{self.name} 收到状态帧", timeout)

    def send(self, obj: dict) -> None:
        self.ws.send(json.dumps(obj, ensure_ascii=False))

    def call(self, op: str, rid: int, **kw) -> dict | None:
        self.send(dict({"op": op, "id": rid}, **kw))
        return self.ack(rid)

    def close(self) -> None:
        try:
            self.ws.close()
        except Exception:                               # noqa: BLE001
            pass


# ---- 场景 --------------------------------------------------------------------


def test_codec() -> None:
    """握手 + 三种长度的帧。300 KB 走的是 64 位长度分支。"""
    print("帧编解码")
    port = _free_port()
    accepted: list[ws.Conn] = []
    ready = threading.Event()

    def serve() -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", port))
        srv.listen(1)
        ready.set()
        conn, _ = srv.accept()
        try:
            c = ws.Conn.accept(conn)
            accepted.append(c)
            while True:
                got = c.recv(timeout=5)
                if got is None:
                    return
                opcode, payload = got
                if opcode == ws.OP_TEXT:
                    c.send_text("echo:" + payload.decode())
                else:
                    c.send_binary(payload)
        finally:
            srv.close()

    threading.Thread(target=serve, daemon=True).start()
    ready.wait(2)
    with connect(f"ws://127.0.0.1:{port}/probe?t=abc", open_timeout=5,
                 max_size=None) as client:
        client.send("你好")
        check(client.recv(timeout=3) == "echo:你好", "文本帧原样回显")
        blob = bytes(range(256)) * 1200                 # 307200 字节
        client.send(blob)
        got = client.recv(timeout=5)
        check(isinstance(got, bytes) and got == blob, "300 KB 二进制帧完整")
        client.ping()
    check(accepted and accepted[0].path.startswith("/probe"),
          "请求路径被解析出来（token 挂在 query 上）")


def test_gateway(fake: FakeRobot) -> None:
    print("网关")
    port = _free_port()
    token = "s3cret"
    gw = Gateway(host="127.0.0.1", port=port, token=token,
                 socket_path=fake.path, preset="sim", robot_manage="attach",
                 host_flag=None, verbose=False)
    gw.vla = FakeVla()                                 # type: ignore[assignment]
    cam_path = Path(tempfile.mkdtemp(prefix="go2-cam-")) / "cam.jpg"
    gw.cam = FakeCam(cam_path)                         # type: ignore[assignment]
    threading.Thread(target=gw.serve, daemon=True, name="gateway").start()
    wait_for(lambda: gw.port != 0, "网关开始监听")
    url = f"ws://127.0.0.1:{gw.port}/?t={token}"

    # 口令
    bad = Peer(url.replace(token, "wrong"), "bad")
    bad.send({"op": "hello", "token": "wrong"})
    err = bad.wait(lambda m: m.get("op") == "error", "错口令被拒")
    check(err is not None and err.get("code") == 401, "错口令回 401")
    wait_for(lambda: bad.closed.is_set(), "错口令连接被关")

    a = Peer(url, "A")
    a.send({"op": "hello", "token": token, "name": "手机"})
    welcome = a.wait(lambda m: m.get("op") == "welcome", "A 收到 welcome")
    check(welcome is not None, "hello 之后拿到 welcome")
    if welcome:
        check(abs(welcome["limits"]["vx"] - limits.VX_MAX) < 1e-9,
              "welcome 带上限，App 不用猜 VX_MAX")

    # 操作权
    check(a.call("lease", 1, action="acquire")["ok"], "A 取得操作权")
    b = Peer(url, "B")
    b.send({"op": "hello", "token": token, "name": "平板"})
    b.wait(lambda m: m.get("op") == "welcome", "B 收到 welcome")
    denied = b.call("lease", 1, action="acquire")
    check(denied is not None and denied["code"] == 409, "B 抢不到操作权（409）")
    check("手机" in denied["msg"], "拒绝信息里带上是谁在控制")

    # 模式闸门：还没进手动，速度指令要被拒
    refused = a.call("setpoint", 2, vx=0.2)
    check(refused is not None and refused["code"] == 409,
          "非手动模式下拒绝速度指令")

    # 切手动
    check(a.call("mode", 3, mode="manual")["ok"], "请求切手动")
    wait_for(lambda: gw.mode == "manual", "网关进入手动模式", timeout=10.0)
    check(fake.conns >= 1, "手动模式下连上了机器人")
    check(a.call("lease", 4, action="acquire")["ok"], "切换后重新取操作权")

    # 画面转发：手动模式下来自相机馈送，帧首字节标来源
    payload = b"\xff\xd8" + b"\x00" * 300
    cam_path.write_bytes(payload)
    check(wait_for(lambda: len(a.binary) > 0, "收到第一帧画面", timeout=2.0),
          "手动模式下把相机馈送的帧推给 App")
    if a.binary:
        check(a.binary[-1][0] == FRAME_CAM, "手动模式的帧标成「相机」")
        check(a.binary[-1][1:] == payload, "帧内容原样（只加了来源字节）")

    # 速度下发与夹取
    fake.clear()
    out = a.call("setpoint", 5, vx=9.0, vy=0.0, vyaw=0.0, source="gamepad")
    check(out is not None and abs(out["vx"] - limits.VX_MAX) < 1e-6,
          "超过上限的速度被夹到 VX_MAX")
    check(wait_for(lambda: fake.last("move") is not None, "机器人收到 move"),
          "设定值转成了 move")
    mv = fake.last("move") or {}
    check(abs(mv.get("vx", 0) - limits.VX_MAX) < 1e-6, "move 的 vx 是夹取后的值")

    # 过期停车
    fake.clear()
    check(wait_for(lambda: (fake.last("move") or {}).get("vx") == 0.0,
                   "设定值过期后发零速", timeout=1.5), "250 ms 没新设定值就停车")

    # 命令白名单与租约
    check(a.call("cmd", 6, cmd="park")["code"] == 400, "App 不能直接 park")
    check(b.call("cmd", 2, cmd="jump")["code"] == 403, "没有操作权不能发命令")
    fake.clear()
    check(a.call("cmd", 7, cmd="jump")["ok"], "有操作权可以 jump")
    check(wait_for(lambda: fake.count("jump") == 1, "机器人收到 jump"),
          "命令真的转给了机器人")

    # VLA 事件（UDP → WS）
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = {"kind": "vla", "t": 1.5, "cmd": {"vx": 0.3}, "front_gap": 1.1,
               "sectors": [{"name": "正前", "gap": 1.1, "top": 0.0}],
               "thought": "左边空着", "reply": "我往左走", "reply_seq": 1,
               "seq": 1}
    sock.sendto(json.dumps(payload, ensure_ascii=False).encode(),
                ("127.0.0.1", gw.event_port))
    evt = a.wait(lambda m: m.get("op") == "vla"
                 and (m.get("data") or {}).get("thought") == "左边空着",
                 "A 收到 VLA 事件")
    check(evt is not None and evt["data"]["thought"] == "左边空着",
          "VLA 事件原样转给 App")

    # 切 VLA：机器人必须被 park 掉（否则两个写者抢 lowcmd）
    fake.parked.clear()
    check(a.call("mode", 8, mode="vla")["ok"], "请求切 VLA")
    wait_for(lambda: gw.mode == "vla", "网关进入 VLA 模式", timeout=20.0)
    check(fake.parked.is_set(), "切 VLA 前把机器人服务端 park 掉了")
    check(gw.robot is None, "切 VLA 后网关不再持有机器人")
    check(a.call("setpoint", 9, vx=0.2)["code"] == 409, "VLA 模式下拒绝手动速度")
    check(gw.cam.stops >= 1, "切 VLA 时停掉了相机馈送")
    vla_payload = b"\xff\xd8" + b"\x11" * 300
    a.binary.clear()
    gw.vla.stream_path.write_bytes(vla_payload)
    check(wait_for(lambda: len(a.binary) > 0, "收到 VLA 画面", timeout=2.0),
          "VLA 模式下推它的渲染窗口")
    if a.binary:
        check(a.binary[-1][0] == FRAME_VLA, "VLA 模式的帧标成「渲染」")
    check(a.call("say", 10, text="往前走")["ok"], "VLA 在跑时可以说话")
    check(gw.vla.said == ["往前走"], "话真的进了 VLA 的 stdin")

    # 回手动：VLA 收掉，机器人重新连上
    check(a.call("mode", 11, mode="manual")["ok"], "请求切回手动")
    wait_for(lambda: gw.mode == "manual", "网关回到手动模式", timeout=20.0)
    check(gw.vla.stops == 1, "切回手动时把 VLA 停掉了")
    check(fake.conns >= 2, "手动模式重新连上机器人")

    # 断线：持有操作权的人走了要停车
    a.call("lease", 12, action="acquire")
    a.call("setpoint", 13, vx=0.3)
    fake.clear()
    a.close()
    check(wait_for(lambda: (fake.last("move") or fake.last("stop")) is not None,
                   "A 断开后机器人收到停车", timeout=3.0), "持有者断线即停车")
    le = b.wait(lambda m: m.get("op") == "event" and m.get("kind") == "lease"
                and m.get("holder") is None, "B 看到租约释放", timeout=3.0)
    check(le is not None, "断开后操作权被释放")

    b.close()
    gw.stopped.set()
    time.sleep(0.4)


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def main() -> int:
    tmpdir = tempfile.mkdtemp(prefix="go2-gw-")
    fake = FakeRobot(os.path.join(tmpdir, "robot.sock"))
    fake.start()
    try:
        test_codec()
        test_gateway(fake)
    finally:
        fake.close()
    print()
    if FAILS:
        print(f"{len(FAILS)} 项失败 / 共 {CHECKS[0]} 项")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print(f"全部通过（{CHECKS[0]} 项）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
