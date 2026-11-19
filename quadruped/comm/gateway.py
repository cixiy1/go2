"""网关：把机器狗接到 App 上（手机 / 桌面 / 网页）。

    python -m quadruped.comm.gateway                    # 本机 8780，仿真预设
    python -m quadruped.comm.gateway --preset real --host 0.0.0.0

Flutter 客户端连 `ws://<这台机器>:8780/?t=<token>`。

## 为什么还要一层

`comm/server.py` 是「一条连接、一个客户端」的：第二个连接直接 409。手机、桌面
网页、平板同时看的时候，这条规则会把人挡在门外 —— 而遥控这件事本来就需要
「一个能动的 + 若干个只能看的」。

## 这个包里谁管什么

    child.py       子进程外壳（`comm/` 里的 Popen 只在这里）
    robot_link.py  到 comm.server 的连接，以及那个服务端的生死
    vla_host.py    VLA 子进程的生死 + 它的 stdin
    cam_host.py    相机馈送子进程的生死
    gateway.py     会话、操作权、模式闸门、广播、编排上面三个

**只有 gateway.py 同时持有三个 host。** host 之间互不 import：谁起谁停、
谁先谁后，全部写在 `_switch_manual` / `_switch_vla` / `_switch_off` 三个函数里。
想加第四条控制链路，要改的也是这三个函数，不是散在各处。

## 谁能让狗动

这是整个仓库最硬的一条边界，四类进程按「能不能写 `rt/lowcmd`」分开：

    能写    comm.server（持 Backend）        手动 / 脚本 / 网关转发
            VLA 子进程（PolicyBackend）      自主
            前台 main.py 起的控制器          调试
    只读    percept.feed（相机）             不碰腿
            gateway（本文件）                不碰 DDS
            Flutter 客户端                   只说话，不接线

前三个互斥：`rt/lowcmd` 同一时刻只允许一个发送者。所以网关的「切模式」不是换
一个 UI 状态，是交接控制权；`_switch_vla` 先 `park` 掉服务端再起 VLA，顺序反了
就是两个写者 —— 表现是腿一抽一抽，而不是报错。

`docs/control-ownership.md` 有完整的表，`test/arch_check.py` 规则 5~7 钉住其中
能静态检查的部分。

## 三条安全阀，一条都不许省

1. **操作权**：同一时刻只有一个 App 能发速度。别人要控制得先请求，被拒时
   回一句「iPhone 正在控制」。
2. **驱动看门狗**：`--setpoint-ttl` 毫秒内没收到新的设定值就 `stop`。手机锁屏、
   WiFi 掉、进程被杀，最坏结果是停下。
3. **断线即停**：持有操作权的客户端断开，立刻 `stop` 并释放操作权。

## 它不碰 DDS

网关进程里没有 `ChannelFactoryInitialize`。机器人要么是别人起的 `comm.server`
（网关连上去），要么是网关自己拉起来的子进程（退出时收掉）。唯一的写者是那个
服务端 —— 这也意味着**别同时开着 `main.py keyboard`**，那是第二个写者。
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import queue
import signal
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

from quadruped.limits import CROUCH_MAX, CROUCH_MIN, VX_MAX, VY_MAX, VYAW_MAX
from quadruped.comm import ws
from quadruped.comm.cam_host import CamHost
from quadruped.comm.client import RobotError
from quadruped.comm.protocol import DEFAULT_PORT, DEFAULT_SOCKET
from quadruped.comm.robot_link import POLL_HZ, RobotLink
from quadruped.comm.vla_host import VlaHost

# 状态帧广播频率。和机器人状态轮询同频 —— 轮询比广播慢的话，界面会看到
# 同一个数重复几帧；比它快就是白占 WiFi。
STATE_HZ = POLL_HZ

# 速度设定值的下发频率。和 App 侧的发帧频率解耦：App 可以 20 Hz 发，也可以
# 只在变化时发，这里统一按 20 Hz 转给机器人 —— 机器人那边每来一条 `move`
# 就重设一次目标速度，太密没意义，太稀会出现「一格一格」的顿挫。
SETPOINT_HZ = 20.0

# 设定值多久算过期。超过就 `stop`。
#
# 250 ms 是手感与安全的折中：再短，WiFi 抖一下狗就一顿一顿；再长，手机滑出
# 兜里之后它还会往前走一秒（vx 上限 1.0，就是 0.9 米）。
SETPOINT_TTL = 0.25

# 操作权持有者多久不说话就当作掉线。App 每 2 秒发一次心跳。
LEASE_IDLE = 6.0

# VLA 收尾等待。`runner.run()` 的 finally 要蹲下 → 阻尼 → 交还 mcf，真机上
# 十几秒是正常的。等不到才 SIGKILL，那条路意味着腿没阻尼。
VLA_STOP_GRACE = 25.0

# 单实例锁。两个网关 = 两条到机器人的连接 = 第二个会被 409 挡掉，但用户看到
# 的是「App 连不上」，而不是「你已经开了一个网关」。
LOCK = Path("/tmp/go2-gateway.lock")

# 二进制帧的第一个字节是来源标记。App 用它在画面上标「VLA 渲染」还是「机载相机」：
# 两者都是相机图，但前者叠了想法字幕和视野条，拿它当「实景」判断会误读。
FRAME_VLA = 1
FRAME_CAM = 2

# 高层控制端的名字。这三个字符串是「参数形状」，不是「这个端点能不能用」——
# 能不能用由服务端 `status` 里的 `clients` 回报（仿真只有 ours/policy，真机三个都有）。
# 两份不合并的原因：这一层不许 import 控制栈（arch_check 规则 2），而
# `CLIENTS` 表在 `robot/body.py` 里，import 它等于把 SDK 拖进通讯层。
CONTROLLERS = ("official", "ours", "policy")
CONTROLLER_LABEL = {
    "official": "官方控制器",
    "ours": "自研控制器",
    "policy": "推理模型控制器",
}

# 允许 App 直接转发的命令。`forward` / `turn` / `park` 故意不在里面：
# 前两个是阻塞调用（几秒），会把状态轮询卡住；`park` 会关掉服务端，那是
# 网关自己的事，不该由手机决定。
ALLOWED = ("stand", "uncrouch", "crouch", "stop", "freeze", "jump", "gait", "move",
           # 官方控制端的姿态/表演动作统一走这一条，具体名字由 server 的白名单
           # `ACTS` 收口（见 `comm/server.py`）。这里不逐个列，否则每加一个动作
           # 要改两处，漏一处就是「客户端点了没反应」。
           "act")


# 把浮点夹到区间里。NaN 也当越界处理 —— NaN 比较恒为 False，不显式判会漏过去。
def _clamp(value: float, lo: float, hi: float) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if v != v:
        return 0.0
    return lo if v < lo else (hi if v > hi else v)


# 循环地址判断：只监听回环时可以不设 token，监听 0.0.0.0 必须设。
def _loopback(host: str) -> bool:
    return host in ("127.0.0.1", "localhost", "::1")


class Client:
    """一个 App 连接。`send` 里带锁，广播线程和应答可能同时写。"""

    def __init__(self, cid: int, conn: ws.Conn, addr: str):
        self.id = cid
        self.conn = conn
        self.addr = addr
        self.name = "App"
        self.hello = False
        self.camera = True
        self.last_seen = time.monotonic()
        self.lease = False
        self._lock = threading.Lock()

    def send(self, obj: dict) -> None:
        try:
            line = json.dumps(obj, ensure_ascii=False, default=float)
        except (TypeError, ValueError):
            return
        with self._lock:
            self.conn.send_text(line)

    def send_bytes(self, data: bytes) -> None:
        with self._lock:
            self.conn.send_binary(data)

    @property
    def closed(self) -> bool:
        return self.conn.closed


class Gateway:
    """会话、操作权、模式、广播。所有跨线程状态都在 `self._lock` 下。"""

    def __init__(self, *, host: str, port: int, token: str, socket_path: str,
                 preset: str, robot_manage: str, host_flag: str | None,
                 client: str | None = None, iface: str | None = None,
                 events_port: int = 0, camera: bool = True,
                 cam_hz: float = 30.0, verbose: bool = True):
        self.host = host
        self.port = port
        self.token = token
        self.verbose = verbose
        self.started = time.monotonic()
        self.clients: dict[int, Client] = {}
        self._lock = threading.RLock()
        self._next_id = 1
        self.mode = "off"                # off / switching / manual / vla
        self.mode_note = ""
        self.lease: int | None = None    # 持有操作权的 client id
        self.setpoint = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
        self.setpoint_ts = 0.0
        self.setpoint_src = "-"
        self.driving = False
        self.robot: RobotLink | None = None
        self.iface = iface
        self._robot_args = dict(socket_path=socket_path, host=host_flag,
                                port=DEFAULT_PORT, preset=preset,
                                manage=robot_manage, client=client,
                                iface=iface, echo=verbose)
        # VLA 事件从 UDP 进来，端口由内核分配，再告诉子进程。
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 端口给 0 由内核分配（默认）。给固定端口是为了让「VLA 跑在另一台机器
        # 上」也能把事件发过来 —— 那时候网关不知道对方的端口，对方也不知道
        # 网关的，必须约定一个。
        self._udp.bind(("127.0.0.1", int(events_port or 0)))
        self._udp.settimeout(0.5)
        self.event_port = self._udp.getsockname()[1]
        self.vla = VlaHost(preset=preset, event_port=self.event_port,
                           echo=verbose, iface=iface)
        # 手动模式下的相机馈送。VLA 在跑时不用它 —— 那一侧自己就在取相机。
        self.camera = bool(camera)
        self.cam = CamHost(preset=preset, hz=cam_hz, echo=verbose, iface=iface)
        self.vla_event: dict = {}
        self.vla_replies: deque[dict] = deque(maxlen=40)
        # 雷达：由 `percept.feed` 写 JSON，这里按 mtime 变化读。
        # 独立于 VLA —— 手动模式下没有 VLA 进程，雷达照样要有。
        self._radar_cache: dict = {}
        self._radar_mtime = 0.0
        self._radar_read_at = 0.0
        self._jobs: queue.Queue = queue.Queue()
        self.stopped = threading.Event()
        self._stream_mtime = 0.0
        self._stream_key = ""            # 换源时 mtime 的比较基准要重置
        self._listener: socket.socket | None = None

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[gateway] {time.strftime('%H:%M:%S')} {msg}", flush=True)

    # -- 广播 ------------------------------------------------------------

    def _broadcast(self, obj: dict, skip: Client | None = None) -> None:
        with self._lock:
            targets = list(self.clients.values())
        for c in targets:
            if c is skip or c.closed:
                continue
            c.send(obj)

    def _event(self, kind: str, text: str, **extra) -> None:
        self._broadcast(dict({"op": "event", "kind": kind, "text": text}, **extra))
        self._log(f"{kind}: {text}")

    # -- 状态帧 ----------------------------------------------------------

    # 雷达是感知层的东西，跟 VLA 没关系。原来 `sectors` 只在 VLA 帧里
    # （`control/embodied/runner.py`），手动模式没有 VLA 进程，雷达图就永远空白 ——
    # 而遥控最需要看四周。现在由 `percept.feed` 单独写一份 JSON，这里读进来。
    #
    # 按 mtime 变化才读，10 Hz 的状态循环里一次 stat 是微秒级。
    def _radar(self) -> dict:
        path = self.cam.radar_path
        try:
            mtime = path.stat().st_mtime
        except OSError:
            return {"ok": False, "why": "雷达没起来", "sectors": []}
        if mtime != self._radar_mtime:
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as err:
                return {"ok": False, "why": f"雷达文件读不了：{type(err).__name__}",
                        "sectors": []}
            self._radar_mtime = mtime
            self._radar_cache = data
        if not self._radar_cache:
            return {"ok": False, "why": "雷达还没出数据", "sectors": []}
        out = dict(self._radar_cache)
        # 超过 1.5 秒没更新就当断了：拿旧数据画雷达比「没有雷达」更危险，
        # 人会以为那是当前四周。
        out["age"] = round(max(0.0, time.time() - float(out.get("t") or 0)), 2)
        if out["age"] > 1.5:
            out["ok"] = False
            out["why"] = f"雷达数据 {out['age']:.1f} 秒没更新"
        return out

    def _state(self, client: Client) -> dict:
        with self._lock:
            holder = self.lease
            mode = self.mode
            note = self.mode_note
            sp = dict(self.setpoint)
            sp_ts = self.setpoint_ts
            src = self.setpoint_src
            n = len(self.clients)
        robot = self.robot
        rs = robot.status if robot is not None else {}
        age = None if not sp_ts else round((time.monotonic() - sp_ts) * 1000.0)
        return {
            "op": "state",
            "t": round(time.time(), 3),
            "mode": mode,
            "note": note,
            "lease": {"holder": holder, "mine": holder == client.id,
                      "clients": n},
            "link": {
                "ok": robot is not None and robot.error is None and bool(rs),
                "error": None if robot is None else robot.error,
                "preset": None if robot is None else robot.preset,
                "target": rs.get("target"),
                "rtt_ms": None if robot is None else robot.rtt_ms,
                # 实际生效的控制端（服务端说的）与这个端点可选的列表。
                "controller": None if robot is None else robot.controller,
                "available": [] if robot is None else robot.available,
            },
            "robot": {
                "pos": rs.get("pos"),
                "rpy": rs.get("rpy"),
                "height": rs.get("height"),
                "gait": rs.get("gait"),
                "busy": rs.get("busy"),
                "moving": rs.get("moving"),
                "loop": rs.get("loop"),
                "error": rs.get("error"),
            },
            "input": {"vx": sp["vx"], "vy": sp["vy"], "vyaw": sp["vyaw"],
                      "age_ms": age, "source": src, "driving": self.driving},
            "vla": self.vla.status(),
            "radar": self._radar(),
            "cam": self.cam.status(),
            "limits": {"vx": VX_MAX, "vy": VY_MAX,
                       "vyaw": VYAW_MAX,
                       "crouch": [CROUCH_MIN, CROUCH_MAX]},
            "server": {"uptime": round(time.monotonic() - self.started, 1),
                       "proto": 1},
        }

    def _state_loop(self) -> None:
        period = 1.0 / STATE_HZ
        tick = time.perf_counter()
        while not self.stopped.is_set():
            with self._lock:
                targets = list(self.clients.values())
            for c in targets:
                if not c.hello or c.closed:
                    continue
                c.send(self._state(c))
            self._reap_lease()
            tick += period
            rest = period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()

    # 操作权持有者久不说话就收回。手机锁屏、WiFi 掉线都不会发 FIN，
    # 只靠「断开」判是判不出来的。
    def _reap_lease(self) -> None:
        with self._lock:
            holder = self.lease
            client = self.clients.get(holder) if holder is not None else None
            stale = client is None or time.monotonic() - client.last_seen > LEASE_IDLE
        if holder is None:
            return
        if stale:
            self._stop_motion()
            self._drop_lease(f"客户端 {LEASE_IDLE:.0f} 秒没心跳，收回操作权")

    # -- 驱动 ------------------------------------------------------------

    # 把当前设定值转给机器人。20 Hz，只在有变化或需要心跳时发。
    def _drive_loop(self) -> None:
        period = 1.0 / SETPOINT_HZ
        tick = time.perf_counter()
        last_sent = None
        last_ts = 0.0
        while not self.stopped.is_set():
            with self._lock:
                sp = dict(self.setpoint)
                ts = self.setpoint_ts
                mode = self.mode
            robot = self.robot
            fresh = ts > 0 and (time.monotonic() - ts) <= SETPOINT_TTL
            if robot is None or mode != "manual":
                self.driving = False
                last_sent = None
            elif not fresh:
                # 过期：发一次 stop，然后把 `driving` 清掉，别每帧都发。
                if self.driving:
                    try:
                        robot.call("move", vx=0.0, vy=0.0, vyaw=0.0, timeout=2.0)
                    except (OSError, RobotError, TimeoutError, socket.timeout, ConnectionError):
                        pass
                    self.driving = False
                    self._log("设定值过期，停车")
                last_sent = None
            else:
                key = (round(sp["vx"], 4), round(sp["vy"], 4), round(sp["vyaw"], 4))
                # 值没变也每隔 100 ms 重发一次：机器人服务端有 3 秒空闲停车，
                # 而 App 那边可能因为「摇杆没动」而不发帧。
                if key != last_sent or time.monotonic() - last_ts > 0.1:
                    try:
                        robot.call("move", vx=key[0], vy=key[1], vyaw=key[2],
                                   timeout=2.0)
                        last_sent = key
                        last_ts = time.monotonic()
                        self.driving = any(abs(v) > 1e-6 for v in key)
                    except (OSError, RobotError, TimeoutError, socket.timeout, ConnectionError) as err:
                        self._log(f"下发速度失败：{err}")
                        self.driving = False
            tick += period
            rest = period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()

    def _stop_motion(self) -> None:
        with self._lock:
            self.setpoint = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
            self.setpoint_ts = 0.0
            self.driving = False
        robot = self.robot
        if robot is None:
            return
        try:
            robot.call("stop", timeout=3.0)
        except (OSError, RobotError, TimeoutError, socket.timeout, ConnectionError):
            pass

    # -- 操作权 ----------------------------------------------------------

    def _grant_lease(self, client: Client) -> dict:
        with self._lock:
            holder = self.lease
            if holder is not None and holder != client.id:
                other = self.clients.get(holder)
                name = other.name if other is not None else "另一台设备"
                return {"ok": False, "code": 409, "msg": f"{name} 正在控制"}
            self.lease = client.id
            client.lease = True
            client.last_seen = time.monotonic()
        self._broadcast({"op": "event", "kind": "lease",
                         "text": f"{client.name} 取得操作权", "holder": client.id})
        return {"ok": True, "holder": client.id}

    def _drop_lease(self, why: str = "") -> None:
        with self._lock:
            holder = self.lease
            self.lease = None
            if holder is not None and holder in self.clients:
                self.clients[holder].lease = False
        self._broadcast({"op": "event", "kind": "lease", "text": why or "操作权已释放",
                         "holder": None})

    # -- 模式 ------------------------------------------------------------

    # 切模式要几步、每步几秒（park 收尾、VLA 起来接管策略），不能在客户端
    # 读线程里做 —— 那会把这个客户端的应答卡住，看起来像网关死了。
    # 所以排进单线程队列，立刻回一句「在切了」。
    def request_mode(self, target: str) -> dict:
        if target not in ("manual", "vla", "off"):
            return {"ok": False, "code": 400, "msg": f"没有这个模式：{target}"}
        with self._lock:
            if self.mode == target:
                return {"ok": True, "mode": target, "already": True}
            self.mode = "switching"
            self.mode_note = f"切到{'手动' if target == 'manual' else 'VLA'}…"
        self._stop_motion()
        self._jobs.put(("mode", target))
        return {"ok": True, "switching": True, "target": target}

    def _job_loop(self) -> None:
        while not self.stopped.is_set():
            try:
                work = self._jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                action, arg = work
                if action == "mode":
                    if arg == "manual":
                        self._switch_manual()
                    elif arg == "vla":
                        self._switch_vla()
                    else:
                        self._switch_off()
                else:
                    self._switch_controller(arg)
            except Exception as err:                       # noqa: BLE001
                with self._lock:
                    self.mode = "off"
                    self.mode_note = f"切换失败：{type(err).__name__}: {err}"
                self._event("error", self.mode_note)

    # 手动：先把 VLA 收掉，再连机器人。
    #
    # 顺序不能反。VLA 还在跑的时候连上去，管理论上有两个写者；实际上
    # `comm.server` 的 `connect()` 会 BalanceStand，和策略抢腿。
    def _switch_manual(self) -> None:
        if self.vla.running:
            self._event("mode", "正在停止 VLA（蹲下 → 阻尼 → 交还）…")
            self.vla.stop()
        robot = self.robot
        if robot is None:
            robot = RobotLink(**self._robot_args)
            self.robot = robot
        robot.open()
        if self.camera:
            self.cam.start()
        with self._lock:
            self.mode = "manual"
            self.mode_note = ""
        self._event("mode", "手动遥控已就绪")
        self._publish_vla_event({})

    # VLA：先把机器人服务端收掉（park），再起 VLA。
    #
    # `park` 是远端关停：停步 → 蹲下 → 阻尼 → 服务端退出、套接字消失。
    # 只断开连接是不够的 —— 那个进程还握着 `rt/lowcmd`。
    def _switch_vla(self) -> None:
        # 先停相机馈送：真机上 videohub 只认一个订阅者，不停掉 VLA 就拿不到帧
        # （`GetImageSample` 返回 code != 0，画面一片空白，日志里什么都不说）。
        self.cam.stop()
        robot = self.robot
        if robot is not None:
            self._event("mode", "正在交还机器人控制权（park）…")
            robot.shutdown()
            self.robot = None
        out = self.vla.start()
        with self._lock:
            self.mode = "vla"
            self.mode_note = ""
        self._event("mode", "VLA 已启动，自主接管" if out.get("ok") else "VLA 起不来")

    # 切控制端。先过四道守卫，再排进作业队列（重启服务端要几秒）。
    #
    # 只在手动模式下允许：VLA 自己就是控制端的一种，切它没意义；off 模式下
    # 没有服务端可重启。
    def request_controller(self, kind: str) -> dict:
        if kind not in CONTROLLERS:
            return {"ok": False, "code": 400,
                    "msg": f"没有这个控制端：{kind}，可选 {'/'.join(CONTROLLERS)}"}
        with self._lock:
            mode = self.mode
        if mode != "manual":
            return {"ok": False, "code": 409,
                    "msg": f"当前是 {mode} 模式；切控制端要先切到手动遥控"}
        robot = self.robot
        if robot is None:
            return {"ok": False, "code": 503, "msg": "还没连上机器人"}
        if not robot.owns_server:
            return {"ok": False, "code": 409,
                    "msg": "服务端是你自己起的（--robot attach），换控制端要用它自己的 "
                           f"--client 重启。当前：{robot.controller or '未知'}"}
        allowed = robot.available
        if allowed and kind not in allowed:
            names = " / ".join(CONTROLLER_LABEL.get(k, k) for k in allowed)
            return {"ok": False, "code": 400,
                    "msg": f"这个端点只能用{names}"}
        if robot.controller == kind:
            return {"ok": True, "controller": kind, "already": True}
        # 换控制端要重启服务端：先停步、收回操作权。操作权不保留 —— 链路会断，
        # 保留一个「名义上还持有」的租约比收回更危险。
        self._stop_motion()
        self._drop_lease("切换控制端，操作权已收回")
        with self._lock:
            self.mode_note = f"正在切到{CONTROLLER_LABEL.get(kind, kind)}…"
        self._jobs.put(("controller", kind))
        return {"ok": True, "switching": True, "controller": kind}

    def _switch_controller(self, kind: str) -> None:
        robot = self.robot
        if robot is None:
            raise RuntimeError("机器人链路已经不在了")
        label = CONTROLLER_LABEL.get(kind, kind)
        self._event("mode", f"正在切换到{label}（服务端要重启，狗会先蹲下）")
        robot.restart(kind)
        with self._lock:
            self.mode_note = ""
        # 重启之后服务端会自己做 BalanceStand，但腿还在抬；这里不假装知道高度，
        # 让状态帧如实反映服务端报的值。
        self._event("mode", f"控制端已切到{label}")

    def _switch_off(self) -> None:
        self.cam.stop()
        if self.vla.running:
            self.vla.stop()
        robot = self.robot
        if robot is not None:
            robot.shutdown()
            self.robot = None
        self._stop_motion()
        with self._lock:
            self.mode = "off"
            self.mode_note = ""
        self._event("mode", "已收工")

    # -- VLA 事件 --------------------------------------------------------

    def _publish_vla_event(self, payload: dict) -> None:
        self.vla_event = payload
        self._broadcast({"op": "vla", "data": payload})

    def _udp_loop(self) -> None:
        while not self.stopped.is_set():
            try:
                data, _ = self._udp.recvfrom(65535)
            except (TimeoutError, socket.timeout):
                continue
            except OSError:
                return
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if payload.get("kind") != "vla":
                continue
            # 回话单独留一份：App 中途接进来也要看得到刚才说了什么。
            reply = payload.get("reply")
            if reply and (not self.vla_replies or
                          self.vla_replies[-1].get("seq") != payload.get("reply_seq")):
                self.vla_replies.append({"seq": payload.get("reply_seq"),
                                         "text": reply, "t": payload.get("t")})
            self._publish_vla_event(payload)

    # -- 画面 ------------------------------------------------------------

    # VLA 把窗口渲染成 JPEG 写到文件（`--stream`），这里按 mtime 变化推给 App。
    # 用文件而不是套接字：那一侧已经在写这个文件了，多一条通道就多一处要同步。
    # 画面转发。两个源，一个出口：
    #
    #   VLA 在跑   → 它渲染的窗口（带想法字幕、视野条）
    #   手动模式   → `percept.feed` 的纯相机图
    #
    # 帧前加一字节标明来源，App 据此决定标签。混在一起的话，用户会以为
    # VLA 那张带字幕的合成图就是机载实景。
    def _camera_loop(self) -> None:
        while not self.stopped.is_set():
            # 4 ms 一次 stat：转发延迟直接决定「画面看起来跟不跟手」。
            # 原来睡 0.15 s，光这一层就把帧率封在 6.6 Hz —— 60 fps 的源也只剩 6 帧。
            # 一次 stat 是微秒级的，200 Hz 轮询的代价可以忽略。
            time.sleep(0.004)
            if not self.camera:
                continue
            if self.vla.running:
                path, kind = self.vla.stream_path, FRAME_VLA
                key = "vla"
            elif self.cam.running:
                path, kind = self.cam.stream_path, FRAME_CAM
                key = "cam"
            else:
                continue
            if key != self._stream_key:
                # 换源：新文件的 mtime 可能比旧源小，不重置就永远不发第一帧。
                self._stream_key = key
                self._stream_mtime = 0.0
            try:
                mtime = path.stat().st_mtime
            except OSError:
                continue
            if mtime <= self._stream_mtime:
                continue
            try:
                data = path.read_bytes()
            except OSError:
                continue
            if len(data) < 128:
                continue
            self._stream_mtime = mtime
            with self._lock:
                targets = [c for c in self.clients.values() if c.camera]
            for c in targets:
                if not c.closed:
                    c.send_bytes(bytes((kind,)) + data)

    # -- 会话 ------------------------------------------------------------

    def _on_text(self, client: Client, text: str) -> None:
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            client.send({"op": "error", "msg": "不是合法 JSON"})
            return
        if not isinstance(msg, dict):
            return
        client.last_seen = time.monotonic()
        op = msg.get("op")
        rid = msg.get("id")

        if op == "hello":
            if not self._check_token(msg.get("token")):
                client.send({"op": "error", "code": 401, "msg": "token 不对",
                             "id": rid})
                client.conn.close(4001, "bad token")
                return
            client.hello = True
            client.name = str(msg.get("name") or "App")[:24]
            with self._lock:
                holder = self.lease
            client.send({
                "op": "welcome",
                "proto": 1,
                "server": {"uptime": round(time.monotonic() - self.started, 1)},
                "lease": {"holder": holder, "mine": holder == client.id},
                "limits": {"vx": VX_MAX, "vy": VY_MAX,
                           "vyaw": VYAW_MAX},
                "mode": self.mode,
                "vla": self.vla.status(),
            "cam": self.cam.status(),
                "vla_last": self.vla_event,
                "dialog": list(self.vla_replies),
            })
            self._log(f"{client.name} 接入（{client.addr}，共 {len(self.clients)} 个）")
            return

        if not client.hello:
            client.send({"op": "error", "code": 401, "msg": "先发 hello", "id": rid})
            return

        try:
            out = self._dispatch(client, op, msg)
        except (OSError, RobotError, TimeoutError, socket.timeout, ConnectionError) as err:
            out = {"ok": False, "code": 503, "msg": f"{type(err).__name__}: {err}"}
        if rid is not None:
            client.send(dict({"op": "ack", "id": rid}, **out))

    def _dispatch(self, client: Client, op: str, msg: dict) -> dict:
        if op == "ping":
            return {"ok": True, "t": msg.get("ts")}

        if op == "lease":
            action = msg.get("action")
            if action == "acquire":
                return self._grant_lease(client)
            if action == "release":
                with self._lock:
                    mine = self.lease == client.id
                if mine:
                    self._stop_motion()
                    self._drop_lease("操作权已交还")
                return {"ok": True}
            return {"ok": False, "code": 400, "msg": "action 只能是 acquire/release"}

        if op == "setpoint":
            with self._lock:
                holder = self.lease
            if holder != client.id:
                return {"ok": False, "code": 403, "msg": "没有操作权"}
            if self.mode != "manual":
                return {"ok": False, "code": 409,
                        "msg": f"当前是 {self.mode} 模式，手动遥控要先切过去"}
            vx = _clamp(msg.get("vx", 0.0), -VX_MAX, VX_MAX)
            vy = _clamp(msg.get("vy", 0.0), -VY_MAX, VY_MAX)
            vyaw = _clamp(msg.get("vyaw", 0.0), -VYAW_MAX, VYAW_MAX)
            with self._lock:
                self.setpoint = {"vx": vx, "vy": vy, "vyaw": vyaw}
                self.setpoint_ts = time.monotonic()
                self.setpoint_src = str(msg.get("source") or "-")[:16]
            return {"ok": True, "vx": vx, "vy": vy, "vyaw": vyaw}

        if op == "cmd":
            cmd = str(msg.get("cmd") or "")
            if cmd not in ALLOWED:
                return {"ok": False, "code": 400,
                        "msg": f"App 不能直接发 {cmd}；可用：{'/'.join(ALLOWED)}"}
            with self._lock:
                holder = self.lease
            if holder != client.id:
                return {"ok": False, "code": 403, "msg": "没有操作权"}
            if self.mode not in ("manual", "vla"):
                return {"ok": False, "code": 409, "msg": f"当前是 {self.mode}"}
            if self.mode == "vla" and cmd not in ("stop", "freeze"):
                return {"ok": False, "code": 409, "msg": "VLA 模式下只能急停"}
            robot = self.robot
            if robot is None:
                return {"ok": False, "code": 503, "msg": "还没连上机器人"}
            data = robot.call(cmd, **(msg.get("args") or {}))
            return {"ok": True, "data": data}

        if op == "controller":
            return self.request_controller(str(msg.get("kind") or ""))

        if op == "mode":
            target = str(msg.get("mode") or "")
            return self.request_mode(target)

        if op == "vla":
            action = msg.get("action")
            if action == "start":
                if self.mode != "vla":
                    return {"ok": False, "code": 409, "msg": "先切到 VLA 模式"}
                if not self.vla.running:
                    return {"ok": False, "code": 409, "msg": "VLA 已退出，重新切换模式"}
                return {"ok": True, "vla": self.vla.status()}
            if action == "stop":
                with self._lock:
                    self.mode = "switching"
                    self.mode_note = "正在停止 VLA…"
                self._jobs.put(("mode", "off"))
                return {"ok": True, "stopping": True}
            return {"ok": False, "code": 400, "msg": "action 只能是 start/stop"}

        if op == "say":
            text = str(msg.get("text") or "").strip()
            if not text:
                return {"ok": False, "code": 400, "msg": "说点什么呢"}
            if not self.vla.running:
                return {"ok": False, "code": 409, "msg": "VLA 没在跑"}
            ok = self.vla.say(text)
            if not ok:
                return {"ok": False, "code": 503, "msg": "VLA 的 stdin 写不进去"}
            self.vla_replies.append({"seq": None, "text": text, "t": -1,
                                     "mine": True})
            return {"ok": True}

        if op == "camera":
            client.camera = bool(msg.get("on", True))
            return {"ok": True, "camera": client.camera}

        if op == "log":
            return {"ok": True, "lines": list(self.vla.log)[-int(msg.get("n") or 80):]}

        return {"ok": False, "code": 400, "msg": f"没有这个 op：{op}"}

    def _check_token(self, given) -> bool:
        if not self.token:
            return True
        return str(given or "") == self.token

    def _serve(self, client: Client) -> None:
        try:
            while not self.stopped.is_set() and not client.closed:
                try:
                    got = client.conn.recv(timeout=0.5)
                except (TimeoutError, socket.timeout):
                    continue
                except OSError:
                    break
                if got is None:
                    break
                opcode, payload = got
                if opcode == ws.OP_TEXT:
                    try:
                        self._on_text(client, payload.decode("utf-8"))
                    except Exception as err:               # noqa: BLE001
                        self._log(f"处理消息失败：{type(err).__name__}: {err}")
                # 客户端不发二进制；收到就丢，不回错误 —— 回了也没人看。
        finally:
            self._drop(client)

    def _drop(self, client: Client) -> None:
        with self._lock:
            self.clients.pop(client.id, None)
            was_holder = self.lease == client.id
        try:
            client.conn.close()
        except OSError:
            pass
        if was_holder:
            self._stop_motion()
            self._drop_lease(f"{client.name} 断开，停车")
        self._log(f"{client.name} 离开（剩 {len(self.clients)} 个）")

    # -- 入口 ------------------------------------------------------------

    def serve(self) -> int:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(8)
        listener.settimeout(0.5)
        self._listener = listener
        # 端口给 0 时由内核分配。测试和「随便找个空口」都靠这一行拿到真实端口。
        self.port = listener.getsockname()[1]
        where = f"ws://{self.host}:{self.port}/"
        print(f"网关监听 {where}", flush=True)
        if self.token:
            print(f"token：{self.token}", flush=True)
        else:
            print("token：未设置（只监听回环时可用）", flush=True)

        for target in (self._state_loop, self._drive_loop, self._udp_loop,
                       self._camera_loop, self._job_loop):
            threading.Thread(target=target, daemon=True,
                             name=target.__name__).start()

        try:
            while not self.stopped.is_set():
                try:
                    sock, addr = listener.accept()
                except (TimeoutError, socket.timeout):
                    continue
                except OSError:
                    break
                try:
                    conn = ws.Conn.accept(sock)
                except (ws.HandshakeError, OSError, TimeoutError, socket.timeout) as err:
                    self._log(f"握手失败（{addr[0]}）：{err}")
                    try:
                        sock.close()
                    except OSError:
                        pass
                    continue
                with self._lock:
                    cid = self._next_id
                    self._next_id += 1
                    client = Client(cid, conn, f"{addr[0]}:{addr[1]}")
                    self.clients[cid] = client
                threading.Thread(target=self._serve, args=(client,),
                                 daemon=True, name=f"client-{cid}").start()
        finally:
            self._shutdown()
        return 0

    def _shutdown(self) -> None:
        if self.stopped.is_set():
            return
        self.stopped.set()
        print("\n收工：停 VLA、还机器人", flush=True)
        self._stop_motion()
        self.cam.stop()
        if self.vla.running:
            self.vla.stop()
        if self.robot is not None:
            self.robot.shutdown()
            self.robot = None
        with self._lock:
            targets = list(self.clients.values())
        for c in targets:
            c.send({"op": "event", "kind": "bye", "text": "网关已关闭"})
            c.conn.close(1001, "gateway shutdown")
        try:
            self._udp.close()
        except OSError:
            pass
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass


# ---- 入口 --------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="机器狗 App 网关：WebSocket 多路复用 + 操作权 + VLA 托管")
    ap.add_argument("--host", default="127.0.0.1",
                    help="监听地址。给 0.0.0.0 才能让手机连进来（此时必须有 token）")
    ap.add_argument("--port", type=int, default=8780, help="监听端口")
    ap.add_argument("--token", default=os.environ.get("GO2_GATEWAY_TOKEN"),
                    help="接入口令。不给：回环口不校验，非回环自动生成")
    ap.add_argument("--socket", default=os.environ.get("GO2_COMM_SOCKET",
                                                       DEFAULT_SOCKET),
                    help="机器人服务端的 Unix 套接字路径")
    ap.add_argument("--preset", choices=("sim", "real"), default="sim",
                    help="仿真还是真机。网关自己要起服务端 / VLA 时用这个")
    ap.add_argument("--client", choices=CONTROLLERS, default=None,
                    help="高层控制端。不给：仿真 ours、真机 official（服务端自己挑）")
    # 网卡名。不写死：真机默认是 "en5"，那是 macOS 的编号；扩展坞上是 "eth10"。
    # 不给就按 preset 的默认值，走错网卡时 `require_iface` 会报「网卡不存在」，
    # 看起来像没插线，实际只是名字不对。
    ap.add_argument("--iface", default=os.environ.get("GO2_IFACE") or None,
                    help="DDS 走的网卡，默认按 preset 推（可用 GO2_IFACE 环境变量）")
    ap.add_argument("--robot", choices=("auto", "attach", "spawn"), default="auto",
                    help="auto=有服务端就连、没有就起；attach=只连不起；"
                         "spawn=总是自己起一个")
    ap.add_argument("--cam-hz", type=float, default=30.0,
                    help="相机馈送出图频率（1~60）。仿真那边要同步：SIM_VIDEO_HZ")
    ap.add_argument("--events-port", type=int, default=0,
                    help="VLA 事件（UDP）的固定端口，默认由内核分配")
    ap.add_argument("--no-camera", action="store_true", help="不转发 VLA 画面")
    ap.add_argument("--quiet", action="store_true", help="少打印")
    return ap


# 单实例锁。第二个网关不该抢同一个端口，更不该在 close 时把别人的服务端收掉。
def _lock() -> object | None:
    LOCK.parent.mkdir(parents=True, exist_ok=True)
    fh = open(LOCK, "w")
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fh.close()
        return None
    return fh


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not _loopback(args.host) and not args.token:
        # 这个接口能让狗动。监听外网又不要口令，等于把狗挂在公网上。
        print("监听非回环地址时必须给 --token（或 GO2_GATEWAY_TOKEN）",
              file=sys.stderr)
        return 2
    token = args.token
    if not token and _loopback(args.host):
        token = ""
    elif not token:
        token = f"{int(time.time()) % 1000000:06d}"

    holder = _lock()
    if holder is None:
        print("已经有一个网关在跑（/tmp/go2-gateway.lock）", file=sys.stderr)
        return 1

    gw = Gateway(host=args.host, port=args.port, token=token,
                 socket_path=args.socket, preset=args.preset,
                 robot_manage=args.robot, host_flag=None, client=args.client,
                 iface=args.iface,
                 events_port=args.events_port,
                 camera=not args.no_camera,
                 cam_hz=max(1.0, min(60.0, args.cam_hz)),
                 verbose=not args.quiet)

    def stop(signum, frame):                        # noqa: ARG001
        gw.stopped.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        return gw.serve()
    finally:
        holder.close()                              # type: ignore[union-attr]
        try:
            LOCK.unlink()
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
