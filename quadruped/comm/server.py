"""机器人服务端：常驻进程，独占 DDS 与 rt/lowcmd，对外提供套接字接口。

    python -m quadruped.comm.server                          # 连仿真（lo0 / domain 1）
    python -m quadruped.comm.server --preset real --kd 3.0    # 连真机，网卡自动挑
    python -m quadruped.comm.server --host 0.0.0.0            # 换 TCP，给另一台机器用

## 它不做什么

不排队、不规划、不做决策。决策在外部（Yuki、脚本、网页后端），这里只做一件事：
把收到的命令转给 `Backend`。**同一时刻只允许一个客户端**，第二个连接直接回 409 ——
两台控制器对同一台狗下指令一定会打架，这条在 `launcher.py` 的冲突检查里已经立过一次。

## 三个安全阀

1. **空闲停车**：有速度指令但 `--idle-stop` 秒内没收到任何请求，自动 `stop`。
   客户端崩了、网断了，最坏结果是停下，不会是「一直走」。
2. **断开即停**：客户端连接一断，立刻 `stop`（不是 park —— 每次断线都蹲下太难用）。
3. **退出才 park**：Ctrl+C / SIGTERM 才停步 → 蹲下 → 阻尼。异常退出不会从 27 cm
   直接掉下来。

关闭空闲停车用 `--idle-stop 0`，但只在调试时这么做。
"""

import argparse
import os
import signal
import socket
import sys
import threading
import time

from quadruped.comm.protocol import (
    API_UNSUPPORTED,
    BAD_REQUEST,
    BUSY,
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_SOCKET,
    MAX_LINE,
    ProtocolError,
    error,
    ok,
    parse_request,
)


# ---- 命令执行 ----------------------------------------------------------------


class CommandFailed(Exception):
    """命令在机器人那边没成功。code 用机器人原生返回码，或本层的 5xx。"""

    def __init__(self, code: int, msg: str):
        super().__init__(msg)
        self.code = code
        self.msg = msg


class RobotService:
    """持有 Backend，把命令翻译成对它的调用，并记住「现在是不是在走」。"""

    def __init__(self, backend, idle_stop: float = 3.0, verbose: bool = True):
        self.b = backend
        self.idle_stop = float(idle_stop)
        self.verbose = verbose
        self.moving = False
        self.stopped = False
        self.session_active = False
        self.started = time.monotonic()
        self.last_cmd = time.monotonic()
        self._in_call = False
        self._closed = False
        # 由 main 填：本端点可选的控制端。网关据此告诉 App「哪个能点」，
        # 免得 App 自己抄一份 CLIENTS 表然后和 body.py 漂移。
        self.clients: tuple[str, ...] = ()
        self._watchdog_stop = threading.Event()

    # -- 生命周期 --------------------------------------------------------

    def start_watchdog(self) -> None:
        if self.idle_stop <= 0:
            return
        threading.Thread(target=self._watchdog, daemon=True,
                         name="comm-watchdog").start()

    # 把机器人站到标称高度再开门。`connect()` 里的 BalanceStand 是异步的，
    # 返回时腿还在抬（实测 0.11 m，站高 0.27 m），这时候接第一条 move，
    # 等于从蹲姿起步 —— 速度跟踪会差一大截，而调用方看不出原因。
    def ensure_standing(self, timeout: float = 6.0) -> bool:
        from quadruped.control.controller import NOMINAL_HEIGHT

        need = NOMINAL_HEIGHT * 0.85
        try:
            if float(self.b.height) >= need:
                return True
            self.b.stand()
        except Exception as err:                            # noqa: BLE001
            self._log(f"起立失败：{err}")
            return False

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self.stopped:
            time.sleep(0.1)
            try:
                if float(self.b.height) >= need:
                    return True
            except Exception:                               # noqa: BLE001
                continue
        self._log(f"起立超时（{timeout:.0f}s 内没到 {need:.2f} m）")
        return False

    def _watchdog(self) -> None:
        while not self._watchdog_stop.wait(0.2):
            if self.stopped or self._in_call or not self.moving:
                continue
            if time.monotonic() - self.last_cmd < self.idle_stop:
                continue
            self._log(f"空闲 {self.idle_stop:.1f}s 无请求，停车")
            try:
                self.b.stop_motion()
            except Exception as err:                        # noqa: BLE001
                self._log(f"空闲停车失败：{err}")
            self.moving = False

    # 断开连接时调。正在走就停车，不蹲下 —— 蹲下留给退出流程。
    def release(self) -> None:
        if self.moving:
            self._log("客户端断开，停车")
            try:
                self.b.stop_motion()
            except Exception as err:                        # noqa: BLE001
                self._log(f"停车失败：{err}")
            self.moving = False

    # 收工：停步 → 蹲下 → 阻尼。重复调用只生效一次。
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._watchdog_stop.set()
        try:
            self.b.close()
        except Exception as err:                            # noqa: BLE001
            self._log(f"收工失败：{err}")

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[comm] {time.strftime('%H:%M:%S')} {msg}", flush=True)

    # -- 分发 ------------------------------------------------------------

    # 处理一行请求，返回一行响应。任何异常都在这里收口，不让它把会话打断。
    def handle_line(self, line: bytes) -> bytes:
        try:
            req_id, cmd, args = parse_request(line)
        except ProtocolError as err:
            return error(None, BAD_REQUEST, str(err))

        self.last_cmd = time.monotonic()
        fn = getattr(self, f"_cmd_{cmd}", None)
        if fn is None:                                      # 命令表有、实现没写
            return error(req_id, API_UNSUPPORTED, f"命令 {cmd} 还没实现")

        self._in_call = True
        try:
            data = fn(args)
        except CommandFailed as err:
            return error(req_id, err.code, err.msg)
        except NotImplementedError as err:
            # 抽象层用 NotImplementedError 表示「这个控制端没有该能力」，
            # 消息里已经写明是哪个后端缺什么，直接透传比再造一句准。
            return error(req_id, API_UNSUPPORTED,
                         str(err) or f"当前控制端不支持 {cmd}")
        except Exception as err:                            # noqa: BLE001
            self._log(f"{cmd} 失败：{type(err).__name__}: {err}")
            return error(req_id, 500, f"{type(err).__name__}: {err}")
        finally:
            self._in_call = False
            self.last_cmd = time.monotonic()
        return ok(req_id, _plain(data))

    # 机器人返回码非 0 就抛。原生语义：0 成功，4xxx 是具体失败原因。
    #
    # 两个后端两种返回：官方 / 自研给的是原生返回码（int），推理控制端给的是
    # `MotionService.set_velocity` 的状态 dict。原来这里一律 `int(code)`，在推理
    # 控制端上每条 move 都抛 `TypeError: int() argument must be ... not 'dict'`，
    # 现象是「一切到推理控制端，摇杆就一点反应都没有」，而 App 上只看得到一句
    # 500。dict 的失败信号在 `error` 字段里。
    @staticmethod
    def _check(code, what: str) -> None:
        if isinstance(code, dict):
            err = code.get("error")
            if err:
                raise CommandFailed(503, f"{what} 失败：{err}")
            return
        if int(code) != 0:
            raise CommandFailed(int(code), f"{what} 返回 {code}")

    # -- 具体命令 --------------------------------------------------------

    def _cmd_ping(self, args):
        return {"pong": True, "uptime_s": round(time.monotonic() - self.started, 1)}

    def _cmd_status(self, args):
        data = {
            "target": self.b.target,
            "moving": self.moving,
            # 当前生效的控制端。App 靠它显示「实际在用哪个」，而不是显示自己
            # 请求过的那个 —— 两者不一致时（比如 attach 到别人起的服务端）要看得出来。
            "client": getattr(self.b, "kind", ""),
            "clients": list(self.clients),
        }
        try:
            data["pos"] = [round(float(v), 4) for v in self.b.pos]
            data["rpy"] = [round(float(v), 4) for v in self.b.rpy]
            # 推理控制器那侧没有运动学模型，报 None。直接 round(None) 会把整条
            # status 推进 except，调用方看到的是「status 出错」而不是「高度不可知」。
            h = self.b.height
            data["height"] = None if h is None else round(float(h), 4)
            data["busy"] = bool(self.b.busy)
            data["gait"] = self.b.loco_gait
            data["loop"] = self.b.loop_text.strip()
        except Exception as err:                            # noqa: BLE001
            data["error"] = f"{type(err).__name__}: {err}"
        return data

    def _cmd_stand(self, args):
        self._check(self.b.stand(), "stand")
        self.moving = False
        return {"stood": True}

    def _cmd_uncrouch(self, args):
        self._check(self.b.uncrouch(), "uncrouch")
        return {"stood": True}

    def _cmd_crouch(self, args):
        depth = float(args["depth"])
        self._check(self.b.crouch(depth), "crouch")
        self.moving = False
        return {"depth": depth}

    def _cmd_stop(self, args):
        self._check(self.b.stop_motion(), "stop")
        self.moving = False
        return {"moving": False}

    def _cmd_freeze(self, args):
        self._check(self.b.freeze(), "freeze")
        self.moving = False
        return {"frozen": True}

    def _cmd_move(self, args):
        vx, vy, vyaw = args["vx"], args["vy"], args["vyaw"]
        self._check(self.b.move(vx, vy, vyaw), "move")
        self.moving = any(abs(v) > 1e-6 for v in (vx, vy, vyaw))
        return {"moving": self.moving, "vx": vx, "vy": vy, "vyaw": vyaw}

    def _cmd_forward(self, args):
        out = self.b.forward(args["distance"], vx=args["vx"])
        self.moving = False
        return dict(out) if isinstance(out, dict) else {"result": out}

    def _cmd_turn(self, args):
        out = self.b.turn(args["degrees"], yaw_rate=args["yaw_rate"])
        self.moving = False
        return dict(out) if isinstance(out, dict) else {"result": out}

    def _cmd_jump(self, args):
        self._check(self.b.jump(), "jump")
        return {"jumping": True}

    def _cmd_gait(self, args):
        style = args["style"]
        if not style:
            return {"gait": self.b.toggle_loco_gait()}
        return {"gait": self.b.set_gait_style(style)}

    def _cmd_park(self, args):
        self.close()
        self.stopped = True
        return {"bye": True}


# numpy 标量不是 JSON 原生类型，递归转成 float / list，否则编码器会抛 TypeError。
def _plain(obj):
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_plain(v) for v in obj]
    if isinstance(obj, bool) or obj is None or isinstance(obj, str):
        return obj
    if isinstance(obj, (int, float)):
        return float(obj) if isinstance(obj, float) else int(obj)
    try:
        return float(obj)                                   # numpy.float32/64、np.int64
    except (TypeError, ValueError):
        return str(obj)


# ---- 套接字 ------------------------------------------------------------------


# 建监听套接字。给了 host 走 TCP，否则走 Unix 域套接字（同一台机器推荐）。
def listen(args) -> socket.socket:
    if args.host:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.host, args.port))
        where = f"tcp://{args.host}:{args.port}"
    else:
        path = args.socket
        if os.path.exists(path):
            os.unlink(path)                 # 上次异常退出留下的陈旧文件
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(path)
        os.chmod(path, 0o600)               # 只有本用户能连：这个接口能让狗动
        where = f"unix://{path}"
    sock.listen(4)
    sock.settimeout(0.5)                    # 让 accept 能定期回头看 stopped 标志
    print(f"监听 {where}", flush=True)
    return sock


# 一个客户端会话。逐行读、逐行回；客户端断开或服务端要退出就返回。
def session(conn: socket.socket, svc: RobotService) -> None:
    conn.settimeout(0.5)
    buf = b""
    while not svc.stopped:
        try:
            chunk = conn.recv(65536)
        except TimeoutError:
            continue
        except OSError:
            return
        if not chunk:
            return
        buf += chunk
        if len(buf) > MAX_LINE and b"\n" not in buf:
            conn.sendall(error(None, BAD_REQUEST, f"单行超过 {MAX_LINE} 字节"))
            return
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                conn.sendall(svc.handle_line(line))
            except OSError:
                return


def install_signals(svc: RobotService) -> None:
    def handler(signum, frame):             # noqa: ARG001
        svc._log(f"收到信号 {signum}，收工")
        svc.stopped = True

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


# ---- 入口 --------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    from quadruped.control.virtual.robot import add_endpoint_args

    ap = argparse.ArgumentParser(
        description="机器狗通讯服务端：常驻持机器人，对外提供套接字命令接口")
    add_endpoint_args(ap)                   # --iface / --preset / --domain / --client

    net = ap.add_argument_group("通讯")
    net.add_argument("--socket", default=os.environ.get("GO2_COMM_SOCKET", DEFAULT_SOCKET),
                     help="Unix 域套接字路径")
    net.add_argument("--host", default=None,
                     help=f"给 TCP 就填监听地址（如 {DEFAULT_HOST}）；不给走 Unix 套接字")
    net.add_argument("--port", type=int, default=DEFAULT_PORT, help="TCP 端口")

    gain = ap.add_argument_group("控制参数")
    gain.add_argument("--kp", type=float, default=None, help="关节刚度，不给用控制器默认")
    gain.add_argument("--kd", type=float, default=None, help="关节阻尼，不给用控制器默认")
    gain.add_argument("--tau", type=float, default=None, help="重力前馈强度 0~1")

    ap.add_argument("--idle-stop", type=float, default=3.0,
                    help="有动作时多久没收到请求就自动停车（秒），0=关闭")
    ap.add_argument("--quiet", action="store_true", help="不打印心跳与停车日志")
    return ap


def main(argv=None) -> int:
    from quadruped.control.virtual.robot import (
        clients_for, connect, default_client, resolve_endpoint,
    )

    args = build_parser().parse_args(argv)

    # 服务端不弹菜单：它是被脚本/系统拉起来的，stdin 可能是空的。
    ep = resolve_endpoint(args.preset, iface=args.iface, domain=args.domain)
    # 不指定控制端时跟交互菜单同一个默认值：仿真 policy、真机 official。
    # 网关拉起服务端时不会传 --client，所以这里才是遥控真正走哪条步态的地方。
    kind = args.client or default_client(ep.preset)
    print(f"端点 {ep.label} / 控制端 {kind}", flush=True)
    try:
        backend = connect(ep, kind, kp=args.kp, kd=args.kd, tau=args.tau)
    except RuntimeError as err:
        print(f"连不上：{err}", file=sys.stderr)
        return 1
    print(f"已连上 {backend.target}", flush=True)
    print(f"步态 {backend.loco_gait}（`gait` 命令可切换）", flush=True)

    svc = RobotService(backend, idle_stop=args.idle_stop, verbose=not args.quiet)
    svc.clients = clients_for(ep.preset)
    # 推理控制端要先接管：`PolicyBackend.start()` 里是「摆到策略站姿 1.5 s +
    # 起 50 Hz 线程 + 原地热身 1.5 s」，一共三秒多。如果留到第一条 `move` 才做，
    # 网关那条调用（timeout=2.0）必然超时，而超时会让一问一答的会话错位一格，
    # 之后每条命令都读到上一条的响应 —— 现象是「切到推理控制端，摇杆一点反应
    # 都没有」。提前接管，第一条速度指令就是即时返回的。
    #
    # 也因为它测不到机身高度（策略只发关节指令，没有本体状态），这里不能走
    # `ensure_standing()`；`start()` 自己会在倾角超过 45° 时拒绝，等价的那道闸。
    if backend.kind == "policy":
        try:
            backend.start()
            print("推理控制端已接管（站姿 + 热身完成）", flush=True)
        except Exception as err:                           # noqa: BLE001
            print(f"推理控制端接管失败：{err}", file=sys.stderr)
            return 1
    elif not svc.ensure_standing():
        print("警告：没能站到标称高度，第一条动作会从当前姿态起步", file=sys.stderr)
    install_signals(svc)
    listener = listen(args)
    svc.start_watchdog()
    try:
        while not svc.stopped:
            try:
                conn, _ = listener.accept()
            except TimeoutError:
                continue
            except OSError:
                break
            # 第二个连接照样 accept，但立刻回 409 —— 不 accept 的话对方会
            # 一直卡在 backlog 里等到 30 秒超时，看起来像服务端死了。
            if svc.session_active:
                try:
                    conn.sendall(error(None, BUSY, "已有客户端在控制，等它断开再来"))
                finally:
                    conn.close()
                continue
            svc.session_active = True
            threading.Thread(target=_serve_conn, args=(conn, svc),
                             daemon=True, name="comm-session").start()
    finally:
        listener.close()
        svc.close()
        if not args.host and os.path.exists(args.socket):
            os.unlink(args.socket)
        print("已退出", flush=True)
    return 0


# 一个连接一个线程。机器人同一时刻只让一个线程碰（session_active 守卫在入口），
# 线程只负责把这一条连接读干净、写回去。
def _serve_conn(conn: socket.socket, svc: RobotService) -> None:
    with conn:
        svc._log("客户端接入")
        try:
            session(conn, svc)
        finally:
            svc.session_active = False
            svc._log("客户端断开")
            svc.release()


if __name__ == "__main__":
    sys.exit(main())
