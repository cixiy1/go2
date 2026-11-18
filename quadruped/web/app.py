"""真机网页控制面：起停 VLA 进程、看状态、看日志。

    python -m quadruped.web.app --iface en5 --port 8080
    # 浏览器打开 http://192.168.123.x:8080/?t=<token>
    python -m quadruped.web.app --token abc123          # 自己定 token

## 它不碰 DDS

这一层**只做进程管理**：起一个 `quadruped.control.embodied.vla` 子进程，读它的
stdout，把每 2 秒那行状态解析成 JSON 给页面。策略、感知、DDS、lowcmd 全在子进程
里 —— 网页和运动控制之间没有第二条链路，也就不存在「两个写者抢 lowcmd」。

要把页面做成能直接调 `PolicyBackend`，就得让网页进程也持有 DDS：那样网页自己崩了
或者被开两个标签页，就可能出现第二个写者。`LinkBusy` 是进程内锁，拦不住进程外。

## 停止 = SIGINT，不是 SIGKILL

子进程的收尾（蹲下 → 阻尼 → 交还 mcf）写在 `runner.run()` 的 finally 里，只有
Ctrl+C 那条路会走到。所以「停止」发 SIGINT，等 8 秒，还活着才 SIGKILL。SIGTERM
在 `vla.main()` 里被转成 KeyboardInterrupt，systemd 停服务时是同一套收尾。

## token

`--token` / 环境变量 `GO2_WEB_TOKEN`，都没给就随机生成 6 位并打印在启动日志里。
页面把 token 存在 localStorage，之后所有请求带 `X-Token`。这是防「同一个局域网里
别人顺手打开」的，不是防攻击 —— 别把这台机器暴露到公网。
"""

from __future__ import annotations

import argparse
import errno
import fcntl
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PAGE = Path(__file__).resolve().parent / "page.html"

# 单实例锁。两个网页进程各起一个 VLA，就是两个写者抢 lowcmd —— 这不是理论问题，
# 是「谁最后发谁生效」，表现为狗一抽一抽。
LOCK = Path("/tmp/quadruped-web.lock")

# 子进程收尾最多等这么久。
# 8 秒不够：`thinker.stop()` 要等推理线程（最多 3 s），后面还有 viewer/world/
# driver 的 close。等不到就 SIGKILL —— 那意味着腿没阻尼、mcf 没交还，真机上
# 要重启才能拿回官方遥控器。
STOP_GRACE = 15.0


# fork 之后、exec 之前跑。只做两件事，都是 async-signal-safe 的：
#
#   setsid()                      新会话 —— 停止时 `killpg` 能一次收掉子进程和
#                                 它自己拉起的相机/雷达线程
#   SIGINT → SIG_DFL              shell 的后台任务把 SIGINT 设成 SIG_IGN，fork
#                                 会继承；而 CPython 只在 SIGINT 是 SIG_DFL 时
#                                 才装 KeyboardInterrupt 处理器。不重置的话
#                                 子进程对「停止」完全没反应，只能等 SIGKILL。
def _child_setup() -> None:
    os.setsid()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

# 状态行正则，对应 `runner.py` 每 2 秒打的那一行：
#   12.0s  vx=+0.40  策略 50 Hz  前方 1.20  推理 12 次 / 1.03s  位置 (+1.2, +0.3) m  朝向 +45°
STATUS_RE = re.compile(
    r"^\s*(?P<t>[\d.]+)s\s+vx=(?P<vx>[+-]?[\d.]+)\s+策略 (?P<hz>\d+) Hz\s+"
    r"前方 (?P<gap>\S+)\s+推理 (?P<th>\d+) 次 / (?P<gen>[\d.]+)s\s+"
    r"位置 \((?P<x>[+-][\d.]+), (?P<y>[+-][\d.]+)\) m"
    r"(?:\s+朝向 (?P<yaw>[+-][\d.]+)°)?")

# 回话那一行，对应 `Thinker._log("回话：…")` 打出来的 `[L] 回话：…`。
# 抓它是为了让页面把「它刚说了什么」显示出来 —— 只靠日志尾巴的话，人得自己翻。
REPLY_RE = re.compile(r"^\[L\] 回话：(.+)$")

# 子进程确认收到一句人话。`Thinker` 在 `_take()` 之后打这一行。
#
# 有它才能把「我说的」和「它回的」配成对：只说「它说」的话，模型对**指令**
# 常常只改命令、不写 reply，页面一片安静，人以为话没送到 —— 2026-11-16 用户
# 报的「指令只能发一次」就是这个观感。
ACK_RE = re.compile(r"^\[L\] 收到：(.+)$")

# 页面上保留几轮对话。再多也没人翻，而且每轮都要序列化一遍。
DIALOG_KEEP = 6


# 从一行日志里抽出状态。抽不到返回 None —— 页面就显示「还没起来」。
def parse_status(line: str) -> dict | None:
    m = STATUS_RE.match(line)
    if not m:
        return None
    gap = m.group("gap")
    try:
        gap_v = float(gap)
    except ValueError:
        gap_v = None                                  # 「无视野」这类
    return dict(t=float(m.group("t")), vx=float(m.group("vx")),
                hz=int(m.group("hz")), gap=gap_v, thoughts=int(m.group("th")),
                gen=float(m.group("gen")), x=float(m.group("x")),
                y=float(m.group("y")),
                yaw=None if m.group("yaw") is None else float(m.group("yaw")))


class Supervisor:
    """一个子进程 + 一坨状态。线程安全靠一把锁，不做队列。"""

    def __init__(self, cmd: list[str], log_path: Path, frame_path: Path):
        self.cmd = cmd
        self.log_path = log_path
        # 子进程每 5 Hz 导一张画面到这里（`Viewer._export`）。网页只读文件，
        # 不跟子进程建立第二条通讯 —— 图片这条路上没有任何能被写的东西。
        self.frame_path = frame_path
        self.proc: subprocess.Popen | None = None
        self.state = "idle"          # idle / starting / running / stopping / exited
        self.status: dict | None = None
        self.last_line = ""
        self.reply = ""             # 它最近一次回给人的话
        # 我说的话和它回的，配成对。每条：{t, you, got, it}
        #   got  子进程有没有确认收到（`[L] 收到：`）
        #   it   它的回话（可能一直是空 —— 指令轮次它只改命令、不说话）
        self.dialog: list[dict] = []
        self.started_at: float | None = None
        self.exit_code: int | None = None
        self.tail: list[str] = []
        # 子进程会不会读 stdin。只有带 `--interactive` 才会 —— 不带的话写进去的
        # 话堆在管道里没人取，满了之后 `write` 会阻塞住整个网页线程。
        self.interactive = "--interactive" in cmd
        self._lock = threading.Lock()
        # `dialog` 单独一把锁：`say()` 写 stdin 时不能握着 `_lock`，见那里的注释。
        self._dlg_lock = threading.Lock()
        self._reader: threading.Thread | None = None

    # 起。已经在跑就不重复起 —— 页面上连点两下不该出两个进程。
    def start(self) -> dict:
        with self._lock:
            if self.proc is not None and self.proc.poll() is None:
                return {"ok": False, "why": "已经在跑"}
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as log:
                log.write(f"\n=== {time.strftime('%F %T')} 启动 {' '.join(self.cmd)}\n")
            self.proc = subprocess.Popen(
                self.cmd, cwd=str(ROOT), stdout=subprocess.PIPE,
                # stdin 必须是管道，不能让它继承网页进程的 stdin：不接的话子进程
                # 读到的是 /dev/null（systemd 下）或者 terminal，前者的 `input()`
                # 立刻 EOF —— 网页上说什么它都收不到。
                stdin=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
                encoding="utf-8", errors="replace",
                # 独立进程组：停止时把 vla 拉起的子孙一起收掉。
                #
                # 顺手把 SIGINT 恢复成默认。网页本身常常是后台起的（shell 的
                # `&`、systemd），那种情况下 SIGINT 是 SIG_IGN，fork 出来又被
                # 继承；Python 看到 SIG_IGN 就不装 KeyboardInterrupt 处理器，
                # 子进程于是对「停止」完全免疫，只能 SIGKILL。`vla.py` 自己也装
                # 了一遍，这里再兜一层是为了别的子命令（测试脚本）也不中招。
                preexec_fn=_child_setup)
            self.state, self.status, self.last_line = "starting", None, ""
            self.reply = ""
            with self._dlg_lock:
                self.dialog = []
            self.started_at, self.exit_code = time.time(), None
            self.tail = []
            self._reader = threading.Thread(target=self._pump, daemon=True)
            self._reader.start()
            return {"ok": True, "pid": self.proc.pid, "log": str(self.log_path)}

    # 把子进程的输出抄一份到日志文件，同时留最近 200 行给页面，顺手解析状态行。
    #
    # 不解析就只剩「进程还活着」这一条信息 —— 而这一层要回答的恰恰是「狗现在在
    # 干什么」：回路频率掉没掉、前方还有多远、模型多慢。
    def _pump(self) -> None:
        proc = self.proc
        if proc is None or proc.stdout is None:
            return
        with open(self.log_path, "a", encoding="utf-8") as log:
            for line in proc.stdout:
                line = line.rstrip("\n")
                log.write(line + "\n")
                log.flush()
                self.last_line = line
                self.tail.append(line)
                if len(self.tail) > 200:
                    del self.tail[:-200]
                st = parse_status(line)
                if st is not None:
                    self.status = st
                    self.state = "running"
                got = REPLY_RE.match(line)
                if got is not None:
                    self.reply = got.group(1).strip()
                    self._fill_reply(self.reply)
                ack = ACK_RE.match(line)
                if ack is not None:
                    self._mark_got(ack.group(1).strip())
            # 子进程退出的那一行也留个记号
            log.write(f"=== {time.strftime('%F %T')} 退出，"
                      f"code {proc.poll()}\n")
        self.state = "exited"

    def stop(self) -> dict:
        with self._lock:
            if self.proc is None or self.proc.poll() is not None:
                return {"ok": False, "why": "没在跑"}
            self.state = "stopping"
            self._signal(signal.SIGINT)
        # 等收尾（蹲下 → 阻尼 → 交还 mcf）。不持锁等，否则状态查询会被卡住。
        deadline = time.time() + STOP_GRACE
        while time.time() < deadline:
            if self.proc.poll() is not None:
                return {"ok": True, "exit": self.proc.returncode}
            time.sleep(0.2)
        self._signal(signal.SIGKILL)
        return {"ok": True, "killed": True}

    def _signal(self, sig) -> None:
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except OSError:
            pass

    # 子进程确认收到一句。先进先出配对：连着说两句时，「收到」的顺序就是发送
    # 顺序，找第一条匹配文本的未确认条目。
    def _mark_got(self, text: str) -> None:
        with self._dlg_lock:
            for item in self.dialog:
                if not item["got"] and item["you"] == text:
                    item["got"] = True
                    return
            # 文本对不上（子进程可能做了空白规整）就退化为「第一条未确认的」。
            for item in self.dialog:
                if not item["got"]:
                    item["got"] = True
                    return

    # 它的回话配给「已经确认、还没有回话」的第一条。
    #
    # 自驱轮次也会打 `[L] 回话：`（模型自言自语），那种没有对应的「你」，
    # 直接丢弃 —— 不然会把上一句的会话内容覆盖掉。
    def _fill_reply(self, text: str) -> None:
        with self._dlg_lock:
            for item in self.dialog:
                if item["got"] and not item["it"]:
                    item["it"] = text
                    return

    # 把一句话递进子进程的 stdin。`runner._stdin_loop` 那边 `input()` 读一行，
    # 转给 `Thinker.say()` —— 和终端里敲一句完全同一条路，不另开控制通道。
    #
    # 不走「再开一个 DDS 客户端」那条路：lowcmd 只能有一个写者，`LinkBusy`
    # 是进程内锁，拦不住第二个进程。
    def say(self, text: str) -> dict:
        text = " ".join((text or "").split())
        if not text:
            return {"ok": False, "why": "空话"}
        if not self.interactive:
            return {"ok": False, "why": "这个子进程没开 --interactive，收不到话"}
        # 取引用要在锁里，写 stdin 要在锁外。
        #
        # 不能在持锁时写：管道满（子进程没在读）会让 `write` 阻塞，而 `_pump`
        # 那边拿同一把锁更新 `dialog` —— 它一卡住就不再读子进程 stdout，子进程
        # stdout 满又不再读 stdin，两边互等。这条链真能死锁，虽然要子进程先卡住
        # 才触发。
        with self._lock:
            proc = self.proc
        if proc is None or proc.poll() is not None or proc.stdin is None:
            return {"ok": False, "why": "没在跑"}
        try:
            proc.stdin.write(text + "\n")
            proc.stdin.flush()
        except (OSError, ValueError) as err:
            return {"ok": False, "why": f"{type(err).__name__}: {err}"}
        with self._dlg_lock:
            self.dialog.append({"t": time.time(), "you": text,
                                "got": False, "it": ""})
            del self.dialog[:-DIALOG_KEEP]
        return {"ok": True, "text": text}

    def snapshot(self) -> dict:
        proc = self.proc
        alive = proc is not None and proc.poll() is None
        if proc is not None and not alive and self.state != "idle":
            self.state, self.exit_code = "exited", proc.returncode
        try:
            frame_ts = self.frame_path.stat().st_mtime
        except OSError:
            frame_ts = None
        with self._dlg_lock:
            dialog = [dict(item) for item in self.dialog]
        out = dict(state=self.state, pid=None if proc is None else proc.pid,
                   alive=alive, status=self.status, exit_code=self.exit_code,
                   started_at=self.started_at,
                   uptime=None if self.started_at is None else
                   round(time.time() - self.started_at, 1),
                   log=str(self.log_path), last_line=self.last_line,
                   reply=self.reply, interactive=self.interactive,
                   dialog=dialog, frame_ts=frame_ts)
        return out


class Handler(BaseHTTPRequestHandler):
    sup: Supervisor
    token: str
    frame: Path
    server_version = "quadruped-web"

    def log_message(self, fmt, *args):        # 别把每个请求都刷到终端
        pass

    # -- 工具 ------------------------------------------------------------

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj) -> None:
        self._send(code, json.dumps(obj, ensure_ascii=False).encode(), "application/json")

    def _authed(self) -> bool:
        if not self.token:
            return True
        got = self.headers.get("X-Token", "")
        if not got:
            q = urllib.parse.urlparse(self.path).query
            got = urllib.parse.parse_qs(q).get("t", [""])[0]
        return got == self.token

    # -- 路由 ------------------------------------------------------------

    def do_GET(self) -> None:
        path = urllib.parse.urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            return
        if not self._authed():
            self._json(401, {"ok": False, "why": "token 不对"})
            return
        if path == "/api/status":
            self._json(200, self.sup.snapshot())
            return
        # 画面版本号。页面按 10 Hz 问这个（几十字节），变了才去下整张图 ——
        # 没变的时候重下 32 KB 是纯浪费，而且会把局域网和浏览器的连接占满。
        if path == "/api/tick":
            try:
                ts = Handler.frame.stat().st_mtime
            except OSError:
                ts = 0.0
            self._json(200, {"ts": round(ts * 1000)})
            return
        # 画面。和窗口里那张是同一个画布（相机 + 想法 + 视野条，带标注），
        # 子进程按 20 Hz 导出来。读不到就是还没渲染 —— 404，页面显示占位。
        if path == "/api/frame":
            try:
                body = Handler.frame.read_bytes()
            except OSError:
                self._json(404, {"ok": False, "why": "还没有画面"})
                return
            self._send(200, body, "image/jpeg")
            return
        if path == "/api/log":
            n = 60
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            try:
                n = max(1, min(400, int(q.get("n", ["60"])[0])))
            except ValueError:
                pass
            try:
                lines = self.sup.log_path.read_text(errors="replace").splitlines()
            except OSError:
                lines = []
            self._json(200, {"lines": lines[-n:]})
            return
        self._json(404, {"ok": False, "why": "没有这个接口"})

    def do_POST(self) -> None:
        if not self._authed():
            self._json(401, {"ok": False, "why": "token 不对"})
            return
        path = urllib.parse.urlparse(self.path).path
        if path == "/api/start":
            self._json(200, self.sup.start())
            return
        if path == "/api/stop":
            self._json(200, self.sup.stop())
            return
        if path == "/api/say":
            self._json(200, self.sup.say(self._read_text()))
            return
        self._json(404, {"ok": False, "why": "没有这个接口"})

    # 从请求里抠出那句话。两种写法都支持：JSON body（页面用）和 query
    # （`curl -X POST '…/api/say?text=往前走'`，调试用）。
    def _read_text(self) -> str:
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        text = q.get("text", [""])[0]
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            n = 0
        if n > 0:
            try:
                obj = json.loads(self.rfile.read(n).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                obj = None
            if isinstance(obj, dict):
                text = str(obj.get("text") or obj.get("say") or text)
        return text


# 拿锁。拿不到说明已经有一个在跑 —— 直接退出，不要起第二个。
def take_lock() -> bool:
    fd = os.open(LOCK, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as err:
        if err.errno in (errno.EAGAIN, errno.EACCES):
            return False
        raise
    os.write(fd, str(os.getpid()).encode())
    globals()["_LOCK_FD"] = fd                   # 进程活着期间别被回收
    return True


# systemd 停服务发 SIGTERM。它必须走到 `main()` 的 finally，把子进程按「蹲下 →
# 阻尼 → 交还 mcf」收掉；只靠默认处理的话进程直接死，systemd 超时后把整个 cgroup
# SIGKILL，子进程连 finally 都跑不完 —— 狗就停在最后一条命令上。
def _on_sigterm(signum, frame):
    raise KeyboardInterrupt


def local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.123.1", 9))           # 不真发包，只为问路由
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "127.0.0.1"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="真机网页控制面")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--iface", default=os.environ.get("GO2_IFACE", "en5"),
                    help="真机网卡名。扩展坞/本体上不一定是 en5，用 ip addr 查")
    ap.add_argument("--preset", default="real", choices=("real", "sim"),
                    help="页面「开始」拉起的 VLA 跑在哪：real（网卡）或 sim（local）")
    ap.add_argument("--scene", default="maze",
                    help="仿真场景名（只在 --preset sim 时有用；仿真器本身要另外起）")
    ap.add_argument("--source", default=None,
                    choices=("real+vision", "proprio", "sim+vision", "sim"))
    ap.add_argument("--geom", default="on", choices=("on", "off"))
    ap.add_argument("--token", default=os.environ.get("GO2_WEB_TOKEN", ""))
    ap.add_argument("--open", action="store_true",
                    help="起完服务自动用默认浏览器打开页面（本机跑时用）")
    ap.add_argument("--log-dir", default=str(ROOT / "logs"))
    ap.add_argument("--frame", default="",
                    help="画面 JPEG 的落盘路径，默认 <log-dir>/frame.jpg")
    ap.add_argument("--cmd", default="",
                    help="覆盖要起的命令（调试用，比如 python -c 睡 60 秒）")
    a = ap.parse_args(argv)

    if not take_lock():
        print(f"已经有一个网页进程在跑（{LOCK}）。要重开先停掉它。")
        return 2

    log_dir = Path(a.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    frame_path = Path(a.frame) if a.frame else log_dir / "frame.jpg"

    # 子进程跑真机还是仿真。**原来这里是写死的 `--preset real`** —— launcher
    # 明明按场景算好了 `--preset sim` 传进来，被 `embodied/web.py` 的 `_strip`
    # 摘掉扔了，于是仿真里点「开始」起的是真机 VLA，连 en5 连不上就退出。
    # 2026-11-18 用户问「仿真为什么不能网页」，就是这一行。
    sim = a.preset == "sim"
    iface = "lo0" if sim else a.iface
    source = a.source or ("sim+vision" if sim else "real+vision")

    if a.cmd:
        cmd = a.cmd.split()
    else:
        cmd = [sys.executable, "-m", "quadruped.control.embodied.vla",
               "--preset", a.preset, "--iface", iface, "--source", source,
               # 没有图形会话时 `--no-watch` 是必须的：viewer 在扩展坞上起不来，
               # 而它抛的错会被当成「窗口关了」直接结束主循环
               "--no-watch", "--geom", a.geom,
               # 网页上说的话从这里进 `Thinker.say()`。不带这一项，命令行照样跑，
               # 只是页面上的输入框会一直回「收不到话」。
               "--interactive",
               # 画面：窗口里那张（相机 + 想法 + 视野条）按 5 Hz 导成 JPEG。
               # `--no-watch` 也照样渲染，导出的图就是本机窗口里能看到的那张。
               "--stream", str(frame_path)]

    token = a.token or f"{int(time.time()) % 100000:05d}"
    Handler.sup = Supervisor(cmd, log_dir / "web-vla.log", frame_path)
    Handler.token = token
    Handler.frame = frame_path

    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    ip = local_ip()
    print(f"网页控制面 http://{ip}:{a.port}/?t={token}")
    print(f"  要起的命令：{' '.join(cmd)}")
    print(f"  日志：{Handler.sup.log_path}")
    print(f"  画面：{frame_path}（子进程按 5 Hz 覆盖，页面自己进来取）")
    print("  Ctrl+C 退出（不会自动停机器人 —— 先按页面上的『停止』）", flush=True)

    if a.open:
        # 走 127.0.0.1 而不是刚才那个局域网 IP：本机开会话时它更稳（多网卡时
        # `local_ip()` 可能选到没路由的那张）。等一下再开，`serve_forever` 还没
        # 进循环时浏览器会拿到 connection refused。
        def _open():
            time.sleep(0.6)
            try:
                webbrowser.open(f"http://127.0.0.1:{a.port}/?t={token}")
            except Exception as err:                        # noqa: BLE001
                print(f"  开浏览器失败（自己点上面的地址）：{err}", flush=True)

        threading.Thread(target=_open, name="open-browser", daemon=True).start()

    # systemd stop 发 SIGTERM。它必须走到下面的 finally，把子进程按「蹲下 → 阻尼 →
    # 交还 mcf」收掉；否则 systemd 会在超时后 SIGKILL 整个 cgroup，子进程连 finally
    # 都跑不完 —— 狗就那么站在最后一条命令上。
    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        if Handler.sup.snapshot()["alive"]:
            Handler.sup.stop()
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
