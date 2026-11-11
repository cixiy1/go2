"""虚拟控制器 · 键盘遥控 Go2，带加速度限幅和按键保持。

「虚拟」指它跑在仿真器上（`lo0`）。同一份代码也能驱动真机，那是环境切换
不是另一个控制器 —— 真机专有的诊断/只读工具放 `quadruped/control/physical/`。

两个控制端，对外开放的接口完全一样（都是 SportClient 那套 Init / SetTimeout /
BalanceStand / Move / StopMove / Damp）：

    1) 官方控制器   unitree_sdk2py 的 SportClient，走真机上的原生运动服务
    2) 自研控制器   quadruped/control/high_level.py 的 HighLevelController

后面的连接、按键循环、收工各只有一份代码，区别只在构造那一步。

环境由**网卡**决定，不再区分 sim/real 参数：lo0（回环）对着仿真器，其它网卡
对着真机。可选的控制端由环境决定（Go2Robot.CLIENTS）：

    仿真网卡 → 只有自研一个，不问，直接用
    真实网卡 → 官方 / 自研两个，问一句

仿真的桥接层只有 rt/lowcmd 与 rt/sportmodestate，没有运动服务，官方控制器
发过去的请求只会等到超时，所以那边根本不列出来。

用法（走项目入口，环境由它替你问）：
    uv run python main.py                          # 问用哪个控制器，再问网口
    uv run python main.py keyboard -- --iface en5  # 跳过选择，直接真机
直接跑这个模块也行：
    uv run python -m quadruped.control.virtual.keyboard              # 问网口、问控制器
    uv run python -m quadruped.control.virtual.keyboard --iface en5  # 不问网口，真机
    uv run python -m quadruped.control.virtual.keyboard --iface en5 --client ours
    uv run python -m quadruped.control.virtual.keyboard --max-speed 0.25
    uv run python -m quadruped.control.virtual.keyboard --duration 15  # 最多跑 15 秒
    uv run python -m quadruped.control.virtual.keyboard --keys terminal  # 强制终端按键
    uv run python -m quadruped.control.virtual.keyboard --keys-anywhere  # 全局监听
    uv run python -m quadruped.control.virtual.keyboard --line-mode   # 指令模式
    uv run python -m quadruped.control.virtual.keyboard --dry-run     # 不连狗

不带 --iface / --preset 时先列本机网卡让你选（回车 = 仿真），选完再按那个
环境列出可选的控制端。非交互终端（管道、重定向）两项都直接用默认。

按键（全局监听，多个方向可以同时按）：
    w / s     前进 / 后退        a / d   左移 / 右移（横移）
    q / e     左转 / 右转        方向键   同 w s q e
    空格      跳跃               Ctrl    蹲下 / 起立（切换）
    Enter     停止并站定         b       站立平衡（BalanceStand）
    1..9      最大速度档 0.1 ~ 0.9 m/s
    h         显示 / 隐藏帮助    ESC     退出

    终端按键模式没有 Ctrl（修饰键在终端里不产生字符），其余一致。

设计要点：
- 方向是**状态**不是事件：按着就走，松开（keyup）立即按加速度限幅减速到 0，
  跟游戏一样。w 和 q 同时按就是边走边转，互不打断。
- 终端模式读不到 keyup（字符流里没有），退回「每次按键刷新 0.55 秒意图」的
  老办法：意图过期即视作松手。这个值要大于 macOS 的键盘首次重复延迟（约 0.5s）。
  并且终端只会重复最后按下的那个键，所以按住 w 再按 q 时 w 会先掉 —— 要真正的
  多键同时按，就用全局监听。
- 空格 / Ctrl / Enter 是一次性动作，按下才触发一次，长按不会连跳。
- **动作不阻塞**：跳、蹲这类动作只是往控制回路里排一段序列（controller.pose_sequence），
  排完立刻返回，20 Hz 主循环照常跑。动作期间不发速度指令（腿被序列占着），
  但方向键的状态一直记着，动作一播完就接着走，不用重新按键。
- Move 是无回复调用，机器人会一直保持上一个速度指令，退出时必须 StopMove。
- 全程 finally 兜底，Ctrl+C / 异常 / 到时都会停车并蹲下收工。
"""

import argparse
import math
import os
import queue
import select
import socket
import sys
import termios
import threading
import time
import tty

# 项目根进 sys.path：直接 `python quadruped/control/virtual/keyboard.py` 也能跑，
# 也让 IDE 能静态解析下面的 from quadruped.robot.body。
# 本文件在 quadruped/control/virtual/ 下，往上三级才是项目根。
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from quadruped.robot.body import Go2Robot

# ---- 入口注册信息 ---------------------------------------------------------
# 项目根 main.py 的启动器会扫 quadruped/control/{virtual,physical}/*.py，
# 导入模块读这个字典来列菜单。加一个新控制器 = 加一个带 META 的文件。
#   owns_cmd   会往 rt/lowcmd 发指令（两个同时选会互相覆盖，启动器会拦）
#   owns_keys  接管键盘（两个同时选抢按键，启动器也会拦）
META = dict(
    title="键盘遥控",
    env="virtual",
    desc="w/a/s/d/q/e 边走边转，空格跳跃、Ctrl 蹲起，1~9 调速度档",
    owns_cmd=True,
    owns_keys=True,
)

# 每个方向的单位向量 (vx, vy, vyaw)。vy 正为左，vyaw 正为逆时针。
DIRS = {
    "w": (1, 0, 0),
    "s": (-1, 0, 0),
    "a": (0, 1, 0),
    "d": (0, -1, 0),
    "q": (0, 0, 1),
    "e": (0, 0, -1),
}

HOLD = 0.55          # 按键意图保持时间，秒。要大于 macOS 键盘首次重复延迟（约 0.5s）
ACCEL_LIN = 1.0      # 线加速度上限 m/s^2
ACCEL_ANG = 2.5      # 角加速度上限 rad/s^2
RATE = 20.0          # 控制频率 Hz
EPS = 1e-3

# 方向键的转义序列第三字节 -> 等价按键
ARROWS = {"A": "w", "B": "s", "C": "e", "D": "q"}

CLIENT_NAMES = {"official": "官方控制器", "ours": "自研控制器"}

# 菜单里给每个控制端的一行说明
CLIENT_DESC = {
    "official": "官方控制器    SportClient —— 真机上的原生运动服务",
    "ours": "自研控制器    HighLevelController —— 本仓库实现",
}

HELP = """
按键（多个方向可以同时按）：
  w / s   前进 / 后退        a / d   左移 / 右移（横移）
  q / e   左转 / 右转        ↑↓←→   同 w s q e
  空格    跳跃               Ctrl    蹲下 / 起立
  Shift   切换风格（机械 / 仿生）  Tab   同上，跳和跑一起换
  Enter   停止并站定         b       站立平衡
  1..9    最大速度 0.1~0.9 m/s
  h       开关本帮助         ESC     退出

方向按住就走、松开就停。空格 / Ctrl / Enter 是单次动作，不连发。
动作不会挡住后面的按键：跳到一半也能按方向，落地那一下接着走。
"""

LINE_HELP = """
命令行模式：终端只能按行转发，用「方向 + 秒数」下指令，每条回车执行。
    w 1.5     前进 1.5 秒          d 0.8     右移 0.8 秒
    q 1       左转 1 秒            w         省略秒数按 1 秒
    jump      跳跃                 crouch    蹲下 / 起立
    gait      切换风格（机械 / 仿生）  jgait   同上（跳和跑一起换）
    stop      停车并站定           speed 0.3 设最大线速度
    quit      退出
"""


class TerminalKeys:
    """终端字符流。没有 keyup，方向靠「意图超时」维持，也读不到 Ctrl 键。

    优先用 stdin；stdin 不是终端时（IDE 内置终端、被重定向、从管道启动）
    改读 /dev/tty，否则按键会走行缓冲，按单键毫无反应。两种情况都拿不到终端
    就退化成逐行读，此时必须按回车。
    """

    live = False        # 没有真实的按下/松开状态

    def __init__(self):
        self.owned = False          # /dev/tty 是自己打开的，要负责关闭
        self.saved = None
        self.tty_mode = False
        self._q = []

        if sys.stdin.isatty():
            self.fd, self.source = sys.stdin.fileno(), "stdin"
        else:
            try:
                self.fd = os.open("/dev/tty", os.O_RDONLY | os.O_NONBLOCK)
                self.owned = True
                self.source = "/dev/tty"
            except OSError:
                self.fd, self.source = sys.stdin.fileno(), "stdin(非终端)"

        if os.isatty(self.fd):
            try:
                self.saved = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                # 回读确认 ICANON 真的关了，否则单键仍然读不到
                self.tty_mode = (termios.tcgetattr(self.fd)[3] & termios.ICANON) == 0
            except termios.error:
                pass

    def pump(self, timeout: float) -> None:
        if not select.select([self.fd], [], [], timeout)[0]:
            return
        data = os.read(self.fd, 64)
        if not data:
            raise EOFError
        self._parse(data.decode("utf-8", errors="ignore"))

    def _parse(self, s: str) -> None:
        i = 0
        while i < len(s):
            ch = s[i]
            if ch == "\x1b":
                seq = s[i + 2:i + 3]
                if s[i + 1:i + 2] == "[" and seq in ARROWS:
                    self._q.append(("dir", ARROWS[seq]))
                    i += 3
                    continue
                self._q.append(("quit", None))      # 单独一个 ESC = 退出
                i += 1
                continue
            if ch == "\x03":                        # Ctrl+C
                self._q.append(("quit", None))
            elif ch in "\r\n":                      # Enter
                self._q.append(("stop", None))
            elif ch == " ":
                self._q.append(("jump", None))
            elif ch in DIRS:
                self._q.append(("dir", ch))
            elif ch.isdigit() and ch != "0":
                self._q.append(("speed", int(ch)))
            elif ch == "b":
                self._q.append(("balance", None))
            elif ch == "\t":
                self._q.append(("loco_gait", None))
            elif ch == "g":
                self._q.append(("loco_gait", None))
            elif ch == "G":
                self._q.append(("jump_gait", None))
            elif ch == "h":
                self._q.append(("help", None))
            i += 1

    def events(self):
        out, self._q = self._q, []
        return out

    def pressed(self, name: str) -> bool:
        return False

    def dir_presses(self) -> int:
        return 0

    @property
    def front_name(self) -> str:
        return "终端字符流"

    def restore(self):
        if self.saved is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            except termios.error:
                pass
            self.saved = None
        if self.owned:
            os.close(self.fd)
            self.owned = False


class GlobalKeys:
    """系统级键盘监听（pynput / CGEventTap）：真正的按下与松开。

    终端字符流只能拿到「按了什么」，拿不到「什么时候松开」，还被系统限制成
    只重复最后按下的那个键 —— 按住 w 再按 q，w 就断了。游戏里没这问题，因为
    读的是按键状态位。这里走同一套：on_press 记进 down 集合，on_release 移除，
    方向由主循环按帧问「现在还按着吗」。

    全局意味着别的窗口也能收到，所以默认只在**前台是终端 / IDE** 时响应 ——
    否则你在旁边文档里打字，狗会跟着动。判断不了前台是谁就放行（宁可不好用，
    也不要莫名其妙没反应）。

    macOS 首次运行会要求「输入监控」权限（系统设置 → 隐私与安全性）。
    """

    live = True

    # 前台是这些应用时才吃按键。**认 bundle id，不认显示名** —— localizedName
    # 跟着系统语言变，中文 macOS 上 Terminal.app 叫「终端」，拿英文名去比永远不中，
    # 结果就是监听装着却一个键都不响应。
    TERMINAL_BUNDLES = {
        "com.apple.Terminal",
        "com.googlecode.iterm2",
        "dev.warp.Warp-Stable", "dev.warp.Warp",
        "com.github.wez.wezterm",
        "io.alacritty",
        "net.kovidgoyal.kitty",
        "co.zeit.hyper",
        "org.tabby", "io.tabby",
        "com.jetbrains.pycharm", "com.jetbrains.pycharm.ce",
        "com.jetbrains.pycharm.eap",
        "com.jetbrains.intellij", "com.jetbrains.intellij.ce",
        "com.jetbrains.goland", "com.jetbrains.clion",
        "com.microsoft.VSCode", "com.microsoft.VSCodeInsiders",
    }

    # bundle id 拿不到时退回显示名，只收特征明确的词（"code" 会误伤 Xcode，不放）
    TERMINAL_NAMES = (
        "terminal", "iterm", "warp", "wezterm", "alacritty", "kitty", "hyper",
        "tabby", "pycharm", "intellij", "goland", "clion", "cursor",
        "终端", "命令行", "控制台",
    )

    def __init__(self, only_terminal: bool = True):
        from pynput import keyboard as pk

        self._down: set[str] = set()        # 方向名 + "kind:val" 的一次性动作标记
        self._lock = threading.Lock()
        self._q: queue.Queue = queue.Queue()
        self._dir_presses = 0

        self._only_terminal = only_terminal
        self._front_ok = True
        self._front_name = "?"
        self._front_t = 0.0
        self._tty_fd: int | None = None
        self._tty_saved = None

        self._table = {
            pk.Key.space: ("jump", None),
            pk.Key.enter: ("stop", None),
            pk.Key.ctrl: ("crouch", None),
            pk.Key.ctrl_l: ("crouch", None),
            pk.Key.ctrl_r: ("crouch", None),
            pk.Key.tab: ("loco_gait", None),
            pk.Key.shift: ("jump_gait", None),
            pk.Key.shift_l: ("jump_gait", None),
            pk.Key.shift_r: ("jump_gait", None),
            pk.Key.esc: ("quit", None),
            pk.Key.up: ("dir", "w"),
            pk.Key.down: ("dir", "s"),
            pk.Key.left: ("dir", "q"),
            pk.Key.right: ("dir", "e"),
        }

        self._listener = pk.Listener(on_press=self._on_press,
                                     on_release=self._on_release)
        self._listener.start()
        if not self._listener.running:
            raise RuntimeError("键盘监听没起来（多半是没给「输入监控」权限）")
        self._mute_tty()

    def _mute_tty(self) -> None:
        """关掉终端回显。按键已经被监听见了，再回显只会糊在状态行上，
        还会把字符堆进 tty 输入缓冲，等下谁读 stdin 就一口吃进去。"""
        if not sys.stdin.isatty():
            return
        try:
            fd = sys.stdin.fileno()
            self._tty_saved = termios.tcgetattr(fd)
            self._tty_fd = fd
            tty.setcbreak(fd)               # 关 ECHO，同时关行缓冲
        except (termios.error, OSError):
            self._tty_saved = None

    def _resolve(self, key):
        char = getattr(key, "char", None)
        if char:
            c = char.lower()
            if c in DIRS:
                return ("dir", c)
            if c in "123456789":
                return ("speed", int(c))
            if c == "b":
                return ("balance", None)
            if c == "h":
                return ("help", None)
            return None
        return self._table.get(key)

    def _on_press(self, key):
        r = self._resolve(key)
        if r is None:
            return
        kind, val = r
        if kind == "dir":
            with self._lock:
                self._down.add(val)
                self._dir_presses += 1      # 给开跑前的「你按了方向键」提示用
            return
        # 一次性动作：系统连发的重复 keydown 只算第一次
        tag = f"{kind}:{val}"
        with self._lock:
            if tag in self._down:
                return
            self._down.add(tag)
        self._q.put((kind, val))

    def _on_release(self, key):
        r = self._resolve(key)
        if r is None:
            return
        kind, val = r
        with self._lock:
            self._down.discard(val if kind == "dir" else f"{kind}:{val}")

    def pump(self, timeout: float) -> None:
        time.sleep(timeout)         # 监听在后台线程，主循环只管睡

    def front_ok(self) -> bool:
        """前台是不是终端 / IDE。0.25 秒查一次，别每帧都问系统。"""
        if not self._only_terminal:
            return True
        now = time.monotonic()
        if now - self._front_t < 0.25:
            return self._front_ok
        self._front_t = now
        try:
            from AppKit import NSWorkspace
            app = NSWorkspace.sharedWorkspace().frontmostApplication()
            if app is None:
                self._front_ok, self._front_name = True, "取不到前台，放行"
                return True
            bundle = str(app.bundleIdentifier() or "")
            name = str(app.localizedName() or "")
        except Exception:
            self._front_ok, self._front_name = True, "判断不了，放行"
            return True
        self._front_name = name
        self._front_ok = self.is_terminal_app(bundle, name)
        return self._front_ok

    @classmethod
    def is_terminal_app(cls, bundle: str, name: str) -> bool:
        """bundle id 命中白名单就放行；没命中再看显示名（新终端不至于被一刀切）。"""
        if bundle and bundle in cls.TERMINAL_BUNDLES:
            return True
        low = (name or "").lower()
        return any(h in low for h in cls.TERMINAL_NAMES)

    @property
    def front_name(self) -> str:
        self.front_ok()
        return self._front_name

    def events(self):
        allow_all = self.front_ok()     # 焦点在别处：动作丢掉，但退出放行
        out = []
        while True:
            try:
                e = self._q.get_nowait()
            except queue.Empty:
                return out
            if allow_all or e[0] == "quit":
                out.append(e)
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                return out

    def pressed(self, name: str) -> bool:
        if not self.front_ok():
            return False
        with self._lock:
            return name in self._down

    def dir_presses(self) -> int:
        """按下过多少次方向键。看总数而不是看「此刻按着」，短按也不会漏。"""
        with self._lock:
            return self._dir_presses

    def restore(self):
        self._listener.stop()
        if self._tty_saved is not None and self._tty_fd is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._tty_saved)
            except (termios.error, OSError):
                pass
            self._tty_saved = None


def make_keys(mode: str, anywhere: bool = False):
    """按 --keys 选按键源。auto：能用全局就用，不行退回终端。"""
    if mode in ("auto", "global"):
        try:
            return GlobalKeys(only_terminal=not anywhere)
        except Exception as e:
            if mode == "global":
                raise
            print(f"全局键盘监听不可用（{e}），退回终端按键。")
            print("  终端模式下 Ctrl 读不到，且按住 w 再按 q 会掉一个方向。")
    return TerminalKeys()


def approach(cur: float, target: float, step: float) -> float:
    if cur < target:
        return min(cur + step, target)
    return max(cur - step, target)


class Controller:
    def __init__(self, max_lin: float, max_ang: float):
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.deadline = dict.fromkeys(DIRS, 0.0)
        self.vx = self.vy = self.vyaw = 0.0

    def press(self, ch: str, now: float):
        if ch in DIRS:
            self.deadline[ch] = now + HOLD
            return True
        return False

    def target(self, now: float, active=None):
        """active 给了就按它算（全局监听：当前按着的那几个键）；
        不给就用 deadline 里的意图（终端模式）。"""
        if active is None:
            active = [k for k in DIRS if self.deadline[k] > now]
        tx = ty = tz = 0.0
        for key in active:
            dx, dy, dz = DIRS[key]
            tx += dx
            ty += dy
            tz += dz
        # 斜向不加速：只要方向和，不做归一化以外的事
        tx = max(-1.0, min(1.0, tx)) * self.max_lin
        ty = max(-1.0, min(1.0, ty)) * self.max_lin
        tz = max(-1.0, min(1.0, tz)) * self.max_ang
        return tx, ty, tz

    def step(self, now: float, dt: float, active=None):
        tx, ty, tz = self.target(now, active)
        self.vx = approach(self.vx, tx, ACCEL_LIN * dt)
        self.vy = approach(self.vy, ty, ACCEL_LIN * dt)
        self.vyaw = approach(self.vyaw, tz, ACCEL_ANG * dt)
        return self.vx, self.vy, self.vyaw

    def stop_now(self):
        self.deadline = dict.fromkeys(DIRS, 0.0)
        self.vx = self.vy = self.vyaw = 0.0

    @property
    def moving(self) -> bool:
        return max(abs(self.vx), abs(self.vy), abs(self.vyaw)) > EPS


class MotionTracker:
    """用里程计位置差算实测速度，比 sport state 里的 velocity 字段可靠。"""

    def __init__(self):
        self.last = None
        self.vx = self.vy = 0.0

    def update(self, pos, yaw: float, now: float):
        if pos is None:
            return None
        if self.last is None:
            self.last = (now, pos[0], pos[1])
            return (0.0, 0.0)
        t0, x0, y0 = self.last
        dt = now - t0
        if dt < 0.08:
            return (self.vx, self.vy)
        dx, dy = pos[0] - x0, pos[1] - y0
        c, s = math.cos(yaw), math.sin(yaw)
        self.vx = (c * dx + s * dy) / dt
        self.vy = (-s * dx + c * dy) / dt
        self.last = (now, pos[0], pos[1])
        return (self.vx, self.vy)


class SessionLog:
    """把会话写到文件，事后能复盘「指令发了没有、狗动了没有」。"""

    def __init__(self, path):
        self.f = None
        try:
            self.f = open(path, "w", buffering=1)
            self.f.write(f"# start {time.strftime('%H:%M:%S')}\n")
            self.f.write("# t  cmd_vx cmd_vy cmd_vyaw  move_ret  real_vx real_vy\n")
        except OSError:
            self.f = None

    def write(self, *parts):
        if self.f:
            self.f.write("  ".join(str(p) for p in parts) + "\n")

    def close(self):
        if self.f:
            self.f.write("# end\n")
            self.f.close()
            self.f = None


def render(vx: float, vy: float, vyaw: float, max_lin: float, active: str,
           elapsed: float, measured, gait: str | None = None,
           loco: str | None = None, loop: str = "") -> str:
    arrow = lambda v: "▮" * int(round(abs(v) / max(max_lin, 1e-6) * 8))
    if measured is None:
        real = "实测 --    "
    else:
        real = f"实测 {measured[0]:+.2f} {measured[1]:+.2f}"
    tag = ""
    if gait or loco:
        # 跳和跑现在是同一档风格，标签里只显示一次，省得状态行抻太长
        name = (loco or gait or "").split("（")[0]
        tag = f" 风格:{name}"
    return (
        f"\r{elapsed:5.1f}s  指令 vx {vx:+.2f} {arrow(vx):<8} "
        f"vy {vy:+.2f} vyaw {vyaw:+.2f} | {real} max {max_lin:.2f}"
        f" [{active or '·':<5}]{tag}{loop}"
    )


def run_segment(client, ctrl: "Controller", key: str, duration: float) -> float:
    """走一段：加速到位、保持 duration、再减速到 0。返回本段实际发过的峰值线速度。"""
    dx, dy, dz = DIRS[key]
    tx, ty, tz = dx * ctrl.max_lin, dy * ctrl.max_lin, dz * ctrl.max_ang
    peak = 0.0
    t0 = last = time.monotonic()
    while True:
        now = time.monotonic()
        dt = min(now - last, 0.2)
        last = now
        ctrl.vx = approach(ctrl.vx, tx, ACCEL_LIN * dt)
        ctrl.vy = approach(ctrl.vy, ty, ACCEL_LIN * dt)
        ctrl.vyaw = approach(ctrl.vyaw, tz, ACCEL_ANG * dt)
        peak = max(peak, abs(ctrl.vx), abs(ctrl.vy))
        if client:
            client.Move(ctrl.vx, ctrl.vy, ctrl.vyaw)
        if now - t0 >= duration:
            break
        time.sleep(1.0 / RATE)

    last = time.monotonic()
    while ctrl.moving:
        now = time.monotonic()
        dt = min(now - last, 0.2)
        last = now
        ctrl.vx = approach(ctrl.vx, 0.0, ACCEL_LIN * dt)
        ctrl.vy = approach(ctrl.vy, 0.0, ACCEL_LIN * dt)
        ctrl.vyaw = approach(ctrl.vyaw, 0.0, ACCEL_ANG * dt)
        if client:
            client.Move(ctrl.vx, ctrl.vy, ctrl.vyaw)
        time.sleep(1.0 / RATE)
    if client:
        client.StopMove()
    return peak


def require_iface(iface: str) -> None:
    """网卡不在就早点说清楚，别等 DDS 报一句看不懂的错。"""
    if Go2Robot.interface_exists(iface):
        return
    avail = ", ".join(n for _, n in socket.if_nameindex())
    raise RuntimeError(f"本机没有网卡 {iface}。当前可用：{avail}")


# 真机在线调参：命令行 --kp/--kd/--tau 写这里，OursBackend 建控制器时读。
# 放在模块级是为了不改 make_backend 的签名（它已经被三处调用）。
TUNE: dict = {}


class OursBackend:
    """自研：Go2Robot → Go2Controller → HighLevelController。

    高层本身就实现了 SportClient 那一套方法，所以 client 就是它本身，
    不需要再包一层。位置和姿态取自控制器在 connect 时订阅的 rt/sportmodestate。
    """

    kind = "ours"
    label = "自研控制器 HighLevelController"

    def __init__(self, preset, domain, iface):
        self.robot = Go2Robot(preset, domain=domain, iface=iface,
                              kp=TUNE.get("kp"), kd=TUNE.get("kd"),
                              tau_ff=TUNE.get("tau"))
        c = self.robot.controller
        self.target = f"网卡 {c.iface} / domain {c.domain}"
        require_iface(c.iface)
        if not self.robot.connect():
            raise RuntimeError(
                f"{c.iface}（domain {c.domain}）3 秒内没收到 rt/lowstate。"
                "仿真先起 ./quadruped/sim/run_sim.sh；真机先确认 setup_en5.sh 过了")
        self.client = c.high_level

    @property
    def loop_text(self) -> str:
        """控制回路节奏，形如「 回路5.0/6.2」。真机抖不抖先看这个：
        均值超过 6 ms 或 max 跳到十几毫秒，就是时序问题，不是参数问题。"""
        st = self.robot.controller.loop_stats()
        if not st.get("n"):
            return ""
        return f" 回路{st['mean']:.1f}/{st['max']:.1f}ms"

    @property
    def pos(self):
        return self.robot.pos

    @property
    def rpy(self):
        return self.robot.rpy

    @property
    def height(self):
        return self.robot.controller.body_height()

    @property
    def busy(self):
        """腿正被跳跃 / 蹲伏占着。这时候别再发速度，等动作播完自然接着走。"""
        return self.robot.controller.busy

    def jump(self) -> int:
        return self.client.jump_async()          # 排完就返回，不等落地

    @property
    def loco_gait(self) -> str:
        return self.client.gait_style_label

    def toggle_loco_gait(self) -> str:
        """机械 ↔ 仿生：跑法跟着换，跳跃动作也一起换（同一档风格）。"""
        self.client.toggle_jump_style()
        return self.client.gait_style_label

    @property
    def jump_gait(self) -> str:
        return self.client.jump_style_label

    def toggle_jump_gait(self) -> str:
        """机械 ↔ 仿生：跳跃动作跟着换，跑法也一起换（同一档风格）。"""
        self.client.toggle_gait_style()
        return self.client.jump_style_label

    def crouch(self) -> int:
        return self.client.crouch_async(0.08)

    def uncrouch(self) -> int:
        return self.client.uncrouch_async()

    def stand(self) -> int:
        return self.client.stand_async()

    def stop_motion(self) -> int:
        """停车，不等收步完成 —— 20 Hz 主循环不能在这里睡着。"""
        return self.client.stop_async()

    def freeze(self) -> int:
        """停止：中断动作并站定，仍然不阻塞。"""
        return self.client.stand_async()

    def close(self):
        """停步 → 蹲下 → 阻尼。仿真桥不发指令就没有力矩，必须自己蹲好再松。"""
        self.client.park()


class OfficialBackend:
    """官方：unitree_sdk2py 的 SportClient，调真机上的原生运动服务。

    只在真机可用 —— 仿真的桥接层只收发 rt/lowcmd 和 rt/sportmodestate，
    没有 rt/api/sport/request 的服务端，请求会一直等到超时。
    """

    kind = "official"
    label = "官方控制器 SportClient"

    def __init__(self, preset, domain, iface):
        require_iface(iface)
        ChannelFactoryInitialize(domain, iface)
        self._msg = None
        self.target = f"网卡 {iface} / domain {domain}"
        ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(
            lambda m: setattr(self, "_msg", m), 10)
        self.client = SportClient()
        self.client.SetTimeout(3.0)
        self.client.Init()

    @property
    def pos(self):
        return tuple(self._msg.position) if self._msg is not None else None

    @property
    def rpy(self):
        return tuple(self._msg.imu_state.rpy) if self._msg is not None else None

    @property
    def height(self):
        return None          # 原生状态里的 position 与 base_link 差一个常数，不做换算

    @property
    def busy(self):
        """原生服务在机器上跑，这些动作都是同步 RPC，这边没有「占着腿」的状态。"""
        return False

    def jump(self) -> int:
        return self.client.FrontJump()      # 原生只有前跳，没有原地跳

    @property
    def loco_gait(self) -> str | None:
        return None                         # 原生的运动步态在机器上

    def toggle_loco_gait(self) -> str | None:
        return None

    @property
    def jump_gait(self) -> str | None:
        return None                         # 原生步态在机器上，改不了

    def toggle_jump_gait(self) -> str | None:
        return None

    def crouch(self) -> int:
        """原生没有「蹲下」这个接口，用坐下代替（后腿折叠，前腿撑地）。"""
        return self.client.Sit()

    def uncrouch(self) -> int:
        return self.client.RiseSit()

    def stand(self) -> int:
        return self.client.BalanceStand()

    def stop_motion(self) -> int:
        return self.client.StopMove()

    def freeze(self) -> int:
        self.client.StopMove()
        return self.client.BalanceStand()

    def close(self):
        """原生运动服务自己有平衡环，停住就行，不需要 Damp 趴下。"""
        self.client.StopMove()


def make_backend(kind: str, preset: str, domain: int, iface: str):
    return (OfficialBackend if kind == "official" else OursBackend)(
        preset, domain, iface)


def ask_iface(default: str) -> str:
    """开跑前问一句用哪块网口。回车 = 仿真（lo0）。

    列的是本机真有的网卡（ifconfig 解析），加上真机的默认网卡 —— 那块没插的
    时候也列出来，标一句「未检测到」，省得人以为程序没认出来。
    """
    sim = Go2Robot.sim_network()
    real_pre = Go2Robot.PRESETS["real"]["iface"]

    items: list[dict] = [dict(name=sim["iface"], ip="",
                              note=f"仿真器（domain {sim['domain']}）")]
    seen = {sim["iface"]}

    for it in Go2Robot.list_ifaces():
        n = it["name"]
        if n in seen or n.startswith("lo"):
            continue
        # 没激活又没地址的虚拟网卡（Thunderbolt/USB 口的占位名）不列
        if not it["active"] and not it["ip"] and n != real_pre:
            continue
        if it["ip"]:
            off = "" if Go2Robot.is_real_robot_net(it["ip"]) \
                else "，非 192.168.123 网段"
            note = f"{it['ip']:<15s} 真机（domain 0{off}）"
        else:
            note = "真机默认网卡，当前未检测到"
        items.append(dict(name=n, ip=it["ip"], note=note))
        seen.add(n)

    if real_pre not in seen:
        items.append(dict(name=real_pre, ip="", note="真机默认网卡，当前未检测到"))

    # 接机器人的网段（192.168.123.x）排前面，其次才是普通网卡
    head, tail = items[:1], items[1:]
    head += [x for x in tail if Go2Robot.is_real_robot_net(x["ip"])]
    others = [x for x in tail if not Go2Robot.is_real_robot_net(x["ip"])]

    print("\n网口：")
    for i, it in enumerate(head + others, 1):
        print(f"  {i}) {it['name']:<6s} {it['note']}")

    order = head + others
    default_no = next((i for i, it in enumerate(order, 1)
                       if it["name"] == default), 1)
    if not sys.stdin.isatty():
        print(f"  非交互终端，用 {order[default_no - 1]['name']}")
        return order[default_no - 1]["name"]

    while True:
        try:
            prompt = (f"  选择 1-{len(order)}，回车 = {default_no}：" if len(order) > 1
                      else "  回车继续：")
            ans = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return order[default_no - 1]["name"]
        if not ans:
            return order[default_no - 1]["name"]
        try:
            n = int(ans)
        except ValueError:
            print(f"  只认 1 到 {len(order)}。")
            continue
        if not 1 <= n <= len(order):
            print(f"  只认 1 到 {len(order)}。")
            continue
        return order[n - 1]["name"]


def ask_client(options: tuple[str, ...], default: str, preset: str) -> str:
    """开跑前问一句用哪个控制端。

    options 由环境给出（Go2Robot.CLIENTS）：仿真只有自研一个，真机两个都有。
    只有一个的时候也停下来问一句 —— 让人看清自己连的是哪个控制器，回车即可，
    比"程序自己决定了"清楚。
    """
    print("\n控制端：")
    for i, kind in enumerate(options, 1):
        print(f"  {i}) {CLIENT_DESC[kind]}")
    if len(options) == 1 and preset == "sim":
        print("  仿真桥只有 rt/lowcmd 和 rt/sportmodestate，没有运动服务，"
              "官方控制器在这边连不上，所以只给这一个。")

    default_key = str(options.index(default) + 1)
    if not sys.stdin.isatty():
        print(f"  非交互终端，按默认选：{CLIENT_NAMES[default]}")
        return default

    while True:
        try:
            prompt = (f"  选择 1-{len(options)}，回车 = {default_key}：" if len(options) > 1
                      else "  回车继续：")
            ans = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return default
        if not ans:
            return default
        try:
            n = int(ans)
        except ValueError:
            print(f"  只认 1 到 {len(options)}。")
            continue
        if not 1 <= n <= len(options):
            print(f"  只认 1 到 {len(options)}。")
            continue
        return options[n - 1]


def line_control(backend, ctrl: "Controller") -> float:
    """给只按行转发输入的终端（IDEA Run 控制台）用的指令模式。返回峰值线速度。"""
    client = backend.client if backend is not None else None
    print(LINE_HELP)
    peak = 0.0
    crouched = False
    while True:
        try:
            line = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue

        parts = line.split()
        cmd = parts[0]

        if cmd in ("quit", "exit"):
            break
        if cmd == "stop":
            ctrl.stop_now()
            if client:
                client.StopMove()
                client.BalanceStand()
            print("  已停")
            continue
        if cmd == "jump":
            if backend:
                print(f"  跳跃 -> {backend.jump()}")
            else:
                print("  （dry-run，不连狗）")
            continue
        if cmd in ("gait", "jgait"):
            fn = (backend.toggle_jump_gait if cmd == "jgait"
                  else backend.toggle_loco_gait) if backend else None
            label = fn() if fn else None
            print(f"  风格 -> {label}" if label
                  else "  （官方控制器的步态在机器上 / dry-run）")
            continue
        if cmd == "crouch":
            crouched = not crouched
            if backend:
                code = backend.crouch() if crouched else backend.uncrouch()
                print(f"  {'蹲下' if crouched else '起立'} -> {code}")
            else:
                print(f"  （dry-run）{'蹲下' if crouched else '起立'}")
            continue
        if cmd == "speed" and len(parts) > 1:
            try:
                ctrl.max_lin = max(0.05, min(float(parts[1]), 1.0))
            except ValueError:
                print("  用法: speed 0.3")
                continue
            print(f"  最大线速度 {ctrl.max_lin:.2f} m/s")
            continue
        if cmd in DIRS:
            try:
                dur = float(parts[1]) if len(parts) > 1 else 1.0
            except ValueError:
                print("  用法: w 1.5")
                continue
            dur = max(0.1, min(dur, 5.0))
            print(f"  执行 {cmd} {dur:.1f}s …")
            peak = max(peak, run_segment(client, ctrl, cmd, dur))
            print("  完成")
            continue
        if cmd == "help":
            print(LINE_HELP)
            continue
        print("  不认识的指令，输入 help 看用法")

    return peak


def main(argv=None) -> int:
    """argv 为 None 时读 sys.argv；启动器直接调用本函数传参。"""
    ap = argparse.ArgumentParser(description="键盘遥控 Go2（官方 SportClient / 自研高层控制器）")
    ap.add_argument("--iface", default=None,
                    help="网卡名：lo0 = 仿真（域号读仿真器 config.py），其它 = 真机 domain 0")
    ap.add_argument("--preset", choices=("sim", "real"), default=None,
                    help="快捷方式：sim=lo0/domain1，real=en5/domain0")
    ap.add_argument("--domain", type=int, default=None, help="直接指定 DDS 域号，最高优先级")
    ap.add_argument("--client", choices=("official", "ours"), default=None,
                    help="跳过询问，直接用指定控制器（仿真只有 ours 可用）")
    ap.add_argument("--max-speed", type=float, default=0.4, help="最大线速度 m/s，默认 0.4")
    ap.add_argument("--max-yaw", type=float, default=0.8, help="最大角速度 rad/s，默认 0.8")
    ap.add_argument("--duration", type=float, default=0.0, help="最长运行秒数，0=不限")
    ap.add_argument("--dry-run", action="store_true", help="不连狗，只打印速度")
    ap.add_argument("--yes", action="store_true", help="跳过开始前确认")
    ap.add_argument("--keys", choices=("auto", "global", "terminal"), default="auto",
                    help="按键源：global=系统级监听（多键同时按），terminal=终端字符流")
    ap.add_argument("--keys-anywhere", action="store_true",
                    help="全局监听时不限制前台应用（默认只在前台是终端/IDE 时响应）")
    ap.add_argument("--debug-keys", action="store_true", help="把每次按键事件打出来，排查无反应")
    ap.add_argument("--line-mode", action="store_true",
                    help="命令行模式：用 'w 1.5' 这种指令控制，适合只能按行转发的终端")
    ap.add_argument("--log", default="/tmp/go2_kb.log", help="会话日志路径")
    ap.add_argument("--kp", type=float, default=None,
                    help="关节刚度，不给就用控制器默认（110）")
    ap.add_argument("--kd", type=float, default=None,
                    help="关节阻尼，不给就用控制器默认（1.5）")
    ap.add_argument("--tau", type=float, default=None,
                    help="重力前馈强度 0~1（1=完全补偿体重），真机默认 1")
    args = ap.parse_args(argv)
    for k in ("kp", "kd", "tau"):
        if getattr(args, k) is not None:
            TUNE[k] = getattr(args, k)

    # 环境由网卡决定：回环 = 仿真，其它 = 真机。域号跟着走（仿真 1 / 真机 0）。
    # 没在命令行指定网口/预设时先问一句，选完才知道这个环境能用哪些控制端。
    if args.iface or args.preset:
        cfg = Go2Robot.resolve(args.preset, domain=args.domain, iface=args.iface)
    else:
        iface = ask_iface(default=Go2Robot.sim_network()["iface"])
        cfg = Go2Robot.resolve(iface=iface, domain=args.domain)
    in_sim = cfg["preset"] == "sim"
    options = Go2Robot.clients_for(cfg["preset"])     # 仿真 1 个，真机 2 个
    default = "ours" if in_sim else "official"
    print(f"\n网口 {cfg['iface']}，DDS 域 {cfg['domain']}"
          f"（{'仿真' if in_sim else '真机'}）")

    # 选控制端放在建 Keyboard 之前：这时终端还是正常模式，输入会回显
    if args.client:
        kind = args.client
        if kind not in options:
            print(f"{'仿真' if in_sim else '真机'}只能用"
                  f"{' / '.join(CLIENT_NAMES[k] for k in options)}，"
                  f"没有{CLIENT_NAMES[kind]}。")
            return 2
    else:
        kind = ask_client(options, default, cfg["preset"])

    # --line-mode 用 input() 读整行，全局监听会把终端设成关回显的原始模式，两者别混
    kb = (TerminalKeys() if args.line_mode
          else make_keys(args.keys, anywhere=args.keys_anywhere))
    print(f"按键输入: {type(kb).__name__}")
    if kb.live:
        print("  全局监听：按住就走、松开就停，多个方向可以同时按。")
        if args.keys_anywhere:
            print("  任意窗口都收按键 —— 去别的窗口打字也会驱动机器人。")
        else:
            ok = kb.front_ok()
            print(f"  只在前台是终端 / IDE 时响应（当前前台: {kb.front_name}，"
                  f"{'对得上' if ok else '不在白名单，按键会被丢掉'}）。")
            if not ok:
                print("  点一下终端窗口让它到最前，或加 --keys-anywhere。")
        print("  首次运行 macOS 会要「输入监控」权限（系统设置 → 隐私与安全性）。")
    else:
        print("  终端字符流，方向靠 0.55 秒意图维持，按住 w 再按 q 会掉一个。")
        if kb.tty_mode:
            print("  单键立即生效。Ctrl 在终端里读不到，蹲下请用 --line-mode 或全局监听。")
        else:
            print("  警告: 拿不到控制终端，按键走行缓冲，必须按回车才生效。")
            print("  建议改用系统 Terminal.app / iTerm2 运行。")
    print("  若按键仍无反应，把输入法切到英文（ABC）——中文输入法会截走按键。")

    backend = None
    client = None
    ctrl = Controller(args.max_speed, args.max_yaw)
    help_on = not args.dry_run
    peak = 0.0
    stopped = True
    crouched = False
    send_err = 0
    got_key = False
    hint_done = False

    t_start = time.monotonic()
    log = None

    try:
        if not args.dry_run:
            try:
                backend = make_backend(kind, cfg["preset"], cfg["domain"], cfg["iface"])
            except RuntimeError as e:
                print(f"\n连不上：{e}")
                return 1
            client = backend.client
            print(f"\n控制端 {backend.label}，目标 {backend.target}")

            # 两个控制端都实现了这套方法，下面不再区分
            client.SetTimeout(3.0)
            client.Init()
            code = client.BalanceStand()
            print(f"BalanceStand -> {code}")
            if code != 0:
                print("对端没响应。官方控制器先 ping 192.168.123.161 再试。")
                return 1

            time.sleep(1.0)
            h = backend.height
            rpy = backend.rpy
            if h is None and rpy is None:
                print("收不到 rt/sportmodestate，实测速度无法显示（不影响控制）")
            else:
                parts = []
                if h is not None:
                    parts.append(f"站立高度 {h:.3f} m")
                if rpy is not None:
                    parts.append("姿态 rpy=(" + ", ".join(f"{v:+.2f}" for v in rpy) + ")")
                print("机器人状态: " + "  ".join(parts))

            print(HELP)
            print(f"最大线速度 {args.max_speed:.2f} m/s，最大角速度 {args.max_yaw:.2f} rad/s")
            print("确认狗周围至少 1 米空旷，人别站在它正前方。")

            if not args.yes:
                print("\n还没开始。按 Enter 开始，ESC 取消。"
                      "（方向键这会儿不会让狗动）")
                if kb.live and not kb.front_ok():
                    print(f"  ⚠ 前台是「{kb.front_name}」，不在白名单里，"
                          "按键会被丢掉 —— 点一下终端窗口，或加 --keys-anywhere。")
                kb.events()                     # 丢掉这之前攒下的事件
                waiting = True
                nudged = False
                base_presses = kb.dir_presses()

                def nudge() -> None:
                    nonlocal nudged
                    if not nudged:
                        nudged = True
                        print("  方向键收到了，但还没开始 —— 先按 Enter。")

                while waiting:
                    kb.pump(1.0)
                    for name, _ in kb.events():
                        if name == "quit":
                            print("已取消。")
                            return 0
                        if name == "stop":          # Enter
                            waiting = False
                            break
                        if name == "dir":           # 终端模式的字符流才有这条
                            nudge()
                    # 全局监听的方向键不进事件队列，看按下计数
                    if kb.dir_presses() > base_presses:
                        nudge()
                print("开始。")

        if args.line_mode:
            peak = line_control(backend, ctrl)     # 峰值与时长交给 finally 统一打印
            return 0

        t_start = last = time.monotonic()
        tracker = MotionTracker()
        log = SessionLog(args.log)
        log_tick = 0
        if log.f:
            print(f"会话日志: {args.log}")

        def dispatch(name: str) -> None:
            """一次性动作：交给后端，**发完就返回**。

            所有动作都不阻塞 —— 跳、蹲、起立的序列在控制回路里播，主循环照常
            20 Hz 跑。所以跳着的时候还能按别的键，落地那一下该走的方向已经接上了。
            """
            nonlocal crouched, stopped
            if name == "loco_gait":
                if backend is None:
                    print("\n切换风格（dry-run，不连狗）", flush=True)
                    return
                label = backend.toggle_loco_gait()
                print(f"\n风格 → {label}（跳和跑一起换）" if label else
                      "\n官方控制器的步态在机器上，改不了", flush=True)
                return
            if name == "jump_gait":
                # 切风格只是改下一跳怎么跳、下一步怎么迈，腿上的动作照旧，
                # 别顺手把车停了
                if backend is None:
                    print("\n切换风格（dry-run，不连狗）", flush=True)
                    return
                label = backend.toggle_jump_gait()
                print(f"\n风格 → {label}（跳和跑一起换）" if label else
                      "\n官方控制器只有一种前跳，切不了步态", flush=True)
                return
            if name == "stop":
                ctrl.stop_now()                 # 只有「停止」要把速度意图清零
            if client and not backend.busy:
                backend.stop_motion()
            stopped = True
            if name == "stop":
                print("\n停止：收步并站定", flush=True)
            elif name == "balance":
                print("\n站立平衡", flush=True)
            elif name == "jump":
                print("\n跳跃", flush=True)
            else:
                crouched = not crouched
                print("\n蹲下" if crouched else "\n起立", flush=True)
            if backend is None:
                return
            if name == "stop":
                code = backend.freeze()
            elif name == "balance":
                code = backend.stand()
            elif name == "jump":
                code = backend.jump()
            else:
                code = backend.crouch() if crouched else backend.uncrouch()
            print(f"  -> {code}", flush=True)

        while True:
            kb.pump(1.0 / RATE)
            now = time.monotonic()
            dt = min(now - last, 0.2)
            last = now

            quit_now = False
            for name, val in kb.events():
                if args.debug_keys:
                    sys.stdout.write(f"\nKEY {name} {'' if val is None else val}\n")
                if not got_key:
                    got_key = True
                    sys.stdout.write("\n✓ 键盘输入已连通\n")
                if name == "dir":
                    ctrl.press(val, now)          # 终端模式靠它续期意图
                elif name == "speed":
                    ctrl.max_lin = val / 10.0
                    print(f"\n最大线速度 {ctrl.max_lin:.2f} m/s")
                elif name == "help":
                    help_on = not help_on
                    print(HELP if help_on else "\n(帮助已关闭)")
                elif name == "quit":
                    quit_now = True
                else:                             # jump / crouch / stop / balance
                    dispatch(name)

            if not got_key and not hint_done and now - t_start > 5:
                hint_done = True
                sys.stdout.write("\n还没收到任何按键。焦点不在终端时先点一下终端窗口；"
                                 "IDEA 的 Run 控制台只按行转发输入，请加 --line-mode。\n")

            if quit_now:
                break

            # 全局监听有真实的按住状态，直接用；终端模式没有，退回 deadline 意图
            active = {k for k in DIRS if kb.pressed(k)} if kb.live else None
            vx, vy, vyaw = ctrl.step(now, dt, active)
            peak = max(peak, abs(vx), abs(vy))

            ret: int | None = None
            if client:
                if backend.busy:
                    # 腿在被跳跃 / 蹲伏占着，这会儿别插速度指令。方向键仍照常
                    # 记着，动作一播完（下一帧 busy 变 False）就接着走。
                    pass
                elif ctrl.moving:
                    ret = client.Move(vx, vy, vyaw)
                    if ret != 0:
                        send_err += 1
                        if send_err >= 5:
                            print("\n连续 5 次发送失败，停车退出")
                            break
                    stopped = False
                elif not stopped:
                    ret = backend.stop_motion()     # 非阻塞：收步交给控制回路
                    stopped = True

            pos = backend.pos if backend is not None else None
            rpy = backend.rpy if backend is not None else None
            yaw = rpy[2] if rpy else 0.0
            real = tracker.update(pos, yaw, now)
            held = active if active is not None else {
                k for k in DIRS if ctrl.deadline[k] > now}
            elapsed = now - t_start
            sys.stdout.write(render(vx, vy, vyaw, ctrl.max_lin,
                                    "".join(sorted(held)), elapsed, real,
                                    backend.jump_gait if backend else None,
                                    backend.loco_gait if backend else None,
                                    getattr(backend, "loop_text", "")))
            sys.stdout.flush()

            log_tick += 1
            if log and log_tick % 5 == 0:
                log.write(f"{elapsed:6.2f} {vx:+.2f} {vy:+.2f} {vyaw:+.2f}"
                          f"  {ret if ret is not None else '-':<3}"
                          f"{(real or (0, 0))[0]:+.2f} {(real or (0, 0))[1]:+.2f}")

            if args.duration and elapsed >= args.duration:
                print("\n到时长上限，停车")
                break

    except (KeyboardInterrupt, EOFError):
        print("\n中断")
    finally:
        kb.restore()
        if log:
            log.close()
        ctrl.stop_now()
        if backend is not None:
            backend.close()
            if backend.kind == "ours":
                print("\n收工：停步 → 蹲下 → 阻尼。进程退出后没人发 lowcmd，"
                      "腿会直接失力，所以先蹲好再松。")
            else:
                print("\n收工：已 StopMove，原生运动服务继续保持站立。")
        if args.dry_run or backend is not None:
            print(f"峰值速度 {peak:.2f} m/s，运行 {time.monotonic() - t_start:.1f}s")

    return 0


if __name__ == "__main__":
    sys.exit(main())
