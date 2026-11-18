"""项目入口的启动器：先问你要跑哪个控制器，再把它拉起来。

启动器不登记任何控制器，也不登记分组。它只做一件事：扫 `quadruped/control/`
下的每个子包，谁在自己的 `__init__.py` 里声明了 `GROUP`，谁就出现在菜单里。

    physical/__init__.py    GROUP = dict(title="物理控制器 · 直接驱动机器人", order=1)
    embodied/__init__.py    GROUP = dict(title="具身智能 · 决策层", order=2)

没声明 `GROUP` 的子包就是不上菜单的内部实现（`virtual/` 是抽象层，属于这种）。
组里每个模块的形状由 `control/virtual/protocol.py` 定义：模块级 `META` +
`main(argv)`。所以**加一个控制器 = 加一个文件，加一个分组 = 加一个目录**，
两件事都不用回来改这里。

为什么要开子进程而不是 import 进来直接跑：`ChannelFactoryInitialize` 一个
进程只能调一次，第二个控制器在同一进程里初始化 DDS 会静默拿不到数据。多选
时每个控制器一个子进程，各跑各的。

用法：
    uv run python main.py                # 交互：环境 → 运控 → 控制器
    uv run python main.py keyboard       # 跳过选择，直接跑键盘遥控（默认仿真 / MPC+WBC）
    uv run python main.py --sim          # 只起 MuJoCo 仿真器，占住当前终端
    uv run python main.py --preset real --motion mpc keyboard
    uv run python main.py --list         # 只列清单
    uv run python main.py keyboard -- --iface en5     # -- 之后原样传给控制器
"""

import argparse
import importlib
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))          # quadruped/cli
ROOT = os.path.dirname(os.path.dirname(_HERE))              # 项目根

# `python quadruped/cli/launcher.py` 时 sys.path[0] 是 `quadruped/cli`，项目根
# 不在里面，下面那句 `import quadruped` 直接 ModuleNotFoundError。`main.py` 与
# `test/` 下几个脚本都有同样的引导，这里补上，两个入口都能直接跑。
if __package__ in (None, ""):
    sys.path.insert(0, ROOT)

from quadruped.cli.prompt import Form, ask_line, pad, parse_indices, pick  # noqa: E402
from quadruped.control.virtual.protocol import check_module, normalize_group  # noqa: E402
from quadruped.sim.scenes import scene_menu, scene_path  # noqa: E402

CONTROL_DIR = os.path.join(os.path.dirname(_HERE), "control")


class Group:
    """一个分组。标题、顺序由该目录自己的 `__init__.py` 声明，这里不登记。"""

    def __init__(self, name, title, order=50.0, error=None):
        self.name = name
        self.title = title
        self.order = order
        self.error = error

    @property
    def ok(self):
        return self.error is None


# 扫 control/ 下的子包，读 `GROUP`。没声明的跳过，声明的按 order 排。
#
# 导入失败的包也留一条 —— 一个包整个 import 不进来，菜单里空着比报错更难查。
def discover_groups() -> list[Group]:
    groups: list[Group] = []
    for name in sorted(os.listdir(CONTROL_DIR)):
        if name.startswith((".", "_")) or name == "__pycache__":
            continue
        d = os.path.join(CONTROL_DIR, name)
        if not os.path.isfile(os.path.join(d, "__init__.py")):
            continue                                  # 不是包（比如 controller.py 那种单文件）
        try:
            mod = importlib.import_module(f"quadruped.control.{name}")
        except Exception as e:                        # noqa: BLE001
            groups.append(Group(name, name, error=f"{type(e).__name__}: {e}"))
            continue
        raw = getattr(mod, "GROUP", None)
        if raw is None:
            continue                                  # 不上菜单的内部实现
        try:
            meta = normalize_group(raw, name)
        except ValueError as e:
            groups.append(Group(name, name, error=str(e)))
            continue
        groups.append(Group(name, meta["title"], meta["order"]))
    groups.sort(key=lambda g: (g.order, g.name))
    return groups


class Entry:
    """一个控制器。导入失败也留个条目，菜单里标出来，别假装它不存在。"""

    def __init__(self, group, stem, module, meta=None, error=None):
        self.group = group
        self.stem = stem
        self.module = module
        self.meta = meta or {}
        self.error = error

    @property
    def title(self):
        return self.meta.get("title", self.stem)

    @property
    def desc(self):
        return self.meta.get("desc", "")

    @property
    def owns_cmd(self):
        return bool(self.meta.get("owns_cmd", False))

    @property
    def owns_keys(self):
        return bool(self.meta.get("owns_keys", False))

    @property
    def runnable(self):
        return self.error is None


# 在每个分组目录里扫模块，导入后按协议校验 META。
def discover(groups: list[Group]) -> list[Entry]:
    found: list[Entry] = []
    for g in groups:
        if not g.ok:
            continue                                  # 组本身 import 不进来，菜单里单独标错
        d = os.path.join(CONTROL_DIR, g.name)
        for fn in sorted(os.listdir(d)):
            if not fn.endswith(".py") or fn.startswith("_"):
                continue
            stem = fn[:-3]
            module = f"quadruped.control.{g.name}.{stem}"
            try:
                mod = importlib.import_module(module)
            except Exception as e:                    # noqa: BLE001 任何导入错误都不该挡住菜单
                found.append(Entry(g.name, stem, module,
                                   error=f"{type(e).__name__}: {e}"))
                continue
            meta, err = check_module(mod, g.name, stem)
            if meta is None and err is None:
                continue                              # 没有 META 的不是控制器，静默跳过
            found.append(Entry(g.name, stem, module, meta=meta, error=err))
    return found


# 打印菜单，返回 编号 → 控制器。
def render_menu(groups: list[Group], entries: list[Entry]) -> dict[int, Entry]:
    print("\nGo2 统一入口\n")
    n = 0
    index: dict[int, Entry] = {}
    for g in groups:
        print(f"  {g.title}")
        if not g.ok:
            print(f"    （导入失败：{g.error}）\n")
            continue
        members = [e for e in entries if e.group == g.name]
        if not members:
            print("    （还没有，留空占位）\n")
            continue
        for e in members:
            n += 1
            index[n] = e
            if e.runnable:
                print(f"    {n}  {pad(e.title, 12)}{e.module}")
                if e.desc:
                    print(f"       {pad('', 12)}{e.desc}")
            else:
                print(f"    {n}  {pad(e.title, 12)}不可用 —— {e.error}")
        print()
    return index


# 问一句选哪个。回车 = 第一个能跑的。
def ask(groups: list[Group], entries: list[Entry]) -> list[Entry]:
    index = render_menu(groups, entries)
    if not index:
        print("目录里没有控制器。")
        return []
    default = next((k for k, e in index.items() if e.runnable), None)
    hint = f"回车 = {default}" if default else "没有可用的"
    while True:
        raw = ask_line(f"选哪个（多选用 1,2 或 1 2；q 退出；{hint}）：")
        if raw in ("q", "Q", "quit", "exit"):
            return []
        if not raw:
            if default is None:
                print("上面没有能跑的控制器。")
                return []
            return [index[default]]
        got = parse_indices(raw, len(index))
        if got is None:
            print(f"  「{raw}」不是菜单里的编号")
            continue
        picked: list[Entry] = []
        bad = False
        for n in got:
            e = index[n]
            if not e.runnable:
                print(f"  {e.title} 现在跑不了：{e.error}")
                bad = True
                break
            picked.append(e)
        if bad or not picked:
            continue
        return picked


# ---- 环境与运控 --------------------------------------------------------------
#
# 这两项不进控制器的 argparse：
#   环境用 `--preset`，各控制器早就支持（`add_endpoint_args` 里注册的）；
#   运控用环境变量 `GO2_MOTION`，由 `make_gait` 读 —— 所有入口的步态最后都过
#   那里，一处覆盖全部。启动器只负责收集，往下传。
#
# 运控默认开（三层是本体，vmc 退成回滚选项）。代价是回路周期 p95 从 5.8 ms
# 涨到 8.2 ms，超过 5 ms 的标称周期（见 TODO.md Step 3）；单帧计算只有 1.0 ms，
# 多出来的是 50 Hz 线程与 200 Hz 主回路抢 GIL。要单层给 `--motion vmc`。

ENVS = [
    ("sim-new", "仿真（一起起）", "先拉仿真器，就绪后接控制器"),
    ("sim", "仿真（已在跑）", "直接锁 lo0 / domain 1"),
    ("real", "真机", "列网口让你选（en5 / USB 网卡都可能）"),
]

# 每一档对应 (preset, 要不要先拉仿真器)。
ENV_SPEC = {
    "sim-new": ("sim", True),
    "sim": ("sim", False),
    "real": ("real", False),
    # 不替用户定环境：不传 `--preset`，交给控制器把网口列出来。点名启动
    # （`main.py keyboard`）走这一档 —— 那条路径以前默认锁死仿真，想连真机
    # 没有入口，等于网口菜单被顶层吞掉。
    "ask": (None, False),
}

# 只在 `SIM_PERCEPT=0`（仿真侧不发感知）时用的兜底等待。正常情况下不睡秒数，
# 由 `_wait_ready()` 收到第一帧感知来判断 —— 见那个函数的说明。
SIM_BOOT = 6.0

MOTIONS = [
    ("mpc", "开启（MPC + WBC）", "默认。三层；单帧 1.0 ms，回路 p95 8~9 ms"),
    ("vmc", "关闭（VMC）", "单层反馈；单帧 0.5 ms，回路 p95 5.8 ms"),
]

# 控制端。仿真桥只有 rt/lowcmd 和 rt/sportmodestate，没有运动服务，官方那条
# 在仿真连不上 —— 选错由控制端自己 `choose_client` 报错，这里不拦。
# 顺序有含义：`pick()` 拿列表下标当默认候选，`ask_settings()` 的初值又必须能在
# 这张表里找到，所以推荐项排第一。
#
# 「交给控制端自选」还在，但排到最后：它的实际结果是仿真落自研、真机落官方，
# 而自研才会开本仓库的 MPC + WBC。把它放在第一位，等于让人一路回车落进运控。
CLIENTS = [
    ("policy", "推理模型", "默认。策略网络直连 rt/lowcmd，不用运控"),
    ("ours", "自研", "本仓库的 HighLevelController，会建 Go2Controller"),
    ("official", "官方", "真机上的原生运动服务，仿真不可用"),
]

# 几何测距。一个开关，两种实现，由环境决定，**不是**同一个东西：
#
#   仿真 → MuJoCo 射线求交（`percept/sim.py`，percept_pub 发 UDP）
#   真机 → Go2 的 L1 雷达（`percept/lidar.py`，rt/utlidar/cloud）
#
# 之前 `--source lidar` 指向的其实是仿真射线，日志却写着 radar —— 混在一起没法
# 排查。现在这个开关只管「要不要测距」，具体接谁由 runner 按环境二选一，
# 启动时打一行「测距：…」说明实际用的是哪一个。
GEOMS = [
    ("on", "开启", "安全闸、栅格记忆、G 层选向都要它"),
    ("off", "关闭", "只用画面 + 本体遥测，撞了才知道"),
]

# 仿真场景。只在「仿真（一起起）」这一档有意义 —— 已经在跑的那个仿真器早就
# 按自己的场景起好了，真机更没有场景可换。
_SCENES = scene_menu()
_SCENE_TEXT = {k: f"{label}  {note}" for k, label, note in _SCENES}

_ENV_TEXT = {k: f"{label}  {note}" for k, label, note in ENVS}
_MOTION_TEXT = {k: label for k, label, _ in MOTIONS}
_CLIENT_TEXT = {k: label for k, label, _ in CLIENTS}


# 这一屏最终会落到哪个控制端。
#
# 用户还没选（`默认`）时按环境推：真机落官方（机器上本来就在跑 mcf，用它是
# 零摩擦的默认），仿真落推理模型（仿真桥只有 rt/lowcmd 和 rt/sportmodestate，
# 没有运动服务，官方在那头连不上；自研要额外建 Go2Controller，不是「什么都不选」
# 时该发生的事）。
#
# 运控那一行必须按这个实际结果说话，不能因为「还没选」就先把「开启（MPC + WBC）」
# 写上去：真机走官方、仿真走推理，两种都不执行本仓库的 MPC/WBC。
def _eff_client(all_: dict) -> str:
    cur = all_.get("client") or "__none__"
    if cur != "__none__":
        return cur
    return "official" if all_.get("env") == "real" else "policy"


# 运控这一行该写什么。三种控制端只有 `ours` 会建 Go2Controller。
def _motion_text(all_: dict) -> str:
    eff = _eff_client(all_)
    if eff == "policy":
        return "不适用（推理模型直连 rt/lowcmd）"
    if eff == "official":
        return "不适用（官方控制器走机器上的原生运控）"
    return _MOTION_TEXT[all_["motion"]]

# 放进表里是为了「临时试一下」不用记 `--` 透传，但必须标清楚它不是普通选项。
_NUM_NOTE = ("注：kp / kd 与模型一起标定（仿真与真机同一套），一般不要动。\n"
             "    改了下表只对当前环境成立，症状是「某个速度段开始打滑」，\n"
             "    不容易联想到刚度。回车 = 用默认。")


def _show(key: str):
    """字段的显示函数。第二个参数是整张表，用于跨字段的「不适用」。"""
    if key == "env":
        return lambda v, _all: _ENV_TEXT[v]
    if key == "motion":
        # 推理模型直连 rt/lowcmd，官方控制器发给机器上的原生运控 —— 两者都不建
        # Go2Controller，MPC/WBC 一次都不执行。这里写「开启」就是在骗人。
        return lambda v, all_: _motion_text(all_)
    if key == "client":
        # `__none__` = 还没定，按环境推（`_eff_client`）。要显示推出来的那个，
        # 不能把 `__none__` 直接写出来 —— 菜单是给人看的。
        def client_text(v, all_):
            if v in (None, "__none__"):
                return f"默认（{_CLIENT_TEXT[_eff_client(all_)]}）"
            return _CLIENT_TEXT[v]
        return client_text
    if key == "scene":
        def scene_text(v, all_):
            if all_.get("env") != "sim-new":
                return "不适用（只有「仿真（一起起）」要选场景）"
            return _SCENE_TEXT[v]
        return scene_text
    if key == "geom":
        # 同一项在不同环境落到不同实现，摘要里就把实现写出来，别只写「开启」。
        def geom_text(v, all_):
            if v == "off":
                return "关闭（只用画面，没有测距）"
            via = ("仿真射线" if all_.get("env") in ("sim", "sim-new")
                   else "Go2 雷达")
            return f"开启（{via}）"
        return geom_text
    return lambda v, _all: "默认" if v is None else f"{v:g}   ← 已偏离默认"


def _ask_number(name: str, cur):
    """读一个数。回车 = 回到默认（None），不是「保持当前」。"""
    raw = ask_line(f"  {name}（回车 = 用默认）：")
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        print(f"  「{raw}」不是数字，保持 {cur if cur is not None else '默认'}")
        return cur


# 两段式配置。
#
# 第一段只问「跑什么环境、接哪个控制端」。运控是不是要开，取决于控制端：三种
# 控制端里只有自研会建 `Go2Controller`，推理模型直连 `rt/lowcmd`，官方把指令发给
# 机器上的原生运控。所以运控这一项必须等控制端定下来之后才出现 —— 同一屏里先列
# 一个「开启（MPC + WBC）」再让人去选控制端，等于先给结论再问前提。
def ask_settings() -> dict:
    def edit_env(cur, _all):
        return pick("环境", ENVS, [k for k, *_ in ENVS].index(cur))

    def edit_client(cur, all_):
        # 当前是「默认」时，光标停在按环境推出来的那个上，回车 = 用它。
        # 之前这里直接 `index(cur)`，`__none__` 不在表里会抛 ValueError。
        keys = [k for k, *_ in CLIENTS]
        default = keys.index(cur) if cur in keys else keys.index(_eff_client(all_))
        return pick("控制端", CLIENTS, default)

    def edit_motion(cur, _all):
        return pick("运控", MOTIONS, [k for k, *_ in MOTIONS].index(cur))

    def edit_scene(cur, _all):
        return pick("场景", _SCENES, [k for k, *_ in _SCENES].index(cur))

    def edit_geom(cur, _all):
        return pick("几何测距", GEOMS, [k for k, *_ in GEOMS].index(cur or "on"))

    head = Form(
        [
            ("环境", "env", _show("env"), edit_env),
            ("场景", "scene", _show("scene"), edit_scene),
            ("控制端", "client", _show("client"), edit_client),
        ],
        # 初始值必须是 `__none__`，不能写死某一个控制端。
        #
        # 这里原来是 `client="policy"`，于是「按环境推」（`_eff_client`）永远走不到：
        # 用户把环境改成「真机」，控制端那行还停在「推理模型」，回车提交就带着
        # `--client policy` 下去了 —— 真机上键盘遥控直接走策略网络，
        # `claim()` 一拿写入权就 `ReleaseMode()`，用户的遥控器失效。
        # 用户看到的现象是「我选了官方，怎么用的是推理」，而他其实一次都没选过。
        dict(env="sim-new", scene="flat", client="__none__"),
        title="第 1 步 · 环境、场景与控制端",
    ).run()

    eff = _eff_client(head)
    rows = []
    if eff == "ours":
        # 只有这条路会建 `Go2Controller`，运控才有开不开的问题。
        rows.append(("运控", "motion", _show("motion"), edit_motion))
    note = _NUM_NOTE
    if eff != "ours":
        whose = ("推理模型控制端直连 rt/lowcmd" if eff == "policy"
                 else "官方控制器把指令发给机器上的原生运控")
        note = f"{whose}，本仓库的 MPC + WBC 不执行，所以这里不问运控。\n{_NUM_NOTE}"

    body = Form(
        rows + [
            # 默认开启。关掉之后安全闸不触发、记忆不进格子 —— 那是给「只想看
            # 纯视觉能走成什么样」用的，不是常态。
            ("几何测距", "geom", _show("geom"), edit_geom),
            ("刚度 kp", "kp", _show("kp"), lambda cur, _a: _ask_number("kp", cur)),
            ("阻尼 kd", "kd", _show("kd"), lambda cur, _a: _ask_number("kd", cur)),
        ],
        dict(head, motion="mpc", geom="on", kp=None, kd=None),
        title=f"第 2 步 · 参数（控制端 {_CLIENT_TEXT[eff]}）",
        note=note,
    ).run()
    return body


# 起 MuJoCo 仿真器。路径与 mjpython 的判断都在脚本里，这里只管把终端换过去。
# 仿真器是长驻 GUI，会占住这个终端，所以和控制器必须分两个终端跑。
def run_sim(scene: str | None = None) -> int:
    script = os.path.join(ROOT, "quadruped", "sim", "run_sim.sh")
    if not os.access(script, os.X_OK):
        print(f"找不到可执行的 {script}")
        return 2
    if scene is None:
        scene = pick("场景", _SCENES, 0)
    os.environ["SIM_SCENE"] = str(scene_path(scene))
    os.environ["GO2_SCENE"] = scene
    print(f"启动 MuJoCo 仿真器（场景 {_SCENE_TEXT[scene]}）…")
    print("控制器另开一个终端：uv run python main.py\n")
    sys.stdout.flush()
    os.chdir(ROOT)
    os.execv(script, [script])


# 仿真器 + 控制器一起起：一条命令、一个终端。
#
# 两个都必须是独立进程：mjpython 是 GUI 外壳（GUI 得在主线程），而
# `ChannelFactoryInitialize` 一个进程只能调一次，控制器也不能塞进本进程。
# 所以这里起两个子进程，自己只当管理者：等仿真器就绪 → 起控制器 → 控制器
# 退出或 Ctrl+C 时把仿真器收掉，不留孤儿窗口。
SIM_SCRIPT = os.path.join(ROOT, "quadruped", "sim", "run_sim.sh")

# 无窗口版本。物理和 DDS 链路和带窗口的完全是同一条（都在 `UnitreeSdk2Bridge`
# 里），差别只是不调 `launch_passive`。
#
# 需要一个「没有窗口也能跑」的入口，是因为从自动化脚本、远程会话、CI 里起仿真时
# `launch_passive` 会直接挂住：进程活着、一行输出都没有、DDS 上什么都没有，
# 表现是控制器报「3 秒内没收到 rt/lowstate」。这个错误看着像 DDS 的问题，
# 实际上是根本没人给它建窗口。
SIM_HEADLESS = os.path.join(ROOT, "quadruped", "sim", "unitree_mujoco",
                            "simulate_python", "headless.py")
SIM_PYTHON = os.path.join(ROOT, ".venv", "bin", "python")

# 等仿真真的开始发布感知帧，最多等这么久。
#
# 不睡固定秒数。窗口版启动一次要 mjpython 起一个 .app、建窗口、加载
# scene_maze.xml、建两个相机的渲染器，实测 4~12 s 都有；而控制器自己只等 3 s
# （`PolicyBackend` → `DirectLink.connect`）。两边一叠加，仿真慢一点就报
# 「3 秒内没收到 rt/lowstate」—— 这个报错看着像 DDS 故障，实际上仿真还在加载。
SIM_READY_LIMIT = 30.0


# 42300 有没有人在收。起仿真之前先问这一句。
#
# 必须在 `Popen` 之前查：等仿真起来再查就已经晚了 —— 第二个仿真已经在往同一个
# DDS 域发布，两边互相覆盖，狗会抽。这一步只花一次 bind，代价可以忽略。
def _percept_port_free() -> bool:
    from quadruped.percept.sim import default_port

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", default_port()))
        return True
    except OSError:
        return False
    finally:
        sock.close()


# 等到收到第一帧感知，才算仿真起来了。返回 "ready" / "busy" / "died" / "timeout"。
#
# 判据用感知帧而不是 lowstate：`percept_pub` 和 lowstate 都是从同一个仿真循环里
# 发出来的，收到第一帧射线就说明物理已经在推进。走 UDP 探一眼就走，不碰 DDS。
#
# 这里**故意不设 SO_REUSEADDR**。设了的话，即使别的进程已经在 42300 上收感知，
# 这个 bind 照样成功，于是两个 socket 抢同一批帧 —— 等于把正在跑的那一路的感知
# 偷走，症状是那边突然「感知已停」。bind 失败恰恰是要报的情况：上一次运行还没退。
def _wait_ready(sim: subprocess.Popen, limit: float = SIM_READY_LIMIT) -> str:
    from quadruped.percept.sim import default_port

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", default_port()))
    except OSError:
        sock.close()
        return "busy"
    sock.settimeout(0.2)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < limit:
            if sim.poll() is not None:
                return "died"
            try:
                sock.recvfrom(65535)
                return "ready"
            except (socket.timeout, BlockingIOError):
                continue
        return "died" if sim.poll() is not None else "timeout"
    finally:
        sock.close()



def run_with_sim(cmd: list[str], env: dict, scene: str = "flat",
                 window: bool = True) -> int:
    script = SIM_SCRIPT if window else SIM_HEADLESS
    if not os.access(script, os.R_OK):
        print(f"找不到 {script}")
        return 2
    path = scene_path(scene)
    if not path.is_file():
        print(f"场景文件不存在：{path}")
        return 2
    if not _percept_port_free():
        from quadruped.percept.sim import default_port
        print(f"端口 {default_port()} 已被占用：已经有另一个程序（上一次运行）"
              "在收感知。")
        print("  先停掉它再跑：pgrep -fl unitree_mujoco.py。"
              "两边同时往 rt/lowcmd 发命令会让狗抽。")
        return 2
    sim_env = dict(os.environ)
    sim_env["SIM_SCENE"] = str(path)
    title = _SCENE_TEXT.get(scene, scene)
    print(f"起 MuJoCo 仿真器（{'窗口' if window else '无窗口'}，场景 {title}，"
          f"{path.name}）…")
    sys.stdout.flush()
    if window:
        # 单独一个进程组。`mjpython` 是个包装器，它 exec 出来的 `.app` 才是真正
        # 的仿真进程，`terminate()` 只杀得掉包装器 —— 里面的 app 变成孤儿继续往
        # 同一个 DDS 域发布。实测两个孤儿活了一小时，之后每次运行读到的都是
        # 「一台翻倒的狗 + 一台站着的狗」交替，症状是 tilt 恒 180°、位置在
        # 两个值之间跳，而日志里看不出任何异常（因为进程都还在，报错也没有）。
        sim = subprocess.Popen([SIM_SCRIPT], cwd=ROOT, env=sim_env,
                               start_new_session=True)
    else:
        # 无窗口这一路给了 `--seconds`，但控制器先退出时下面的 `_kill_group`
        # 会先把它收掉，秒数只是兜底。
        sim = subprocess.Popen([SIM_PYTHON, SIM_HEADLESS, "--seconds", "3600"],
                               cwd=os.path.dirname(SIM_HEADLESS), env=sim_env,
                               start_new_session=True)
    ctrl = None
    code = 1
    try:
        # `SIM_PERCEPT=0` 的时候没有感知帧可等，退回睡固定秒数。
        percept_off = os.environ.get("SIM_PERCEPT", "1") == "0"
        print("等仿真器开始发布感知 …" if not percept_off else "等仿真器启动 …")
        sys.stdout.flush()
        t_boot = time.monotonic()
        if percept_off:
            time.sleep(SIM_BOOT)
            state = "ready"
        else:
            state = _wait_ready(sim)
        boot = time.monotonic() - t_boot
        if sim.poll() is not None:
            print(f"仿真器自己退了（返回码 {sim.returncode}），看上面的报错。")
            return sim.returncode or 1
        if state == "busy":
            from quadruped.percept.sim import default_port
            print(f"端口 {default_port()} 已被占用：已经有另一个程序在收感知，"
                  "多半是上一次运行还没退。")
            print("  先停掉它再跑：pgrep -fl unitree_mujoco.py；"
                  "两边同时往 rt/lowcmd 发命令会让狗抽。")
            return 1
        if state != "ready":
            print(f"仿真器 {SIM_READY_LIMIT:.0f} 秒内没有发布感知帧，先不接控制器。")
            print("  窗口版需要能显示窗口的图形会话；从脚本、远程会话、CI 里跑时"
                  "加 --no-window。")
            return 1
        print(f"仿真器就绪（{boot:.1f} s），接控制器\n")
        sys.stdout.flush()
        ctrl = subprocess.Popen(cmd, cwd=ROOT, env=env)
        code = ctrl.wait()
    except KeyboardInterrupt:
        print("\n中断 …")
        code = 130
    finally:
        # 先等控制器收尾，再关仿真器。顺序反了的话，控制器那一步「保持站姿 +
        # 断 lowcmd」会连不上已经死掉的 DDS，白等一轮超时。
        if ctrl is not None and ctrl.poll() is None:
            try:
                ctrl.wait(timeout=6)
            except subprocess.TimeoutExpired:
                ctrl.terminate()
        if sim.poll() is None:
            print("收仿真器 …")
            _kill_group(sim)
    return code


# 结束仿真器的整个进程组。
#
# 只 `terminate()` 是不够的（见上面起进程那段注释）：包装器收到 SIGTERM 就退，
# 它 exec 出来的子进程还活着。先 SIGTERM 整组，5 秒不退再 SIGKILL 整组。
def _kill_group(proc: subprocess.Popen) -> None:
    import signal
    try:
        pgid = os.getpgid(proc.pid)
    except OSError:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except OSError:
            return
        try:
            proc.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            continue


# 两个控制器同时占 lowcmd / 键盘，DDS 上会互相覆盖，先拦一下。
def check_conflict(entries: list[Entry], assume_yes: bool) -> bool:
    problems = []
    if sum(e.owns_cmd for e in entries) > 1:
        names = "、".join(e.title for e in entries if e.owns_cmd)
        problems.append(f"{names} 都会往 rt/lowcmd 发指令，同时跑会互相覆盖，"
                        "看起来就是「腿一跳一跳的」。")
    if sum(e.owns_keys for e in entries) > 1:
        names = "、".join(e.title for e in entries if e.owns_keys)
        problems.append(f"{names} 都会抢键盘。")
    if not problems:
        return True
    print()
    for p in problems:
        print(f"  警告：{p}")
    if assume_yes:
        print("  --yes：继续。")
        return True
    try:
        ans = input("  还是要一起跑？[y/N] ").strip().lower()
    except EOFError:
        ans = ""
    return ans in ("y", "yes")


def launch(entries: list[Entry], extra: list[str], cfg: dict,
           assume_yes: bool = False, with_sim: bool = False,
           window: bool = True) -> int:
    if not entries:
        return 1
    if extra and len(entries) > 1:
        print("多个控制器时不能带额外参数（会被塞给每一个）。先只选一个，或去掉 --。")
        return 2
    if not check_conflict(entries, assume_yes):
        print("已取消。")
        return 2

    preset, auto_sim = ENV_SPEC[cfg["env"]]
    with_sim = with_sim or auto_sim
    motion = cfg["motion"]

    # 只有仿真才替用户锁死网口（就那一个 lo0，多问一步没意义）。
    #
    # 真机**不传 preset**：`choose_endpoint` 只在 `preset is None and iface is
    # None` 时才列网口，传了 real 就把人钉死在 en5 —— 而真机可能接在 en5，
    # 也可能是枚举成 en0 的 USB 网卡，猜错等于连不上。让它把网口列出来自己挑。
    base: list[str] = []
    if preset == "sim":
        base += ["--preset", "sim"]
    # kp / kd 只在用户显式改过时传，默认留空让控制器用内置那一套 —— 那个值是
    # 跟模型一起标定的，不是在命令行上随手给的。
    for key in ("kp", "kd"):
        if cfg.get(key) is not None:
            base += [f"--{key}", f"{cfg[key]:g}"]

    scene = cfg.get("scene") or "flat"
    env = dict(os.environ)
    env["GO2_MOTION"] = motion
    # 场景走环境变量，和运控一样：控制器不一定关心它，但关心的那几个（VLA 要
    # 按迷宫挂提示）得读得到，而命令行不能给所有控制器都塞一个 --scene。
    env["GO2_SCENE"] = scene
    # 测距开关同理。只有感知层读它，别的控制器不认识 `--geom`。
    env["GO2_GEOM"] = "off" if cfg.get("geom") == "off" else "on"

    where = {"sim": "仿真", "real": "真机", None: "按网口选"}[preset]
    # 命令行用 `-- --client policy` 透传，不在 cfg 里。摘要要照实说运控用不用，
    # 所以把 extra 里那一段也读出来。
    eff_client = cfg.get("client")
    for i, tok in enumerate(extra):
        if tok == "--client" and i + 1 < len(extra):
            eff_client = extra[i + 1]
        elif tok.startswith("--client="):
            eff_client = tok.split("=", 1)[1]
    if eff_client in (None, "__none__"):
        # 模块自己声明了用哪个控制端就照它说。VLA 这一类走策略网络、
        # 不建 `Go2Controller`，如果按环境推成「自研」，摘要会写着「运控 mpc」
        # 而实际上根本没开 —— 这和菜单里挂着「运控开启」骗人是同一件事。
        declared = entries[0].meta.get("client")
        if declared:
            eff_client = declared
            shown_client = f"{_CLIENT_TEXT.get(declared, declared)}（模块声明）"
        else:
            # 用 `cfg["env"]` 而不是 `preset`：真机那一档 ENV_SPEC 给的是 None
            # （不传 `--preset`，让控制器把网口列出来），拿 preset 判永远不成立。
            eff_client = "official" if cfg["env"] == "real" else "policy"
            shown_client = f"默认（{_CLIENT_TEXT[eff_client]}）"
    else:
        shown_client = _CLIENT_TEXT.get(eff_client, eff_client)
        # 用户选的客户端和模块声明打架时，**不启动**。
        #
        # 原来这里只打印一行「对这条命令不生效」，然后照跑模块 —— 用户选了官方，
        # 摘要改回「推理模型」，接着策略网络接管 lowcmd。他以为自己的选择生效了，
        # 实际没有。这类「说了等于没说」的提示不如直接拒绝。
        declared = entries[0].meta.get("client")
        if declared and declared != eff_client:
            print(f"\n{entries[0].title} 只能用「{_CLIENT_TEXT.get(declared, declared)}」，"
                  f"你选的是「{_CLIENT_TEXT.get(eff_client, eff_client)}」。\n"
                  f"要么改「控制端」，要么换一个功能。\n")
            return 2
    if eff_client == "policy":
        motion_note = "不适用（推理模型直连 rt/lowcmd）"
    elif eff_client == "official":
        motion_note = "不适用（官方控制器走机器上的原生运控）"
    else:
        motion_note = motion
    extra_note = f" / 场景 {_SCENE_TEXT.get(scene, scene)}" if with_sim else ""
    # 测距单独一行说清楚。同一项在仿真落到射线、在真机落到雷达，写在一起会混。
    geom = "off" if cfg.get("geom") == "off" else "on"
    geom_note = ("关闭（只用画面）" if geom == "off"
                 else ("仿真射线（MuJoCo）" if preset == "sim"
                       else "Go2 雷达（rt/utlidar/cloud）"))
    print(f"\n环境 {where} / 控制端 {shown_client} / 运控 {motion_note}{extra_note}")
    print(f"几何测距 {geom_note}")

    # 控制端**一定**要往下传，包括「按环境推」推出来的那个。
    #
    # 不传的话控制器会自己再问一次，而它那套默认和这里的 `_eff_client` 不是一个：
    # `choose_client` 在仿真下默认 `ours`（自研），这里默认 `policy`。摘要写着
    # 「推理模型」，实际跑起自研 —— 比不问还糟。
    # `extra` 排最后，用户 `-- --client ours` 这种透传仍然能覆盖。
    cmd_extra = base + ["--client", eff_client] + extra
    os.chdir(ROOT)                       # 让 `-m` 从项目根找包
    cmds = [[sys.executable, "-m", e.module] + cmd_extra for e in entries]

    if with_sim:
        # 一个仿真器只能带一个控制器：两个都往 rt/lowcmd 发，会互相覆盖。
        if len(cmds) > 1:
            print("一起起仿真器时只能选一个控制器。")
            return 2
        return run_with_sim(cmds[0], env, scene, window=window)

    if len(cmds) == 1:
        # 单片：直接换掉自己。Ctrl+C 就是给控制器本身，信号路径最短。
        print(f"启动 {entries[0].title} …\n")
        sys.stdout.flush()
        if motion:
            os.environ["GO2_MOTION"] = motion
        os.execv(sys.executable, cmds[0])

    # 多片：各一个子进程，输出共用这个终端（控制器会自己画状态行）。
    procs = []
    for e, cmd in zip(entries, cmds):
        procs.append(subprocess.Popen(cmd, cwd=ROOT, env=env))
        print(f"启动 {e.title} …")
    print()
    code = 0
    try:
        for e, p in zip(entries, procs):
            rc = p.wait()
            print(f"{e.title} 退出，返回码 {rc}")
            code = code or rc
    except KeyboardInterrupt:
        print("\n中断，正在停所有控制器 …")
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        code = 130
    return code


# `--preset` 到配置屏那一档的映射。None 不是「仿真」，是「没指定」——
# 这两个必须分开，否则点名启动时网口菜单就被默认值吞掉了。
def _env_key(preset: str | None) -> str:
    if preset == "real":
        return "real"
    return "sim" if preset == "sim" else "ask"


def main(argv=None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if "--" in argv:
        i = argv.index("--")
        head, extra = argv[:i], argv[i + 1:]
    else:
        head, extra = argv, []

    ap = argparse.ArgumentParser(
        prog="main.py",
        description="Go2 统一入口：选环境、运控、控制器，或者单独起仿真器")
    ap.add_argument("controllers", nargs="*",
                    help="控制器文件名，如 keyboard；不给就交互选")
    ap.add_argument("--preset", choices=("sim", "real"), default=None,
                    help="环境：sim=仿真（直接 lo0），real=真机（列网口让你选）")
    ap.add_argument("--motion", choices=("vmc", "mpc"), default=None,
                    help="运控：vmc=关闭（单层反馈），mpc=开启（MPC+WBC 三层）")
    ap.add_argument("--sim", action="store_true",
                    help="只起 MuJoCo 仿真器（长驻 GUI，占住当前终端）")
    ap.add_argument("--scene", default=None,
                    choices=tuple(k for k, *_ in _SCENES),
                    help="仿真场景：flat=平地 / step=台阶 / maze=迷宫 / "
                         "terrain=起伏地形。不给就交互时问")
    ap.add_argument("--no-window", action="store_true",
                    help="仿真器不建窗口（无显示器 / 脚本里跑时用，"
                         "窗口版会卡在 launch_passive）")
    ap.add_argument("--with-sim", action="store_true",
                    help="连仿真器一起起：一个终端、一条命令，退出时一起收")
    ap.add_argument("--geom", choices=("on", "off"), default="on",
                    help="几何测距：on（默认）仿真走 MuJoCo 射线、真机走 Go2 雷达；"
                         "off 只用画面")
    ap.add_argument("--list", action="store_true", help="只列清单，不运行")
    ap.add_argument("--yes", "-y", action="store_true", help="冲突时不再问")
    args = ap.parse_args(head)

    if args.sim:
        return run_sim(args.scene)

    groups = discover_groups()
    entries = discover(groups)
    if args.list:
        render_menu(groups, entries)
        print(f"  环境：{args.preset or '交互时选'}    "
              f"运控：{args.motion or '默认 mpc'}")
        print("  起仿真器：uv run python main.py --sim\n")
        return 0

    if args.controllers:
        # 直接点名：运控用默认（MPC+WBC），环境只看 `--preset` ——
        # 没给就交给控制器列网口，不替人锁死仿真（否则真机连不上）。
        cfg = dict(env=_env_key(args.preset),
                   motion=args.motion or "mpc", client="__none__",
                   scene=args.scene or "flat", geom=args.geom)
        by_stem = {e.stem: e for e in entries}
        picked = []
        for name in args.controllers:
            e = by_stem.get(name)
            if e is None:
                print(f"没有叫 {name} 的控制器。可选：{', '.join(by_stem) or '（无）'}")
                return 2
            if not e.runnable:
                print(f"{e.title} 现在跑不了：{e.error}")
                return 2
            picked.append(e)
    else:
        # 交互：命令行没给环境/运控，就摆一屏配置让人改，再选控制器。
        if args.preset is None and args.motion is None:
            cfg = ask_settings()
        else:
            cfg = dict(env=_env_key(args.preset),
                       motion=args.motion or "mpc", client="__none__",
                       scene=args.scene or "flat", geom=args.geom)
        picked = ask(groups, entries)

    if args.with_sim:
        if cfg["env"] == "real":
            print("真机不能一起起仿真器。")
            return 2
        cfg["env"] = "sim-new"           # 明确要求一起起，就覆盖配置里的那一档

    return launch(picked, extra, cfg, args.yes, with_sim=args.with_sim,
                  window=not args.no_window)


if __name__ == "__main__":
    sys.exit(main())
