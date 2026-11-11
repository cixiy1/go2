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
}

# 仿真器起来到能收 lowstate 要多久。mjpython 解包 + 加载 scene.xml + 720 个
# 三角形的地形，给 4 s 是留足余量；短了控制器会在 connect 的 3 s 超时里空等。
SIM_BOOT = 4.0

MOTIONS = [
    ("mpc", "开启（MPC + WBC）", "默认。三层；单帧 1.0 ms，回路 p95 8~9 ms"),
    ("vmc", "关闭（VMC）", "单层反馈；单帧 0.5 ms，回路 p95 5.8 ms"),
]

# 控制端。仿真桥只有 rt/lowcmd 和 rt/sportmodestate，没有运动服务，官方那条
# 在仿真连不上 —— 选错由控制端自己 `choose_client` 报错，这里不拦。
CLIENTS = [
    ("__none__", "默认", "由控制端自己挑"),
    ("ours", "自研", "本仓库的 HighLevelController"),
    ("official", "官方", "真机上的原生运动服务，仿真不可用"),
]

_ENV_TEXT = {k: f"{label}  {note}" for k, label, note in ENVS}
_MOTION_TEXT = {k: label for k, label, _ in MOTIONS}
_CLIENT_TEXT = {k: label for k, label, _ in CLIENTS}

# 放进表里是为了「临时试一下」不用记 `--` 透传，但必须标清楚它不是普通选项。
_NUM_NOTE = ("注：kp / kd 与模型一起标定（仿真与真机同一套），一般不要动。\n"
             "    改了下表只对当前环境成立，症状是「某个速度段开始打滑」，\n"
             "    不容易联想到刚度。回车 = 用默认。")


def _show(key: str):
    """字段的显示函数。数值项要把「已偏离默认」标出来。"""
    if key == "env":
        return lambda v: _ENV_TEXT[v]
    if key == "motion":
        return lambda v: _MOTION_TEXT[v]
    if key == "client":
        return lambda v: _CLIENT_TEXT[v]
    return lambda v: "默认" if v is None else f"{v:g}   ← 已偏离默认"


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


# 一屏配置。
def ask_settings() -> dict:
    def edit_env(cur):
        return pick("环境", ENVS, [k for k, *_ in ENVS].index(cur))

    def edit_motion(cur):
        return pick("运控", MOTIONS, [k for k, *_ in MOTIONS].index(cur))

    def edit_client(cur):
        return pick("控制端", CLIENTS, [k for k, *_ in CLIENTS].index(cur))

    return Form(
        [
            ("环境", "env", _show("env"), edit_env),
            ("运控", "motion", _show("motion"), edit_motion),
            ("刚度 kp", "kp", _show("kp"), lambda cur: _ask_number("kp", cur)),
            ("阻尼 kd", "kd", _show("kd"), lambda cur: _ask_number("kd", cur)),
            ("控制端", "client", _show("client"), edit_client),
        ],
        dict(env="sim-new", motion="mpc", kp=None, kd=None, client="__none__"),
        note=_NUM_NOTE,
    ).run()


# 起 MuJoCo 仿真器。路径与 mjpython 的判断都在脚本里，这里只管把终端换过去。
# 仿真器是长驻 GUI，会占住这个终端，所以和控制器必须分两个终端跑。
def run_sim() -> int:
    script = os.path.join(ROOT, "quadruped", "sim", "run_sim.sh")
    if not os.access(script, os.X_OK):
        print(f"找不到可执行的 {script}")
        return 2
    print("启动 MuJoCo 仿真器 …")
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


def run_with_sim(cmd: list[str], env: dict) -> int:
    if not os.access(SIM_SCRIPT, os.X_OK):
        print(f"找不到可执行的 {SIM_SCRIPT}")
        return 2
    print("起 MuJoCo 仿真器 …")
    sys.stdout.flush()
    sim = subprocess.Popen([SIM_SCRIPT], cwd=ROOT)
    code = 1
    try:
        time.sleep(SIM_BOOT)
        if sim.poll() is not None:
            print(f"仿真器自己退了（返回码 {sim.returncode}），看上面的报错。")
            return sim.returncode or 1
        print(f"仿真器就绪（等了 {SIM_BOOT:.0f} s），接控制器\n")
        sys.stdout.flush()
        ctrl = subprocess.Popen(cmd, cwd=ROOT, env=env)
        code = ctrl.wait()
    except KeyboardInterrupt:
        print("\n中断 …")
        code = 130
    finally:
        if sim.poll() is None:
            print("收仿真器 …")
            sim.terminate()
            try:
                sim.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sim.kill()
    return code


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
           assume_yes: bool = False, with_sim: bool = False) -> int:
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
    if cfg.get("client") and cfg["client"] != "__none__":
        base += ["--client", cfg["client"]]
    cmd_extra = base + extra          # 用户在 `--` 后给的排最后，才能覆盖

    env = dict(os.environ)
    env["GO2_MOTION"] = motion

    print(f"\n环境 {preset} / 运控 {motion}")
    os.chdir(ROOT)                       # 让 `-m` 从项目根找包
    cmds = [[sys.executable, "-m", e.module] + cmd_extra for e in entries]

    if with_sim:
        # 一个仿真器只能带一个控制器：两个都往 rt/lowcmd 发，会互相覆盖。
        if len(cmds) > 1:
            print("一起起仿真器时只能选一个控制器。")
            return 2
        return run_with_sim(cmds[0], env)

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
    ap.add_argument("--with-sim", action="store_true",
                    help="连仿真器一起起：一个终端、一条命令，退出时一起收")
    ap.add_argument("--list", action="store_true", help="只列清单，不运行")
    ap.add_argument("--yes", "-y", action="store_true", help="冲突时不再问")
    args = ap.parse_args(head)

    if args.sim:
        return run_sim()

    groups = discover_groups()
    entries = discover(groups)
    if args.list:
        render_menu(groups, entries)
        print(f"  环境：{args.preset or '交互时选'}    "
              f"运控：{args.motion or '默认 mpc'}")
        print("  起仿真器：uv run python main.py --sim\n")
        return 0

    if args.controllers:
        # 直接点名：不再问，缺的用默认（仿真 / MPC+WBC）。
        cfg = dict(env="real" if args.preset == "real" else "sim",
                   motion=args.motion or "mpc", client="__none__")
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
            cfg = dict(env="real" if args.preset == "real" else "sim",
                       motion=args.motion or "mpc", client="__none__")
        picked = ask(groups, entries)

    if args.with_sim:
        if cfg["env"] == "real":
            print("真机不能一起起仿真器。")
            return 2
        cfg["env"] = "sim-new"           # 明确要求一起起，就覆盖配置里的那一档

    return launch(picked, extra, cfg, args.yes, with_sim=args.with_sim)


if __name__ == "__main__":
    sys.exit(main())
