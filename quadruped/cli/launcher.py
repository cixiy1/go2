"""项目入口的启动器：先问你要跑哪个控制器，再把它拉起来。

控制器按分组放在 `quadruped/control/` 下的三个目录里：

    virtual/     虚拟 · 对仿真器（lo0）
    physical/    物理 · 对真机（en5）
    embodied/    具身智能 · 决策层，输出动作而不是关节指令

前两组按**运行环境**分，第三组按**分层**分：具身智能控制器坐在高层之上，
自己不碰 DDS 也不碰步态，仿真和真机用的是同一份。

每个模块在模块级暴露一个 `META` 字典，本文件扫描目录、导入读 `META`、列成
菜单。所以**加一个控制器就是加一个文件**，不用回来改这里。

为什么要开子进程而不是 import 进来直接跑：`ChannelFactoryInitialize` 一个
进程只能调一次，第二个控制器在同一进程里初始化 DDS 会静默拿不到数据。多选
时每个控制器一个子进程，各跑各的。

用法：
    uv run python main.py                # 交互选，可多选
    uv run python main.py keyboard       # 跳过选择，直接跑键盘遥控
    uv run python main.py --list         # 只列清单
    uv run python main.py keyboard -- --iface en5     # -- 之后原样传给控制器
"""

import argparse
import importlib
import os
import subprocess
import sys
import unicodedata

_HERE = os.path.dirname(os.path.abspath(__file__))          # quadruped/cli
ROOT = os.path.dirname(os.path.dirname(_HERE))              # 项目根
CONTROL_DIR = os.path.join(os.path.dirname(_HERE), "control")

# 目录名 → 菜单标题。顺序就是菜单顺序。
GROUPS = (
    ("virtual", "虚拟 · 仿真器 lo0"),
    ("physical", "物理 · 真机 en5"),
    ("embodied", "具身智能 · 决策层（仿真/真机通用）"),
)


def _width(s: str) -> int:
    """终端显示宽度：中日韩字符占两格。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def _pad(s: str, n: int) -> str:
    return s + " " * max(0, n - _width(s))


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


def discover() -> list[Entry]:
    """扫三个目录，导入每个模块读 META。"""
    found: list[Entry] = []
    for group, _ in GROUPS:
        d = os.path.join(CONTROL_DIR, group)
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if not fn.endswith(".py") or fn.startswith("_"):
                continue
            stem = fn[:-3]
            module = f"quadruped.control.{group}.{stem}"
            try:
                mod = importlib.import_module(module)
            except Exception as e:                    # noqa: BLE001 任何导入错误都不该挡住菜单
                found.append(Entry(group, stem, module,
                                   error=f"{type(e).__name__}: {e}"))
                continue
            meta = getattr(mod, "META", None)
            if meta is None:
                # 没有 META 的模块不当控制器，静默跳过（比如辅助模块）。
                continue
            found.append(Entry(group, stem, module, meta=dict(meta)))
    return found


def render_menu(entries: list[Entry]) -> dict[int, Entry]:
    """打印菜单，返回 编号 → 控制器。"""
    print("\nGo2 控制器\n")
    n = 0
    index: dict[int, Entry] = {}
    for key, heading in GROUPS:
        print(f"  {heading}")
        members = [e for e in entries if e.group == key]
        if not members:
            print("    （还没有，留空占位）\n")
            continue
        for e in members:
            n += 1
            index[n] = e
            if e.runnable:
                print(f"    {n}  {_pad(e.title, 12)}{e.module}")
                if e.desc:
                    print(f"       {_pad('', 12)}{e.desc}")
            else:
                print(f"    {n}  {_pad(e.title, 12)}不可用 —— {e.error}")
        print()
    return index


def ask(entries: list[Entry]) -> list[Entry]:
    """问一句选哪个。回车 = 第一个能跑的。"""
    index = render_menu(entries)
    if not index:
        print("两个目录下都没有控制器。")
        return []
    default = next((k for k, e in index.items() if e.runnable), None)
    hint = f"回车 = {default}" if default else "没有可用的"
    while True:
        try:
            raw = input(f"选哪个（多选用 1,2 或 1 2；q 退出；{hint}）：").strip()
        except EOFError:
            print()
            return []
        if raw in ("q", "Q", "quit", "exit"):
            return []
        if not raw:
            if default is None:
                print("上面没有能跑的控制器。")
                return []
            return [index[default]]
        picked: list[Entry] = []
        bad = False
        for tok in raw.replace(",", " ").split():
            if not tok.isdigit() or int(tok) not in index:
                print(f"  「{tok}」不是菜单里的编号")
                bad = True
                break
            e = index[int(tok)]
            if not e.runnable:
                print(f"  {e.title} 现在跑不了：{e.error}")
                bad = True
                break
            if e not in picked:
                picked.append(e)
        if bad or not picked:
            continue
        return picked


def check_conflict(entries: list[Entry], assume_yes: bool) -> bool:
    """两个控制器同时占 lowcmd / 键盘，DDS 上会互相覆盖，先拦一下。"""
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


def launch(entries: list[Entry], extra: list[str], assume_yes: bool = False) -> int:
    if not entries:
        return 1
    if extra and len(entries) > 1:
        print("多个控制器时不能带额外参数（会被塞给每一个）。先只选一个，或去掉 --。")
        return 2
    if not check_conflict(entries, assume_yes):
        print("已取消。")
        return 2

    os.chdir(ROOT)                       # 让 `-m` 从项目根找包
    cmds = [[sys.executable, "-m", e.module] + extra for e in entries]

    if len(cmds) == 1:
        # 单片：直接换掉自己。Ctrl+C 就是给控制器本身，信号路径最短。
        print(f"\n启动 {entries[0].title} …\n")
        sys.stdout.flush()
        os.execv(sys.executable, cmds[0])

    # 多片：各一个子进程，输出共用这个终端（控制器会自己画状态行）。
    procs = []
    for e, cmd in zip(entries, cmds):
        procs.append(subprocess.Popen(cmd, cwd=ROOT))
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
        prog="main.py", description="Go2 控制器入口：选一个（或几个）跑起来")
    ap.add_argument("controllers", nargs="*",
                    help="控制器文件名，如 keyboard；不给就交互选")
    ap.add_argument("--list", action="store_true", help="只列清单，不运行")
    ap.add_argument("--yes", "-y", action="store_true", help="冲突时不再问")
    args = ap.parse_args(head)

    entries = discover()
    if args.list:
        render_menu(entries)
        return 0

    if args.controllers:
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
        picked = ask(entries)

    return launch(picked, extra, args.yes)


if __name__ == "__main__":
    sys.exit(main())
