"""架构边界检查：内部代码不依赖通讯层，通讯层不在模块级拖入重依赖。

    .venv/bin/python test/arch_check.py

两条规则，都是「一旦破了，坏处不会立刻报错」的那种：

1. `quadruped/` 里除 `quadruped/comm/` 之外的代码，不许 import `quadruped.comm`。
   内部（控制栈、Yuki 工具包）直接对接抽象层 `virtual.robot.connect()`；通讯层是
   给仓库外项目的边界。内部一旦绕过它去 import client，就会出现「同一个进程里
   既开 DDS 又连套接字」，路径多一条，排查时多一层。

2. `quadruped/comm/*.py` 的**模块级** import 只允许标准库和 `quadruped.comm.*`，
   加一个例外 `quadruped.limits`（纯常量表，import 不拉起任何东西）。
   `unitree_sdk2py`、`quadruped.robot`、`quadruped.control.*` 只能写在函数里。
   破了这条，`import quadruped.comm.client` 就会把 DDS 和整个 control 层拖进来
   （实测 147 个模块、0.115 s），在没有 SDK 的机器上连客户端都 import 不了。

3. `quadruped/limits.py` 自己也得是轻的：模块级只 import 标准库。它是规则 2 的
   例外，例外一旦开始 import numpy 或 controller，整条边界就漏了。

4. `quadruped/motion/`（推理控制器）不许 import `quadruped.control.*`、
   `quadruped.robot.*`、`quadruped.comm.*`。它只认 `LowLink` 那九个成员，依赖方向
   只能是 control → motion。破了这条不会报错，只会在某天有人顺手调 `move()` 之后
   才表现出来。

5. `ChannelFactoryInitialize` 只允许出现在「一个进程只跑一件事」的入口文件里。
   它是进程级单例：第一次的 domain 和网卡会被后面所有调用沿用，而且不报错。
   通讯层里调它等于网关自己握住了 DDS —— 那正是本层存在的理由（不碰 DDS）。

6. `subprocess.Popen` 只允许出现在三个「进程所有者」文件里：`comm/child.py`
   （网关的子进程）、`cli/launcher.py`（仿真 + 控制器）、`web/app.py`（网页托管的
   VLA）。多一个发起点，就多一处「谁杀了谁」的答案。

7. `quadruped/percept/` 的**模块级** import 不许碰 `control` / `robot` / `comm`。
   感知层要能在没有控制栈的机器上 import；`feed.py` 解析端点那一步写在 `main()`
   里，属于函数内依赖，不算破坏。

8. 只有 `comm/gateway.py` 能 import 三个 host（robot_link / vla_host / cam_host）。
   host 之间互不 import，「谁先谁后」只写在网关的三个 `_switch_*` 函数里。

`test/` 自己不受规则 1 约束：那些脚本扮演的是外部调用方。
`if TYPE_CHECKING:` 块不算模块级 —— 它运行时不执行，不产生导入开销。

任一条不过就返回非零。
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# 通讯层包名。只有它自己内部可以互相 import。
COMM = "quadruped.comm"

# 内部代码所在位置。规则 1 扫这些。
INTERNAL = ("quadruped", "main.py")

# 规则 1 的豁免：通讯层自身。
INTERNAL_EXEMPT = Path("quadruped/comm")

# 规则 2 的例外：纯常量模块。它被允许出现在通讯层的模块级 import 里，
# 条件是规则 3 对它成立。
LIGHT = ("quadruped.limits",)


# 收集一个模块里的 import，分成「模块级」和「函数/类体内」两拨。
#
# `if TYPE_CHECKING:` 整块跳过：只有类型检查器看得见，运行时不执行。
def imports_of(tree: ast.AST) -> tuple[list[tuple[str, int]], list[tuple[str, int]]]:
    top: list[tuple[str, int]] = []
    nested: list[tuple[str, int]] = []

    def is_type_checking(node: ast.If) -> bool:
        return "TYPE_CHECKING" in ast.dump(node.test)

    def walk(node: ast.AST, inside: bool) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.If) and is_type_checking(child):
                continue
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef,
                                  ast.ClassDef)):
                walk(child, True)
                continue
            if isinstance(child, ast.Import):
                bucket = nested if inside else top
                bucket.extend((a.name, a.lineno) for a in child.names)
            elif isinstance(child, ast.ImportFrom):
                # level > 0 是相对导入，模块名可能为 None；对本包内部无所谓。
                if child.level == 0 and child.module:
                    bucket = nested if inside else top
                    bucket.append((child.module, child.lineno))
            walk(child, inside)

    walk(tree, False)
    return top, nested


# 模块名是否落在 `pkg` 这棵子树里（含 pkg 自身）。
def under(module: str, pkg: str) -> bool:
    return module == pkg or module.startswith(pkg + ".")


def check_internal(root: Path) -> list[str]:
    bad: list[str] = []
    files: list[Path] = []
    for entry in INTERNAL:
        target = root / entry
        if target.is_file():
            files.append(target)
        else:
            files.extend(sorted(target.rglob("*.py")))

    for path in files:
        rel = path.relative_to(root)
        if rel.is_relative_to(INTERNAL_EXEMPT):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as err:
            bad.append(f"{rel}:{err.lineno} 语法错误，无法检查：{err.msg}")
            continue
        top, nested = imports_of(tree)
        for module, line in top + nested:
            if under(module, COMM):
                bad.append(f"{rel}:{line} 内部代码 import 了通讯层 {module}"
                           f"（内部走 virtual.robot.connect()）")
    return bad


def check_comm_light(root: Path) -> list[str]:
    bad: list[str] = []
    for path in sorted((root / "quadruped/comm").glob("*.py")):
        rel = path.relative_to(root)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as err:
            bad.append(f"{rel}:{err.lineno} 语法错误，无法检查：{err.msg}")
            continue
        top, _ = imports_of(tree)
        for module, line in top:
            if module.split(".")[0] in sys.stdlib_module_names:
                continue
            if under(module, COMM):
                continue
            if any(under(module, pkg) for pkg in LIGHT):
                continue
            bad.append(f"{rel}:{line} 模块级 import 了 {module}"
                       f"（重依赖放函数里，否则 import 客户端要等 SDK）")
    return bad


def check_limits_light(root: Path) -> list[str]:
    bad: list[str] = []
    for pkg in LIGHT:
        path = root / (pkg.replace(".", "/") + ".py")
        if not path.is_file():
            bad.append(f"{pkg} 不存在，但通讯层在 import 它")
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for module, line in imports_of(tree)[0]:
            if module.split(".")[0] in sys.stdlib_module_names:
                continue
            bad.append(f"{path.relative_to(root)}:{line} 模块级 import 了 {module}"
                       f"（{pkg} 必须是纯常量，不能拉起别的模块）")
    return bad


def check_motion_isolated(root: Path) -> list[str]:
    """规则 4：推理控制器不许认识控制栈。

    `quadruped/motion/` 是策略这一侧的全部。它一旦 import `quadruped.control.*`
    或 `quadruped.robot.*`，「独立于高层控制器」这句话就作废了 —— 而且不会报错，
    只会在某天有人顺手调了个 `move()` 之后表现出来。依赖方向只能是 control → motion。
    """
    forbidden = ("quadruped.control", "quadruped.robot", "quadruped.comm")
    bad: list[str] = []
    for path in sorted((root / "quadruped/motion").glob("*.py")):
        rel = path.relative_to(root)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as err:
            bad.append(f"{rel}:{err.lineno} 语法错误，无法检查：{err.msg}")
            continue
        top, nested = imports_of(tree)
        for module, line in top + nested:
            for pkg in forbidden:
                if under(module, pkg):
                    bad.append(f"{rel}:{line} import 了 {module}"
                               f"（推理层只能依赖 LowLink）")
    return bad


# 规则 5 的白名单：一个进程只跑一件事的入口。
DDS_ENTRY = {
    "quadruped/control/controller.py",        # 自研控制器（后台 200 Hz 回路）
    "quadruped/control/high_level/official.py",  # 官方 SportClient
    "quadruped/motion/direct.py",             # 推理控制器的 low 层直连
    "quadruped/percept/feed.py",              # 相机馈送（只读，不写 lowcmd）
    "quadruped/percept/radar_feed.py",        # 雷达馈送（独立进程，只订阅点云）
}

# 规则 6 的白名单：进程所有者。
PROCESS_ENTRY = {
    "quadruped/comm/child.py",     # 网关的三个子进程
    "quadruped/cli/launcher.py",   # 仿真器 + 控制器
    "quadruped/web/app.py",        # 网页托管的 VLA
}

# 规则 8：这三个模块只能由 gateway.py 拉起来。
HOST_MODULES = {
    "quadruped.comm.robot_link",
    "quadruped.comm.vla_host",
    "quadruped.comm.cam_host",
}


# 收集所有「函数调用名 → 行号」。只看名字，不看对象是谁 —— 这三条规则要拦的
# 就是「换个写法绕过去」，所以 `subprocess.Popen(...)` 和 `from subprocess import
# Popen; Popen(...)` 都得算。
def calls_of(tree: ast.AST) -> list[tuple[str, int]]:
    out: list[tuple[str, int]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if isinstance(fn, ast.Name):
            out.append((fn.id, node.lineno))
        elif isinstance(fn, ast.Attribute):
            out.append((fn.attr, node.lineno))
    return out


def _python_files(base: Path) -> list[Path]:
    return sorted(base.rglob("*.py")) if base.is_dir() else []


def check_dds_singleton(root: Path) -> list[str]:
    """规则 5：DDS 初始化只在进程入口。"""
    bad: list[str] = []
    for path in _python_files(root / "quadruped"):
        rel = str(path.relative_to(root))
        # 仿真器与它的示例是第三方代码（`quadruped/sim/unitree_mujoco/`），
        # 单独进程、单独配置，不参与本仓库的进程纪律。
        if rel.startswith("quadruped/sim/"):
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as err:
            bad.append(f"{rel}:{err.lineno} 语法错误，无法检查：{err.msg}")
            continue
        if rel in DDS_ENTRY:
            continue
        for name, line in calls_of(tree):
            if name == "ChannelFactoryInitialize":
                bad.append(f"{rel}:{line} 调了 ChannelFactoryInitialize"
                           f"（进程单例，只许出现在 {sorted(DDS_ENTRY)} 里）")
        # 别名绕不过去：`from ... import ChannelFactoryInitialize as C` 然后 `C(1, "lo")`
        # 的调用名是 C，只看调用名就漏了。导入本身就是意图，这里一起拦。
        for node in ast.walk(tree):
            if (isinstance(node, ast.ImportFrom)
                    and node.module == "unitree_sdk2py.core.channel"
                    and any(a.name == "ChannelFactoryInitialize" for a in node.names)):
                bad.append(f"{rel}:{node.lineno} import 了 ChannelFactoryInitialize"
                           f"（进程单例，只许出现在 {sorted(DDS_ENTRY)} 里）")
    return bad


def check_process_owners(root: Path) -> list[str]:
    """规则 6：子进程只在三个所有者里起。"""
    bad: list[str] = []
    for path in _python_files(root / "quadruped"):
        rel = str(path.relative_to(root))
        if rel in PROCESS_ENTRY:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError:
            continue
        for name, line in calls_of(tree):
            if name == "Popen":
                bad.append(f"{rel}:{line} 起了子进程"
                           f"（只有 {sorted(PROCESS_ENTRY)} 能起；网关走 comm/child.py）")
    return bad


def check_percept_light(root: Path) -> list[str]:
    """规则 7：感知层模块级不碰控制栈。"""
    forbidden = ("quadruped.control", "quadruped.robot", "quadruped.comm")
    bad: list[str] = []
    for path in sorted((root / "quadruped/percept").glob("*.py")):
        rel = path.relative_to(root)
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as err:
            bad.append(f"{rel}:{err.lineno} 语法错误，无法检查：{err.msg}")
            continue
        top, _ = imports_of(tree)
        for module, line in top:
            for pkg in forbidden:
                if under(module, pkg):
                    bad.append(f"{rel}:{line} 模块级 import 了 {module}"
                               f"（要用的放函数里，感知层得能在没有控制栈的机器上 import）")
    return bad


def check_single_orchestrator(root: Path) -> list[str]:
    """规则 8：编排只有网关一处。"""
    bad: list[str] = []
    for path in sorted((root / "quadruped/comm").glob("*.py")):
        rel = path.relative_to(root)
        if rel.name == "gateway.py":
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError:
            continue
        top, nested = imports_of(tree)
        for module, line in top + nested:
            if module in HOST_MODULES:
                bad.append(f"{rel}:{line} import 了 {module}"
                           f"（只有 gateway.py 能拉起 host，host 之间互不 import）")
            if under(module, "quadruped.comm.gateway"):
                bad.append(f"{rel}:{line} 反向依赖网关"
                           f"（依赖方向只能是 gateway → host）")
    return bad


def main() -> int:
    rules = (
        ("规则 1  内部不依赖通讯层", check_internal),
        ("规则 2  通讯层模块级只 import 标准库", check_comm_light),
        ("规则 3  常量模块保持轻", check_limits_light),
        ("规则 4  推理层不认识控制栈", check_motion_isolated),
        ("规则 5  DDS 初始化只在进程入口", check_dds_singleton),
        ("规则 6  子进程只在三个所有者里起", check_process_owners),
        ("规则 7  感知层模块级不碰控制栈", check_percept_light),
        ("规则 8  编排只有网关一处", check_single_orchestrator),
    )
    failed = 0
    for title, fn in rules:
        bad = fn(ROOT)
        if bad:
            failed += 1
            print(f"✗ {title}")
            for line in bad:
                print(f"    {line}")
        else:
            print(f"✓ {title}")
    print("\n边界成立" if not failed else f"\n{failed} 条规则被破坏")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
