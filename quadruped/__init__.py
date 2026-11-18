"""四足控制栈：从关节层到步态，仿真与真机共用同一份代码。

按职责分五个目录，依赖单向（cli → robot → control，control 不碰除 DDS 之外的 IO）：

    quadruped/
    ├── control/          控制算法
    │   ├── controller.py    Go2Controller        低层：运动学、步态、力矩，200 Hz 回路
    │   ├── high_level/      HighLevelController  高层：站、蹲、跳、按速度走
    │   └── virtual/         接哪台、用哪个控制端，产出 Backend
    ├── robot/
    │   └── body.py          Go2Robot             装配：选环境、造控制器、转发读数
    ├── cli/
    │   └── main.py          命令行入口，一个子命令一个动作
    ├── comm/                通讯层：套接字协议，只给仓库外的项目用
    └── sim/                 仿真资产（不是 Python 包）
        ├── run_sim.sh
        └── unitree_mujoco/  宇树官方仿真器（第三方，勿改）

`comm/` 是包边界，不是内部组件：仓库内和 Yuki 工具包直接 import
`control.virtual.robot`，外部项目只能走 `comm/`。`import quadruped.comm.client`
不碰 DDS，这条由 `test/arch_check.py` 守着。

三层的包含关系是字面意义上的：

    Go2Robot.controller               →  Go2Controller
    Go2Robot.controller.high_level    →  HighLevelController
    HighLevelController.controller    →  指回 Go2Controller

入口两种跑法都行，脚本模式会自己把项目根插进 sys.path：

    uv run python -m quadruped.cli.main stand
    uv run python quadruped/cli/main.py stand
"""

from typing import TYPE_CHECKING, Any

# 这里是**惰性**导出，不是 `from ... import ...`。
#
# 包 `__init__` 一旦 import 控制器，`import quadruped.comm.client` 也会把
# cyclonedds、unitree_sdk2py、整个 control 层一起拖进来（实测 147 个模块、0.11 s）。
# 后果有两个：`quadruped.comm.client` 声称「只要一个套接字」，却在没有装 SDK 的
# 机器上 import 失败；`main.py --list` 这种只想读菜单的路径也要等 DDS 加载。
#
# 改成 PEP 562 的模块级 `__getattr__` 之后，`from quadruped import Go2Robot` 照旧
# 能用（第一次访问时才真正 import），`import quadruped.comm.client` 则是干净的。
_LAZY = {
    "Go2Controller": "quadruped.control.controller",
    "GaitParams": "quadruped.control.controller",
    "NOMINAL_HEIGHT": "quadruped.control.controller",
    "make_gait": "quadruped.control.controller",
    "Go2Robot": "quadruped.robot.body",
}

__all__ = ["Go2Controller", "GaitParams", "NOMINAL_HEIGHT", "make_gait",
           "Go2Robot"]

if TYPE_CHECKING:                       # 只给类型检查看，运行时不执行
    from quadruped.control.controller import (GaitParams, Go2Controller,
                                              NOMINAL_HEIGHT, make_gait)
    from quadruped.robot.body import Go2Robot


def __getattr__(name: str) -> Any:
    module = _LAZY.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module
    return getattr(import_module(module), name)


def __dir__() -> list[str]:
    return sorted(__all__)
