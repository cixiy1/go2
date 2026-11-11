"""四足控制栈：从关节层到步态，仿真与真机共用同一份代码。

按职责分四个目录，依赖单向（cli → robot → control，control 不碰除 DDS 之外的 IO）：

    quadruped/
    ├── control/          控制算法
    │   ├── controller.py    Go2Controller        低层：运动学、步态、力矩，200 Hz 回路
    │   └── high_level.py    HighLevelController  高层：站、蹲、跳、按速度走
    ├── robot/
    │   └── body.py          Go2Robot             装配：选环境、造控制器、转发读数
    ├── cli/
    │   └── main.py          命令行入口，一个子命令一个动作
    └── sim/                 仿真资产（不是 Python 包）
        ├── run_sim.sh
        └── unitree_mujoco/  宇树官方仿真器（第三方，勿改）

三层的包含关系是字面意义上的：

    Go2Robot.controller               →  Go2Controller
    Go2Robot.controller.high_level    →  HighLevelController
    HighLevelController.controller    →  指回 Go2Controller

入口两种跑法都行，脚本模式会自己把项目根插进 sys.path：

    uv run python -m quadruped.cli.main stand
    uv run python quadruped/cli/main.py stand
"""

from quadruped.control.controller import (Go2Controller, GaitParams,
                                          NOMINAL_HEIGHT, make_gait)
from quadruped.robot.body import Go2Robot

__all__ = ["Go2Controller", "GaitParams", "NOMINAL_HEIGHT", "make_gait",
           "Go2Robot"]
