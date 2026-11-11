"""Go2 统一入口：选环境、运控、控制器，或者单独起仿真器。

    uv run python main.py                              # 交互：环境 → 运控 → 控制器
    uv run python main.py --sim                        # 只起 MuJoCo 仿真器
    uv run python main.py --preset sim --motion mpc keyboard
    uv run python main.py --list                       # 只看清单
    uv run python main.py keyboard -- --iface en5      # -- 之后原样传给控制器

`--preset` 选仿真/真机，`--motion` 选是否开启 MPC+WBC 运控（**默认开**）。
两项都可以不给，交互时会问；不想要运控时给 `--motion vmc`，或者直接
`GO2_MOTION=vmc`。

控制器放在 quadruped/control/ 下的分组目录里。每个分组在自己的 `__init__.py`
声明 `GROUP`（标题 + 顺序）就上菜单，没声明的（`virtual/` 抽象层）是内部实现：

    physical/    直接驱动机器人（键盘、手柄、固定动作脚本）
    embodied/    具身智能，决策层

加控制器 = 在分组目录里加一个带 `META` + `main(argv)` 的文件；
加分组 = 加一个目录并在 `__init__.py` 写 `GROUP`。两者都不用改启动器。
"""

import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quadruped.cli.launcher import main   # noqa: E402  要在 sys.path 之后导入

if __name__ == "__main__":
    sys.exit(main())
