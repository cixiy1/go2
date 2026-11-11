"""Go2 项目入口：问你要跑哪个控制器，然后拉起来。

    uv run python main.py                  # 交互选，可多选
    uv run python main.py --list           # 只看清单
    uv run python main.py keyboard         # 跳过选择
    uv run python main.py keyboard -- --iface en5    # -- 之后原样传给控制器

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
