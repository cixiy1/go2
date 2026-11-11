"""Go2 项目入口：问你要跑哪个控制器，然后拉起来。

    uv run python main.py                  # 交互选，可多选
    uv run python main.py --list           # 只看清单
    uv run python main.py keyboard         # 跳过选择
    uv run python main.py keyboard -- --iface en5    # -- 之后原样传给控制器

控制器按运行环境分开放：
    quadruped/control/virtual/     对仿真器（lo0）
    quadruped/control/physical/    对真机（en5）—— 现在是空的

加控制器 = 在对应目录加一个带模块级 `META` 字典、能独立运行的文件。
"""

import os
import sys

_ROOT = os.path.dirname(os.path.abspath(__file__))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from quadruped.cli.launcher import main   # noqa: E402  要在 sys.path 之后导入

if __name__ == "__main__":
    sys.exit(main())
