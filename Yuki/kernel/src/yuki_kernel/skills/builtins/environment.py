"""内置工具：环境信息。"""

import os
import platform
import sys
from pathlib import Path


def get_environment_info() -> str:
    """返回当前运行环境信息，帮助模型生成正确的命令。"""
    return "\n".join(
        [
            f"操作系统: {platform.platform()}",
            f"Python: {sys.version.split()[0]}",
            f"工作目录: {Path.cwd()}",
            f"Shell: {os.environ.get('SHELL', '未知')}",
            f"PATH: {os.environ.get('PATH', '')}",
        ]
    )
