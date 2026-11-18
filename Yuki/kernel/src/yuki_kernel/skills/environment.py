"""执行环境：内核注册时由宿主注入，所有工具共用，不归某个工具私有。

一个 Environment 同时承载两面：

- 隔离面（run）：命令/工具在何种 OS 隔离下执行（降权、资源阀、网络/文件范围），
  内部委托 ``Sandbox`` 实现。
- 运行面（runtime）：用哪个 python 解释器、基础环境变量（venv 的 PATH/PYTHONPATH 等），
  供 python 工具与命令解析使用。

宿主可注入由外部软件创建的虚拟环境，也可注入外部推入的系统环境；内核只照执行，
不自行创建环境、不判断权限。终端工具用的就是这个环境，且它并非终端专属——
python 工具与 command 工具同样经此环境执行。
"""

from __future__ import annotations

import sys
from pathlib import Path

from .sandbox import BasicSandbox, RunResult, Sandbox


class Environment:
    """执行环境抽象：工具统一经它执行命令。"""

    python_path: str = sys.executable

    def __init__(self) -> None:
        self.base_env: dict[str, str] = {}

    def run(
        self,
        command: list[str],
        cwd: Path,
        input_text: str,
        timeout: float = 30,
        env: dict[str, str] | None = None,
    ) -> RunResult:
        raise NotImplementedError


class BasicEnvironment(Environment):
    """默认环境：当前用户、宿主完整环境、标准库 BasicSandbox 隔离阀门。

    仅作开发兜底；生产应由宿主注入更强的 Environment（venv / 容器 / Seatbelt）。
    """

    def __init__(
        self,
        sandbox: Sandbox | None = None,
        python_path: str | None = None,
        base_env: dict[str, str] | None = None,
    ):
        super().__init__()
        self._sandbox = sandbox or BasicSandbox()
        self.python_path = python_path or sys.executable
        self.base_env = base_env or {}

    def run(
        self,
        command: list[str],
        cwd: Path,
        input_text: str,
        timeout: float = 30,
        env: dict[str, str] | None = None,
    ) -> RunResult:
        return self._sandbox.run(command, cwd, input_text, timeout, env)
