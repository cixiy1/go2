"""内置工具：在终端/shell 里执行命令。

本工具不自己构造沙箱：命令经子进程执行，统一由宿主注入内核的
`ToolExecutor` 沙箱托管（降权、资源上限、环境清空、命令白名单都归那一层）。
工具只负责拼命令、跑、把结果格式化为文本。具体运行环境（是否降权、以哪个
用户、什么环境）由宿主推入的 `Sandbox` 设定决定，内核只照执行、不自行判断。
"""

import subprocess
from pathlib import Path


def run_terminal(cmd: str) -> str:
    """在 shell 中执行命令并返回结果文本。

    cmd: 要执行的命令字符串（经 /bin/sh -c 运行，适配各 POSIX 终端）。
    沙箱与权限由宿主注入的 executor 沙箱决定，不在本工具内配置。
    """
    if not cmd or not cmd.strip():
        return "错误：命令为空"
    try:
        proc = subprocess.run(
            ["/bin/sh", "-c", cmd],
            check=False,
            cwd=Path.cwd(),
            input="",
            text=True,
            capture_output=True,
            timeout=30,
        )
    except subprocess.TimeoutExpired:
        return "错误：命令执行超时"
    except Exception as err:  # noqa: BLE001 - 命令失败需回喂模型而非崩溃
        return f"错误：命令执行失败：{err}"
    parts = [f"退出码: {proc.returncode}"]
    if proc.stdout.strip():
        parts.append(f"标准输出:\n{proc.stdout.strip()}")
    if proc.stderr.strip():
        parts.append(f"标准错误:\n{proc.stderr.strip()}")
    if proc.returncode != 0 and not proc.stdout.strip() and not proc.stderr.strip():
        parts.append("(无输出)")
    return "\n".join(parts)
