"""把 stdout 同时写进一个文件。

跑一次迷宫要一分多钟，日志几百行，而终端窗口通常比屏幕宽 —— 右边被切掉的那
半正好是「位置 / 朝向 / 转向还差多少」。2026-11-15 排查「撞墙」花了三轮截图，
每一轮都读不全，最后是靠猜哪几个数字对得上。

这里不引 logging 模块：全项目用的是 `print(..., flush=True)`，换成 logging 要
改几十处，而且日志里那些缩进和分行本来就是给人看的。只加一个 tee，不动输出格式。

    install("/repo/logs/vla-20261115-1300.log")

写文件失败（目录没权限、磁盘满）时静默退回纯终端：日志丢一行可以接受，
因为写不了日志让整条控制链路起不来不行。
"""

from __future__ import annotations

import io
import os
import sys

_installed: "_Tee | None" = None


class _Tee(io.TextIOBase):
    def __init__(self, stream, path: str):
        self._stream = stream
        self._file = open(path, "w", buffering=1, encoding="utf-8")
        self.path = path

    def write(self, text: str) -> int:
        self._stream.write(text)
        self._file.write(text)
        return len(text)

    def flush(self) -> None:
        self._stream.flush()
        self._file.flush()

    # 有些库（pygame、tqdm）会问 `isatty` / `fileno`。后者不能直接抛：仿真那边
    # 用 `subprocess` 继承 stdout，拿不到 fd 会直接崩。
    def isatty(self) -> bool:
        return False

    def fileno(self) -> int:
        return self._stream.fileno()

    def close(self) -> None:
        try:
            self._file.close()
        finally:
            super().close()


def install(path: str):
    """开始同时写文件。重复调用以第一次为准（不会嵌套两层 tee）。"""
    global _installed
    if _installed is not None:
        return _installed
    try:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tee = _Tee(sys.stdout, path)
    except Exception:                                       # noqa: BLE001
        return None
    sys.stdout = tee
    _installed = tee
    print(f"日志同时写入 {path}")
    return tee


def current() -> str:
    return _installed.path if _installed is not None else ""
