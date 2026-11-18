"""Yuki 示例聊天外壳：只保留主循环。"""

import asyncio
import signal
import threading

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

from .commands import handle_command
from .rendering import render_turn

EXIT_COMMANDS = {"exit", "quit", "q", "退出"}


def _read_stdin(lines: "asyncio.Queue[str | None]", loop: asyncio.AbstractEventLoop) -> None:
    while True:
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            loop.call_soon_threadsafe(lines.put_nowait, None)
            return
        loop.call_soon_threadsafe(lines.put_nowait, line)


async def run(app: App) -> None:
    loop = asyncio.get_running_loop()
    lines: asyncio.Queue = asyncio.Queue()
    threading.Thread(target=_read_stdin, args=(lines, loop), daemon=True).start()

    main_task = asyncio.current_task()

    def _on_sigint() -> None:
        if main_task is not None:
            main_task.cancel()

    handler_installed = False
    try:
        loop.add_signal_handler(signal.SIGINT, _on_sigint)
        handler_installed = True
    except (NotImplementedError, RuntimeError):
        pass

    try:
        while True:
            print("user：", end="", flush=True)
            try:
                raw = await lines.get()
            except asyncio.CancelledError:
                break
            if raw is None:
                break
            line = raw.strip()
            if not line:
                continue
            if line.lower() in EXIT_COMMANDS:
                break
            if line.startswith("/"):
                await handle_command(app, line)
                continue
            await render_turn(app, line)
    finally:
        if handler_installed:
            try:
                loop.remove_signal_handler(signal.SIGINT)
            except (NotImplementedError, RuntimeError):
                pass
