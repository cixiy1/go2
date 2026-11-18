"""Yuki 斜杠命令分发入口。"""

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

from .packages import handle_pkg
from .sessions import handle_session


async def handle_command(app: App, line: str) -> bool:
    parts = line.split(maxsplit=1)
    cmd = parts[0].lower()
    arg = parts[1].strip() if len(parts) > 1 else ""

    if cmd == "/pkg":
        await handle_pkg(app, arg)
    else:
        await handle_session(app, cmd, arg)
    return True
