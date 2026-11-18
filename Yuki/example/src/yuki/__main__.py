"""Yuki 入口。"""

import asyncio
import sys
import traceback
from pathlib import Path
from typing import cast

sys.path.insert(0, str(Path(cast(str, __file__)).resolve().parent.parent))

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

# noinspection PyUnresolvedReferences
from yuki_kernel.core.memory import SessionStore

# noinspection PyUnresolvedReferences
from yuki_kernel.skills.package_manager import PackageManager

from yuki.approver import cli_approver
from yuki.cli import run
from yuki.settings import EXAMPLE_ROOT, load_settings


def _env_mtime(project_root: Path):
    env_file = project_root / ".env"
    if not env_file.exists():
        return None
    return env_file.stat().st_mtime


async def watch_env(app: App, stop: asyncio.Event):
    last = _env_mtime(EXAMPLE_ROOT)
    while not stop.is_set():
        await asyncio.sleep(2)
        current = _env_mtime(EXAMPLE_ROOT)
        if current is not None and current != last:
            last = current
            await app.reload(load_settings())
            print("检测到 .env 变化，配置已热加载")


async def main():
    settings = load_settings()
    assert settings.data_dir is not None
    assert settings.packages_dir is not None
    store = SessionStore(settings.data_dir)
    package_manager = PackageManager(settings.packages_dir)
    app = App(settings, store, package_manager, approver=cli_approver)
    scan = app.registry.package_scan
    for package_id in scan.packages:
        print(f"发现外置工具包：{package_id}")
    for name, reason in scan.skipped:
        print(f"跳过外置工具包 {name}：{reason}")
    if scan.available:
        print(f"可用外置工具包：{'、'.join(scan.available)}")
    else:
        print("可用外置工具包：无")
    for package in app.registry.available_packages:
        if package["loaded"]:
            tools = "、".join(package["tools"]) or "无"
            prompts = "、".join(package["prompts"]) or "无"
            print(f"已加载 {package['id']}：工具 {tools}；提示词 {prompts}")
    stop = asyncio.Event()
    watcher = asyncio.create_task(watch_env(app, stop))

    try:
        await app.agent.start()
        await run(app)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print(".\n手动退出....")
    finally:
        stop.set()
        watcher.cancel()
        try:
            await watcher
        except asyncio.CancelledError:
            pass
        err = sys.exc_info()[1]
        if err is not None:
            traceback.print_exception(*sys.exc_info())
        try:
            await app.agent.close()
            print("\n程序结束，资源清理完成")
        except Exception as err:  # noqa: BLE001 - CLI 退出前兜底清理，需捕获一切异常
            print(f"资源清理异常：{err!r}")
        sys.stdout.flush()
        sys.stderr.flush()
        sys.exit(1 if err is not None else 0)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
