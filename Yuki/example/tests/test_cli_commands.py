"""CLI 斜杠命令契约。"""

# noinspection PyUnresolvedReferences
import pytest

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

# noinspection PyUnresolvedReferences
from yuki_kernel.providers import ChatChunk

# noinspection PyUnresolvedReferences
from yuki_kernel.skills.package_manager import PackageManager

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider

from yuki.commands import handle_command


@pytest.mark.asyncio
async def test_session_commands(settings, store, tmp_path):
    app = App(
        settings,
        store,
        PackageManager(tmp_path / "packages"),
    )
    app.agent.provider = FakeProvider(
        script=[[ChatChunk(content="ok"), ChatChunk(done=True)]],
        settings=settings,
    )
    app.session.messages.append({"role": "user", "content": "你好"})

    await handle_command(app, "/save 测试会话")
    assert [meta.name for meta in store.list_sessions()] == ["测试会话"]

    await handle_command(app, "/new")
    assert [message["role"] for message in app.agent.memory] == ["system"]

    await handle_command(app, "/load 测试会话")
    assert app.agent.session.name == "测试会话"
    assert app.agent.memory[-1] == {"role": "user", "content": "你好"}
    assert app.session is app.agent.session

    await handle_command(app, "/sessions")
    assert [meta.name for meta in store.list_sessions()] == ["测试会话"]


@pytest.mark.asyncio
async def test_pkg_install_prints_scan(settings, store, tmp_path, capsys, weather_package):
    app = App(settings, store, PackageManager(tmp_path / "installed"))
    await handle_command(app, f"/pkg install {weather_package / 'weather'}")

    out = capsys.readouterr().out
    assert "已安装：weather 1.0.0" in out
    assert "发现外置工具包：weather" in out
    assert "可用外置工具包：weather" in out
