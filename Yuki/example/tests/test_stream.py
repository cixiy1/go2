"""流式渲染与无头收集契约。"""

# noinspection PyUnresolvedReferences
import pytest

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

# noinspection PyUnresolvedReferences
from yuki_kernel.core.context import collect_stream

# noinspection PyUnresolvedReferences
from yuki_kernel.providers import ChatChunk

# noinspection PyUnresolvedReferences
from yuki_kernel.skills.package_manager import PackageManager

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider

from yuki.rendering import render_turn


@pytest.mark.asyncio
async def test_render_turn_keeps_original_content(settings, store, tmp_path, capsys):
    app = App(settings, store, PackageManager(tmp_path / "packages"))
    app.agent.provider = FakeProvider(
        script=[[ChatChunk(content="纽约22°C。</think>纽约22°C。"), ChatChunk(done=True)]],
        settings=settings,
    )
    await render_turn(app, "纽约天气")
    assert "纽约22°C。</think>纽约22°C。" in capsys.readouterr().out


@pytest.mark.asyncio
async def test_collect_stream():
    async def stream():
        yield ChatChunk(thinking="思考中")
        yield ChatChunk(content="你好")
        yield ChatChunk(done=True)

    collected = await collect_stream(stream())
    assert collected.thinking == "思考中"
    assert collected.content == "你好"
