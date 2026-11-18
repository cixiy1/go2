"""重试与工具错误回传契约。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.core.errors import ProviderError
from yuki_kernel.providers import ChatChunk
from yuki_kernel.skills import ToolRegistry
from yuki_kernel.skills.types import Tool

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider


@pytest.mark.asyncio
async def test_provider_retry(settings):
    fake = FakeProvider(
        errors=[ProviderError("boom")],
        script=[[ChatChunk(content="ok"), ChatChunk(done=True)]],
        settings=settings,
    )
    agent = Agent("fake", ToolRegistry(None), settings, provider=fake)
    text = ""
    async for chunk in agent.send_message("hi"):
        if chunk.content:
            text += chunk.content
    assert text == "ok"
    assert fake.calls == 2


def test_tool_error_returned(tmp_path):
    module_dir = tmp_path / "tools"
    module_dir.mkdir()
    (module_dir / "boom.py").write_text(
        "def run():\n    raise RuntimeError('bad')\n",
        encoding="utf-8",
    )
    registry = ToolRegistry(None)
    registry.register_tool(
        Tool(
            name="boom",
            description="",
            parameters={"type": "object", "properties": {}},
            entry={
                "type": "python",
                "module": "boom.py",
                "handler": "run",
            },
            package="test",
            package_dir=str(module_dir),
        )
    )
    assert registry.execute("boom", {}) == "工具执行失败：bad"
