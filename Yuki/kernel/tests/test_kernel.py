"""内核可嵌入性契约：turn、命名空间记忆、会话导入导出。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.core.app import App
from yuki_kernel.core.memory import MemoryStore, Session, SessionStore
from yuki_kernel.providers import ChatChunk
from yuki_kernel.skills import ToolRegistry
from yuki_kernel.skills.package_manager import PackageManager

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider, env_tool_call_chunk


@pytest.mark.asyncio
async def test_agent_turn(settings, tmp_path):
    registry = ToolRegistry(None)
    fake = FakeProvider(
        script=[
            [env_tool_call_chunk()],
            [ChatChunk(content="环境信息已返回。"), ChatChunk(done=True)],
        ],
        settings=settings,
    )
    store = MemoryStore(tmp_path / "data")
    agent = Agent(
        "fake",
        registry,
        settings,
        provider=fake,
        memory_store=store,
    )

    result = await agent.turn("看看环境")

    assert "环境信息" in result.content
    assert result.tool_calls
    assert any(
        "看看环境" in hit.content for hit in store.search("看看环境", 5)
    )


@pytest.mark.asyncio
async def test_app_reload_closes_old_provider(settings, store, tmp_path):
    app = App(settings, store, PackageManager(tmp_path / "packages"))
    old_provider = app.agent.provider

    await app.reload(settings)

    assert old_provider.closed


def test_memory_namespace_isolated(tmp_path):
    store_a = MemoryStore(tmp_path / "data", namespace="robot_a")
    store_b = MemoryStore(tmp_path / "data", namespace="robot_b")
    store_a.add("s1", "机器人A的秘密")
    store_b.add("s2", "机器人B的秘密")

    assert any("机器人A" in hit.content for hit in store_a.search("机器人", 5))
    assert not any("机器人A" in hit.content for hit in store_b.search("机器人", 5))


def test_session_import_export(tmp_path):
    session = Session(name="test", messages=[{"role": "user", "content": "你好"}])
    path = tmp_path / "session.jsonl"

    SessionStore.export(session, path)
    imported = SessionStore.import_file(path, name="test")

    assert imported.messages == session.messages
    assert imported.name == "test"
