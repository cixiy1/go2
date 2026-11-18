"""长期记忆契约。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.core.memory import MemoryStore
from yuki_kernel.skills import ToolRegistry

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider


def test_memory_search(tmp_path):
    store = MemoryStore(tmp_path / "data")
    store.add("s1", "用户：你好\n助手：你好呀")
    store.add("s2", "用户：纽约天气\n助手：纽约22度")

    hits = store.search("纽约", limit=5)
    assert any("纽约" in hit.content for hit in hits)


def test_search_memory_tool(tmp_path):
    store = MemoryStore(tmp_path / "data")
    store.add("old", "用户：纽约天气\n助手：纽约22度")
    registry = ToolRegistry(
        None,
        memory_searcher=lambda query: store.search_text(query, 5),
    )

    content = registry.execute("search_memory", {"query": "纽约"})
    assert "纽约" in content


def test_search_memory_disabled():
    registry = ToolRegistry(None)
    assert registry.execute("search_memory", {"query": "纽约"}) == "长期记忆未启用"


@pytest.mark.asyncio
async def test_remember_stores_turn(settings, tmp_path):
    store = MemoryStore(tmp_path / "data")
    agent = Agent(
        "fake",
        ToolRegistry(None),
        settings,
        provider=FakeProvider(settings=settings),
        memory_store=store,
    )

    await agent.remember("你好", "你好呀")

    assert any("你好呀" in hit.content for hit in store.search("你好", 5))
