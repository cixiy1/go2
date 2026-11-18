"""上下文摘要契约。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.core.context.budget import estimate_tokens
from yuki_kernel.providers import ChatChunk
from yuki_kernel.skills import ToolRegistry

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider


def test_estimate_tokens_chinese_aware():
    # 中文按 1 字符≈1 token；纯字母按 ~4 字符≈1 token
    assert estimate_tokens("你好世界") == 4
    assert estimate_tokens("hello") == 2  # ceil(5/4)=2
    assert estimate_tokens("hi你好") == 3  # 2 拉丁 + 2 中文
    assert estimate_tokens("") == 0


@pytest.mark.asyncio
async def test_summarize_oldest(settings):
    settings.max_context_tokens = 60
    settings.keep_recent_messages = 1
    fake = FakeProvider(
        script=[[ChatChunk(content="这是历史摘要。")]],
        settings=settings,
    )
    agent = Agent("fake", ToolRegistry(None), settings, provider=fake)
    agent.memory.append({"role": "user", "content": "a" * 100})
    agent.memory.append({"role": "assistant", "content": "b" * 100})
    agent.memory.append({"role": "user", "content": "最近消息"})

    await agent._context.ensure()

    assert fake.calls == 1
    assert any(
        message.get("content", "").startswith("[历史摘要]")
        for message in agent.memory
    )


@pytest.mark.asyncio
async def test_no_summary_when_under_budget(settings):
    settings.max_context_tokens = 100000
    fake = FakeProvider(
        script=[[ChatChunk(content="ok"), ChatChunk(done=True)]],
        settings=settings,
    )
    agent = Agent("fake", ToolRegistry(None), settings, provider=fake)

    async for _ in agent.send_message("你好"):
        pass

    assert fake.calls == 1
