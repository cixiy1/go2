"""中间件与事件总线契约。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.core.events import AgentEvent, EventBus, Middleware
from yuki_kernel.providers import ChatChunk
from yuki_kernel.skills import ToolRegistry

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider


class Recorder(Middleware):
    def __init__(self):
        self.kinds = []

    async def before(self, event: AgentEvent) -> AgentEvent:
        self.kinds.append(("before", event.kind))
        return event

    async def after(self, event: AgentEvent) -> None:
        self.kinds.append(("after", event.kind))


@pytest.mark.asyncio
async def test_hooks_and_bus(settings):
    recorder = Recorder()
    bus = EventBus()
    bus_kinds = []

    async def on_chunk(event):
        bus_kinds.append(event.kind)

    bus.subscribe("assistant_chunk", on_chunk)
    fake = FakeProvider(
        script=[[ChatChunk(content="你好。"), ChatChunk(done=True)]],
        settings=settings,
    )
    agent = Agent(
        "fake",
        ToolRegistry(None),
        settings,
        provider=fake,
        middlewares=[recorder],
        bus=bus,
    )

    async for _ in agent.send_message("你好"):
        pass
    await agent.close()

    assert ("before", "user_message") in recorder.kinds
    assert ("before", "assistant_chunk") in recorder.kinds
    assert ("before", "session_end") in recorder.kinds
    assert "assistant_chunk" in bus_kinds
