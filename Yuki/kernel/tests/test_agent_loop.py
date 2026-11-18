"""工具闭环契约：turn_stream 事件。"""

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.providers import ChatChunk, Provider
from yuki_kernel.skills import ToolRegistry

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider, env_tool_call_chunk


@pytest.mark.asyncio
async def test_turn_stream_tool_loop(settings):
    registry = ToolRegistry(None)
    fake = FakeProvider(
        script=[
            [env_tool_call_chunk()],
            [ChatChunk(content="环境信息已返回。"), ChatChunk(done=True)],
        ],
        settings=settings,
    )
    agent = Agent("fake", registry, settings, provider=fake)

    kinds = []
    content = ""
    async for event in agent.turn_stream("看看环境"):
        kinds.append(event.kind)
        if event.kind == "content":
            content += event.text

    assert "tool_calls" in kinds
    assert "tool_result" in kinds
    assert "环境信息" in content


@pytest.mark.asyncio
async def test_turn_auto_loads_available_package(settings, weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])
    fake = FakeProvider(
        script=[
            [
                ChatChunk(
                    tool_calls=[
                        {
                            "index": 0,
                            "id": "call_1",
                            "function": {
                                "name": "weather_now",
                                "arguments": '{"city":"New York"}',
                            },
                        }
                    ]
                )
            ],
            [ChatChunk(content="纽约 22°C。"), ChatChunk(done=True)],
        ],
        settings=settings,
    )
    agent = Agent("fake", registry, settings, provider=fake)

    texts = []
    async for event in agent.turn_stream("纽约天气"):
        if event.kind == "tool_result":
            texts.append(event.text)

    assert "22°C" in texts
    assert registry.active_packages == []


@pytest.mark.asyncio
async def test_list_packages_then_concrete_tool(settings, weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])
    list_call = {
        "index": 0,
        "id": "call_list",
        "function": {"name": "list_packages", "arguments": "{}"},
    }
    weather_call = {
        "index": 0,
        "id": "call_weather",
        "function": {"name": "weather_now", "arguments": '{"city":"New York"}'},
    }
    fake = FakeProvider(
        script=[
            [ChatChunk(tool_calls=[list_call])],
            [ChatChunk(tool_calls=[weather_call])],
            [ChatChunk(content="纽约 22°C。"), ChatChunk(done=True)],
        ],
        settings=settings,
    )
    agent = Agent("fake", registry, settings, provider=fake)

    results = []
    content = ""
    async for event in agent.turn_stream("纽约天气"):
        if event.kind == "tool_result":
            results.append(event.text)
        if event.kind == "content":
            content += event.text

    assert any("22°C" in text for text in results)
    assert "纽约 22°C。" in content
    assert registry.active_packages == []


class _RecordingProvider(Provider):
    def __init__(self, settings, script):
        super().__init__("rec", settings)
        self.script = script
        self.requests = []
        self.calls = 0

    async def chat(self, messages, tools=None, **kwargs):
        del tools, kwargs
        self.requests.append([dict(message) for message in messages])
        chunks = self.script[min(self.calls, len(self.script) - 1)]
        self.calls += 1
        for chunk in chunks:
            yield chunk

    def build_tool_messages(self, tool_calls, results):
        assistant = {
            "role": "assistant",
            "content": None,
            "tool_calls": [
                {
                    "id": call.get("id"),
                    "type": "function",
                    "function": {"name": call["name"], "arguments": "{}"},
                }
                for call in tool_calls
            ],
        }
        return [
            assistant,
            *[
                {
                    "role": "tool",
                    "tool_call_id": call.get("id"),
                    "content": result["content"],
                }
                for call, result in zip(tool_calls, results)
            ],
        ]


@pytest.mark.asyncio
async def test_tool_result_is_fed_back_to_next_request(settings, weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])
    list_call = {
        "index": 0,
        "id": "call_list",
        "function": {"name": "list_packages", "arguments": "{}"},
    }
    weather_call = {
        "index": 0,
        "id": "call_weather",
        "function": {"name": "weather_now", "arguments": '{"city":"New York"}'},
    }
    provider = _RecordingProvider(
        settings,
        [
            [ChatChunk(tool_calls=[list_call])],
            [ChatChunk(tool_calls=[weather_call])],
            [ChatChunk(content="纽约 22°C。"), ChatChunk(done=True)],
        ],
    )
    agent = Agent("rec", registry, settings, provider=provider)

    async for _event in agent.turn_stream("纽约天气"):
        pass

    assert len(provider.requests) == 3
    second = provider.requests[1]
    assert any(
        message.get("role") == "assistant"
        and message.get("tool_calls")
        and message["tool_calls"][0]["function"]["name"] == "list_packages"
        for message in second
    )
    assert any(
        message.get("role") == "tool"
        and "weather" in (message.get("content") or "")
        for message in second
    )
    third = provider.requests[2]
    assert any(
        message.get("role") == "tool"
        and "22°C" in (message.get("content") or "")
        for message in third
    )
    assert any(message.get("role") == "tool" for message in agent.memory)
