"""内核内置 provider 契约。"""

import json
from types import SimpleNamespace

import pytest

from yuki_kernel.config import Settings
from yuki_kernel.providers import (
    AnthropicProvider,
    OpenAIProvider,
    create_provider,
    register_provider,
)
from yuki_kernel.testing import FakeProvider


def _openai_settings():
    return Settings(provider="openai", model="gpt", openai_api_key="x")


def _anthropic_settings():
    return Settings(provider="anthropic", model="claude", anthropic_api_key="x")


def _openai_chunk(content=None, thinking=None, done=False):
    delta = SimpleNamespace(
        content=content,
        reasoning_content=thinking,
        tool_calls=None,
    )
    choice = SimpleNamespace(delta=delta, finish_reason="stop" if done else None)
    return SimpleNamespace(choices=[choice])


class FakeOpenAIStream:
    def __init__(self, chunks):
        self.chunks = chunks

    async def __aiter__(self):
        for chunk in self.chunks:
            yield chunk

    async def close(self):
        pass


class FakeOpenAIClient:
    def __init__(self, chunks):
        self.chunks = chunks
        self.seen = {}

    async def create(self, **kwargs):
        self.seen = kwargs
        return FakeOpenAIStream(self.chunks)


@pytest.mark.asyncio
async def test_openai_translation():
    settings = _openai_settings()
    provider = OpenAIProvider(settings.model, settings)
    provider.client = SimpleNamespace(
        chat=SimpleNamespace(
            completions=FakeOpenAIClient(
                [
                    _openai_chunk(thinking="想"),
                    _openai_chunk(content="你好"),
                    _openai_chunk(done=True),
                ]
            )
        )
    )

    chunks = [chunk async for chunk in provider.chat([{"role": "user", "content": "hi"}])]

    assert chunks[0].thinking == "想"
    assert "".join(c.content or "" for c in chunks) == "你好"
    assert chunks[-1].done is True


def test_openai_build_tool_messages():
    provider = OpenAIProvider("gpt", _openai_settings())
    messages = provider.build_tool_messages(
        [{"id": "c1", "name": "get_env", "arguments": {}}],
        [{"content": "ok"}],
    )
    assert messages[0]["role"] == "assistant"
    assert messages[0]["tool_calls"][0]["id"] == "c1"
    assert messages[1]["role"] == "tool"
    assert messages[1]["tool_call_id"] == "c1"


def _anthropic_event(kind, **payload):
    return SimpleNamespace(type=kind, **payload)


class FakeAnthropicMessages:
    def __init__(self, events):
        self.events = events
        self.seen = {}

    async def create(self, **kwargs):
        self.seen = kwargs
        return self._stream()

    async def _stream(self):
        for event in self.events:
            yield event


@pytest.mark.asyncio
async def test_anthropic_text_and_tool_translation():
    settings = _anthropic_settings()
    provider = AnthropicProvider(settings.model, settings)
    events = [
        _anthropic_event("content_block_start", index=0, content_block=SimpleNamespace(type="text")),
        _anthropic_event("content_block_delta", index=0, delta=SimpleNamespace(type="text_delta", text="你好")),
        _anthropic_event("content_block_stop", index=0),
        _anthropic_event(
            "content_block_start",
            index=1,
            content_block=SimpleNamespace(type="tool_use", id="t1", name="get_env"),
        ),
        _anthropic_event(
            "content_block_delta",
            index=1,
            delta=SimpleNamespace(type="input_json_delta", partial_json='{"city": "纽约"}'),
        ),
        _anthropic_event("content_block_stop", index=1),
        _anthropic_event("message_stop"),
    ]
    provider.client = SimpleNamespace(messages=FakeAnthropicMessages(events))

    chunks = [chunk async for chunk in provider.chat([{"role": "user", "content": "hi"}])]

    assert "".join(c.content or "" for c in chunks) == "你好"
    tool_calls = [call for chunk in chunks for call in chunk.tool_calls]
    assert tool_calls[0]["function"]["name"] == "get_env"
    assert json.loads(tool_calls[0]["function"]["arguments"]) == {"city": "纽约"}
    assert chunks[-1].done is True


def test_anthropic_tools_converted():
    settings = _anthropic_settings()
    provider = AnthropicProvider(settings.model, settings)
    fake = FakeAnthropicMessages([])
    provider.client = SimpleNamespace(messages=fake)
    tools = [
        {
            "type": "function",
            "function": {
                "name": "get_env",
                "description": "环境",
                "parameters": {"type": "object", "properties": {}},
            },
        }
    ]

    import asyncio

    async def run():
        async for _ in provider.chat([{"role": "user", "content": "hi"}], tools=tools):
            pass

    asyncio.run(run())

    assert fake.seen["tools"][0]["name"] == "get_env"
    assert fake.seen["tools"][0]["input_schema"]["type"] == "object"


def test_anthropic_build_tool_messages():
    provider = AnthropicProvider("claude", _anthropic_settings())
    messages = provider.build_tool_messages(
        [{"id": "t1", "name": "get_env", "arguments": {}}],
        [{"content": "ok"}],
    )
    assert messages[0]["role"] == "assistant"
    assert messages[0]["content"][0]["type"] == "tool_use"
    assert messages[1]["role"] == "user"
    assert messages[1]["content"][0]["type"] == "tool_result"
    assert messages[1]["content"][0]["tool_use_id"] == "t1"


def test_registry_builtin_and_custom():
    settings = Settings(
        provider="openai",
        model="gpt",
        openai_api_key="x",
        anthropic_api_key="x",
    )
    assert isinstance(create_provider("openai", "gpt", settings), OpenAIProvider)
    assert isinstance(create_provider("anthropic", "claude", settings), AnthropicProvider)

    register_provider(
        "dummy",
        lambda model, runtime_settings: FakeProvider(runtime_settings, model=model),
    )
    assert isinstance(create_provider("dummy", "m", settings), FakeProvider)
    with pytest.raises(ValueError):
        create_provider("missing", "m", settings)
