"""测试辅助：fake provider 与样例工具包，供 kernel/example 测试共用。"""

import json
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any

import pytest

from .config import Settings
from .core.memory import SessionStore
from .providers import ChatChunk, Provider, register_provider


class FakeProvider(Provider):
    def __init__(
        self,
        settings: Settings,
        script: Any = None,
        model: str = "fake",
        errors: Any = None,
    ):
        super().__init__(model, settings)
        self.script = list(script or [])
        self.errors = list(errors or [])
        self.calls = 0
        self.closed = False

    async def chat(
        self,
        messages,
        tools=None,
        **kwargs,
    ) -> AsyncIterator[ChatChunk]:
        del messages, tools, kwargs
        if self.calls < len(self.errors):
            err = self.errors[self.calls]
            self.calls += 1
            raise err
        chunks = self.script[min(self.calls, len(self.script) - 1)] if self.script else []
        self.calls += 1
        for chunk in chunks:
            yield chunk

    async def close(self, skip_unload: bool = False):
        self.closed = True

    def build_tool_messages(
        self,
        tool_calls: list[dict[str, Any]],
        results: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        assistant = {
            "role": "assistant",
            "content": "",
            "tool_calls": [
                {"function": {"name": call["name"], "arguments": call["arguments"]}}
                for call in tool_calls
            ],
        }
        return [
            assistant,
            *[{"role": "tool", "content": result["content"]} for result in results],
        ]


def env_tool_call_chunk() -> ChatChunk:
    return ChatChunk(
        tool_calls=[
            {
                "index": 0,
                "id": "call_1",
                "function": {
                    "name": "get_environment_info",
                    "arguments": "{}",
                },
            }
        ]
    )


def register_fake_provider() -> None:
    register_provider(
        "fake",
        lambda model, settings: FakeProvider(settings=settings, model=model),
    )


def make_weather_package(dest: Path) -> Path:
    weather = dest / "weather"
    weather.mkdir(parents=True)
    (weather / "manifest.json").write_text(
        json.dumps(
            {
                "id": "weather",
                "name": "天气工具包",
                "version": "1.0.0",
                "description": "查询指定城市当前气温",
                "tools": [
                    {
                        "name": "weather_now",
                        "description": "查询指定城市当前气温",
                        "parameters": {
                            "type": "object",
                            "required": ["city"],
                            "properties": {
                                "city": {"type": "string", "description": "城市英文名"}
                            },
                        },
                        "entry": {
                            "type": "python",
                            "module": "tool.py",
                            "handler": "weather_now",
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    (weather / "tool.py").write_text(
        "def weather_now(city):\n    return '22°C'\n",
        encoding="utf-8",
    )
    return weather


register_fake_provider()

__all__ = ["FakeProvider", "env_tool_call_chunk", "pytest"]


@pytest.fixture
def settings(tmp_path):
    return Settings(
        provider="fake",
        model="fake",
        data_dir=tmp_path / "data",
        packages_dir=tmp_path / "packages",
        retry_max=2,
        retry_base=0.01,
        max_context_tokens=100000,
        keep_recent_messages=2,
    )


@pytest.fixture
def store(settings):
    assert settings.data_dir is not None
    return SessionStore(settings.data_dir)


@pytest.fixture
def weather_package(tmp_path):
    dest = tmp_path / "packages"
    make_weather_package(dest)
    return dest
