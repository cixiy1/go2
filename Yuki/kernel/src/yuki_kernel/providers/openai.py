"""OpenAI 兼容 provider：内核内置，懒加载 SDK。"""

import json
from collections.abc import AsyncIterator, Mapping, Sequence
from typing import Any, cast

from ..config import Settings
from .base import ChatChunk, Provider


class OpenAIProvider(Provider):
    def __init__(
        self,
        model: str,
        settings: Settings,
        api_key: str | None = None,
        base_url: str | None = None,
    ):
        super().__init__(model, settings)
        try:
            from openai import AsyncOpenAI
        except ImportError as err:
            raise ImportError("OpenAIProvider 需要安装 openai") from err
        self.client = AsyncOpenAI(
            api_key=api_key or self.settings.openai_api_key,
            base_url=base_url or self.settings.openai_base_url,
        )

    async def chat(
        self,
        messages: Sequence[Mapping[str, Any]],
        tools: Sequence[Mapping[str, Any]] | None = None,
        **kwargs,
    ) -> AsyncIterator[ChatChunk]:
        kwargs.pop("stream", None)
        if self.settings.think:
            extra_body = kwargs.setdefault("extra_body", {})
            extra_body.setdefault("thinking", {"type": "enabled"})
        stream = cast(
            Any,
            await self.client.chat.completions.create(
                model=cast(Any, self.model),
                messages=cast(Any, messages),
                tools=cast(Any, tools),
                stream=True,
                **kwargs,
            ),
        )
        try:
            async for chunk in stream:
                choices = getattr(chunk, "choices", None) or []
                if not choices:
                    continue
                choice = choices[0]
                delta = getattr(choice, "delta", None)
                if delta is None:
                    continue
                thinking = getattr(delta, "reasoning_content", None) or getattr(delta, "thinking", None)
                yield ChatChunk(
                    thinking=thinking,
                    content=getattr(delta, "content", None),
                    tool_calls=list(getattr(delta, "tool_calls", None) or []),
                    done=bool(getattr(choice, "finish_reason", False)),
                )
        finally:
            try:
                await stream.close()
            except (AttributeError, RuntimeError, OSError):
                pass

    async def close(self, skip_unload: bool = False):
        await self.client.close()

    def build_tool_messages(
        self,
        tool_calls: list[dict[str, Any]],
        results: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        assistant = {
            "role": "assistant",
            "content": None,
            "tool_calls": [
                {
                    "id": call.get("id") or f"call_{index}",
                    "type": "function",
                    "function": {
                        "name": call["name"],
                        "arguments": json.dumps(call["arguments"], ensure_ascii=False),
                    },
                }
                for index, call in enumerate(tool_calls)
            ],
        }
        tool_messages = [
            {
                "role": "tool",
                "tool_call_id": call.get("id") or f"call_{index}",
                "content": result["content"],
            }
            for index, (call, result) in enumerate(zip(tool_calls, results))
        ]
        return [assistant] + tool_messages
