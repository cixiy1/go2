"""Anthropic provider：内核内置，懒加载 SDK，完整支持工具调用。"""

import json
from collections.abc import AsyncIterator, Mapping, Sequence
from typing import Any, cast

from ..config import Settings
from .base import ChatChunk, Provider

THINKING_BUDGET = 1024


class AnthropicProvider(Provider):
    def __init__(self, model: str, settings: Settings):
        super().__init__(model, settings)
        try:
            from anthropic import AsyncAnthropic
        except ImportError as err:
            raise ImportError("AnthropicProvider 需要安装 anthropic") from err
        self.client = AsyncAnthropic(
            api_key=self.settings.anthropic_api_key,
            base_url=self.settings.anthropic_base_url,
        )

    async def chat(
        self,
        messages: Sequence[Mapping[str, Any]],
        tools: Sequence[Mapping[str, Any]] | None = None,
        **kwargs,
    ) -> AsyncIterator[ChatChunk]:
        kwargs.pop("stream", None)
        request: dict[str, Any] = {
            "model": self.model,
            "messages": [
                message for message in messages if message.get("role") != "system"
            ],
            "stream": True,
            **kwargs,
        }
        system = "\n\n".join(
            str(message.get("content", ""))
            for message in messages
            if message.get("role") == "system"
        )
        if system:
            request["system"] = system
        anthropic_tools = [self._to_anthropic_tool(tool) for tool in (tools or [])]
        if anthropic_tools:
            request["tools"] = anthropic_tools
        if self.settings.think:
            request["thinking"] = {"type": "enabled", "budget_tokens": THINKING_BUDGET}

        stream = cast(
            Any,
            await self.client.messages.create(**request),
        )
        tool_blocks: dict[int, dict[str, Any]] = {}
        async for event in stream:
            event_type = getattr(event, "type", None)
            if event_type == "content_block_start":
                block = getattr(event, "content_block", None)
                if block is not None and getattr(block, "type", None) == "tool_use":
                    tool_blocks[getattr(event, "index", 0)] = {
                        "id": getattr(block, "id", ""),
                        "name": getattr(block, "name", ""),
                        "arguments": "",
                    }
            elif event_type == "content_block_delta":
                delta = getattr(event, "delta", None)
                if delta is None:
                    continue
                delta_type = getattr(delta, "type", None)
                if delta_type == "text_delta":
                    yield ChatChunk(content=getattr(delta, "text", None))
                elif delta_type == "thinking_delta":
                    yield ChatChunk(thinking=getattr(delta, "thinking", None))
                elif delta_type == "input_json_delta":
                    index = getattr(event, "index", 0)
                    if index in tool_blocks:
                        tool_blocks[index]["arguments"] += getattr(delta, "partial_json", "")
            elif event_type == "content_block_stop":
                index = getattr(event, "index", 0)
                if index in tool_blocks:
                    block = tool_blocks.pop(index)
                    try:
                        arguments = json.loads(block["arguments"]) if block["arguments"] else {}
                    except json.JSONDecodeError:
                        arguments = {}
                    yield ChatChunk(
                        tool_calls=[
                            {
                                "index": index,
                                "id": block["id"],
                                "function": {
                                    "name": block["name"],
                                    "arguments": json.dumps(arguments, ensure_ascii=False),
                                },
                            }
                        ]
                    )
            elif event_type == "message_stop":
                yield ChatChunk(done=True)

    @staticmethod
    def _to_anthropic_tool(tool: Mapping[str, Any]) -> dict[str, Any]:
        function = tool.get("function", tool)
        return {
            "name": function["name"],
            "description": function.get("description", ""),
            "input_schema": function.get(
                "parameters",
                {"type": "object", "properties": {}},
            ),
        }

    def build_tool_messages(
        self,
        tool_calls: list[dict[str, Any]],
        results: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        assistant_content = []
        tool_results = []
        for index, (call, result) in enumerate(zip(tool_calls, results)):
            tool_use_id = call.get("id") or f"tool_use_{index}"
            assistant_content.append(
                {
                    "type": "tool_use",
                    "id": tool_use_id,
                    "name": call["name"],
                    "input": call["arguments"],
                }
            )
            tool_results.append(
                {
                    "type": "tool_result",
                    "tool_use_id": tool_use_id,
                    "content": result["content"],
                }
            )
        return [
            {"role": "assistant", "content": assistant_content},
            {"role": "user", "content": tool_results},
        ]
