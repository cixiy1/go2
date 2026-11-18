"""无头流式收集与内容清洗。"""

from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from typing import Any

from ...providers import ChatChunk

THINK_TAGS = ("<think>", "</think>")


def clean_content(text: str, pending: str) -> tuple[str, str, bool]:
    """去掉内容流里的 think 标签，返回清理文本、待续标签、是否出现过标签。"""
    combined = pending + text
    saw_tag = any(tag in combined for tag in THINK_TAGS)
    for tag in THINK_TAGS:
        combined = combined.replace(tag, "")
    for tag in THINK_TAGS:
        for size in range(len(tag) - 1, 0, -1):
            if combined.endswith(tag[:size]):
                return combined[:-size], combined[-size:], saw_tag
    return combined, "", saw_tag

@dataclass
class Collected:
    thinking: str = ""
    content: str = ""
    tool_calls: list[Any] = field(default_factory=list)


@dataclass
class StreamEvent:
    """turn_stream 输出的事件，供外部软件渲染。"""

    kind: str  # thinking | content | tool_calls | tool_result | package_restored | done
    text: str = ""
    calls: list[Any] = field(default_factory=list)


class TagFilter:
    """只去掉 think 标签，保留其余文本。"""

    def __init__(self):
        self.pending = ""

    def feed(self, text: str) -> str:
        clean, self.pending, _ = clean_content(text, self.pending)
        return clean


async def collect_stream(stream: AsyncIterator[ChatChunk]) -> Collected:
    """无头消费流式输出，只清理 think 标签，不做渲染。"""
    result = Collected()
    pending = ""
    buffer = ""
    async for chunk in stream:
        if chunk.thinking:
            result.thinking += chunk.thinking
        if chunk.tool_calls:
            result.tool_calls.extend(chunk.tool_calls)
        if chunk.content is not None:
            clean, pending, _ = clean_content(chunk.content, pending)
            buffer += clean
    result.content = buffer
    return result
