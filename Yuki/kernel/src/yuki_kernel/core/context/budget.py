"""上下文预算与摘要。"""

import asyncio
import math
import re
from collections.abc import Callable
from typing import Any

from ...config import Settings
from ..errors import ProviderError
from ..memory import MemoryStore

# CJK 统一表意文字区（含扩展 A），1 字符≈1 token；
# 其余字符（拉丁字母、标点、空白等）按 ~4 字符≈1 token。
_CJK_RE = re.compile(r"[㐀-䶿一-鿿豈-﫿]")


def estimate_tokens(text: str) -> int:
    """估算一段文本的 token 数，对中文按 1 字符≈1 token 计。"""
    if not text:
        return 0
    cjk = len(_CJK_RE.findall(text))
    latin = len(text) - cjk
    return cjk + math.ceil(latin / 4)

MemoryProvider = Callable[[], list[dict[str, Any]]]
Chat = Callable[..., Any]
SessionIdProvider = Callable[[], str]


class ContextManager:
    def __init__(
        self,
        settings: Settings,
        memory_provider: MemoryProvider,
        chat: Chat,
        memory_store: MemoryStore | None,
        session_id_provider: SessionIdProvider,
    ):
        self.settings = settings
        self.memory_provider = memory_provider
        self.chat = chat
        self.memory_store = memory_store
        self.session_id_provider = session_id_provider

    def estimate(self) -> int:
        total = 0
        for message in self.memory_provider():
            content = message.get("content") or ""
            total += estimate_tokens(content) + 4
        return total

    async def ensure(self) -> None:
        if self.settings.max_context_tokens <= 0:
            return
        for _ in range(3):
            if self.estimate() <= self.settings.max_context_tokens:
                return
            if not await self._summarize_oldest():
                self._drop_oldest()

    async def _summarize_oldest(self) -> bool:
        memory = self.memory_provider()
        keep = self.settings.keep_recent_messages
        if len(memory) - 1 <= keep:
            return False
        target = [
            message
            for message in memory[1 : len(memory) - keep]
            if not (
                message.get("role") == "system"
                and str(message.get("content", "")).startswith("[历史摘要]")
            )
        ]
        if not target:
            return False
        prompt = [
            {"role": "system", "content": "把以下对话压缩成简洁中文摘要，保留关键事实、结论和未完成事项。"},
            *target,
        ]
        try:
            summary = await self._collect(prompt)
        except (ProviderError, ConnectionError, TimeoutError, OSError):
            return False
        if not summary.strip():
            return False
        memory[1 : len(memory) - keep] = [
            {"role": "system", "content": "[历史摘要] " + summary.strip()}
        ]
        if self.memory_store is not None:
            await asyncio.to_thread(
                self.memory_store.add,
                self.session_id_provider(),
                "摘要：" + summary.strip(),
            )
        return True

    async def _collect(self, messages: list[dict[str, Any]]) -> str:
        parts = []
        async for chunk in self.chat(messages, tools=None):
            if chunk.content:
                parts.append(chunk.content)
        return "".join(parts)

    def _drop_oldest(self) -> None:
        memory = self.memory_provider()
        for index, message in enumerate(memory):
            if message.get("role") in {"user", "assistant"}:
                memory.pop(index)
                return
