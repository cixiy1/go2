"""Provider 抽象与统一流式消息。"""

from abc import ABC, abstractmethod
from collections.abc import AsyncIterator, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from ..config import Settings


@dataclass
class ChatChunk:
    """统一的流式消息片段，屏蔽不同 provider 的消息结构差异。"""

    thinking: str | None = None
    content: str | None = None
    tool_calls: list = field(default_factory=list)
    done: bool = False


class Provider(ABC):
    def __init__(self, model: str, settings: Settings):
        self.model = model
        self.settings = settings

    @abstractmethod
    def chat(
        self,
        messages: Sequence[Mapping[str, Any]],
        tools: Sequence[Mapping[str, Any]] | None = None,
        **kwargs,
    ) -> AsyncIterator[ChatChunk]:
        """调用模型并返回统一格式的异步流式结果。"""

    @abstractmethod
    def build_tool_messages(
        self,
        tool_calls: list[dict[str, Any]],
        results: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        """把工具调用和结果构造成可继续对话的消息。"""

    @staticmethod
    async def start() -> bool:
        return True

    async def close(self, skip_unload: bool = False):
        return None
