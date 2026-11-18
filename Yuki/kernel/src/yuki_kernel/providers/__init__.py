"""Provider 抽象、内置厂商与注册口。

内置 Provider 采用懒加载：仅当首次创建对应 Provider 时才 import 厂商 SDK，
裸装 yuki-kernel（不装 openai / anthropic extras）也能 import 内核并使用自定义 Provider。
"""

from collections.abc import Callable

from ..config import Settings
from .anthropic import AnthropicProvider
from .base import ChatChunk, Provider
from .openai import OpenAIProvider

__all__ = [
    "AnthropicProvider",
    "ChatChunk",
    "OpenAIProvider",
    "Provider",
    "create_provider",
    "register_provider",
]

_PROVIDERS: dict[str, Callable[[str, Settings], Provider]] = {
    "openai": OpenAIProvider,
    "anthropic": AnthropicProvider,
}


def _resolve(name: str) -> Callable[[str, Settings], Provider]:
    factory = _PROVIDERS.get(name)
    if factory is None:
        raise ValueError(f"未注册的 provider：{name}，可选 {list(_PROVIDERS)}")
    return factory


def register_provider(name: str, factory: Callable[[str, Settings], Provider]) -> None:
    _PROVIDERS[name] = factory


def create_provider(
    name: str,
    model: str,
    settings: Settings,
) -> Provider:
    return _resolve(name)(model, settings)
