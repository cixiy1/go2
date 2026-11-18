"""事件总线：只负责广播，不做控制流。"""

from collections import defaultdict
from collections.abc import Awaitable, Callable

from .events import AgentEvent

Handler = Callable[[AgentEvent], Awaitable[None]]


class EventBus:
    def __init__(self):
        self._handlers: dict[str, list[Handler]] = defaultdict(list)

    def subscribe(self, kind: str, handler: Handler) -> None:
        self._handlers[kind].append(handler)

    async def publish(self, event: AgentEvent) -> None:
        for handler in list(self._handlers.get(event.kind, [])):
            await handler(event)
