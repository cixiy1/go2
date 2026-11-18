"""中间件链：before 可改写或中止事件，after 只做收尾。"""

from collections.abc import Awaitable, Callable

from .events import AgentEvent


class Middleware:
    async def before(self, event: AgentEvent) -> AgentEvent:
        return event

    async def after(self, event: AgentEvent) -> None:
        return None


async def run_before(
    middlewares: list[Middleware],
    event: AgentEvent,
) -> AgentEvent:
    for middleware in middlewares:
        event = await middleware.before(event)
    return event


async def run_after(middlewares: list[Middleware], event: AgentEvent) -> None:
    for middleware in reversed(middlewares):
        await middleware.after(event)


EventHandler = Callable[[AgentEvent], Awaitable[None]]
