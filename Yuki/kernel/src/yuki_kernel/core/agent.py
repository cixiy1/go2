"""Agent：异步会话闭环，包含重试、摘要、审批与钩子。"""

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable
from dataclasses import dataclass, field
from typing import Any

from ..config import Settings
from ..providers import ChatChunk, Provider, create_provider
from ..skills import ToolRegistry
from ..skills.tools import merge_tool_calls
from .context import ContextManager, StreamEvent, TagFilter, clean_content
from .errors import is_transient
from .events import AgentEvent, EventBus, Middleware, run_after, run_before
from .memory import MemoryStore, Session
from .policy import ApprovalGate

Approver = Callable[[str, dict[str, Any]], Awaitable[str]]


@dataclass
class TurnResult:
    thinking: str = ""
    content: str = ""
    tool_calls: list[Any] = field(default_factory=list)
    changed_packages: list[str] = field(default_factory=list)


class Agent:
    def __init__(
        self,
        model: str,
        registry: ToolRegistry,
        settings: Settings,
        session: Session | None = None,
        provider: str | Provider = "openai",
        system_prompt: str = "",
        middlewares: list[Middleware] | None = None,
        bus: EventBus | None = None,
        approver: Approver | None = None,
        memory_store: MemoryStore | None = None,
    ):
        self.model = model
        self.settings = settings
        self.registry = registry
        self.session = session or Session()
        self.system_prompt = system_prompt
        self.memory = self.session.messages
        self.middlewares = list(middlewares or [])
        self.bus = bus or EventBus()
        self.approver = approver
        self.memory_store = memory_store
        if isinstance(provider, Provider):
            self.provider = provider
        else:
            self.provider = create_provider(provider, model, settings)
        self._approval = ApprovalGate(self.registry, lambda: self.session, lambda: self.approver)
        self._context = ContextManager(
            settings,
            lambda: self.memory,
            self._chat_with_retry,
            memory_store,
            lambda: self.session.session_id,
        )
        self._sync_system_prompt()

    def _sync_system_prompt(self):
        parts = [self.system_prompt] if self.system_prompt else []
        registry_prompt = self.registry.system_prompt()
        if registry_prompt:
            parts.append(registry_prompt)
        content = "\n\n".join(parts)
        if content:
            message = {"role": "system", "content": content}
            if self.memory and self.memory[0].get("role") == "system":
                self.memory[0] = message
            else:
                self.memory.insert(0, message)
        elif self.memory and self.memory[0].get("role") == "system":
            self.memory.pop(0)

    async def _emit(self, kind: str, payload: Any, **context: Any) -> AgentEvent:
        event = AgentEvent(kind=kind, payload=payload, context=context)
        event = await run_before(self.middlewares, event)
        await self.bus.publish(event)
        await run_after(self.middlewares, event)
        return event

    async def start(self):
        await self._emit("session_start", self.session)
        return await self.provider.start()

    async def close(self, skip_unload: bool = False):
        await self._emit("session_end", self.session)
        return await self.provider.close(skip_unload=skip_unload)

    def switch_session(self, session: Session) -> None:
        self.session = session
        self.memory = session.messages
        self._sync_system_prompt()

    async def send_message(self, user_message: str) -> AsyncIterator[ChatChunk]:
        event = await self._emit("user_message", user_message)
        if event.context.get("abort"):
            return
        self.memory.append({"role": "user", "content": user_message})
        async for chunk in self._stream_with_hooks(self.memory, self.registry.tools):
            yield chunk

    async def continue_with_tools(
        self,
        tool_calls: list[Any],
        results: list[dict[str, Any]],
    ) -> AsyncIterator[ChatChunk]:
        self._append_tool_messages(tool_calls, results)
        async for chunk in self._stream_with_hooks(self.memory, self.registry.tools):
            yield chunk

    def _append_tool_messages(
        self,
        tool_calls: list[Any],
        results: list[dict[str, Any]],
    ) -> None:
        calls = merge_tool_calls(tool_calls)
        self.memory.extend(self.provider.build_tool_messages(calls, results))

    async def turn(self, user_message: str) -> TurnResult:
        """无头完整对话回合，等价于收集 turn_stream 的事件。"""
        result = TurnResult()
        async for event in self.turn_stream(user_message):
            if event.kind == "thinking":
                result.thinking += event.text
            elif event.kind == "content":
                result.content += event.text
            elif event.kind == "tool_calls":
                result.tool_calls.extend(event.calls)
            elif event.kind == "package_restored":
                result.changed_packages.append(event.text)
        result.content, _, _ = clean_content(result.content, "")
        return result

    async def turn_stream(self, user_message: str) -> AsyncIterator[StreamEvent]:
        """流式完整对话回合：工具循环、包还原、记忆写入都在内核内完成。"""
        active_packages = self.registry.active_packages
        tag_filter = TagFilter()
        full_content = ""

        async def consume(stream: AsyncIterator[ChatChunk], calls: list[Any]):
            nonlocal full_content
            calls.clear()
            async for chunk in stream:
                if chunk.thinking:
                    yield StreamEvent(kind="thinking", text=chunk.thinking)
                if chunk.tool_calls:
                    calls.extend(chunk.tool_calls)
                    yield StreamEvent(kind="tool_calls", calls=list(chunk.tool_calls))
                if chunk.content is not None:
                    full_content += chunk.content
                    yield StreamEvent(kind="content", text=chunk.content)

        tool_calls: list[Any] = []
        async for event in consume(self.send_message(user_message), tool_calls):
            yield event
        while tool_calls:
            results = await self.execute_tool_calls(tool_calls)
            for result in results:
                yield StreamEvent(kind="tool_result", text=result["content"])
            self._append_tool_messages(tool_calls, results)
            async for event in consume(
                self._stream_with_hooks(self.memory, self.registry.tools),
                tool_calls,
            ):
                yield event

        changed = await self.restore_packages(active_packages)
        for package_id in changed:
            yield StreamEvent(kind="package_restored", text=package_id)
        memory_content = tag_filter.feed(full_content)
        if memory_content:
            self.memory.append({"role": "assistant", "content": memory_content})
            await self.remember(user_message, memory_content)
        yield StreamEvent(kind="done")

    async def _stream_with_hooks(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> AsyncIterator[ChatChunk]:
        await self._context.ensure()
        await self._emit("before_model", {"messages": messages, "tools": tools})
        async for chunk in self._chat_with_retry(messages, tools=tools):
            await self._emit("assistant_chunk", chunk)
            yield chunk

    async def remember(self, user_content: str, assistant_content: str) -> None:
        if self.memory_store is None:
            return
        content = f"用户：{user_content}\n助手：{assistant_content}"
        await asyncio.to_thread(self.memory_store.add, self.session.session_id, content)

    async def execute_tool_calls(self, tool_calls: list[Any]) -> list[dict[str, Any]]:
        results = []
        for call in merge_tool_calls(tool_calls):
            event = await self._emit("tool_call", call)
            if event.context.get("abort"):
                content = "用户已中止"
            elif not await self._approval.check(call["name"], call["arguments"]):
                content = "用户拒绝执行"
            else:
                content = await asyncio.to_thread(
                    self.registry.execute,
                    call["name"],
                    call["arguments"],
                )
            results.append({"role": "tool", "name": call["name"], "content": content})
            await self._emit("tool_result", {"name": call["name"], "content": content})
        self._sync_system_prompt()
        return results

    async def restore_packages(self, package_ids: list[str]) -> list[str]:
        changed = self.registry.restore_packages(package_ids)
        for package_id in changed:
            kind = "package_load" if package_id in self.registry.active_packages else "package_unload"
            await self._emit(kind, package_id)
        self._sync_system_prompt()
        return changed

    async def _chat_with_retry(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]] | None = None,
    ) -> AsyncIterator[ChatChunk]:
        for attempt in range(self.settings.retry_max):
            try:
                async for chunk in self.provider.chat(messages, tools=tools):
                    yield chunk
                return
            except Exception as err:
                if not is_transient(err) or attempt + 1 >= self.settings.retry_max:
                    raise
                await asyncio.sleep(self.settings.retry_base * (2**attempt))
