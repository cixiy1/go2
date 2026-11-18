"""审批门契约。"""

import json

import pytest

from yuki_kernel.core.agent import Agent
from yuki_kernel.skills import ToolRegistry
from yuki_kernel.skills.types import Tool

# noinspection PyUnresolvedReferences
from yuki_kernel.testing import FakeProvider


def _registry(tmp_path):
    module_dir = tmp_path / "tools"
    module_dir.mkdir(exist_ok=True)
    (module_dir / "danger.py").write_text(
        "def run(path=None):\n    return 'ok'\n",
        encoding="utf-8",
    )
    registry = ToolRegistry(None)
    registry.register_tool(
        Tool(
            name="danger",
            description="危险工具",
            parameters={
                "type": "object",
                "properties": {"path": {"type": "string"}},
            },
            requires_approval=True,
            entry={
                "type": "python",
                "module": "danger.py",
                "handler": "run",
            },
            package="test",
            package_dir=str(module_dir),
        )
    )
    return registry


def _echo_package_registry(tmp_path):
    package_dir = tmp_path / "packages" / "echo"
    package_dir.mkdir(parents=True)
    (package_dir / "manifest.json").write_text(
        json.dumps(
            {
                "id": "echo",
                "name": "回声包",
                "version": "1.0.0",
                "description": "回声",
                "tools": [
                    {
                        "name": "echo_text",
                        "description": "回声工具",
                        "parameters": {"type": "object", "properties": {}},
                        "entry": {"type": "command", "command": ["echo", "hi"]},
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    return ToolRegistry(tmp_path / "packages", available=["echo"])


@pytest.mark.asyncio
async def test_approve_once_not_remembered(settings, tmp_path):
    calls = {"count": 0}

    async def approver(_name, _arguments):
        calls["count"] += 1
        return "y"

    agent = Agent(
        "fake",
        _registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
        approver=approver,
    )
    results = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {}}}]
    )
    assert results[0]["content"] == "ok"
    assert not agent.session.is_approved("danger")
    assert calls["count"] == 1


@pytest.mark.asyncio
async def test_approve_remember_session(settings, tmp_path):
    calls = {"count": 0}

    async def approver(_name, _arguments):
        calls["count"] += 1
        return "ya"

    agent = Agent(
        "fake",
        _registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
        approver=approver,
    )
    first = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {}}}]
    )
    second = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {}}}]
    )
    assert first[0]["content"] == "ok"
    assert second[0]["content"] == "ok"
    assert calls["count"] == 1
    assert agent.session.is_approved("danger")


@pytest.mark.asyncio
async def test_approve_denied(settings, tmp_path):
    async def approver(_name, _arguments):
        return "n"

    agent = Agent(
        "fake",
        _registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
        approver=approver,
    )
    results = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {}}}]
    )
    assert results[0]["content"] == "用户拒绝执行"


@pytest.mark.asyncio
async def test_auto_loaded_command_tool_still_requires_approval(settings, tmp_path):
    async def approver(_name, _arguments):
        return "n"

    agent = Agent(
        "fake",
        _echo_package_registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
        approver=approver,
    )
    results = await agent.execute_tool_calls(
        [{"function": {"name": "echo_text", "arguments": {}}}]
    )
    assert results[0]["content"] == "用户拒绝执行"


@pytest.mark.asyncio
async def test_approver_receives_real_arguments(settings, tmp_path):
    received = {}

    async def approver(name, arguments):
        received["name"] = name
        received["arguments"] = arguments
        return "y"

    agent = Agent(
        "fake",
        _registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
        approver=approver,
    )
    await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {"path": "/etc"}}}]
    )
    assert received == {"name": "danger", "arguments": {"path": "/etc"}}


@pytest.mark.asyncio
async def test_approver_can_be_swapped_after_construction(settings, tmp_path):
    received = []

    async def approver(name, arguments):
        received.append((name, arguments))
        return "y"

    agent = Agent(
        "fake",
        _registry(tmp_path),
        settings,
        provider=FakeProvider(settings=settings),
    )
    denied = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {"path": "/etc"}}}]
    )
    assert denied[0]["content"] == "用户拒绝执行"

    agent.approver = approver
    approved = await agent.execute_tool_calls(
        [{"function": {"name": "danger", "arguments": {"path": "/etc"}}}]
    )
    assert approved[0]["content"] == "ok"
    assert received == [("danger", {"path": "/etc"})]
