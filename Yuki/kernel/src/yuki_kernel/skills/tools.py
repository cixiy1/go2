"""解析模型输出的工具调用。"""

import json
from typing import Any


def parse_tool_call(call: Any) -> tuple[str, Any]:
    if isinstance(call, dict):
        function = call.get("function") or {}
        name = function.get("name") or call.get("name", "")
        arguments = function.get("arguments") or {}
    else:
        function = getattr(call, "function", None)
        name = getattr(function, "name", "")
        arguments = getattr(function, "arguments", {})
    return name, arguments


def merge_tool_calls(tool_calls: list[Any]) -> list[dict[str, Any]]:
    """把 OpenAI 兼容的流式 tool_calls 分片按 index 合并成完整调用。"""
    merged: dict[int, dict[str, Any]] = {}

    for call in tool_calls:
        if isinstance(call, dict) and "index" in call:
            function = call.get("function") or {}
            entry = merged.setdefault(call["index"], {"name": "", "arguments": ""})
            if call.get("id"):
                entry["id"] = call["id"]
            if function.get("name"):
                entry["name"] += function["name"]
            args = function.get("arguments")
            if args:
                entry["arguments"] += args
        else:
            name, arguments = parse_tool_call(call)
            merged[len(merged)] = {"name": name, "arguments": arguments}

    calls = []
    for entry in merged.values():
        arguments = entry["arguments"]
        if isinstance(arguments, str):
            try:
                arguments = json.loads(arguments)
            except json.JSONDecodeError:
                arguments = {}
        call = {"name": entry["name"], "arguments": arguments}
        if entry.get("id"):
            call["id"] = entry["id"]
        calls.append(call)
    return calls
