"""工具相关类型定义。"""

from typing import Any, TypedDict


class ToolEntry(TypedDict, total=False):
    type: str
    module: str
    handler: str
    method: str
    command: list[str]


class Tool(TypedDict, total=False):
    name: str
    description: str
    parameters: dict[str, Any]
    entry: ToolEntry
    package: str
    package_dir: str
    requires_approval: bool
