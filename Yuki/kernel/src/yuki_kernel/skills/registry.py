"""工具注册表：统一内置工具与外置工具包的 schema 与执行。"""

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, cast

from .builtin import BUILTIN_PROMPTS, BUILTIN_TOOLS
from .environment import BasicEnvironment, Environment
from .executor import ToolExecutor, require_entry
from .external import discover_packages, load_package
from .meta import META_NAMES, META_TOOLS
from .types import Tool

PathLike = str | Path


@dataclass
class PackageScan:
    """包扫描结果，供客户端决定如何展示。"""

    packages: dict[str, dict[str, Any]] = field(default_factory=dict)
    available: dict[str, dict[str, Any]] = field(default_factory=dict)
    skipped: list[tuple[str, str]] = field(default_factory=list)


def to_function_schema(tool: Tool) -> dict[str, Any]:
    """把内部工具定义转成 OpenAI 兼容的函数 schema。"""
    return {
        "type": "function",
        "function": {
            "name": tool["name"],
            "description": tool["description"],
            "parameters": tool["parameters"],
        },
    }


class ToolRegistry:
    """内置工具 + 外置工具包的注册表。"""

    def __init__(
        self,
        packages_dir: PathLike | None = None,
        available: list[str] | None = None,
        preload: list[str] | None = None,
        memory_searcher: Callable[[str], str] | None = None,
        environment: Environment | None = None,
    ):
        self._tools: dict[str, Tool] = {}
        self._prompts: dict[str, dict[str, Any]] = {}
        self._packages: dict[str, dict[str, Any]] = {}
        self._active_packages: set[str] = set()
        self._executor = ToolExecutor(environment or BasicEnvironment())
        self.memory_searcher = memory_searcher
        self.package_scan = PackageScan()
        self.load_builtin()
        if packages_dir is not None:
            self.scan_packages(Path(packages_dir), available=available)
        for package_id in preload or []:
            self.activate_package(package_id)

    @property
    def tools(self) -> list[dict[str, Any]]:
        schemas = [to_function_schema(tool) for tool in self._tools.values()]
        schemas += [to_function_schema(tool) for tool in META_TOOLS]
        return schemas

    @property
    def prompts(self) -> list[dict[str, Any]]:
        return list(self._prompts.values())

    @property
    def available_packages(self) -> list[dict[str, Any]]:
        return [
            {
                "id": package_id,
                "name": package["name"],
                "description": package["description"],
                "tools": [tool["name"] for tool in package["tools"]],
                "prompts": [prompt["name"] for prompt in package["prompts"]],
                "loaded": package_id in self._active_packages,
            }
            for package_id, package in self._packages.items()
        ]

    @property
    def active_packages(self) -> list[str]:
        return sorted(self._active_packages)

    def load_builtin(self) -> None:
        skills_dir = Path(__file__).resolve().parent
        for tool in BUILTIN_TOOLS:
            self.register_tool(
                Tool(
                    name=tool["name"],
                    description=tool["description"],
                    parameters=tool["parameters"],
                    entry=tool["entry"],
                    package="builtin",
                    package_dir=str(skills_dir),
                )
            )
        for prompt in BUILTIN_PROMPTS:
            content = (skills_dir / prompt["path"]).read_text(encoding="utf-8")
            self.register_prompt({**prompt, "content": content})

    def scan_packages(
        self,
        packages_dir: Path,
        available: list[str] | None = None,
    ) -> PackageScan:
        if not packages_dir.is_dir():
            self.package_scan = PackageScan(skipped=[(str(packages_dir), "目录不存在")])
            return self.package_scan

        discovered: dict[str, dict[str, Any]] = {}
        skipped: list[tuple[str, str]] = []
        for package_dir in discover_packages(packages_dir):
            try:
                package = load_package(package_dir)
                discovered[package["id"]] = package
            except Exception as err:  # noqa: BLE001  包加载失败应跳过而非中断整个注册
                skipped.append((package_dir.name, str(err)))

        if available is not None:
            allowed = set(available)
            usable = {
                package_id: package
                for package_id, package in discovered.items()
                if package_id in allowed
            }
        else:
            usable = dict(discovered)
        self._packages = usable
        self.package_scan = PackageScan(
            packages=discovered,
            available=usable,
            skipped=skipped,
        )
        return self.package_scan

    def activate_package(self, package_id: str) -> str:
        if package_id in self._active_packages:
            return f"{package_id} 已加载"
        package = self._packages.get(package_id)
        if package is None:
            return f"未找到可用工具包：{package_id}"

        tools = cast(list[Tool], package["tools"])
        for tool in tools:
            if tool["name"] in self._tools or tool["name"] in META_NAMES:
                return f"加载 {package_id} 失败：工具名冲突 {tool['name']}"
        for prompt in package["prompts"]:
            if prompt["name"] in self._prompts:
                return f"加载 {package_id} 失败：提示词名冲突 {prompt['name']}"

        for tool in tools:
            self.register_tool(tool)
        for prompt in package["prompts"]:
            self.register_prompt(prompt)
        self._active_packages.add(package_id)

        tool_names = "、".join(tool["name"] for tool in package["tools"]) or "无"
        prompt_names = "、".join(prompt["name"] for prompt in package["prompts"]) or "无"
        return f"已加载 {package_id}：工具 {tool_names}；提示词 {prompt_names}"

    def deactivate_package(self, package_id: str) -> str:
        if package_id not in self._active_packages:
            return f"{package_id} 未加载"
        package = self._packages[package_id]
        for tool in package["tools"]:
            self._tools.pop(tool["name"], None)
        for prompt in package["prompts"]:
            self._prompts.pop(prompt["name"], None)
        self._active_packages.remove(package_id)
        return f"已卸载 {package_id}"

    def restore_packages(self, target: list[str]) -> list[str]:
        target_set = set(target)
        changed = []
        for package_id in list(self._active_packages):
            if package_id not in target_set:
                self.deactivate_package(package_id)
                changed.append(package_id)
        for package_id in target:
            if package_id not in self._active_packages:
                self.activate_package(package_id)
                changed.append(package_id)
        return changed

    def register_tool(self, tool: Tool) -> None:
        name = tool["name"]
        if name in META_NAMES:
            raise ValueError(f"工具名 {name} 是保留名")
        if name in self._tools:
            raise ValueError(f"工具名冲突：{name}")
        self._tools[name] = tool

    def register_prompt(self, prompt: dict[str, Any]) -> None:
        name = prompt["name"]
        if name in self._prompts:
            raise ValueError(f"提示词名冲突：{name}")
        self._prompts[name] = prompt

    def needs_approval(self, name: str) -> bool:
        if not self._load_tool_package(name):
            return False
        tool = self._tools[name]
        entry = tool.get("entry") or {}
        return entry.get("type") == "command" or bool(tool.get("requires_approval"))

    def system_prompt(self) -> str:
        parts = []
        for prompt in self._prompts.values():
            parts.append(f"[{prompt['name']}]\n{prompt['content'].strip()}")
        if self._packages:
            parts.append(
                "外置工具包按需加载：当用户需要的能力在当前工具列表里没有时，"
                "先调用 list_packages 查看有哪些外置包可调用；找到对应包后调用 load_package 加载你想使用的工具"
                "不要重复调用 list_packages "
            )
        return "\n\n".join(parts)

    def execute(self, name: str, arguments: dict[str, Any]) -> str:
        if name in META_NAMES:
            return self._execute_meta(name, arguments)

        if not self._load_tool_package(name):
            return f"Unknown tool: {name}"
        tool = self._tools[name]
        try:
            entry = require_entry(tool)
        except ValueError as err:
            return str(err)
        entry_type = entry.get("type")
        if entry_type == "python":
            return self._executor.execute_python(tool, arguments)
        if entry_type == "command":
            return self._executor.execute_command(tool, arguments)
        return f"Unknown entry type: {entry_type if entry_type is not None else 'None'}"

    def _load_tool_package(self, name: str) -> bool:
        if name in self._tools:
            return True
        for package_id, package in self._packages.items():
            if any(tool["name"] == name for tool in package["tools"]):
                self.activate_package(package_id)
                return name in self._tools
        return False

    def _execute_meta(self, name: str, arguments: dict[str, Any]) -> str:
        if name == "list_packages":
            return self._list_packages_text()
        if name == "load_package":
            return self.activate_package(str(arguments.get("package_id", "")))
        if name == "unload_package":
            return self.deactivate_package(str(arguments.get("package_id", "")))
        if name == "search_memory":
            if self.memory_searcher is None:
                return "长期记忆未启用"
            return self.memory_searcher(str(arguments.get("query", "")))
        return f"Unknown meta tool: {name}"

    def _list_packages_text(self) -> str:
        lines = []
        for package in self.available_packages:
            state = "已加载" if package["loaded"] else "未加载"
            tools = "、".join(package["tools"]) or "无"
            prompts = "、".join(package["prompts"]) or "无"
            lines.append(
                f"{package['id']}：{package['description']}（工具：{tools}；提示词：{prompts}；状态：{state}）"
            )
        return "\n".join(lines) or "暂无可用工具包"
