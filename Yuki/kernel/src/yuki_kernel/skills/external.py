"""外置工具包发现与加载。

外置包是一个包含 manifest.json 的目录，manifest 声明工具和提示词。
"""

import json
from pathlib import Path
from typing import Any

MANIFEST_NAME = "manifest.json"


class PackageError(ValueError):
    """manifest 或包内容不合法时抛出。"""


def discover_packages(packages_dir: Path) -> list[Path]:
    """返回 packages_dir 下所有包含 manifest.json 的目录。"""
    if not packages_dir.is_dir():
        return []
    return sorted(
        path
        for path in packages_dir.iterdir()
        if path.is_dir() and (path / MANIFEST_NAME).is_file()
    )


def load_package(package_dir: Path) -> dict[str, Any]:
    """读取并校验一个外置工具包，返回归一化后的包定义。"""
    manifest_path = package_dir / MANIFEST_NAME
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise PackageError("缺少 manifest.json") from None
    except json.JSONDecodeError as err:
        raise PackageError(f"manifest.json 不是合法 JSON：{err}") from None

    package_id = str(manifest.get("id") or package_dir.name)
    return {
        "id": package_id,
        "name": manifest.get("name", package_id),
        "version": manifest.get("version", ""),
        "description": manifest.get("description", ""),
        "tools": [
            _validate_tool(package_dir, package_id, item)
            for item in manifest.get("tools", [])
        ],
        "prompts": [
            _validate_prompt(package_dir, item)
            for item in manifest.get("prompts", [])
        ],
    }


def _validate_tool(package_dir: Path, package_id: str, tool: dict[str, Any]) -> dict[str, Any]:
    required = ["name", "description", "parameters", "entry"]
    missing = [key for key in required if key not in tool]
    if missing:
        raise PackageError(f"工具缺少字段：{', '.join(missing)}")

    entry = tool["entry"]
    if entry.get("type") == "python":
        if not entry.get("module") or not entry.get("handler"):
            raise PackageError("python 入口需要 module 和 handler")
    elif entry.get("type") == "command":
        command = entry.get("command")
        if not isinstance(command, list) or not command or not all(
            isinstance(part, str) for part in command
        ):
            raise PackageError("command 入口需要非空字符串列表")
    else:
        raise PackageError(f"不支持的 entry.type：{entry.get('type')}")

    return {**tool, "package": package_id, "package_dir": str(package_dir)}


def _validate_prompt(package_dir: Path, prompt: dict[str, Any]) -> dict[str, Any]:
    required = ["name", "description", "path"]
    missing = [key for key in required if key not in prompt]
    if missing:
        raise PackageError(f"提示词缺少字段：{', '.join(missing)}")

    prompt_path = package_dir / prompt["path"]
    if not prompt_path.is_file():
        raise PackageError(f"提示词文件不存在：{prompt['path']}")

    return {
        "name": prompt["name"],
        "description": prompt["description"],
        "content": prompt_path.read_text(encoding="utf-8"),
    }
