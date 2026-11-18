"""本地包管理契约。"""

import asyncio
import zipfile

import pytest

from yuki_kernel.skills import ToolRegistry
from yuki_kernel.skills.external import PackageError
from yuki_kernel.skills.package_manager import PackageManager
from yuki_kernel.skills.sources import LocalDirSource, ZipSource


def test_system_prompt_tells_model_to_discover_packages(weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])

    prompt = registry.system_prompt()

    assert "list_packages" in prompt
    assert "load_package" in prompt
    assert "不要重复调用 list_packages" in prompt


def test_execute_auto_loads_available_package(weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])

    assert registry.active_packages == []
    result = registry.execute("weather_now", {"city": "New York"})
    assert result == "22°C"
    assert registry.active_packages == ["weather"]


def test_execute_unknown_tool_not_auto_loaded(weather_package):
    registry = ToolRegistry(weather_package, available=["weather"])

    assert registry.execute("not_a_tool", {}) == "Unknown tool: not_a_tool"
    assert registry.active_packages == []




@pytest.mark.asyncio
async def test_install_remove_list(tmp_path, weather_package):
    packages_dir = tmp_path / "installed"
    manager = PackageManager(packages_dir)

    info = await manager.install(LocalDirSource(), str(weather_package / "weather"))
    assert info.id == "weather"
    assert (packages_dir / "weather").is_dir()
    assert [item.id for item in manager.list_installed()] == ["weather"]

    manager.remove("weather")
    assert not (packages_dir / "weather").exists()
    assert manager.list_installed() == []


@pytest.mark.asyncio
async def test_install_from_zip(tmp_path, weather_package):
    zip_path = tmp_path / "weather.zip"
    with zipfile.ZipFile(zip_path, "w") as archive:
        for file in (weather_package / "weather").rglob("*"):
            if file.is_file():
                archive.write(file, file.relative_to(weather_package))

    manager = PackageManager(tmp_path / "installed")
    info = await manager.install(ZipSource(), str(zip_path))
    assert info.id == "weather"


def test_zip_slip_rejected(tmp_path):
    zip_path = tmp_path / "bad.zip"
    with zipfile.ZipFile(zip_path, "w") as archive:
        archive.writestr("../evil.txt", "bad")

    manager = PackageManager(tmp_path / "installed")
    with pytest.raises(PackageError):
        asyncio.run(manager.install(ZipSource(), str(zip_path)))
