"""内置工具契约。"""

import json

from yuki_kernel.skills import ToolRegistry


def test_environment_info_available():
    registry = ToolRegistry(None)
    names = [tool["function"]["name"] for tool in registry.tools]
    prompt_names = [prompt["name"] for prompt in registry.prompts]

    assert "get_environment_info" in names
    assert "search_memory" in names
    assert "environment_guide" in prompt_names
    assert "yuki_identity" not in prompt_names
    assert "get_name_guide" not in prompt_names
    guide = next(
        prompt["content"]
        for prompt in registry.prompts
        if prompt["name"] == "environment_guide"
    )
    assert "不要重复调用" in guide

    content = registry.execute("get_environment_info", {})
    assert "操作系统" in content
    assert "Python" in content


def test_sleep_tools_in_registry():
    registry = ToolRegistry(None)

    # 确认两个工具已注册
    names = [tool["function"]["name"] for tool in registry.tools]
    assert "sleep" in names
    assert "run_sleep" in names

    # sleep（output）错误参数
    resp_err = registry.execute("sleep", {"time": 0})
    assert "错误：请输入等待时间" in resp_err

    # sleep（output）正常调用——只输出等待提示，不实际睡眠
    resp_output = registry.execute("sleep", {"time": 3})
    assert "等待：3s" in resp_output

    # run_sleep 错误参数
    resp_err = registry.execute("run_sleep", {"time": 0})
    assert "错误：请输入等待时间" in resp_err

    # run_sleep 正常调用——实际睡眠
    resp_ok = registry.execute("run_sleep", {"time": 3})
    assert "等待完成：3s" in resp_ok


def test_scan_packages_returns_data_without_printing(weather_package, capsys):
    registry = ToolRegistry(None)

    scan = registry.scan_packages(weather_package)

    assert "weather" in scan.packages
    assert scan.available == scan.packages
    assert scan.skipped == []
    assert registry.package_scan.packages == scan.packages
    assert capsys.readouterr().out == ""


def test_scan_packages_missing_dir(tmp_path, capsys):
    registry = ToolRegistry(None)

    scan = registry.scan_packages(tmp_path / "nope")

    assert scan.packages == {}
    assert scan.available == {}
    assert scan.skipped == [(str(tmp_path / "nope"), "目录不存在")]
    assert registry.package_scan.skipped == scan.skipped
    assert capsys.readouterr().out == ""


def test_scan_distinguishes_discovered_and_available(tmp_path, capsys):
    registry = ToolRegistry(None)
    for package_id, handler in (
        ("weather", "weather_now"),
        ("echo", "echo_text"),
    ):
        package_dir = tmp_path / package_id
        package_dir.mkdir()
        (package_dir / "manifest.json").write_text(
            json.dumps(
                {
                    "id": package_id,
                    "name": package_id,
                    "version": "1.0.0",
                    "tools": [
                        {
                            "name": handler,
                            "description": handler,
                            "parameters": {"type": "object", "properties": {}},
                            "entry": {
                                "type": "python",
                                "module": "tool.py",
                                "handler": handler,
                            },
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        (package_dir / "tool.py").write_text(
            f"def {handler}():\n    return 'ok'\n",
            encoding="utf-8",
        )

    scan = registry.scan_packages(tmp_path, available=["weather"])

    assert set(scan.packages) == {"weather", "echo"}
    assert set(scan.available) == {"weather"}
    assert [package["id"] for package in registry.available_packages] == ["weather"]
    assert capsys.readouterr().out == ""


def test_preload_updates_available_packages_without_printing(weather_package, capsys):
    registry = ToolRegistry(weather_package, preload=["weather"])

    loaded = [package for package in registry.available_packages if package["loaded"]]
    assert [package["id"] for package in loaded] == ["weather"]
    assert capsys.readouterr().out == ""
