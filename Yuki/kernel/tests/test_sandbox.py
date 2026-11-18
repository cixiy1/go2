"""工具执行沙箱契约：降权/资源阀门、python/command 均经子进程。"""

import shutil

import pytest

from yuki_kernel.skills import (
    BasicEnvironment,
    BasicSandbox,
    SandboxConfig,
    ToolRegistry,
)


@pytest.mark.skipif(shutil.which("python3") is None, reason="需要 python3")
def test_basic_sandbox_runs_command(tmp_path):
    sandbox = BasicSandbox(SandboxConfig())
    result = sandbox.run(["python3", "-c", "print('ok')"], tmp_path, "", timeout=10)
    assert result.returncode == 0
    assert "ok" in result.stdout


def test_basic_sandbox_rejects_missing_drop_privilege_user():
    with pytest.raises(ValueError, match="沙箱降权用户不存在"):
        BasicSandbox(SandboxConfig(user="__yuki_missing_user__"))


def test_basic_sandbox_whitelist_rejects(tmp_path):
    sandbox = BasicSandbox(SandboxConfig(allowed_binaries=["python3"]))
    result = sandbox.run(["rm", "-rf", "/tmp/nope"], tmp_path, "", timeout=5)
    assert result.returncode == 1
    assert "白名单" in result.stderr


def test_python_tool_runs_out_of_process(tmp_path):
    package_dir = tmp_path / "pkg"
    package_dir.mkdir()
    (package_dir / "tool.py").write_text(
        "def add(a: int, b: int) -> str:\n    return str(a + b)\n",
        encoding="utf-8",
    )
    registry = ToolRegistry(None, environment=BasicEnvironment(BasicSandbox(SandboxConfig())))
    registry.register_tool(
        {
            "name": "add",
            "description": "相加",
            "parameters": {"type": "object", "properties": {}},
            "entry": {"type": "python", "module": "tool.py", "handler": "add"},
            "package": "pkg",
            "package_dir": str(package_dir),
        }
    )
    assert registry.execute("add", {"a": 2, "b": 3}) == "5"


def test_sandbox_applied_to_command_tool(tmp_path):
    package_dir = tmp_path / "pkg"
    package_dir.mkdir()
    registry = ToolRegistry(
        None,
        environment=BasicEnvironment(BasicSandbox(SandboxConfig(allowed_binaries=["python3"]))),
    )
    registry.register_tool(
        {
            "name": "bad_rm",
            "description": "删文件",
            "parameters": {"type": "object", "properties": {}},
            "entry": {"type": "command", "command": ["rm", "-rf", "/tmp/nope"]},
            "package": "pkg",
            "package_dir": str(package_dir),
        }
    )
    # command 不在白名单，沙箱直接拒绝
    assert "白名单" in registry.execute("bad_rm", {})
