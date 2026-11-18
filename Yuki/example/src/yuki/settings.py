"""Yuki 示例：从环境变量构建内核 Settings。"""

import os
from pathlib import Path

# isort: off
# noinspection PyUnresolvedReferences
from yuki_kernel.config import Settings
# noinspection PyUnresolvedReferences
from yuki_kernel.skills import BasicEnvironment
# isort: on

EXAMPLE_ROOT = Path(__file__).resolve().parent.parent.parent


def _bool_env(name: str, default: bool) -> bool:
    return os.getenv(name, str(default)).lower() in {"1", "true", "yes"}


def _int_env(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        return default


def _float_env(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def _csv_env(name: str) -> list[str]:
    value = os.getenv(name, "")
    return [part.strip() for part in value.split(",") if part.strip()]


def _resolve_path(value: str, default: Path) -> Path:
    if not value:
        return default
    path = Path(value)
    return path if path.is_absolute() else EXAMPLE_ROOT / path


def _build_environment() -> BasicEnvironment | None:
    """宿主侧注入执行环境：有虚机环境时让工具真正跑在 venv 里。

    未设置 VIRTUAL_ENV 则返回 None，由内核回退到默认 BasicEnvironment。
    强隔离（容器/Seatbelt/Landlock）不在此处处理，留给宿主把内核整体放进隔离运行。
    """
    venv = os.getenv("VIRTUAL_ENV")
    if not venv:
        return None
    bin_dir = Path(venv) / ("Scripts" if os.name == "nt" else "bin")
    python_path = str(bin_dir / ("python.exe" if os.name == "nt" else "python"))
    return BasicEnvironment(
        python_path=python_path,
        base_env={
            "VIRTUAL_ENV": venv,
            "PATH": f"{bin_dir}{os.pathsep}{os.environ.get('PATH', '')}",
        },
    )


def _load_dotenv() -> None:
    try:
        # noinspection PyUnresolvedReferences
        from dotenv import load_dotenv
    except ImportError:
        return
    load_dotenv(EXAMPLE_ROOT / ".env", override=True)


def load_settings() -> Settings:
    _load_dotenv()
    return Settings(
        provider=os.getenv("AGENT_PROVIDER", "openai"),
        model=os.getenv("AGENT_MODEL", ""),
        think=_bool_env("AGENT_THINK", True),
        openai_base_url=os.getenv("OPENAI_BASE_URL") or None,
        openai_api_key=os.getenv("OPENAI_API_KEY") or None,
        anthropic_base_url=os.getenv("ANTHROPIC_BASE_URL") or None,
        anthropic_api_key=os.getenv("ANTHROPIC_API_KEY") or None,
        packages_dir=_resolve_path(os.getenv("PACKAGES_DIR", ""), EXAMPLE_ROOT / "packages"),
        packages=_csv_env("AGENT_PACKAGES"),
        packages_preload=_csv_env("AGENT_PACKAGES_PRELOAD"),
        max_context_tokens=_int_env("AGENT_MAX_CONTEXT_TOKENS", 12000),
        keep_recent_messages=_int_env("AGENT_KEEP_RECENT_MESSAGES", 10),
        memory_limit=_int_env("AGENT_MEMORY_LIMIT", 5),
        namespace=os.getenv("AGENT_NAMESPACE", "default"),
        retry_max=_int_env("AGENT_RETRY_MAX", 3),
        retry_base=_float_env("AGENT_RETRY_BASE", 0.5),
        data_dir=_resolve_path(os.getenv("DATA_DIR", ""), EXAMPLE_ROOT / "data"),
        environment=_build_environment(),
    )
