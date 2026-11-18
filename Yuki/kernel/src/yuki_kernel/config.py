"""运行参数类型：内核不读取环境，只接收外部软件传入的 Settings。"""

from dataclasses import dataclass, field
from pathlib import Path

from .skills.environment import Environment


@dataclass
class Settings:
    provider: str
    model: str
    think: bool = True
    openai_base_url: str | None = None
    openai_api_key: str | None = None
    anthropic_base_url: str | None = None
    anthropic_api_key: str | None = None
    packages_dir: Path | None = None
    packages: list[str] = field(default_factory=list)
    packages_preload: list[str] = field(default_factory=list)
    max_context_tokens: int = 12000
    keep_recent_messages: int = 10
    memory_limit: int = 5
    namespace: str = "default"
    retry_max: int = 3
    retry_base: float = 0.5
    data_dir: Path | None = None
    environment: Environment | None = None
