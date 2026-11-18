"""内核事件类型。"""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class AgentEvent:
    kind: str
    payload: Any = None
    context: dict[str, Any] = field(default_factory=dict)
