"""会话对象。"""

import time
import uuid
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


class Session:
    def __init__(
        self,
        session_id: str | None = None,
        name: str = "",
        messages: list[dict[str, Any]] | None = None,
        created_at: str | None = None,
    ):
        self.session_id = session_id or uuid.uuid4().hex
        self.name = name
        self.messages = messages or []
        self.approved_tools: dict[str, float] = {}
        self.created_at = created_at or _now()
        self.updated_at = self.created_at

    def approve(self, tool_name: str, minutes: int | None = None) -> None:
        if minutes is None:
            self.approved_tools[tool_name] = float("inf")
        else:
            self.approved_tools[tool_name] = time.time() + minutes * 60

    def is_approved(self, tool_name: str) -> bool:
        expiry = self.approved_tools.get(tool_name)
        if expiry is None:
            return False
        return expiry == float("inf") or expiry > time.time()


@dataclass
class SessionMeta:
    session_id: str
    name: str
    created_at: str
    updated_at: str
