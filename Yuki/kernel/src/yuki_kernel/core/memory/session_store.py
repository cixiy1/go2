"""会话持久化：JSONL 消息 + SQLite 索引。"""

import json
import sqlite3
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from .session import Session, SessionMeta


def _now() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


class SessionStore:
    """会话保存/加载/列表。"""

    def __init__(self, data_dir: Path):
        self.data_dir = data_dir
        self.sessions_dir = data_dir / "sessions"
        self.sessions_dir.mkdir(parents=True, exist_ok=True)
        self.db_path = data_dir / "sessions.db"
        self._init_db()

    def _init_db(self) -> None:
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS sessions (
                    session_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )

    @staticmethod
    def create(name: str = "") -> Session:
        return Session(name=name)

    def save(self, session: Session) -> None:
        path = self.sessions_dir / f"{session.session_id}.jsonl"
        lines = [json.dumps(msg, ensure_ascii=False) for msg in session.messages]
        path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        session.updated_at = _now()
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """
                INSERT INTO sessions (session_id, name, created_at, updated_at)
                VALUES (?, ?, ?, ?)
                ON CONFLICT(session_id) DO UPDATE SET
                    name = excluded.name,
                    updated_at = excluded.updated_at
                """,
                (session.session_id, session.name, session.created_at, session.updated_at),
            )

    @staticmethod
    def export(session: Session, path: Path) -> None:
        """把会话消息导出成 JSONL 文件。"""
        lines = [json.dumps(msg, ensure_ascii=False) for msg in session.messages]
        path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    @staticmethod
    def import_file(path: Path, name: str = "") -> Session:
        """从 JSONL 文件导入会话消息。"""
        messages: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                messages.append(json.loads(line))
        return Session(name=name, messages=messages)

    def load(self, session_id: str) -> Session | None:
        path = self.sessions_dir / f"{session_id}.jsonl"
        if not path.exists():
            return None
        messages: list[dict[str, Any]] = []
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                messages.append(json.loads(line))
        with sqlite3.connect(self.db_path) as conn:
            row = conn.execute(
                "SELECT name, created_at, updated_at FROM sessions WHERE session_id = ?",
                (session_id,),
            ).fetchone()
        if row is None:
            return None
        session = Session(session_id=session_id, name=row[0], messages=messages, created_at=row[1])
        session.updated_at = row[2]
        return session

    def list_sessions(self) -> list[SessionMeta]:
        with sqlite3.connect(self.db_path) as conn:
            rows = conn.execute(
                """
                SELECT session_id, name, created_at, updated_at
                FROM sessions
                ORDER BY updated_at DESC
                """
            ).fetchall()
        return [SessionMeta(*row) for row in rows]
