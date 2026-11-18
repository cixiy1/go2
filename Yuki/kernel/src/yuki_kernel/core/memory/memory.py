"""长期记忆：SQLite 关键词检索。"""

import re
import sqlite3
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path


@dataclass
class MemoryHit:
    session_id: str
    content: str
    created_at: str


def _terms(text: str) -> list[str]:
    """切出检索词：英文按词，中文按双字窗口。"""
    words = re.findall(r"[A-Za-z0-9_]+|[\u4e00-\u9fff]+", text)
    terms = [word for word in words if word.strip()]
    cjk = "".join(re.findall(r"[\u4e00-\u9fff]+", text))
    if len(cjk) >= 2:
        terms.extend(cjk[i : i + 2] for i in range(len(cjk) - 1))
    return list(dict.fromkeys(terms))


class MemoryStore:
    def __init__(self, data_dir: Path, namespace: str = "default"):
        self.data_dir = data_dir
        self.namespace = namespace
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.db_path = data_dir / "memory.db"
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS memories (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    session_id TEXT NOT NULL,
                    content TEXT NOT NULL,
                    created_at TEXT NOT NULL
                )
                """
            )
            columns = [row[1] for row in conn.execute("PRAGMA table_info(memories)")]
            if "namespace" not in columns:
                conn.execute(
                    "ALTER TABLE memories ADD COLUMN namespace TEXT NOT NULL DEFAULT 'default'"
                )

    def add(self, session_id: str, content: str) -> None:
        if not content.strip():
            return
        created_at = datetime.now(UTC).isoformat(timespec="seconds")
        with sqlite3.connect(self.db_path) as conn:
            conn.execute(
                """
                INSERT INTO memories (session_id, content, created_at, namespace)
                VALUES (?, ?, ?, ?)
                """,
                (session_id, content, created_at, self.namespace),
            )

    def search(self, query: str, limit: int = 5) -> list[MemoryHit]:
        terms = _terms(query)
        if not terms:
            return []
        placeholders = " OR ".join(["content LIKE ?"] * len(terms))
        params = [f"%{term}%" for term in terms]
        with sqlite3.connect(self.db_path) as conn:
            rows = conn.execute(
                f"""
                SELECT session_id, content, created_at
                FROM memories
                WHERE namespace = ? AND ({placeholders})
                ORDER BY id DESC
                LIMIT ?
                """,
                [self.namespace, *params, limit],
            ).fetchall()
        return [MemoryHit(*row) for row in rows]

    def search_text(self, query: str, limit: int = 5) -> str:
        hits = self.search(query, limit)
        if not hits:
            return "没有找到相关记忆"
        return "\n".join(f"- {hit.content}" for hit in hits)
