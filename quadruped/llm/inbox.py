"""人说的话：待处理的队列 + 最近几条对话。

`Thinker.say()` 原来只有一个槽（`self._inbox`）：第二句覆盖第一句，处理完就丢。
下一轮提示词里看不到人刚才说过什么 —— 下一条指令够用，对话不行，模型每一轮都像
第一次见到这个人。

两件事分开：

    take()     还没处理的那条，先进先出
    history()  最近几条（人说的 + 它回的），进提示词当上下文

## 为什么排队而不是继续覆盖

一对一的对话里人常连说两句（「往前走」「慢一点」）。覆盖式只剩后半句，前半句的
意图丢了。排到 `PENDING_MAX` 条为止：再多说明这一侧根本没在处理，留着只会让它
执行一串过期指令。

## 为什么历史有上限

提示词里还要放观测、记忆、命令。对话每多一条就挤掉一份观测的注意力 —— 小模型
对长上下文的中间部分最不敏感，`prompt.build_think` 里 `recent` 只喂 3 条是同一个
理由。`KEEP = 6` 是「上下一两轮对话」的量。

## 它的话也要记进来

人问「你看到什么」，模型答「正前两米是空的」，下一句人说「那就往前走」—— 只记
人说的话，模型不知道「那就」指的是什么。所以 `note_reply()` 把它的回话也写进同一
份历史，提示词里用「我 / 它」区分。
"""

from __future__ import annotations

import threading
import time
from collections import deque

# 排队的上限。超过就丢最旧的，并且计数 —— 丢了多少要能看见。
PENDING_MAX = 3

# 历史条数上限。太长会把观测挤出注意力。
KEEP = 6

# 单条最长字数。有人在网页上粘贴一段话，别让它一个人占满历史。
MAX_CHARS = 120


class Inbox:
    """线程安全的对话收件箱。R 层之外的输入都从这里进 L 层。"""

    def __init__(self, *, pending_max: int = PENDING_MAX, keep: int = KEEP):
        self.pending_max = int(pending_max)
        self.keep = int(keep)
        self._lock = threading.Lock()
        self._queue: deque[str] = deque()
        self._dialog: deque[tuple[float, str, str]] = deque(maxlen=self.keep)
        self.dropped = 0          # 因为排队满了被丢掉的条数
        self.total = 0            # 一共收了人多少句话

    # 收一句。空串忽略（网页输入框回车、stdin 空行都会走到这里）。
    def push(self, text: str) -> bool:
        text = " ".join((text or "").split())[:MAX_CHARS]
        if not text:
            return False
        with self._lock:
            if len(self._queue) >= self.pending_max:
                self._queue.popleft()
                self.dropped += 1
            self._queue.append(text)
            self._dialog.append((time.monotonic(), "我", text))
            self.total += 1
        return True

    # 它的回话。只进历史，不进待处理 —— 它说的不该被自己再执行一遍。
    def note_reply(self, text: str) -> bool:
        text = " ".join((text or "").split())[:MAX_CHARS]
        if not text:
            return False
        with self._lock:
            self._dialog.append((time.monotonic(), "它", text))
        return True

    # 取一条待处理的。没有就返回 None。
    def take(self) -> str | None:
        with self._lock:
            return self._queue.popleft() if self._queue else None

    # 最近几条对话，按时间从旧到新。`ago` 是距现在多少秒 —— 提示词里只给相对时间，
    # 模型算不出绝对时刻。
    def history(self) -> list[tuple[float, str, str]]:
        now = time.monotonic()
        with self._lock:
            return [(max(0.0, now - stamp), who, text)
                    for stamp, who, text in self._dialog]

    # 最近一条人说的话。清空之后用它判断「这一轮还有没有人开口」。
    def last_said(self) -> str | None:
        with self._lock:
            for _, who, text in reversed(self._dialog):
                if who == "我":
                    return text
        return None

    @property
    def pending(self) -> int:
        with self._lock:
            return len(self._queue)

    def clear(self) -> None:
        with self._lock:
            self._queue.clear()
            self._dialog.clear()

    def status(self) -> dict:
        with self._lock:
            return {
                "pending": len(self._queue),
                "dialog": len(self._dialog),
                "said": self.total,
                "dropped": self.dropped,
            }
