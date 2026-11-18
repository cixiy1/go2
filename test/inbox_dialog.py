"""输入层的三条线：收件箱、提示词里的对话段、模型回话的解析。

    .venv/bin/python test/inbox_dialog.py

这三件事很容易各做一半：收件箱排队了但提示词不带历史（模型每轮都像第一次见人）、
提示词带了历史但回话解析不出来（页面上永远是上次那句）、回话解析出来了却被当成
命令解析失败（`last_error` 一直红着，状态栏看起来像坏了）。

所以这里逐条钉死：

    1. 排队：先说先执行；队列满丢最旧，丢了几条要能数
    2. 历史：人说的和它回的在同一份里，标了「我 / 它」，按时间从旧到新
    3. 提示词：这一轮的 user 不在「对话」段里重复；没人说话时没有这一段
    4. 解析：reply 写在顶层、写在 command 里、写成「无」三种情况
    5. 只答话的轮次不能算错 —— 空命令是正常结果，不是解析失败
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from quadruped.llm.inbox import KEEP, MAX_CHARS, PENDING_MAX, Inbox   # noqa: E402
from quadruped.llm.prompt import build_think                         # noqa: E402
from quadruped.llm.think import parse_think                          # noqa: E402

ok = 0
bad: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    global ok
    if cond:
        ok += 1
        print(f"  ok   {name}")
    else:
        bad.append(name)
        print(f"  FAIL {name}  {detail}")


def test_inbox() -> None:
    print("\n收件箱")
    box = Inbox()
    box.push("往前走")
    box.push("慢一点")
    check("两句都在排队", box.pending == 2, f"pending={box.pending}")
    check("先说先执行", box.take() == "往前走")
    check("第二句还在", box.take() == "慢一点")
    check("取完是空的", box.take() is None)

    box.push(" ")
    check("空话不入队", box.pending == 0 and box.total == 2,
          f"pending={box.pending} total={box.total}")

    for i in range(PENDING_MAX + 2):
        box.push(f"第{i}句")
    check(f"排队不超过 {PENDING_MAX} 条", box.pending == PENDING_MAX,
          f"pending={box.pending}")
    check("丢掉的最旧那几条有计数", box.dropped == 2, f"dropped={box.dropped}")

    long = "走" * (MAX_CHARS + 50)
    box.clear()
    box.push(long)
    check("超长截断", len(box.history()[0][2]) == MAX_CHARS,
          f"len={len(box.history()[0][2])}")

    box.clear()
    box.push("你看到什么")
    before = box.pending
    box.note_reply("正前两米是空地")
    hist = box.history()
    check("人说的话和它的话都在历史里", len(hist) == 2, f"len={len(hist)}")
    check("按旧到新排", hist[0][1] == "我" and hist[1][1] == "它",
          str([(h[1], h[2]) for h in hist]))
    check("第二条的 ago 不比第一条大", hist[1][0] <= hist[0][0] + 1e-6)
    check("回话不额外占待处理", box.pending == before,
          f"{before} -> {box.pending}")

    box.clear()
    for i in range(KEEP + 3):
        box.push(f"话{i}")
    check(f"历史只留 {KEEP} 条", len(box.history()) == KEEP,
          f"len={len(box.history())}")


def test_prompt() -> None:
    print("\n提示词")
    dialog = [(9.0, "我", "往前走"), (6.0, "它", "好，慢慢走"),
              (1.0, "我", "你看到什么")]
    text = build_think("机身：vx 0.30", user="你看到什么",
                       dialog=dialog)[1]["content"]
    check("有对话段", "对话（从旧到新" in text, text[:200])
    check("这一轮的话不重复出现",
          text.count("你看到什么") == 1, f"count={text.count('你看到什么')}")
    check("历史里那两句在", "往前走" in text and "好，慢慢走" in text)
    check("有人说话就提示用 reply", "用 reply 回他" in text)

    # 没有 user：对话段照旧给（上一轮说过什么仍然有用），但不要求回话。
    text = build_think("机身：vx 0.30", dialog=dialog)[1]["content"]
    check("没人说话时不提 reply 要求", "用 reply 回他" not in text, text[:200])

    text = build_think("机身：vx 0.30")[1]["content"]
    check("没有对话就没有那一段", "对话（从旧到新" not in text)

    system = build_think("机身：vx 0.30", user="你看到什么")[0]["content"]
    check("系统提示词列了 reply 字段", '"reply"' in system or "reply " in system)
    check("系统提示词给了对话示例", "例（用户在说话" in system)


def test_parse() -> None:
    print("\n解析回话")

    thought, reply, fields, notes = parse_think(
        '{"thought": "正前空", "reply": "正前两米是空地", "command": {"vx": 0.3}}',
        "你看到什么")
    check("顶层 reply", reply == "正前两米是空地", reply)
    check("命令照旧解析", fields.get("vx") == 0.3, str(fields))
    check("thought 照旧", thought == "正前空", thought)
    check("没有多余备注", not notes, str(notes))

    _, reply, fields, _ = parse_think(
        '{"reply": "在想", "command": {"reply": "塞里面了", "vx": 0.2}}')
    check("塞进 command 的 reply 也接住", reply == "在想", reply)
    check("command 里的 reply 不进命令", "reply" not in fields, str(fields))

    _, reply, _, _ = parse_think('{"reply": "塞里面了", "command": {"reply": "x"}}')
    check("顶层优先", reply == "塞里面了", reply)

    for word in ("无", "null", "None", "-", " "):
        _, reply, _, _ = parse_think('{"reply": "%s", "command": {"vx": 0.1}}' % word)
        check(f"「{word}」当成没说", reply == "", repr(reply))

    _, reply, fields, notes = parse_think('{"command": {}}')
    check("空命令、没回话", reply == "" and not fields)
    # 空 command 在 `parse_think` 里直接跳过校验，所以不带「一个可用字段都没有」
    # 这条备注 —— 它是聊天轮的正常形状，不该被记成解析失败。
    check("空命令不报解析失败", not notes, str(notes))

    _, reply, _, _ = parse_think('{"reply": "%s"}' % ("说" * 100))
    check("回话截到 60 字", len(reply) == 60, f"len={len(reply)}")

    _, reply, _, _ = parse_think("完全没有 JSON")
    check("没有 JSON 时 reply 是空", reply == "", repr(reply))


def main() -> int:
    test_inbox()
    test_prompt()
    test_parse()
    print(f"\n{ok} 条通过" + (f"，{len(bad)} 条失败：{'、'.join(bad)}" if bad else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
