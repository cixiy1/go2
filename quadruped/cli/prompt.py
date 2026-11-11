"""命令行交互的薄工具层：对齐、单选、多选编号、一屏表单。

原本这些散在 `launcher.py` 里，和「扫目录、校验协议、起子进程」混在一起。
收出来之后有三件事只写一遍：

- **CJK 宽度对齐**。全角字符占两格，不处理就对不齐中文标签。
- **EOF**。管道、重定向、CI 里都收不到回车，`input()` 抛 `EOFError`。这里
  统一当成「取默认」，不让上层每个 `input()` 都包一次 try。
- **编号解析**。单选、多选、表单编辑都是「读一行 → 解析成编号 → 越界提示」，
  三处各写一遍必然长得不一样。

只管「怎么问」，不管「问什么」——选项从哪来、选了之后干什么，留给调用方。
"""

import unicodedata


def width(s: str) -> int:
    """显示宽度。中日韩全角字符占两格。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def pad(s: str, n: int) -> str:
    """右侧补空格到显示宽度 n。"""
    return s + " " * max(0, n - width(s))


def ask_line(prompt: str) -> str:
    """读一行并 strip。EOF 当空串处理（等于「取默认」）。"""
    try:
        return input(prompt).strip()
    except EOFError:
        print()
        return ""


def parse_indices(raw: str, count: int) -> list[int] | None:
    """把 "1,3" / "1 3" 解析成 [1, 3]（1-based，去重保序）。

    有任何一项不是 `1..count` 的整数就返回 None —— 调用方负责报错，因为
    「哪一项不合法」的提示要看它自己的业务（是没这个编号，还是这一项不可用）。
    """
    out: list[int] = []
    for tok in raw.replace(",", " ").split():
        if not tok.isdigit():
            return None
        n = int(tok)
        if not 1 <= n <= count:
            return None
        if n not in out:
            out.append(n)
    return out or None


def pick(title: str, rows: list[tuple], default: int = 0) -> str:
    """单选。rows 是 (key, 标签, 说明)，返回选中的 key。

    回车或 EOF 取 `default`（0-based）。
    """
    print(f"\n{title}")
    for i, (_, label, note) in enumerate(rows, 1):
        print(f"  {i}  {pad(label, 18)}{note}")
    hint = f"回车 = {rows[default][1]}"
    while True:
        raw = ask_line(f"选哪个（{hint}）：")
        if not raw:
            return rows[default][0]
        got = parse_indices(raw, len(rows))
        if got is None or len(got) != 1:
            print(f"  「{raw}」不是一个能选的编号")
            continue
        return rows[got[0] - 1][0]


class Form:
    """一屏表单：列出各字段现值，输编号改某一项，回车提交。

    `rows` 是 (标签, key, 显示函数, 编辑函数)：

    - 显示函数收当前值，返回要打印的字符串；
    - 编辑函数收当前值，返回新值 —— 用户取消、输入非法时就返回原值。

    不做成「逐项都问一遍」，是因为全默认也要敲好几次回车；一屏现状 + 按需
    修改更省事。
    """

    def __init__(self, rows: list[tuple], values: dict, *,
                 title: str = "当前配置",
                 prompt: str = "改哪一项（回车提交）",
                 note: str = ""):
        self.rows = rows
        self.values = dict(values)
        self.title = title
        self.prompt = prompt
        self.note = note

    def render(self) -> None:
        print(f"\n{self.title}（回车 = 按这套跑，输编号改）")
        if self.note:
            for line in self.note.splitlines():
                print(f"  {line}")
        for i, (label, key, show, _) in enumerate(self.rows, 1):
            print(f"  {i}  {pad(label, 10)}{show(self.values[key])}")

    def run(self) -> dict:
        while True:
            self.render()
            raw = ask_line(f"{self.prompt}：")
            if not raw:
                return self.values
            got = parse_indices(raw, len(self.rows))
            if got is None or len(got) != 1:
                print(f"  「{raw}」不是一个能改的编号")
                continue
            _, key, _, edit = self.rows[got[0] - 1]
            self.values[key] = edit(self.values[key])
