"""模型输出到命令字段的翻译层：抠 JSON、校验、夹到训练区间、补默认值。

模型是 1.5B 的通用对话模型，不是专门训出来的结构化输出器。它会干这些事：

    - 用 ```json 围栏包住结果
    - 在 JSON 前面加一句「好的，这是解析结果：」
    - 把「小跑」写成 gait: "小跑" 而不是 "trot"
    - 给一个超出训练区间的数（vx 3.0）

所以这一层不做「相信模型」，只做「接收任何东西，产出一条合法命令」。任何一项
不合法就丢掉那一项，用默认值 —— 丢一项比整条命令失败好，狗还在走。

字段范围全部来自 `walker.COMMAND_RANGE`，不是这里另写一份。超出训练区间的值
策略没见过，输出不可预期（不是走得更快，是摔）。
"""

from __future__ import annotations

import json
import re

from quadruped.policy import gaits
from quadruped.policy.walker import COMMAND_FIELDS, COMMAND_RANGE, default_commands


# 允许被文本控制的字段。`duration`（训练只有一个取值）、`roll`（锁 0）、
# `aux`（部署给 0）不在里面，模型给了也忽略。
FIELDS = (
    "vx", "vy", "vyaw",
    "height", "freq",
    "footswing", "pitch", "width", "length",
)

# `vyaw` 的单位是度，不是 rad/s，也故意不用 `walker.COMMAND_RANGE` 那一套。
#
# 策略吃的是角速度（-1.0~1.0 rad/s），但模型不该做「每秒多少弧度、乘几秒」
# 这个乘法：它看不见自己转得多快，也不知道命令什么时候过期，算出来必然是错的。
# 让它直接说「转 90°」，闭环交给执行层（`think._run_turn`）。
TURN_DEG = (-180.0, 180.0)

# 步态说法到预设名的映射。模型经常直接抄中文词，映射一次比在提示词里反复强调
# 「只能输出英文」更省事。
GAIT_ALIASES = {
    "trot": "trot", "小跑": "trot", "对角": "trot", "快步": "trot",
    "pronk": "pronk", "跳": "pronk", "蹦": "pronk", "跳着走": "pronk",
    "四腿同时": "pronk", "同相": "pronk",
    "bound": "bound", "跑跳": "bound", "前后": "bound", "兔跳": "bound",
    "pace": "pace", "同侧": "pace", "顺拐": "pace", "左右": "pace",
}

# 模型偶尔会把整段话包在围栏里，或者前后带说明。抠第一个 { 到最后一个 }。
_JSON_SPAN = re.compile(r"\{.*\}", re.S)

# 原文里出现这些词，才算「说了要转向」。用来挡模型自己脑补的 vyaw。
# 「偏」也算：模型常把「向左偏一点」当成转向的说法，不含「转」字但意思一样。
TURN_WORDS = ("转", "拐", "绕", "朝向", "面向", "掉头", "偏")

# 同理，挡脑补的 pitch。
PITCH_WORDS = ("俯仰", "低头", "抬头", "前倾", "后倾", "仰起", "倾")


# 按原文校正两个模型靠不住的地方，返回改了什么。
#
# 1.5B 模型实测：提示词写「正数左转」，示例里也放了「左转 → +90」，
# 它仍然把「原地左转」输出成负数，而「右转」也是同一个符号 —— 它只认「转」
# 不认方向。这类错误在提示词层修不掉，只能在拿到原文之后按词纠正。
def _guard(fields: dict, text: str | None) -> list[str]:
    # `None` 和空串是两件事，必须分开：
    #
    #   ""    原文就是空的（用户没说话，或模型没写 thought）—— 照旧检查，
    #         没有依据的 vyaw 当脑补丢掉
    #   None  这次**没有可对照的自然语言**（L 层自驱、模型只给 JSON 命令）——
    #         下面三条都是「按原文纠错」，没有原文就只剩误伤
    #
    # 第二种情况是关掉 thought（`prompt.EMIT_THOUGHT`）之后才有的：原来自驱时拿
    # thought 当依据，thought 一关 body 就是空串，每条 vyaw 都被这里丢掉，实测
    # `{"command": {"vyaw": 80}}` 解析成 `{}`，狗一次都转不了。
    if text is None:
        return []

    notes: list[str] = []
    body = text or ""
    left, right = "左" in body, "右" in body

    # 只出现一侧才敢翻：两边都有（「向左转再往右走」）交给模型自己判断。
    if left != right:
        want = 1.0 if left else -1.0
        for key in ("vyaw", "vy"):
            value = fields.get(key, 0.0)
            if value and value * want < 0:
                fields[key] = -value
                notes.append(f"{key} 符号按原文翻正")

    if fields.get("vyaw") and not any(w in body for w in TURN_WORDS):
        fields.pop("vyaw")
        notes.append("原文没提转向，丢掉 vyaw")

    if fields.get("pitch") and not any(w in body for w in PITCH_WORDS):
        fields.pop("pitch")
        notes.append("原文没提俯仰，丢掉 pitch")

    return notes


# 从模型原文里取出一个 dict。取不到返回 None，调用方决定是重试还是放弃。
def extract_json(raw: str) -> dict | None:
    text = (raw or "").strip()
    if not text:
        return None
    span = _JSON_SPAN.search(text)
    if span is None:
        return None
    try:
        got = json.loads(span.group(0))
    except json.JSONDecodeError:
        return None
    return got if isinstance(got, dict) else None


# 把模型给的值夹进合法区间。非数值、布尔、字符串数字都按「拿不到」处理。
def _clamp(name: str, value) -> float | None:
    lo, hi = TURN_DEG if name == "vyaw" else COMMAND_RANGE[name]
    if isinstance(value, bool):
        return None
    try:
        num = float(value)
    except (TypeError, ValueError):
        return None
    if num != num:                      # NaN
        return None
    return max(lo, min(hi, num))


# 校验一份已经解析好的 dict，返回 (fields, notes)。`parse()` 和 L 层的
# `think` 都走这里 —— 后者拿到的是 `{"thought": ..., "command": {...}}`，字段在
# 内层，不能直接喂 `parse()`。
def validate(data: dict, text: str | None = "") -> tuple[dict, list[str]]:
    fields: dict[str, float] = {}
    notes: list[str] = []

    for key in FIELDS:
        if key not in data:
            continue
        got = _clamp(key, data[key])
        if got is None:
            notes.append(f"{key} 不是数，丢掉")
            continue
        if got != float(data[key]):
            notes.append(f"{key} {data[key]} 夹到 {got:g}")
        fields[key] = got

    # 步态单独处理：它的取值是字符串，映射到相位三元组，不在 FIELDS 里。
    if "gait" in data:
        name = GAIT_ALIASES.get(str(data["gait"]).strip().lower())
        if name is None:
            notes.append(f"步态 {data['gait']!r} 不认识")
        else:
            fields["gait"] = name

    if not fields:
        notes.append("一个可用字段都没有")
        return fields, notes
    return fields, notes + _guard(fields, text)


# 校验一条模型输出，返回 (fields, notes)。`notes` 记录丢了什么、夹了什么，
# 进日志用 —— 模型改了什么必须能看见，不然调提示词全靠猜。
def parse(raw: str, text: str = "") -> tuple[dict, list[str]]:
    data = extract_json(raw)
    if data is None:
        return {}, ["没有 JSON"]
    return validate(data, text)


# 把校验过的 fields 合到默认命令上，返回 `MotionService.set_command` 的 kwargs。
#
# 默认命令是「站住不动、小步频、trot」那一组。只覆盖模型明确给出的项，
# 没给的不动 —— 说「跳着走」不该顺手把速度清零。
#
# `vyaw` 不在这里面：它是角度，不是速度。直接下发会被策略当成 90 rad/s 夹到 1.0，
# 变成「用最大角速度一直转」。由 `think` 拿去开一次转向，执行层负责转到位。
def to_command(fields: dict) -> dict:
    out = {key: value for key, value in fields.items()
           if key not in ("gait", "vyaw")}
    if "gait" in fields:
        # 步态只给名字，相位三元组从预设里取。让模型直接给 phase/offset/bound
        # 三个数，它给出的组合多半是策略没训过的。
        cmd = gaits.apply(default_commands(), fields["gait"])
        for key in ("phase", "offset", "bound"):
            out[key] = float(cmd[COMMAND_FIELDS.index(key)])
    return out
