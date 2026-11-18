"""A 模型动作表的自检：id/槽位、文本解析、裁剪、错误分流。

    .venv/bin/python test/action_check.py

A 模型不是文本模型，输出是「动作 id + 三个连续槽」。这一层的语义就是它学得到的
东西：动作顺序、每个槽的换算区间、哪些动作会占住线程。所以检查的是契约 ——

1. `decode(id, 槽)` 把 [-1, 1] 换算成该动作自己的物理区间；`encode` 是反函数，
   往返要回到原值（裁剪除外）。
2. 输出槽不够是允许的（后面的槽当 0），id 越界必须报 —— 夹到边界会训练出一个
   不存在的动作。
3. 文本入口只认 `Call(Name, 字面量)`。属性访问、嵌套调用、下标一律拒，这条破了
   等于把任意代码执行开给外部。
4. 越界裁剪而不是报错，且 `clamped` 里能看出原值和实际值。
5. 「输出不合法」和「控制端没这个能力」分两种异常：前者 `ActionError`，后者
   `NotImplementedError` 被转成 `ActionError` 但保留后端原文。

任一条不过就返回非零。
"""

from __future__ import annotations

import sys
from pathlib import Path

# 脚本模式跑的时候 sys.path[0] 是 test/，不是仓库根，quadruped 得自己找。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped import limits                      # noqa: E402
from quadruped.action import (                    # noqa: E402
    NAMES,
    SLOTS,
    SPECS,
    ActionError,
    act,
    call,
    call_text,
    decode,
    describe,
    encode,
    parse,
)

CASES_OK = [
    # 文本 → (动作名, 关键字参数)
    ("move(vx=0.2)", ("move", {"vx": 0.2})),
    ("move(0.2, 0, 0)", ("move", {"vx": 0.2, "vy": 0, "vyaw": 0})),
    ("move()", ("move", {})),
    ("forward(0.5)", ("forward", {"distance": 0.5})),
    ("forward(distance=-0.5, vx=0.2)", ("forward", {"distance": -0.5, "vx": 0.2})),
    ("turn(-90)", ("turn", {"degrees": -90})),
    ('set_gait_style("bio")', ("set_gait_style", {"style": "bio"})),
    ("jump()", ("jump", {})),
    ("stop_motion();", ("stop_motion", {})),
    ("  crouch(depth=0.1)  ", ("crouch", {"depth": 0.1})),
]

CASES_BAD = [
    ("", "空"),
    ("move", "不是调用"),
    ("a.b()", "属性访问"),
    ("__import__('os').system('ls')", "属性访问"),
    ("move(vx=0.2) + 1", "不是调用"),
    ("move(vx=os.x)", "嵌套属性"),
    ("move(vx=f(1))", "嵌套调用"),
    ("move(vx=[1])", "列表"),
    ("move(vx=True)", "布尔"),
    ("move(vx=None)", "None"),
    ("move(vx=0.2, vz=1)", "未知参数"),
    ("move(0.2, 0, 0, 1)", "位置参数过多"),
    ("move(0.2, vx=0.3)", "参数给两次"),
    ("forward()", "缺必填"),
    ("moove(vx=0.1)", "未知动作"),
    ("_no_impl('x')", "下划线"),
]


class Stub:
    """记下每次调用，按需返回各种形状的结果。"""

    def __init__(self, codes=None, busy=False):
        self.calls = []
        self.codes = codes or {}
        self.busy = busy

    def move(self, vx=0.0, vy=0.0, vyaw=0.0):
        self.calls.append(("move", vx, vy, vyaw))
        return self.codes.get("move", 0)

    def forward(self, distance, vx=0.3):
        self.calls.append(("forward", distance, vx))
        return {"progress": distance, "target": distance}

    def turn(self, degrees, yaw_rate=0.5):
        self.calls.append(("turn", degrees, yaw_rate))
        return {"turned_deg": degrees}

    def set_gait_style(self, style):
        self.calls.append(("set_gait_style", style))
        return style

    def jump(self):
        self.calls.append(("jump",))
        return 0

    def stop_motion(self):
        self.calls.append(("stop_motion",))
        return 0

    def crouch(self, depth=0.08):
        self.calls.append(("crouch", depth))
        return 0


# 官方后端缺 forward / turn / set_gait_style：调用要抛 NotImplementedError 原文。
class Officialish(Stub):
    def forward(self, *a, **k):
        raise NotImplementedError("假官方没有闭环位移，只有自研控制端实现了它")

    turn = forward
    set_gait_style = forward


def check_units() -> list[str]:
    bad: list[str] = []

    # id 顺序即 NAMES。训练数据按位置存 id，中间插一个会让旧数据整体错位且不报错。
    if NAMES != tuple(SPECS):
        bad.append("NAMES 与 SPECS 顺序不一致")

    # 槽全 0 = 各参数区间的中点。move 对称，中点就是 0。
    got = decode(NAMES.index("move"), [0.0, 0.0, 0.0])
    if got != ("move", {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}):
        bad.append(f"中点解码不对：{got}")

    # 槽打满 ±1 要正好落在物理上下界。
    _, hi = decode(NAMES.index("move"), [1.0, 1.0, 1.0])
    _, lo = decode(NAMES.index("move"), [-1.0, -1.0, -1.0])
    if (hi["vx"], hi["vyaw"]) != (limits.VX_MAX, limits.VYAW_MAX):
        bad.append(f"+1 没打到上界：{hi}")
    if (lo["vx"], lo["vyaw"]) != (-limits.VX_MAX, -limits.VYAW_MAX):
        bad.append(f"-1 没打到下界：{lo}")

    # 非对称区间：forward 的 vx 是 0.05~0.40，中点是 0.225 而不是 0。
    _, mid = decode(NAMES.index("forward"), [0.0, 0.0])
    want = (limits.FORWARD_VX_MIN + limits.VX_MAX) / 2
    if abs(mid["vx"] - want) > 1e-9:
        bad.append(f"非对称区间中点算错：{mid['vx']}，应为 {want}")
    if mid["distance"] != 0.0:
        bad.append(f"distance 的中点是 0 才对：{mid['distance']}")

    # 槽不够时按 0（区间中点）填；无参数动作不吃槽。
    if decode(NAMES.index("jump"), []) != ("jump", {}):
        bad.append("无参数动作不该吃槽")
    _, short = decode(NAMES.index("crouch"), [])
    if abs(short["depth"] - (limits.CROUCH_MIN + limits.CROUCH_MAX) / 2) > 1e-9:
        bad.append(f"缺槽该按中点算：{short}")

    # 字符串参数走可选项索引。
    style_id = NAMES.index("set_gait_style")
    if decode(style_id, [-1.0])[1]["style"] != "mech":
        bad.append("style 的 -1 没落到 mech")
    if decode(style_id, [1.0])[1]["style"] != "search":
        bad.append("style 的 +1 没落到 search")

    # id 越界必须报，不能夹到边界 —— 那等于凭空多出两个动作。
    for bad_id in (-1, len(NAMES)):
        try:
            decode(bad_id, [0.0])
        except ActionError:
            continue
        bad.append(f"id {bad_id} 越界没报错")

    # encode / decode 往返，误差 1e-9 以内。
    samples = (
        ("move", {"vx": 0.23, "vy": -0.11, "vyaw": 0.5}),
        ("forward", {"distance": -1.75, "vx": 0.2}),
        ("turn", {"degrees": 135.0, "yaw_rate": 0.4}),
        ("crouch", {"depth": 0.13}),
        ("set_gait_style", {"style": "bio"}),
    )
    for name, kwargs in samples:
        action_id, slots = encode(name, **kwargs)
        if len(slots) != SLOTS:
            bad.append(f"{name} 的槽位数不对：{len(slots)}")
        back_name, back = decode(action_id, slots)
        if back_name != name:
            bad.append(f"往返动作名变了：{name} → {back_name}")
            continue
        for key, want_value in kwargs.items():
            got_value = back[key]
            if isinstance(want_value, str):
                if got_value != want_value:
                    bad.append(f"{name}.{key} 往返变了：{want_value} → {got_value}")
            elif abs(got_value - want_value) > 1e-9:
                bad.append(f"{name}.{key} 往返误差：{want_value} → {got_value}")

    # 必填参数缺了不能默默填 0，否则会训出一个「距离永远是 0 的 forward」。
    try:
        encode("forward")
    except ActionError:
        pass
    else:
        bad.append("encode forward 缺 distance 没报错")

    # act() 走完整条路：解码后后端收到的是物理量，不是 [-1, 1]。
    stub = Stub()
    out = act(stub, NAMES.index("move"), [0.5, 0.0, 0.0])
    if abs(stub.calls[-1][1] - limits.VX_MAX * 0.5) > 1e-9:
        bad.append(f"act 传给后端的 vx 不对：{stub.calls[-1][1]}")
    if out["action"] != "move":
        bad.append(f"act 返回的动作名不对：{out}")
    return bad


def check_parse() -> list[str]:
    bad = []
    for text, want in CASES_OK:
        try:
            got = parse(text)
        except ActionError as err:
            bad.append(f"该通未通 {text!r}：{err}")
            continue
        if got != want:
            bad.append(f"解析不一致 {text!r}：得到 {got}，期望 {want}")
    for text, why in CASES_BAD:
        try:
            parse(text)
        except ActionError:
            continue
        bad.append(f"该拒未拒 {text!r}（{why}）")
    return bad


def check_call() -> list[str]:
    bad = []

    # 越界裁剪：vx 0.99 → VX_MAX，且 clamped 记下原值与实际值。
    out = call_text(Stub(), "move(vx=0.99, vy=-9, vyaw=0)")
    got = out["args"]
    if got["vx"] != limits.VX_MAX:
        bad.append(f"vx 应裁到 {limits.VX_MAX}，得到 {got['vx']}")
    if got["vy"] != -limits.VY_MAX:
        bad.append(f"vy 应裁到 {-limits.VY_MAX}，得到 {got['vy']}")
    if out.get("clamped", {}).get("vx") != [0.99, limits.VX_MAX]:
        bad.append(f"clamped 没记下原值：{out.get('clamped')}")

    # 默认可省：forward(0.5) 的 vx 用默认值，不应出现在 clamped 里。
    out = call_text(Stub(), "forward(0.5)")
    if out["args"]["vx"] != limits.FORWARD_VX:
        bad.append(f"forward 的 vx 默认值不对：{out['args']}")
    if "vx" in out.get("clamped", {}):
        bad.append("默认值被标成裁剪了")

    # 结果形状：dict 合并、str 走 value、非零码抛。
    if call_text(Stub(), "turn(90)")["turned_deg"] != 90:
        bad.append("turn 的返回字典没合并进结果")
    if call_text(Stub(), 'set_gait_style("bio")')["value"] != "bio":
        bad.append("字符串返回值没放进 value")
    try:
        call(Stub(codes={"move": 4201}), "move", vx=0.1)
    except ActionError as err:
        if "4201" not in str(err):
            bad.append(f"返回码没进错误消息：{err}")
    else:
        bad.append("非零返回码没有抛错")

    # 能力缺失：NotImplementedError 转 ActionError，原文保留。
    try:
        call_text(Officialish(), "forward(0.5)")
    except ActionError as err:
        if "假官方" not in str(err):
            bad.append(f"后端原文丢了：{err}")
    else:
        bad.append("缺能力的动作没有抛错")

    # 阻塞动作碰上占腿：直接拒，不去调。
    stub = Stub(busy=True)
    try:
        call_text(stub, "forward(0.5)")
    except ActionError as err:
        if "占着" not in str(err):
            bad.append(f"占腿提示不对：{err}")
    else:
        bad.append("busy 时仍执行了阻塞动作")
    if stub.calls:
        bad.append(f"busy 时不该调到后端：{stub.calls}")

    # 非阻塞动作不受 busy 影响。
    call_text(Stub(busy=True), "move(vx=0.1)")
    return bad


def check_specs() -> list[str]:
    bad = []
    for name, spec in SPECS.items():
        if name.startswith("_"):
            bad.append(f"动作名不该以下划线开头：{name}")
        for key, param in spec.params.items():
            if param.lo is not None and param.hi is not None and param.lo > param.hi:
                bad.append(f"{name}.{key} 上下限写反了")
            if param.default is not None and param.lo is not None:
                try:
                    param.coerce(key, param.default)
                except ActionError as err:
                    bad.append(f"{name}.{key} 默认值不合法：{err}")
    return bad


def main() -> int:
    failed = 0
    for title, fn in (("id 与槽位", check_units), ("文本解析", check_parse),
                      ("执行", check_call), ("动作表", check_specs)):
        bad = fn()
        if bad:
            failed += 1
            print(f"✗ {title}")
            for line in bad:
                print(f"    {line}")
        else:
            print(f"✓ {title}")

    print(f"\n动作表（{len(SPECS)} 个）：")
    print("\n".join("  " + line for line in describe().splitlines()))
    print("\n契约成立" if not failed else f"\n{failed} 组被破坏")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
