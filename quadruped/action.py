"""A 模型的输出空间：能选哪些动作、参数多大、哪些会占住线程。

A 模型**不是文本模型**，它不生成 token 序列。输出是两样东西：

    action_id  ∈ [0, len(NAMES))   选哪个动作
    values     ∈ [-1, 1]^3         三个参数槽，按该动作的上下限换成物理量

`decode()` 把这两样变成 `("move", {"vx": 0.2, ...})`，`act()` 顺手交给 `Backend`。
名字只出现在日志和提示词里，模型见到的是 id。

## 为什么是 id + 固定槽位

各动作的参数个数不一样（`move` 三个、`jump` 零个）。固定 3 个槽、不用到的填 0，
输出头就是「11 个 logits + 3 个连续值」，形状固定，不用按动作切换。

代价是槽里的物理量随动作变：`move` 的 vx 是 ±0.40，`forward` 的 distance 是 ±3.0。
所以槽里的数一律是 [-1, 1]，由 `decode()` 按该动作自己的上下限做仿射 —— 模型不需要
知道米和弧度，它只需要知道「往这个方向推多少」。

## 文本入口是另一条路

`parse()` / `call_text()` 吃 `move(vx=0.3)` 这样的字符串，给 1 Hz 的文本层和手工调试
用。A 模型不走这条路：它没有字符串可解析。

## 两类动作

    速度类  move                        非阻塞，20 Hz 每帧覆盖上一次
    任务类  forward / turn / jump ...   阻塞或占腿，同一时刻只能一个

任务类执行期间占住调用线程（`forward 1 m` 在仿真里十几秒）。低层 200 Hz 回路不受影响
—— 它有自己的线程。占用的是 A 模型：这段时间新目标排队，或者先用 `freeze()` 打断。

## 越界怎么处理

裁剪到边界并在返回值里标出来，不报错。模型推 3.0 m/s 时，报错的后果是它不知道该改
多少、下一帧还推 3.0；裁到 0.40 至少动起来了，`clamped` 进观测后下一帧会收敛。
"""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from typing import Sequence

import quadruped.limits as limits


class ActionError(ValueError):
    """动作不合法。消息要指向具体字段，它可能直接进观测或日志。"""


# 一个参数：类型、默认值（None = 必填）、上下限（None = 不校验）、字符串可选项。
@dataclass(frozen=True)
class Param:
    type: type = float
    default: object = None
    lo: float | None = None
    hi: float | None = None
    choices: tuple[str, ...] | None = None

    # 物理量区间。没给 lo 的参数用默认值兜（`forward.vx` 这种下限非零的）。
    def span(self, name: str) -> tuple[float, float]:
        lo = self.lo if self.lo is not None else self.default
        if lo is None or self.hi is None:
            raise ActionError(f"参数 {name} 没有上下限，不能从 [-1, 1] 换算")
        return float(lo), float(self.hi)

    # [-1, 1] → 物理量。字符串参数走可选项的索引。
    def from_unit(self, name: str, raw: float):
        x = max(-1.0, min(1.0, float(raw)))
        if self.type is str:
            if not self.choices:
                raise ActionError(f"参数 {name} 是字符串，但没给可选项")
            last = len(self.choices) - 1
            return self.choices[min(last, max(0, int(round((x + 1) / 2 * last))))]
        lo, hi = self.span(name)
        # 保留 9 位：0.5 这种槽算出来是 0.20000000000000007，日志和数据集里难看，
        # 也不利于和人工写的数对上。
        return round(lo + (x + 1) / 2 * (hi - lo), 9)

    # 物理量 → [-1, 1]，训练标注时把示范反过来编码要用。
    def to_unit(self, name: str, value) -> float:
        if self.type is str:
            if not self.choices:
                raise ActionError(f"参数 {name} 是字符串，但没给可选项")
            if value not in self.choices:
                raise ActionError(f"参数 {name} 只能取 {', '.join(self.choices)}，"
                                  f"收到 {value!r}")
            last = len(self.choices) - 1
            index = self.choices.index(value)
            return 2 * index / last - 1 if last else 0.0
        lo, hi = self.span(name)
        if hi <= lo:
            raise ActionError(f"参数 {name} 的上下限写反了：{lo} > {hi}")
        return (float(value) - lo) / (hi - lo) * 2 - 1

    # 文本入口用：类型校验 + 裁剪。返回 (值, 是否被裁剪)。
    def coerce(self, name: str, value: object) -> tuple[object, bool]:
        if self.type is str:
            if not isinstance(value, str):
                raise ActionError(f"参数 {name} 要字符串，收到 {type(value).__name__}")
            if self.choices and value not in self.choices:
                raise ActionError(f"参数 {name} 只能取 {', '.join(self.choices)}，"
                                  f"收到 {value!r}")
            return value, False
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ActionError(f"参数 {name} 要数字，收到 {type(value).__name__}")
        value = float(value)
        if self.lo is not None and value < self.lo:
            return float(self.lo), True
        if self.hi is not None and value > self.hi:
            return float(self.hi), True
        return value, False


@dataclass(frozen=True)
class Spec:
    params: dict[str, Param] = field(default_factory=dict)
    # 执行期间占住调用线程，20 Hz 循环里不能同步调。
    blocking: bool = False
    note: str = ""


# 动作表。键就是 `Backend` 的方法名 —— 不另起别名，模型选中的 id 直接映射到这里。
SPECS: dict[str, Spec] = {
    # 持续速度指令。发一次就一直走，直到下一次 move 或 stop。
    "move": Spec({
        "vx": Param(float, 0.0, -limits.VX_MAX, limits.VX_MAX),
        "vy": Param(float, 0.0, -limits.VY_MAX, limits.VY_MAX),
        "vyaw": Param(float, 0.0, -limits.VYAW_MAX, limits.VYAW_MAX),
    }, note="速度指令，一直生效到被覆盖"),

    # 停车但保持站立。
    "stop_motion": Spec(note="停步站住，不蹲下"),

    # 走够距离才返回。方向由正负号定，不另开后退动作。
    "forward": Spec({
        "distance": Param(float, None, -limits.FORWARD_MAX, limits.FORWARD_MAX),
        "vx": Param(float, limits.FORWARD_VX, limits.FORWARD_VX_MIN, limits.VX_MAX),
    }, blocking=True, note="闭环位移，占住 |distance|/vx + 2 s"),

    # 原地转，正为逆时针。
    "turn": Spec({
        "degrees": Param(float, None, -limits.TURN_MAX, limits.TURN_MAX),
        "yaw_rate": Param(float, limits.TURN_RATE, limits.TURN_RATE_MIN,
                          limits.VYAW_MAX),
    }, blocking=True, note="闭环转向，占住 |degrees|/yaw_rate + 2 s"),

    # 换运动步态风格。字符串参数，靠可选项列表映射到槽里的数值。
    "set_gait_style": Spec({
        "style": Param(str, "mech", choices=("mech", "bio", "search")),
    }, note="mech / bio / search"),

    "toggle_loco_gait": Spec(note="轮换到下一档步态"),

    # 占用腿的动作：排完就返回，播完自己站住。
    "jump": Spec(note="原地跳，不阻塞但占腿"),
    "crouch": Spec({
        "depth": Param(float, limits.CROUCH, limits.CROUCH_MIN, limits.CROUCH_MAX),
    }, note="蹲下，不阻塞但占腿"),
    "uncrouch": Spec(note="站起来"),
    "stand": Spec(note="回标称站高"),

    # 打断：比 stop_motion 更狠，跳跃与蹲伏都会被它中断。
    "freeze": Spec(note="中断当前动作并站定"),
}

# 动作 id 的顺序。**只能在末尾追加**：训练数据里的 id 是按位置存的，中间插一个
# 会让旧数据全错位，而且不报错。
NAMES: tuple[str, ...] = tuple(SPECS)

# 参数槽个数。取参数最多的那个动作（move 三个）。不用的槽填 0。
SLOTS = 3


# id → 动作名。越界要报，不要静默夹到边界 —— 那会让模型学到一个不存在的动作。
def _name_of(action_id: int) -> str:
    index = int(action_id)
    if not 0 <= index < len(NAMES):
        raise ActionError(f"动作 id {index} 越界，可选 0~{len(NAMES) - 1}")
    return NAMES[index]


# 模型输出 → (动作名, 关键字参数)。纯函数，不碰机器人。
#
# 槽不够就当后面的都是 0（中立位），多了忽略：输出头的形状偶尔多一维，为这个报错
# 不值得。要执行再交给 `act()`。
def decode(action_id: int, values: Sequence[float]) -> tuple[str, dict]:
    name = _name_of(action_id)
    spec = SPECS[name]
    vals = [float(v) for v in values]
    kwargs: dict = {}
    for i, (key, param) in enumerate(spec.params.items()):
        raw = vals[i] if i < len(vals) else 0.0
        kwargs[key] = param.from_unit(key, raw)
    return name, kwargs


# 反向：示范动作 → (id, 三个槽)。训练标注和回放都用这条。
#
# 没给的参数用默认值；必填参数没给就报，别默默拿 0 顶上 —— 那会训练出一个
# 「距离永远为 0 的 forward」。
def encode(name: str, **kwargs) -> tuple[int, list[float]]:
    spec = SPECS.get(name)
    if spec is None:
        raise ActionError(f"没有动作 {name!r}，可用：{', '.join(NAMES)}")
    for key in kwargs:
        if key not in spec.params:
            raise ActionError(f"{name} 没有参数 {key!r}")

    slots = [0.0] * SLOTS
    for i, (key, param) in enumerate(spec.params.items()):
        value = kwargs.get(key, param.default)
        if value is None:
            raise ActionError(f"{name} 缺参数 {key}")
        slots[i] = param.to_unit(key, value)
    return NAMES.index(name), slots


# 执行一次模型输出。这是 20 Hz 循环该调的那个。
def act(backend, action_id: int, values: Sequence[float]) -> dict:
    name, kwargs = decode(action_id, values)
    return call(backend, name, **kwargs)


# 校验并执行一个具名动作。返回可直接进观测的字典。
#
# 失败分两种，调用方据此决定下一步：
#   ActionError           输出不合法 —— 该让模型重写，或记进训练数据当负样本
#   NotImplementedError   控制端没这个能力 —— 原样上抛，该让上层换路子
def call(backend, name: str, /, **kwargs) -> dict:
    spec = SPECS.get(name)
    if spec is None:
        raise ActionError(f"没有动作 {name!r}，可用：{', '.join(NAMES)}")

    for key in kwargs:
        if key not in spec.params:
            raise ActionError(
                f"{name} 没有参数 {key!r}，可用：{', '.join(spec.params) or '（无）'}")

    args: dict = {}
    clamped: dict = {}
    for key, param in spec.params.items():
        if key in kwargs:
            value, was_clamped = param.coerce(key, kwargs[key])
        elif param.default is not None:
            value, was_clamped = param.coerce(key, param.default)
            # 默认值本身不标裁剪，它没被模型写过。
            was_clamped = False
        else:
            raise ActionError(f"{name} 缺参数 {key}")
        args[key] = value
        if was_clamped:
            clamped[key] = [kwargs[key], value]

    if spec.blocking and getattr(backend, "busy", False):
        raise ActionError(f"腿正被别的动作占着，{name} 现在执行不了")

    try:
        fn = getattr(backend, name)
    except AttributeError:
        raise ActionError(f"当前控制端没有 {name}") from None

    try:
        out = fn(**args)
    except NotImplementedError as err:
        # 后端明确说「我没有这个能力」，消息里已经写了是哪个后端缺什么。
        raise ActionError(str(err)) from None

    data = {"action": name, "args": args}
    if clamped:
        data["clamped"] = clamped
    if isinstance(out, dict):
        data.update(out)
    elif isinstance(out, str):
        data["value"] = out
    else:
        code = int(out)
        if code != 0:
            raise ActionError(f"{name} 返回 {code}")
        data["code"] = code
    return data


# -- 文本入口（1 Hz 的文本层与手工调试用，A 模型不走这条） --------------------

# 把 `move(vx=0.3)` 解析成 (name, kwargs)。
#
# 用 ast 不用正则：输入是任意文本，`eval` 等于把控制权交出去。`mode="eval"` 之后只认
# `Call(Name, 字面量)`，属性访问、嵌套调用、下标天然被拒 —— 正则看
# `move(vx=__import__("os").system("rm -rf /"))` 只是「括号里有东西」。
#
# 位置参数按声明顺序绑定（模型和人都爱写 `move(0.3, 0, 0)`），不允许同一个参数
# 给两次。
def parse(text: str) -> tuple[str, dict]:
    raw = text.strip().rstrip(";").strip()
    if not raw:
        raise ActionError("空调用")
    try:
        node = ast.parse(raw, mode="eval").body
    except SyntaxError as err:
        raise ActionError(f"不是合法的方法调用：{err.msg}") from None

    if not isinstance(node, ast.Call):
        raise ActionError(f"要写成方法调用，形如 move(vx=0.3)，收到 {type(node).__name__}")
    if not isinstance(node.func, ast.Name):
        raise ActionError("只认裸方法名，不支持 a.b(...) 或下标")
    name = node.func.id
    if name.startswith("_"):
        raise ActionError(f"不许调用 {name}")
    spec = SPECS.get(name)
    if spec is None:
        raise ActionError(f"没有动作 {name!r}，可用：{', '.join(sorted(SPECS))}")

    for arg in node.args:
        if isinstance(arg, ast.Starred):
            raise ActionError("不支持 *args")
    for kw in node.keywords:
        if kw.arg is None:
            raise ActionError("不支持 **kwargs")

    names = list(spec.params)
    kwargs: dict = {}
    if len(node.args) > len(names):
        raise ActionError(f"{name} 最多 {len(names)} 个参数，收到 {len(node.args)}")
    for i, arg in enumerate(node.args):
        kwargs[names[i]] = _literal(arg, names[i])
    for kw in node.keywords:
        if kw.arg not in spec.params:
            raise ActionError(
                f"{name} 没有参数 {kw.arg!r}，可用：{', '.join(names) or '（无）'}")
        if kw.arg in kwargs:
            raise ActionError(f"{name} 的参数 {kw.arg} 给了两次")
        kwargs[kw.arg] = _literal(kw.value, kw.arg)

    # 必填项在解析阶段就报，不留到 call：拿到「缺 distance」才知道下一句补什么。
    for key, param in spec.params.items():
        if param.default is None and key not in kwargs:
            raise ActionError(f"{name} 缺参数 {key}")
    return name, kwargs


# 参数只允许字面量。`f(x)`、`a.b`、`[1]` 一律拒 —— 这一步是 ast 方案的全部意义。
def _literal(node: ast.AST, name: str) -> object:
    if isinstance(node, ast.Constant) and not isinstance(node.value, (bool, type(None))):
        if isinstance(node.value, (int, float, str)):
            return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        inner = node.operand
        if isinstance(inner, ast.Constant) and isinstance(inner.value, (int, float)) \
                and not isinstance(inner.value, bool):
            return -inner.value if isinstance(node.op, ast.USub) else inner.value
    raise ActionError(f"参数 {name} 只能是字面量数字或字符串")


# 解析 + 执行一行文本。文本层与手工调试用。
def call_text(backend, text: str) -> dict:
    name, kwargs = parse(text)
    return call(backend, name, **kwargs)


# 动作表，带 id。喂提示词或写日志用。
def describe() -> str:
    lines = []
    for index, (name, spec) in enumerate(SPECS.items()):
        sig = ", ".join(f"{k}={'-' if p.default is None else p.default}"
                        for k, p in spec.params.items())
        tail = "  [阻塞]" if spec.blocking else ""
        note = f"  # {spec.note}" if spec.note else ""
        lines.append(f"[{index:2d}] {name}({sig}){tail}{note}")
    return "\n".join(lines)
