"""决策层的执行端：把「一句话 / 一次思考」变成动作，交给控制端。

这一层不改机器人怎么动，只决定**动哪个**。三种输入进同一个出口：

    结构化命令    别的进程直接调 `action.act(backend, id, slots)`，不经过这里
    文本           `Brain.say("move(vx=0.3)")`，1 Hz 人手或简单规则
    具身智能       `TextPlanner(complete=...)`，`complete` 是模型，返回一行调用

出口是 `action.act()` —— 和另外两条一样的函数、一样的 id + 三个槽。所以模型换了、
提示词换了、甚至换成语音，机器人那侧一行都不用动。

## 模型怎么接

本文件不 import 任何 SDK、任何 HTTP 客户端。`TextPlanner` 只吃一个
`complete(prompt) -> str` 的可调用对象，谁提供都行：宿主里的 Yuki、本地小模型、
一个固定返回的桩。这样这个仓库不需要知道模型在哪个进程里跑。

## 为什么不能塞进 200 Hz 回路

一次模型推理几百毫秒。回路里等它，就是丢帧、腿悬空。所以这层是低频的、独立的循环
（`period` 秒一拍），而且 `Brain` 自己不发关节指令 —— 它只调控制端的方法。

## 失败怎么处理

`action.call()` 会抛 `ActionError`，两种来源：输出不合法（模型写错了）、控制端没这个
能力（推理控制器不会跳）。两种都收进 `last_errors` 并原样返回，下一拍的观测里带上，
让模型看到「跳」为什么失败，而不是静默重试。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from quadruped import action as A


@dataclass(frozen=True)
class Decision:
    """一次决策的结果。`values` 是三个 [-1, 1] 的槽，见 `action.decode()`。"""

    action_id: int
    values: tuple[float, float, float] = (0.0, 0.0, 0.0)
    reason: str = ""

    @property
    def name(self) -> str:
        return A.NAMES[self.action_id] if 0 <= self.action_id < len(A.NAMES) else "?"


# 一个规划器：看当前状态（和一句人话），给出这一拍要做的事。
#
# 返回列表而不是单个：一句话可能是「往前走两步再蹲下」，拆成两个动作。顺序执行，
# 第一个失败就停 —— 后面的动作用的是同一个身体，前一步没到位就不该接着走。
class Planner(Protocol):
    def __call__(self, state: dict, say: str | None = None) -> list[Decision]: ...


# 写死的一串决策。测试和回放用。
class ScriptedPlanner:
    def __init__(self, script: list[Decision]):
        self.script = list(script)

    def __call__(self, state: dict, say: str | None = None) -> list[Decision]:
        return [self.script.pop(0)] if self.script else []


# 文本规划器：把模型输出的一行调用解析成 `Decision`。
#
# `complete` 拿到的是拼好的提示词，返回形如 `move(vx=0.3)` 的一行。解析用
# `action.parse()`，走 ast，不 eval —— 模型输出是**不可信输入**，这一条不能松。
#
# 模型说了一句解析不了的话，不抛：记进 `last_text` 并返回空列表。规划器每拍都会被
# 调，抛一次就把整个决策循环打死了。
class TextPlanner:
    def __init__(self, complete: Callable[[str], str], system: str | None = None):
        self.complete = complete
        self.system = system or build_prompt()
        self.last_text = ""
        self.last_error = ""

    def __call__(self, state: dict, say: str | None = None) -> list[Decision]:
        prompt = self._render(state, say)
        try:
            text = self.complete(prompt)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"模型调用失败：{type(err).__name__}: {err}"
            return []
        self.last_text = (text or "").strip()
        if not self.last_text:
            return []
        try:
            return [self._to_decision(line) for line in self.last_text.splitlines()
                    if line.strip()]
        except A.ActionError as err:
            self.last_error = str(err)
            return []

    def _to_decision(self, line: str) -> Decision:
        name, kwargs = A.parse(line)
        action_id, slots = A.encode(name, **kwargs)
        return Decision(action_id, tuple(slots), reason=line)

    def _render(self, state: dict, say: str | None) -> str:
        parts = [self.system, "", "当前状态："]
        for key, value in state.items():
            parts.append(f"  {key} = {value}")
        if say:
            parts += ["", f"人话：{say}"]
        parts += ["", "只输出一行动作调用。"]
        return "\n".join(parts)


# 提示词：动作表 + 规则。动作表从 `action.SPECS` 生成，不另抄一份 —— 抄了就会漂。
def build_prompt() -> str:
    return "\n".join([
        "你可以让一台四足机器狗做事。可用的动作：",
        A.describe(),
        "",
        "输出格式：一行，形如 move(vx=0.3)。不要解释，不要代码块。",
        "参数用物理量（米、秒、度），不要归一化。",
        "做不到的动作不要选 —— 现在这个控制端可能没有跳跃或蹲伏。",
    ])


@dataclass
class Brain:
    """一拍一拍地把决策落到控制端上。

    `backend` 是 `virtual.robot.Backend`，也可以是推理控制器那个 `PolicyBackend`。
    本类只调它暴露出来的动作方法，不读 DDS。
    """

    backend: object
    planner: Planner
    period: float = 1.0                 # 决策周期，秒。模型慢，别调小
    verbose: bool = True

    last_state: dict = field(default_factory=dict)
    last_results: list[dict] = field(default_factory=list)
    last_errors: list[str] = field(default_factory=list)
    steps: int = 0

    # 观测：给规划器看的东西。只放它用得上的，别把 12 维关节角塞进去 —— 模型看不懂，
    # 还占提示词。
    def observe(self) -> dict:
        b = self.backend
        state = {"steps": self.steps}
        for key in ("rpy", "height", "pos", "busy"):
            try:
                state[key] = getattr(b, key)
            except Exception:                               # noqa: BLE001
                state[key] = None
        if self.last_errors:
            state["上次失败"] = self.last_errors
        return state

    # 执行一拍：问规划器要动作，按顺序执行，第一个失败就停。
    def tick(self, say: str | None = None) -> list[dict]:
        state = self.observe()
        self.last_state = state
        decisions = self.planner(state, say)
        self.last_errors = []
        results: list[dict] = []

        # 规划器自己也可能失败（模型说了句解析不出的话）。它把原因挂在 `last_error`
        # 上，这里收进观测 —— 否则「模型胡说了」和「模型说不做」在日志里长得一样。
        why = getattr(self.planner, "last_error", "")
        if why and not decisions:
            self.last_errors.append(f"规划器: {why}")
            if self.verbose:
                print(f"[brain] 规划失败：{why}", flush=True)

        for d in decisions:
            try:
                out = A.act(self.backend, d.action_id, d.values)
            except A.ActionError as err:
                # `action.call()` 把 NotImplementedError 也转成 ActionError 了，
                # 所以「模型写错」和「控制端没这个能力」都在这一支。
                self.last_errors.append(f"{d.name}: {err}")
                if self.verbose:
                    print(f"[brain] {d.name} 失败：{err}", flush=True)
                break
            out.setdefault("reason", d.reason)
            results.append(out)
            if self.verbose:
                print(f"[brain] {out['action']} {out.get('args', {})}", flush=True)
        self.last_results = results
        self.steps += 1
        return results

    # 按周期一直跑。`says` 可以是一个无限取值的函数（读队列、读麦克风）。
    def run(self, seconds: float | None = None,
            says: Callable[[], str | None] | None = None) -> None:
        t0 = time.monotonic()
        while seconds is None or time.monotonic() - t0 < seconds:
            self.tick(says() if says else None)
            time.sleep(self.period)


# 手工入口：一行一行喂文本，不需要模型。调试动作表和参数时用它。
def main(argv: list[str] | None = None) -> int:
    import sys

    from quadruped.control.virtual.robot import choose_client, connect, resolve_endpoint

    argv = list(sys.argv[1:] if argv is None else argv)
    preset = argv[0] if argv else "sim"
    ep = resolve_endpoint(preset)
    b = connect(ep, choose_client(ep))
    b.start()
    brain = Brain(b, planner=ScriptedPlanner([]), period=0.0)
    print("输入动作调用，例如 move(vx=0.3)。空行退出。")
    try:
        for line in sys.stdin:
            text = line.strip()
            if not text:
                break
            name, kwargs = A.parse(text)
            aid, slots = A.encode(name, **kwargs)
            brain.planner = ScriptedPlanner([Decision(aid, tuple(slots), reason=text)])
            for out in brain.tick():
                print(out)
    finally:
        b.close()
    return 0
