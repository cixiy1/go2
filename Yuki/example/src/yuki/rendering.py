"""Yuki 外壳的流式渲染：直接输出模型的原始数据。"""

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App


async def render_turn(app: App, line: str) -> None:
    state = "initial"

    def switch_state(next_state: str, label: str) -> None:
        nonlocal state
        if state != next_state:
            if state != "initial":
                print()
            print(label, end="")
            state = next_state

    async for event in app.agent.turn_stream(line):
        if event.kind == "thinking":
            if event.text:
                switch_state("thinking", "思考：")
                print(event.text, end="", flush=True)
        elif event.kind == "tool_calls":
            switch_state("tool_calling", "工具调用：")
            print(event.calls, end="", flush=True)
        elif event.kind == "content":
            if event.text:
                switch_state("ans", "回答：")
                print(event.text, end="", flush=True)
        elif event.kind == "tool_result":
            print(f"\n工具结果：{event.text}")
            state = "initial"
        elif event.kind == "package_restored":
            print(f"\n外置包已还原：{event.text}")
            state = "initial"
    print()
