# yuki_kernel 使用指南

## 1. 内核是什么

`yuki_kernel` 是一个可嵌入的 agent 内核，定位是"空白大脑"：

- 不携带身份、人格或具体业务逻辑。
- 不读取环境变量，不依赖 `.env`、`data/`、`packages/` 等外部文件。
- 不要求固定厂商；内置 `openai` / `anthropic` 适配器，也允许外部软件自定义 Provider。

内核负责这些事：

- 推理编排：一次对话回合里的模型调用、工具闭环、包还原、记忆写入。
- 工具：内置工具、外置工具包、审批规则。
- 记忆：会话消息（短期）与长期记忆检索。
- 上下文：超限时自动压缩历史。
- 扩展：Provider、事件、中间件、错误重试。

外部软件负责：选择或实现 Provider、注入人格、决定会话和包目录、渲染 UI、管理进程生命周期。

## 2. 核心概念与工作原理

### 2.1 三个关键对象

- `Session`：一次会话。持有 `messages`（发给模型的消息列表）和 `approved_tools`（审批记忆）。
- `ChatChunk`：Provider 吐出的统一流式块。
- `StreamEvent`：`turn_stream()` 对外输出的事件，供外部软件渲染。

### 2.2 一次完整回合的流程

你调用 `agent.turn_stream("你好")`，内核内部依次执行：

```text
1. 把 user 消息追加进 session.messages
2. 检查上下文预算，超限先压缩历史
3. 调 provider.chat(...)，逐块消费 ChatChunk
   ├─ thinking   → 产出 StreamEvent(kind="thinking")
   ├─ content    → 产出 StreamEvent(kind="content")
   └─ tool_calls → 累积起来，产出 StreamEvent(kind="tool_calls")
4. 如果模型请求了工具，进入循环：
   ├─ 审批（command 类型或 requires_approval 的工具）
   ├─ 执行工具，拿到文本结果
   ├─ provider.build_tool_messages(...) 把结果翻译回厂商消息格式
   └─ 带着新消息再调模型，直到模型不再请求工具
5. 把外置包状态还原到回合开始时的快照
6. 把清洗后的 assistant 内容写回 session，并写入长期记忆（若启用）
7. 产出 StreamEvent(kind="done")
```

关键点：

- 工具闭环、上下文压缩、审批、包还原、记忆写入全部在内核内完成，外部软件不用自己做。
- 工具闭环不设轮次上限：模型请求工具就执行，结果回喂后继续，直到模型不再请求工具；模型按 `list_packages` 的结果直接调用具体工具，未加载的可用工具会自动加载。
- `turn_stream()` 发的是**原始事件**，不做渲染清洗；think 标签可能出现在事件里，由外壳决定怎么显示。
- 模型调用失败时内核按 `retry_max` 指数退避重试，工具异常会被转成字符串结果回喂模型，不让调用方崩溃。

## 3. 安装与导入

`yuki_kernel` 是一个标准 Python 包，按使用方的不同有三种消费方式。

### 3.1 作为已发布的包（面向互不相干的外部项目）

把内核发到包索引（内部 PyPI / Artifactory 最佳，必要才上公共 PyPI）后，使用方只声明依赖：

```bash
pip install yuki-kernel                   # 仅内核，不含任何厂商 SDK
pip install "yuki-kernel[openai]"          # 需要 OpenAI 兼容接口（智谱等）
pip install "yuki-kernel[anthropic]"       # 需要 Anthropic
pip install "yuki-kernel[openai,anthropic]"  # 两者都要
```

这是"被不同项目使用"的标准方式：版本化、隔离、零源码耦合。内核只对外承诺 §3.3 导出的
稳定 public API，内部模块不当契约。

### 3.2 从源码可编辑安装（本仓库 / 同组织兄弟项目）

开发期或同仓库内消费（例如 `example` 外壳），直接装源码：

```bash
cd kernel  && pip install -e .    # 内核
cd example && pip install -e .    # 示例外壳（依赖上面的内核）
```

可编辑安装把 `kernel/src` 加进解释器路径，改源码即时生效，**无需 `PYTHONPATH`**，
PyCharm / CLI / 测试 / Qodana 统一从 `.venv` 解析。本仓库约定：`kernel/` 与 `example/`
都从各自目录 `pip install -e .`。

### 3.3 导入

内核不对厂商 SDK 做强依赖：`import yuki_kernel` 不会拖入 `openai` / `anthropic`，
用到哪个 Provider 才在实例化时按需加载对应 SDK。未安装对应 extra 就创建该 Provider
会抛出清晰的导入错误，提示安装命令。

```python
from yuki_kernel import (
    Agent,
    App,
    ChatChunk,
    MemoryStore,
    PackageManager,
    Provider,
    SessionStore,
    Settings,
    ToolRegistry,
    TurnResult,
    create_provider,
    register_provider,
)
```

导入进来之后，真正"被不同项目使用"靠的是内核的运行时注入点（不修改内核源码）：
注册自定义 `Provider`、注入 `Environment`、或直接传实例。详见 §5 与 §7.0。
最小可运行示例见 §4。

## 4. 最小可运行示例

```text
import asyncio

from yuki_kernel import Agent, ChatChunk, Provider, Settings, ToolRegistry


class EchoProvider(Provider):
    def build_tool_messages(self, tool_calls, results):
        return []

    async def chat(self, messages, tools=None, **kwargs):
        yield ChatChunk(content="你好，我是回声。")


async def main():
    settings = Settings(provider="echo", model="echo")
    agent = Agent(
        settings.model,
        ToolRegistry(),
        settings,
        provider=EchoProvider(settings.model, settings),
    )
    result = await agent.turn("你好")
    print(result.content)


asyncio.run(main())
```

这个示例不联网，能直接跑通内核最小闭环。原理：

- `EchoProvider` 实现了 Provider 契约的两个方法：`chat` 和 `build_tool_messages`。
- `chat` 是异步生成器，`yield ChatChunk(content=...)` 模拟模型吐出一段回答。
- `provider=EchoProvider(...)` 直接传入实例，内核不会再去查注册表。
- `agent.turn("你好")` 完成"追加消息 → 调模型 → 收集结果"，返回 `TurnResult`。

## 5. Provider：内核与厂商之间的唯一接口

### 5.1 接口契约

实现 `Provider` 必须写两个方法：

```text
class MyProvider(Provider):
    def build_tool_messages(self, tool_calls, results) -> list[dict]:
        # 把"工具调用 + 工具结果"翻译成厂商要求的多轮消息
        ...

    async def chat(self, messages, tools=None, **kwargs):
        # 调用厂商 SDK，逐块翻译成 ChatChunk，用 yield 逐块吐出
        ...
```

- `chat` 必须是异步生成器（函数里有 `yield`），内核用 `async for chunk in provider.chat(...)` 逐块消费。生成器结束代表流结束。
- `build_tool_messages` 在模型请求工具后被调用，把工具结果翻译回厂商消息格式；永远不调用工具时返回 `[]` 即可。
- `start()` / `close()` 基类已有默认实现，不需要写；需要连接池时可以重写。

### 5.2 ChatChunk：统一的流块

```text
ChatChunk(
    thinking="思考过程",   # 思考内容
    content="正文片段",     # 回答内容
    tool_calls=[...],      # 工具调用
    done=False,            # 流结束信号（可选，生成器结束也代表流结束）
)
```

每个 `yield ChatChunk(...)` 等价于"模型吐出了一块内容"。内核只认这个结构，不关心它来自 HTTP 还是固定字符串，所以假 Provider 就能完整测试内核。

### 5.3 自定义厂商的真实转译示例

```text
# 假设厂商流式返回：{"choices": [{"delta": {"content": "你"}, "finish_reason": null}]}
class MyProvider(Provider):
    def build_tool_messages(self, tool_calls, results):
        # 厂商要求 assistant.tool_calls + role=tool 的消息
        return [
            {"role": "assistant", "content": None, "tool_calls": tool_calls},
            *[{"role": "tool", "content": result["content"]} for result in results],
        ]

    async def chat(self, messages, tools=None, **kwargs):
        async for raw in self.vendor.stream(messages, tools):
            choice = raw["choices"][0]
            delta = choice["delta"]
            yield ChatChunk(
                thinking=delta.get("reasoning_content"),
                content=delta.get("content"),
                tool_calls=delta.get("tool_calls") or [],
                done=bool(choice.get("finish_reason")),
            )
```

重试、错误归一由内核在抽象层处理，Provider 不需要自己实现。

### 5.4 注册：传类，不传实例

`register_provider` 存的是"如何创建 Provider"的工厂，不是现成实例：

```text
register_provider("my_provider", MyProvider)
```

- 传 `MyProvider` 这个类，注册那一刻**不会创建任何实例**。
- 之后 `create_provider("my_provider", model, settings)` 内部执行 `MyProvider(model, settings)`，这时才真正实例化。
- 因此类必须满足 `__init__(self, model, settings)`。

不能传实例的原因：实例必须在注册时就创建，那时还不知道 model 和 settings；而且同一个实例会被所有 Agent 共享，互相污染。类本身是可调用的，`MyProvider(model, settings)` 就是"按配方当场造一个"。

### 5.5 三种使用方式

下面三个方式共用同一个完整自定义 Provider `ZhipuProvider`：它的 Key 和地址都从 `Settings` 里读，构造时才初始化客户端。

```text
# 方式一：直接传实例（不经过注册表）
import asyncio

from openai import AsyncOpenAI

from yuki_kernel import Agent, ChatChunk, Provider, Settings, ToolRegistry


class ZhipuProvider(Provider):
    def __init__(self, model: str, settings: Settings):
        super().__init__(model, settings)
        self.client = AsyncOpenAI(
            api_key=settings.openai_api_key,       # Key 从 Settings 读
            base_url=settings.openai_base_url,     # 地址从 Settings 读
        )

    async def chat(self, messages, tools=None, **kwargs):
        stream = await self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            tools=tools,
            stream=True,
        )
        async for chunk in stream:
            choice = chunk.choices[0]
            delta = choice.delta
            yield ChatChunk(
                thinking=getattr(delta, "reasoning_content", None),
                content=delta.content,
                tool_calls=list(delta.tool_calls or []),
                done=bool(choice.finish_reason),
            )

    def build_tool_messages(self, tool_calls, results):
        return [
            {"role": "assistant", "content": None, "tool_calls": tool_calls},
            *[{"role": "tool", "content": result["content"]} for result in results],
        ]

    async def close(self, skip_unload=False):
        await self.client.close()


async def main():
    settings = Settings(
        provider="my_provider",                     # 方式一直接传实例，这个名字不会被用到
        model="glm-4-flash",
        openai_api_key="你的智谱Key",
        openai_base_url="https://open.bigmodel.cn/api/paas/v4/",
    )
    registry = ToolRegistry()
    agent = Agent(
        settings.model,
        registry,
        settings,
        provider=ZhipuProvider(settings.model, settings),   # 直接 new 实例
    )
    await agent.start()
    try:
        result = await agent.turn("你好")
        print(result.content)
    finally:
        await agent.close()


asyncio.run(main())
```

方式一绕过注册表，实例由调用方创建；Key 和地址都写在 `Settings` 里，`ZhipuProvider.__init__` 构造时读取并初始化客户端。

**Settings 值传递规则**

- 类从 `settings` 读什么字段，`Settings` 就填什么值；不读的字段保持默认即可。
- 例如类读 `settings.openai_api_key` / `settings.openai_base_url`，`Settings` 里就要填这两个值。
- 类自己从环境变量、常量或配置文件拿值，`Settings` 就不用填。
- `Settings` 没有的字段，类需要时自己解决：注册时闭包注入、类内读环境变量/配置文件、或扩展 `Settings`。

```text
# 方式二：注册名字，让内核按名字创建
from yuki_kernel import Agent, Settings, ToolRegistry, register_provider

# 复用方式一里定义的 ZhipuProvider
register_provider("my_provider", ZhipuProvider)
settings = Settings(
    provider="my_provider",
    model="glm-4-flash",
    openai_api_key="你的智谱Key",
    openai_base_url="https://open.bigmodel.cn/api/paas/v4/",
)
registry = ToolRegistry()

agent = Agent(settings.model, registry, settings, provider=settings.provider)
```

`provider=settings.provider` 等价于直接写 `provider="my_provider"`，因为 `settings.provider` 的值就是 `"my_provider"`。写成 `settings.provider` 只是让名字只在 `Settings` 里出现一次，避免配置和调用两处不一致；小脚本里直接写死字符串也可以。

`_PROVIDERS` 默认只有 `openai` / `anthropic` 两个名字；示例第一行 `register_provider("my_provider", ZhipuProvider)` 就是在运行时把 `my_provider` 加进注册表。不注册直接使用这个名字会抛 `ValueError`。

方式二里 Key 和地址同样放在 `Settings`，因为 `create_provider` 会调用 `ZhipuProvider(model, settings)`，构造函数自己从 `settings` 读取。

`Settings` 只有 `openai_*` 和 `anthropic_*` 两套凭据字段。如果类的构造函数还需要 `(model, settings)` 之外的额外参数（例如 Settings 里不存在的 `vendor_secret`），有两种做法，任选其一。

做法一：lambda 注入。注册时把类包进 lambda，从环境变量取出凭据，作为额外参数传进构造函数：

```text
class VendorProvider(Provider):
    def __init__(self, model, settings, vendor_secret):   # 多收一个 Settings 里没有的参数
        super().__init__(model, settings)
        self.secret = vendor_secret

    # chat / build_tool_messages 与方式一的 ZhipuProvider 相同，这里省略

register_provider(
    "my_provider",
    lambda model, settings: VendorProvider(
        model,
        settings,
        vendor_secret=os.environ["MY_VENDOR_SECRET"],   # lambda 从环境变量取
    ),
)
```

做法二：类自己拿。构造函数保持 `(model, settings)`，在类内部读环境变量或配置文件：

```text
class VendorProvider(Provider):
    def __init__(self, model, settings):   # 不需要额外参数
        super().__init__(model, settings)
        self.secret = os.environ["MY_VENDOR_SECRET"]   # 类自己读环境变量

    # chat / build_tool_messages 与方式一的 ZhipuProvider 相同，这里省略
```

两种做法任选其一，都能跑：

- 做法一：构造函数显式收参数，类不接触环境变量，测试时方便传假值。
- 做法二：代码更少，但类直接依赖环境变量或配置文件。

"内核不读环境变量"只约束内核本身，外部代码不受限。

```text
# 方式三：使用内置 openai / anthropic，无需注册
from yuki_kernel import Agent, Settings, ToolRegistry

settings = Settings(
    provider="openai",
    model="glm-4-flash",
    openai_api_key="你的Key",
    openai_base_url="https://open.bigmodel.cn/api/paas/v4/",
)
registry = ToolRegistry()

agent = Agent(settings.model, registry, settings, provider=settings.provider)

# 用 anthropic 时换成对应字段：
settings = Settings(
    provider="anthropic",
    model="claude-3-5-sonnet",
    anthropic_api_key="你的Key",
    anthropic_base_url="https://api.anthropic.com",
)
agent = Agent(settings.model, registry, settings, provider=settings.provider)
```

方式三不需要 `register_provider`，因为 `openai` 和 `anthropic` 在导入内核时就已经在注册表里。`Settings` 里的 API Key 和地址只负责喂给对应 Provider 构造。

重要：`Agent` 的 `provider` 参数默认是 `"openai"`，**不会自动读取 `settings.provider`**。直接构造 `Agent` 时必须显式传 `provider=settings.provider`，否则会创建 OpenAI Provider；只有 `App` 组装时才会自动替你传。

### 5.6 内置 Provider

- `openai`：OpenAI 兼容接口（智谱等），支持 thinking、content、tool_calls。
- `anthropic`：Anthropic，完整支持文本、thinking、工具调用。

SDK 懒加载：内核 import 本身不依赖 `openai` / `anthropic`，只有实例化对应 Provider 时才导入。

实现参考：

- [openai.py](../kernel/src/yuki_kernel/providers/openai.py)
- [anthropic.py](../kernel/src/yuki_kernel/providers/anthropic.py)

## 6. 调用方式：三层接口

### 6.1 方式一：`App` 组合完整应用

`App` 是组装容器，负责把 Settings、SessionStore、PackageManager、ToolRegistry、Agent 拼起来，支持配置热加载：

```text
from pathlib import Path

from yuki_kernel import App, PackageManager, SessionStore, Settings

settings = Settings(
    provider="openai",
    model="glm-4-flash",
    openai_api_key="你的Key",
    openai_base_url="https://open.bigmodel.cn/api/paas/v4/",
    packages_dir=Path("packages"),
    packages_preload=["yuki_persona"],
    data_dir=Path("data"),
)

store = SessionStore(settings.data_dir)
package_manager = PackageManager(settings.packages_dir)
app = App(settings, store, package_manager)

await app.agent.start()
try:
    async for event in app.agent.turn_stream("你好"):
        if event.kind == "content":
            print(event.text, end="", flush=True)
finally:
    await app.agent.close()
```

`App` 内部自动完成：扫描外置包、预加载 `packages_preload`、把 `MemoryStore` 接到 `search_memory` 工具、用 `settings.provider` 创建 Provider。

热加载：

```text
new_settings = load_settings()   # 外部软件重新构造 Settings
await app.reload(new_settings)   # 重建 registry / agent / memory_store，保留当前会话并关闭旧 provider
```

### 6.2 方式二：`Agent.turn()` / `turn_stream()`（推荐）

这两个方法完整跑完第 2 节的整个回合流程，外部软件只需要消费结果。

无头回合：

```text
result = await agent.turn("你好")
print(result.content)
print(result.tool_calls)
print(result.changed_packages)
```

流式回合：

```text
async for event in agent.turn_stream("你好"):
    if event.kind == "thinking":
        print("思考：", event.text)
    elif event.kind == "content":
        print(event.text, end="", flush=True)
    elif event.kind == "tool_calls":
        print("工具调用：", event.calls)
    elif event.kind == "tool_result":
        print("工具结果：", event.text)
    elif event.kind == "package_restored":
        print("外置包已还原：", event.text)
```

事件类型：

- `thinking`：思考片段
- `content`：正文片段
- `tool_calls`：模型请求的工具调用
- `tool_result`：工具执行结果
- `package_restored`：外置包还原
- `done`：回合结束

### 6.3 方式三：底层手动流（自己控制工具闭环）

想完全掌控每一步时，可以不用 `turn_stream`，自己组合：

```text
calls: list = []
async for chunk in agent.send_message("北京天气"):
    if chunk.content:
        print(chunk.content, end="", flush=True)
    if chunk.tool_calls:
        calls.extend(chunk.tool_calls)

while calls:
    round_calls = calls
    calls = []
    results = await agent.execute_tool_calls(round_calls)
    for result in results:
        print(f"工具结果：{result['content']}")
    async for chunk in agent.continue_with_tools(round_calls, results):
        if chunk.content:
            print(chunk.content, end="", flush=True)
        if chunk.tool_calls:
            calls.extend(chunk.tool_calls)
```

注意：手动方式不会自动做回合结束时的包还原和 assistant 记忆写入，这些需要自己处理。多数场景用 `turn()` / `turn_stream()` 就够。

### 6.4 三个接口怎么选

| 场景 | 推荐接口 |
| --- | --- |
| 完整应用（包、记忆、热加载） | `App` |
| 无头服务/测试 | `Agent.turn()` |
| CLI / GUI 流式显示 | `Agent.turn_stream()` |
| 深度定制工具循环 | 手动 `send_message` + `execute_tool_calls` + `continue_with_tools` |

## 7. 工具

### 7.0 工具执行环境（沙箱）

外置包可能执行任意 Python 代码或系统命令。内核默认不再在进程内 `exec` 任意代码：
**python 工具也经子进程执行**，与 `command` 入口、内置 `terminal` 工具共用**同一个**
宿主注入的 `Environment`，由它统一决定「命令在什么环境里跑」。工具本身不构造环境——
`terminal` 不自建沙箱，python/command 工具也走这同一层。

`Environment` 同时承载两个面（A 方案合成一个对象，宿主注册内核时一次性注入）：

- **隔离面（`run`）**：命令在何种 OS 隔离下执行（降权、资源阀、网络/文件范围），内部委托 `Sandbox` 实现。
- **运行面（`python_path` / `base_env`）**：用哪个 Python 解释器、基础环境变量（venv 的 `PATH`/`PYTHONPATH` 等），供 python 工具与命令解析使用。

默认 `BasicEnvironment` 只是开发兜底：当前用户、宿主完整环境、标准库 `BasicSandbox` 资源阀
（CPU/内存/单文件上限），`python_path` 为当前解释器、`base_env` 为空。生产应由宿主注入更强的
`Environment`（venv / 容器 / Seatbelt / Landlock）。`BasicEnvironment` 可这样定制：

```text
from yuki_kernel import App, Settings, ToolRegistry
from yuki_kernel.skills import BasicEnvironment, BasicSandbox, SandboxConfig

# 托管给外部软件创建的虚拟环境，并叠加降权与资源阀
env = BasicEnvironment(
    sandbox=BasicSandbox(SandboxConfig(user="nobody", cpu_seconds=10)),
    python_path="/path/to/venv/bin/python",
    base_env={"PATH": "/path/to/venv/bin:/usr/bin", "VIRTUAL_ENV": "/path/to/venv"},
)

settings = Settings(provider="openai", model="glm-4-flash", openai_api_key="你的Key", environment=env)
# App 会自动把 settings.environment 透传给 ToolRegistry；也可直接：
registry = ToolRegistry(packages_dir="packages", environment=env)
```

执行时的环境规则：

- 内置工具（含 `terminal`）继承宿主环境并叠加 `base_env`：`{**base_env, **os.environ}`；
  外置包工具只给 `base_env`（无宿主泄漏）。
- python 工具用 `environment.python_path` 启动解释器，从而跑在注入的 venv 里。
- 资源阀与可选降权由内部 `BasicSandbox` 提供：CPU 时间、地址空间、单文件写入上限；
  降权用户在构造时校验，不存在抛 `ValueError`。

注意：子进程默认继承当前用户权限；`setuid` 到别的 uid（如 `nobody`）**只有 root 能成功**，
普通用户下会失败。普通用户想要真正降权，须把整个内核跑在容器 / Seatbelt / Landlock 等
宿主隔离里，或注入实现了强降级的自定义 `Environment`。

沙箱降低的是「工具能干什么」的上限，不是替代宿主对包来源的信任审查——仍只加载你信任的包。
审批（见 7.4 节）是「是否放行」闸门，环境是「能干什么」阀门，两者叠加使用。

### 7.1 工具注册表

```text
registry = ToolRegistry(
    packages_dir,           # 外置包目录；不传则不扫描
    available=["weather"],  # 白名单；不传表示目录下全部可用
    preload=["weather"],    # 启动即加载的包
)
```

内核扫描与加载不打印任何内容，相关数据通过注册表暴露给外部软件：

- `scan_packages()` 返回 `PackageScan`（`packages` 为发现的包，`available` 为白名单后可用包，`skipped` 为跳过原因），并同时存入 `registry.package_scan`。
- `registry.available_packages` 返回结构化包列表（`id / name / description / tools / prompts / loaded`）。
- 预加载的包在 `available_packages` 中 `loaded=True`；展示由外部软件负责。

注册表自带内置工具和提示词：

- `get_environment_info`：获取操作系统、Python 版本、工作目录等环境信息。
- `environment_guide`：命令生成前的环境信息使用提示。

### 7.2 注册自己的工具

工具必须能通过 `entry` 执行。函数式工具放在一个 `.py` 文件里，`entry` 指向 module + handler：

```text
registry.register_tool({
    "name": "add",
    "description": "两个整数相加",
    "parameters": {
        "type": "object",
        "required": ["a", "b"],
        "properties": {
            "a": {"type": "integer", "description": "第一个数"},
            "b": {"type": "integer", "description": "第二个数"},
        },
    },
    "entry": {
        "type": "python",
        "module": "my_tools.py",
        "handler": "add",
    },
    "package": "my_project",
    "package_dir": "/path/to/my_tools",
})
```

`my_tools.py`：

```text
def add(a: int, b: int) -> str:
    return str(a + b)
```

工具执行约定：

- `python` 入口：加载模块，调用 handler；handler 是类时实例化并调用 `method`（默认 `run`）。
- `command` 入口：子进程执行命令，参数通过 stdin 以 JSON 传入，30 秒超时，`{python}` 替换为当前 Python 解释器。
- 工具返回值必须是字符串；异常会被内核捕获并转成 `工具执行失败：...` 回喂模型。

### 7.3 外置工具包

外置包是一个含 `manifest.json` 的目录，可以声明：

- 工具（python / command 入口）
- 提示词（纯文本，动态进入 system prompt）

```text
registry.activate_package("weather")     # 加载，工具和提示词进入上下文
registry.deactivate_package("weather")   # 卸载，释放上下文
```

manifest 完整格式见 [docs/tools/manifest.md](tools/manifest.md)。

注册表自带四个元工具，模型可以按需调用：

- `list_packages`：列出可用外置包
- `load_package` / `unload_package`：加载/卸载
- `search_memory`：检索长期记忆

可用包里的工具可以直接调用：内核会在执行前自动加载对应包，本轮结束后还原到本轮开始前的状态。

包扫描结果通过 `registry.package_scan` 暴露给外部软件（`packages` 为发现的包，`available` 为可用包，`skipped` 为跳过原因），展示由外部软件负责。

外部软件用 `package_scan` 与 `available_packages` 自行渲染，例如 example 启动时会输出：

```text
发现外置工具包：echo
发现外置工具包：weather
发现外置工具包：writing_style
发现外置工具包：yuki_persona
可用外置工具包：echo、weather、writing_style、yuki_persona
已加载 yuki_persona：工具 无；提示词 yuki_identity
```

### 7.4 审批

满足以下任一条件时，工具执行前会走审批：

- `entry.type == "command"`
- manifest 或工具声明了 `requires_approval: true`

审批器是一个异步回调：

```text
async def my_approver(name: str, arguments: dict) -> str:
    answer = await ask_user(f"工具 {name} 需要审批 (y / ya / y <分钟> / n)：")
    return answer.strip()

agent = Agent(..., approver=my_approver)
```

返回值语义：

- `y`：本次放行
- `ya`：本会话内一直放行
- `y <分钟>`：本会话内放行 N 分钟
- 其它：拒绝，模型收到 `用户拒绝执行`

审批记忆存在 `Session.approved_tools`，会话切换后失效。

## 8. 会话与记忆

### 8.1 会话：换记忆

```text
from pathlib import Path

from yuki_kernel import SessionStore

# 导出当前会话为 JSONL
SessionStore.export(session, Path("robot.jsonl"))

# 从 JSONL 导入，生成新会话
loaded = SessionStore.import_file(Path("robot.jsonl"), name="robot")

# 给 Agent 换记忆
agent.switch_session(loaded)
```

`switch_session` 会替换 `session.messages` 并重建 system prompt，因此"换记忆"是内核原生能力。

### 8.2 会话持久化

```text
store = SessionStore(Path("/your/data"))

new_session = store.create("第一次对话")
store.save(new_session)              # JSONL 消息 + SQLite 索引
loaded = store.load(new_session.session_id)
metas = store.list_sessions()        # 按更新时间倒序
```

`save / load / list_sessions` 使用 JSONL + SQLite 索引，目录完全由外部决定；新会话默认只存在于内存，`save` 才落盘。

### 8.3 长期记忆

```text
from yuki_kernel import MemoryStore

memory = MemoryStore(Path("/your/data"), namespace="robot_a")
memory.add(session_id, "用户：你好\n助手：你好呀")
hits = memory.search("你好", limit=5)
```

- 按 `namespace` 隔离，多个机器人可以共用内核。
- 检索是关键词匹配（英文按词、中文按双字窗口），不是向量检索。
- 模型通过 `search_memory` 工具按需检索，不自动注入全部历史。
- 使用 `App` 时，`MemoryStore.search_text` 会自动接到 `search_memory` 上。

## 9. 上下文管理

每次调模型前，内核会估算 token 用量：

```text
每条消息 ≈ max(1, ceil(len(content) / 3)) + 4
```

超过 `max_context_tokens` 时：

1. 保留最近的 `keep_recent_messages` 条消息。
2. 把更早的消息交给模型，压缩成一条 `[历史摘要]` system 消息。
3. 摘要失败时退化为直接丢弃最早的一条非 system 消息。

```text
settings = Settings(
    ...,
    max_context_tokens=12000,
    keep_recent_messages=10,
)
```

`max_context_tokens <= 0` 表示不限制。

## 10. 事件、中间件与钩子

### 10.1 事件

```text
from yuki_kernel.core.events import EventBus

bus = EventBus()
bus.subscribe("assistant_chunk", lambda event: print("chunk:", event.payload))

agent = Agent(..., bus=bus)
```

事件类型：

```text
user_message / before_model / assistant_chunk / tool_call / tool_result
session_start / session_end / package_load / package_unload
```

`EventBus` 只做观测广播，不参与控制流。

### 10.2 中间件：可以改写或中止

```text
from yuki_kernel.core.events import Middleware


class Gate(Middleware):
    async def before(self, event):
        if event.kind == "user_message":
            event.context["abort"] = True   # 中止这一轮
        return event

    async def after(self, event):
        print("after", event.kind)


agent = Agent(..., middlewares=[Gate()])
```

- `before` 可以改写事件或通过 `event.context["abort"] = True` 中止流程。
- `after` 只做收尾，不能中止。
- `user_message` 和 `tool_call` 支持 abort。

## 11. 错误与重试

异常层级：

```text
AgentError
├── ProviderError    # 模型调用失败
└── ToolError        # 工具执行失败
```

模型调用失败时，内核按瞬时错误判断重试：

- `ProviderError`、`ConnectionError`、`TimeoutError`、`OSError`
- 厂商 SDK 的 API 错误且 `status_code >= 500`

```text
settings = Settings(
    ...,
    retry_max=3,       # 最多尝试次数
    retry_base=0.5,    # 退避基数：0.5s, 1s, 2s, ...
)
```

4xx 错误不重试。工具执行异常不会抛给调用方，而是变成 `工具执行失败：...` 文本回喂模型。

## 12. Settings 全部字段

| 字段 | 必填 | 含义 |
| --- | --- | --- |
| `provider` | 是 | Provider 名字或自定义注册名 |
| `model` | 是 | 模型名 |
| `think` | 否 | 是否启用思考（默认 True） |
| `openai_api_key` | 否 | OpenAI 兼容接口 Key |
| `openai_base_url` | 否 | OpenAI 兼容接口地址 |
| `anthropic_api_key` | 否 | Anthropic Key |
| `anthropic_base_url` | 否 | Anthropic 地址 |
| `packages_dir` | 否 | 外置包目录 |
| `packages` | 否 | 可用包白名单，空列表表示全部可用 |
| `packages_preload` | 否 | 启动即加载的包 |
| `max_context_tokens` | 否 | 上下文预算，默认 12000，<=0 不限制 |
| `keep_recent_messages` | 否 | 压缩时保留的最近消息数，默认 10 |
| `memory_limit` | 否 | 长期记忆检索条数，默认 5 |
| `namespace` | 否 | 长期记忆命名空间，默认 default |
| `retry_max` | 否 | 模型调用重试次数，默认 3 |
| `retry_base` | 否 | 重试退避基数（秒），默认 0.5 |
| `data_dir` | 否 | 会话与长期记忆数据目录 |

内核不读取环境变量，`Settings` 必须由外部软件构造。

## 13. 常见错误排查

**Agent 创建出来不是我的 Provider**

原因：`Agent` 的 `provider` 参数默认是 `"openai"`，不会自动读 `settings.provider`。

```text
# 错
agent = Agent(settings.model, registry, settings)

# 对
agent = Agent(settings.model, registry, settings, provider=settings.provider)
```

**`register_provider` 后报"未注册的 provider"**

原因：注册名与 `Settings(provider=...)` 不一致，或注册发生在创建之后。确认拼写一致，并在创建前注册。

**`register_provider("x", my_instance)` 报错**

原因：注册口要的是"创建函数"，不是实例。传类或 `lambda model, settings: ...`。

**自定义 Provider 报 NotImplementedError**

原因：`Provider` 有两个抽象方法 `chat` 和 `build_tool_messages`，必须都实现；不调用工具时 `build_tool_messages` 返回 `[]` 即可。

**没有外置工具包**

原因：`packages_dir` 为 `None` 或目录下没有 `manifest.json`。检查目录和 manifest 格式（见 [docs/tools/manifest.md](tools/manifest.md)）。

**加载会话后模型答非所问**

原因：会话切换后内核会重建 system prompt；如果加载的 JSONL 里没有对应的人格/提示词，需要外部软件重新注入（例如预加载 `yuki_persona` 包）。

## 14. 示例项目参考

仓库里的 `example/` 是一个完整的参考实现：

- `example/src/yuki/__main__.py`：入口组装 + 热加载 + 生命周期清理。
- `example/src/yuki/cli.py`：主循环与斜杠命令分发。
- `example/src/yuki/rendering.py`：流式渲染，直接输出模型原始数据。
- `example/src/yuki/commands/`：会话与包管理命令。
- `example/src/yuki/approver.py`：CLI 审批实现。
- `example/src/yuki/settings.py`：从环境变量构造内核 Settings。

```bash
cd example
PYTHONPATH=../kernel/src:src python -m yuki
```

示例展示了"外部软件如何调用内核"，内核本身不包含这些 UI 和业务决策。
