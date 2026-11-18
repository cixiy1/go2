# manifest.json 字段参考

每个外置工具包是一个包含 `manifest.json` 的目录。manifest 声明包的元信息、工具和提示词，
agent 启动时读取它，真正执行工具时才加载代码。

## 目录约定

```text
example/packages/
  <package_id>/
    manifest.json
    tool.py            # 可选：python 入口
    scripts/...        # 可选：command 入口
    skill.md           # 可选：提示词正文
```

## 最外层字段

| 字段          | 必填 | 类型   | 说明                               |
| ------------- | ---- | ------ | ---------------------------------- |
| `id`          | 是   | string | 包的唯一标识，用于日志和冲突判断   |
| `name`        | 否   | string | 包的人类可读名称，默认等于 `id`    |
| `version`     | 否   | string | 包版本号                           |
| `description` | 否   | string | 包简介，给人看，不给模型看         |
| `tools`       | 否   | array  | 工具列表，纯提示词包可以省略       |
| `prompts`     | 否   | array  | 提示词列表，没有提示词的包可以省略 |

## tools[] 字段

| 字段                | 必填 | 类型   | 说明                                         |
| ------------------- | ---- | ------ | -------------------------------------------- |
| `name`              | 是   | string | 工具名，模型调用时使用，包内和跨包都不能重复 |
| `description`       | 是   | string | 工具说明，会发给模型                         |
| `parameters`        | 是   | object | 参数结构，JSON Schema                        |
| `entry`             | 是   | object | 工具执行入口                                 |
| `requires_approval` | 否   | bool   | 为 `true` 时执行前需要用户审批               |

## entry 字段

`entry.type` 决定执行方式，二选一。

### python 入口

| 字段      | 必填 | 类型   | 说明                                     |
| --------- | ---- | ------ | ---------------------------------------- |
| `type`    | 是   | string | `"python"`                               |
| `module`  | 是   | string | 包内 Python 文件路径，例如 `tool.py`     |
| `handler` | 是   | string | 模块里的函数名，或类名                   |
| `method`  | 否   | string | `handler` 是类时调用的方法名，默认 `run` |

调用时会把模型参数作为关键字参数传入 handler；`handler` 是函数时直接调用，
是类时会先实例化，再调用 `method` 指定的方法（默认 `run`），返回值转成字符串。

类入口示例：

```python
class Greeter:
    def __init__(self):
        self.prefix = "hi"

    def run(self, name):
        return f"{self.prefix} {name}"
```

```json
{
  "entry": {
    "type": "python",
    "module": "tool.py",
    "handler": "Greeter",
    "method": "run"
  }
}
```

### command 入口

| 字段      | 必填 | 类型          | 说明                                                 |
| --------- | ---- | ------------- | ---------------------------------------------------- |
| `type`    | 是   | string        | `"command"`                                          |
| `command` | 是   | array[string] | 要执行的命令，例如 `["{python}", "scripts/echo.py"]` |

`{python}` 会被替换为当前 Python 解释器路径。调用时模型参数以 JSON 写入 stdin，
程序把结果打印到 stdout，非零退出码视为执行失败。

`command` 入口的工具默认需要审批；`python` 工具只有在 `requires_approval: true`
时才需要审批。

## parameters 字段

`parameters` 是 JSON Schema，常见字段：

| 字段                            | 说明                                             |
| ------------------------------- | ------------------------------------------------ |
| `type`                          | 参数整体类型，函数参数一般用 `"object"`          |
| `required`                      | 必填参数名数组                                   |
| `properties`                    | 具体参数定义，每个键是一个参数名                 |
| `properties.<name>.type`        | 参数类型，如 `"string"`、`"number"`、`"boolean"` |
| `properties.<name>.description` | 参数说明，帮助模型正确传值                       |

## prompts[] 字段

| 字段          | 必填 | 类型   | 说明                                        |
| ------------- | ---- | ------ | ------------------------------------------- |
| `name`        | 是   | string | 提示词名，包内不能重复                      |
| `description` | 是   | string | 提示词描述，说明什么时候应该生效            |
| `path`        | 是   | string | 提示词文件路径，相对包目录，例如 `skill.md` |

提示词内容会被注入系统消息。

## 完整示例

### python 工具 + 提示词

```json
{
  "id": "weather",
  "name": "天气工具包",
  "version": "1.0.0",
  "description": "查询指定城市当前气温",
  "tools": [
    {
      "name": "weather_now",
      "description": "查询指定城市当前气温",
      "parameters": {
        "type": "object",
        "required": ["city"],
        "properties": {
          "city": {
            "type": "string",
            "description": "城市英文名，例如 New York"
          }
        }
      },
      "entry": {
        "type": "python",
        "module": "tool.py",
        "handler": "weather_now"
      }
    }
  ],
  "prompts": [
    {
      "name": "weather_guide",
      "description": "天气查询工具的补充使用说明",
      "path": "skill.md"
    }
  ]
}
```

### command 工具

```json
{
  "id": "echo",
  "name": "回声工具包",
  "version": "1.0.0",
  "description": "演示 command 类型的工具入口",
  "tools": [
    {
      "name": "echo_text",
      "description": "把一段文本原样返回",
      "parameters": {
        "type": "object",
        "required": ["text"],
        "properties": {
          "text": {
            "type": "string",
            "description": "要返回的文本"
          }
        }
      },
      "entry": {
        "type": "command",
        "command": ["{python}", "scripts/echo.py"]
      }
    }
  ]
}
```

### 纯提示词包

```json
{
  "id": "writing_style",
  "name": "写作风格包",
  "version": "1.0.0",
  "description": "让回答保持简洁",
  "prompts": [
    {
      "name": "concise_zh",
      "description": "回答保持简洁、直白",
      "path": "skill.md"
    }
  ]
}
```

## 校验规则

- `manifest.json` 必须是合法 JSON，文件缺失或解析失败会跳过整个包。
- 必填字段缺失、`entry.type` 不支持、提示词文件不存在，都会跳过整个包。
- 工具名或提示词名与已加载项冲突时，跳过整个包。
- 跳过时会在启动日志里打印原因，不影响其他包加载。
- `list_packages`、`load_package`、`unload_package` 是保留工具名，外置包不能用。

## 按需加载

外置包默认只被发现不加载。模型通过三个内置目录工具控制上下文占用：

- `list_packages`：查看可用包及其工具、提示词
- `load_package`：把某个包的工具和提示词加入上下文
- `unload_package`：把某个包从上下文卸载

可用包由 `AGENT_PACKAGES` 白名单限制；`AGENT_PACKAGES_PRELOAD` 可以让包启动时就加载。
模型也可以直接调用可用包里的工具，内核会先自动加载对应包再执行；每轮结束后仍会还原。
每轮对话结束后，外置包状态会自动还原到本轮开始前，临时加载的包会被卸载。

## 内置工具

内置工具不需要 manifest，注册表写在 `kernel/src/yuki_kernel/skills/builtin.py`，
实现代码放在 `kernel/src/yuki_kernel/skills/builtins/`。

`BUILTIN_TOOLS` 里的工具结构与外置包一致：

| 字段          | 说明                                                  |
| ------------- | ----------------------------------------------------- |
| `name`        | 工具名                                                |
| `description` | 工具说明                                              |
| `parameters`  | JSON Schema 参数结构                                  |
| `entry`       | 执行入口，`type` 为 `python` 或 `command`，格式见上文 |

`BUILTIN_PROMPTS` 里的提示词结构：

| 字段          | 说明                                                |
| ------------- | --------------------------------------------------- |
| `name`        | 提示词名                                            |
| `description` | 提示词描述                                          |
| `path`        | 相对 `kernel/src/yuki_kernel/skills/` 的 Markdown 文件路径 |

内置提示词会被注入系统消息，与外置包的 `prompts` 行为一致。

### 工具函数 + 使用说明提示词示例

工具函数的使用说明可以独立成一个提示词，和工具配套注册，但内置提示词常驻上下文，
建议只在需要时启用：

```python
BUILTIN_TOOLS = [
    {
        "name": "get_environment_info",
        "description": "获取当前操作系统、Python 版本、工作目录等环境信息",
        "parameters": {
            "type": "object",
            "properties": {},
        },
        "entry": {
            "type": "python",
            "module": "builtins/environment.py",
            "handler": "get_environment_info",
        },
    },
]

BUILTIN_PROMPTS = [
    {
        "name": "environment_guide",
        "description": "命令生成前的环境信息使用提示",
        "path": "builtins/prompts/environment_guide.md",
    },
]
```
