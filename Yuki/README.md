# Yuki

仓库包含两个独立项目：

- `kernel/`：`yuki-kernel` 可嵌入 agent 内核，空白大脑，不含人格与业务。
- `example/`：`yuki` 示例聊天外壳，基于内核实现，保留会话、包管理、审批、热加载。

## 结构

```text
kernel/
  pyproject.toml
  src/yuki_kernel/         # 内核源码（events/memory/context/policy/providers/skills）
  tests/                   # 内核契约测试
example/
  pyproject.toml
  src/yuki/                # 示例聊天外壳
  packages/                # 外置工具包目录
  scripts/                 # 真实模型联调脚本
  tests/                   # 示例测试
docs/
  kernel.md                # 内核使用指南
  tools/                   # 工具系统开发指南
```

## 安装

```bash
pip install -e kernel
pip install -e example
```

## 运行示例

```bash
yuki
```

或直接用源码运行：

```bash
cd example
PYTHONPATH=../kernel/src:src .venv/bin/python -m yuki
```

`AGENT_PROVIDER` 可选 `openai`、`anthropic`；配置见 `example/.env.example`。

CLI 斜杠命令：

```text
/save <名字> /load <名字> /sessions /new /reload
/pkg install <目录|zip> /pkg remove <id> /pkg list
```

## 测试

```bash
cd kernel && PYTHONPATH=src ../.venv/bin/python -m pytest
cd example && PYTHONPATH=../kernel/src:src ../.venv/bin/python -m pytest
```
