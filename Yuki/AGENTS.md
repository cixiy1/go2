# Yuki 项目协作规范

这份文件同时写给人和 agent：agent 在改代码、提交前按这里执行；人类同事在 review、接手项目时也用它对齐预期。有冲突时，以这份文件为准。

## 项目结构

- `kernel/`：`yuki_kernel` 内核，空白大脑，可嵌入任何软件。
- `example/`：`yuki` 示例外壳，展示如何调用内核做完整聊天应用。
- `docs/`：内核与工具系统的使用文档。

## 怎么跑测试

内核和示例是两个独立包，开发前先 editable 安装到 `.venv`（源码原地可改，无需 PYTHONPATH）：

```bash
./.venv/bin/python -m pip install -e kernel
./.venv/bin/python -m pip install -e example
```

然后分别验证：

```bash
cd kernel && ../.venv/bin/python -m pytest -q
cd example && ../.venv/bin/python -m pytest -q
```

改动后测试必须全绿。

## 提交前检查

每次代码、文档或配置变更提交前，必须完成：

1. 跑上面的测试，确认全绿。
   - 必须自己实际运行测试并确认功能正常，再提交，不得跳过。
   - 任何变更（包括文档）提交前至少跑一次相关测试。
2. 跑 ruff 秒级快检，确认退出码为 0（拦截语法错误、未定义名、未用导入、盲异常等低级问题）：

```bash
cd kernel && ../.venv/bin/ruff check src tests
cd example && ../.venv/bin/ruff check src tests
```

   - ruff 装在仓库 `.venv`（`python -m pip install ruff`），随该 venv 使用，不写进项目依赖。
   - 全量默认规则，存量已由提交 `9144649`/`b42a63a` 清理；新时间戳统一带 `timezone.utc`（提交已修 DTZ005，避免跨时区错乱）。
3. 合并回 `main` 前，跑一次 PyCharm 同引擎语义级检查（Qodana，覆盖 IDE 级数据流/可达性 inspection，慢但保留语义覆盖）：

```bash
# Qodana 官方 CLI：同 PyCharm 引擎，社区版 linter，CI 标准，免费、不需要 token。
# 已实跑验证：native 模式（--within-docker false）不需要 Docker；
# 首次会下载 community linter 运行时（约 3.7G，入 ~/Library/Caches/JetBrains/Qodana 缓存，后续复用）。
# 注意：linter 名用短名 qodana-python-community，不要带 jetbrains/ 前缀（那是 docker 镜像名）。
# 注意：Professional 版（qodana-python）2023.2+ 要求 QODANA_TOKEN，本地无头跑不通，必须用 community。
# 注意：必须显式用 QODANA_PYTHON_PATH 指向项目 .venv（3.14）；否则 Qodana 默认挑
#   /usr/bin/python3（macOS 上为 3.9）当 SDK 语言级别，会把 PEP 604（X | None）/
#   PEP 585（list[str]）当非法注解误报（PyTypeHintsInspection 等），且 profile 里
#   没有 python 字段可配，只能走这个环境变量。
QODANA_PYTHON_PATH="$(pwd)/.venv/bin/python" qodana scan --linter qodana-python-community --within-docker false
```

   - **门槛判法不同于 inspect.sh**：Qodana 退出码 0 只代表「分析跑完」，不代表零问题。判干净要看报告里的问题数，或加 `--fail-threshold` 让超阈值非零退出。报告默认在 `~/Library/Caches/JetBrains/qodana-python-community/results/report`（HTML）。配对 `QODANA_PYTHON_PATH` 后本仓库为 0 problems。
   - 本地无 Docker/不想装 Qodana 时，退回 `inspect.sh` 无头检查（同引擎，慢在每次 `cp` 配置 + JVM 冷启）：

```bash
rm -rf /tmp/yuki-inspect /tmp/pycharm-config-yuki /tmp/pycharm-system-yuki /tmp/pycharm-log-yuki
mkdir -p /tmp/pycharm-system-yuki /tmp/pycharm-log-yuki
cp -R "$HOME/Library/Application Support/JetBrains/PyCharm2026.2" /tmp/pycharm-config-yuki
find /tmp/pycharm-config-yuki -name '*.lock' -delete
PYCHARM_VM_OPTIONS="$PWD/.pycharm-inspect.vmoptions" \
  /Applications/it/ide/PyCharm.app/Contents/bin/inspect.sh \
  "$PWD" "Project Default" /tmp/yuki-inspect \
  -d "$PWD/kernel/src" \
  -d "$PWD/kernel/tests" \
  -d "$PWD/example/src" \
  -d "$PWD/example/tests"
```

   - 该检查门槛是退出码 0，不是解析报告：退出 0 即「按 `Project Default` profile 跑这 4 个目录无 `<problem>` 产出」，不等同于 GUI Problems 视图零警告（GUI 默认 scope/severity 可能不同）。

## 提交格式

提交消息用 `类型: 描述`，中文描述，多个要点用分号连接：

```text
修复: 审批参数丢失，外部回调现在能拿到真实参数
文档: 重写内核使用指南，补充三种 Provider 接入方式
重构: 拆分 CLI 职责到 commands/approver/rendering
```

常用类型：`修复:`、`重构:`、`功能:`、`文档:`、`移除:`、`撤回:`。

默认不推送，只有明确要求时才推送。

## 分支与 main

- 所有代码、文档、配置变更都先在分支开发并提交，不在 `main` 直接提交；`main` 只接受合并。
- 内核相关改动（`kernel/` 源码与 `docs/kernel.md`）在 `kernel` 分支开发，合并用：

```bash
git merge -s ort -Xno-renames kernel
```

- 内核改动先在 `kernel` 分支跑 kernel 测试，合并回 `main` 后必须再跑 example 测试。
- 示例、文档、AGENTS.md 等其余改动开普通分支（如 `codex/*`）开发，用普通 merge 合回 `main`。
- `kernel` 分支是 `main` 去掉 `example` 的精简分支，根结构与 `main` 相同（`kernel/` 与 `main` 同路径），支持普通 `git merge`。
- 该合并会保留 `main` 的 `example/`。不要用 GitHub 默认 PR 合并这个分支，rename 检测会误报冲突。
- `main` 的 `kernel/` 更新后，需要从 `main` 去掉 `example` 重新生成 `kernel` 分支。

## 架构约定

改代码前先理解这几条边界，不要越过：

- 内核是空白大脑：不自动加载人格/身份，不读环境变量，不持有 `data/`、`packages/` 目录。
- 示例负责配置、UI 渲染、审批和进程生命周期，通过依赖调用内核。
- 渲染与内容清洗属于外部 UI 层；尽量保留模型原始数据，不做猜测性改写。
- 内核内置 `openai` / `anthropic` Provider；外部软件可以注册自定义 Provider，也可以直接注入实例。

## 结构风格

- 文件能分类则分类，目录不堆平铺。
- 单文件代码量尽量小，能分发则分发，能组装则组装。
- 内核与示例保持两套独立源码，示例通过依赖调用内核。
- 新增能力先放进现有目录，不另起平铺文件。

## 代码风格

- 保持简洁易读，优先使用项目现有模式和本地工具。
- 改动小而聚焦，不顺手重构无关代码。
- 公开 API 写类型标注；注释只解释不显然的逻辑。
- 不引入新依赖，除非确实需要。

## 文档维护

- 项目文档（README、docs/、AGENTS.md）要经常维护更新。
- 功能、配置、接口或流程规则变更后，必须同步更新对应文档，不能只改代码。
- 文档变更提交前同样跑测试和 PyCharm 无头检查，不得跳过。

## 工作区注意事项

- 不删除或回滚他人未要求的文件/改动；破坏性命令先确认。
- 不修改与任务无关的文件。
- 生成物不入库：`.venv/`、`data/`、`.idea/`、`uv.lock` 等。
