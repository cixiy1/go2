# 外置工具包

外置工具包是放在 `packages/` 目录下的独立文件夹，每个包通过 `manifest.json`
声明自己的工具和提示词。启动时自动加载，删除目录即可禁用。

当前示例：

- `weather/`：python 工具 + 提示词
- `echo/`：command 命令工具
- `writing_style/`：纯提示词包
- `yuki_persona/`：Yuki 身份人格包（内核本身不携带身份）

完整字段说明和开发规范见 [docs/tools/manifest.md](../docs/tools/manifest.md)。
