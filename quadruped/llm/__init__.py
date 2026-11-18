"""文本条件层：一句话变成策略命令。

    quadruped/llm/schema.py   模型输出 → 合法命令字段
    quadruped/llm/prompt.py   系统提示词
    quadruped/llm/model.py    本机 MLX 模型
    quadruped/llm/service.py  自驱循环

分层的原因：模型输出不可信，必须有一层把「任何东西」收成「合法命令」。这一层
和模型分开，换模型（Ollama / 别的权重）时只改 `model.py`。
"""

from quadruped.llm.model import DEFAULT_MODEL, LocalModel
from quadruped.llm.service import TextService

__all__ = ["DEFAULT_MODEL", "LocalModel", "TextService"]
