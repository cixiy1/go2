"""内核异常与瞬时错误判断。"""


class AgentError(Exception):
    """内核基础异常。"""


class ProviderError(AgentError):
    """模型调用失败。"""


class ToolError(AgentError):
    """工具执行失败。"""


def is_transient(err: Exception) -> bool:
    """判断是否值得重试的瞬时错误。"""
    if isinstance(err, (ProviderError, ConnectionError, TimeoutError, OSError)):
        return True
    name = type(err).__name__
    return name in {"APIConnectionError", "APITimeoutError", "APIStatusError"} and (
        getattr(err, "status_code", 0) >= 500
    )
