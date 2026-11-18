"""Yuki 示例的交互审批实现。"""

import asyncio
from typing import Any


async def cli_approver(name: str, arguments: dict[str, Any]) -> str:
    answer = await asyncio.to_thread(
        input,
        f"工具 {name} 需要审批，参数：{arguments} (y / ya / y <分钟> / n)：",
    )
    return answer.strip()
