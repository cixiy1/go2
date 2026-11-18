"""智谱 API 联调测试模版，直接运行即可发送一条测试消息。"""

import asyncio
import sys
from pathlib import Path
from typing import cast

sys.path.insert(0, str(Path(cast(str, __file__)).resolve().parent.parent / "src"))

from yuki.settings import load_settings
from yuki_kernel import Agent, ToolRegistry


async def main():
    settings = load_settings()
    agent = Agent(
        settings.model,
        ToolRegistry(),
        settings,
        provider="openai",
    )
    result = await agent.turn("你好，请用一句话介绍你自己。")
    print(result.content)
    await agent.close()


if __name__ == "__main__":
    asyncio.run(main())
