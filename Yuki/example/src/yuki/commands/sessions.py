"""会话类斜杠命令。"""

import asyncio

# noinspection PyUnresolvedReferences
from yuki_kernel.core.app import App

from ..settings import load_settings


async def handle_session(app: App, cmd: str, arg: str) -> None:
    if cmd == "/save":
        if not arg:
            print("用法：/save <名字>")
            return
        app.session.name = arg
        await asyncio.to_thread(app.store.save, app.session)
        print(f"会话已保存：{arg}")
    elif cmd == "/load":
        if not arg:
            print("用法：/load <名字>")
            return
        meta = None
        for item in app.store.list_sessions():
            if item.name == arg:
                meta = item
                break
        if meta is None:
            print(f"找不到会话：{arg}")
            return
        session = await asyncio.to_thread(app.store.load, meta.session_id)
        if session is None:
            print(f"会话文件缺失：{arg}")
            return
        app.session = session
        app.agent.switch_session(session)
        print(f"已加载会话：{arg}")
    elif cmd == "/sessions":
        metas = app.store.list_sessions()
        if not metas:
            print("暂无已保存会话")
            return
        for meta in metas:
            print(f"{meta.name}  {meta.updated_at}")
    elif cmd == "/new":
        app.agent.switch_session(app.store.create())
        print("已开始新会话")
    elif cmd == "/reload":
        await app.reload(load_settings())
        print("配置已热加载")
    else:
        print(f"未知命令：{cmd}")
