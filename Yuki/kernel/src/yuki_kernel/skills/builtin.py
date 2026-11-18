"""内置工具注册表：只声明内置工具和提示词，具体实现放在 builtins/ 下。"""

from .types import Tool

BUILTIN_TOOLS: list[Tool] = [
    {
        "name": "get_environment_info",
        "description": "获取当前操作系统、Python 版本、工作目录等环境信息；上下文已有环境信息时不要重复调用",
        "parameters": {
            "type": "object",
            "properties": {},
        },
        "entry": {
            "type": "python",
            "module": "builtins/environment.py",
            "handler": "get_environment_info",
        },
    },{
        "name": "terminal",
        "description": "在终端/terminal 中执行命令并返回标准输出与标准错误；适配各类 POSIX 终端。沙箱与降权由宿主注入内核的 Sandbox 设定决定，工具本身不配置权限。",
        "parameters": {
            "type": "object",
            "properties": {
                "cmd": {
                    "type": "string",
                    "description": "要执行的命令字符串，经 /bin/sh -c 运行",
                },
            },
            "required": ["cmd"],
        },
        "entry": {
            "type": "python",
            "module": "builtins/terminal.py",
            "handler": "run_terminal",
        },
    },{
        "name": "sleep",
        "description": "先调用output输出等待提示信息，告知将等待多少秒；调用后需再调用 run_sleep 实际执行等待",
        "parameters": {
            "type": "object",
            "properties": {
                "time": {
                    "type": "number",
                    "description": "要等待的时间，单位秒"
                }
            },
            "required": ["time"],
        },
        "entry": {
            "type": "python",
            "module": "builtins/sleep.py",
            "handler": "Sleep",
            "method": "output",
        },
    },{
        "name": "run_sleep",
        "description": "实际等待指定秒数；先调用 sleep 获得等待提示后再调用本工具执行等待",
        "parameters": {
            "type": "object",
            "properties": {
                "time": {
                    "type": "number",
                    "description": "要等待的时间，单位秒"
                }
            },
            "required": ["time"],
        },
        "entry": {
            "type": "python",
            "module": "builtins/sleep.py",
            "handler": "Sleep",
            "method": "run_sleep",
        },
    }
]

BUILTIN_PROMPTS = [
    {
        "name": "environment_guide",
        "description": "命令生成前的环境信息使用提示",
        "path": "builtins/prompts/environment_guide.md",
    },
]
