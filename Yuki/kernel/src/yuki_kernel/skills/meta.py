"""注册表元工具：包管理与记忆检索。"""

META_NAMES = {"list_packages", "load_package", "unload_package", "search_memory"}

META_TOOLS = [
    {
        "name": "list_packages",
        "description": "列出当前可用的外置工具包；当现有工具无法满足用户需求时，先调用本工具查找可加载的包",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "load_package",
        "description": "加载一个外置工具包，之后它的工具和提示词才进入上下文；需要某项能力但当前没有对应工具时，用本工具加载对应包",
        "parameters": {
            "type": "object",
            "required": ["package_id"],
            "properties": {
                "package_id": {
                    "type": "string",
                    "description": "外置工具包的 id",
                }
            },
        },
    },
    {
        "name": "unload_package",
        "description": "卸载一个外置工具包，释放上下文空间",
        "parameters": {
            "type": "object",
            "required": ["package_id"],
            "properties": {
                "package_id": {
                    "type": "string",
                    "description": "外置工具包的 id",
                }
            },
        },
    },
    {
        "name": "search_memory",
        "description": "按关键词检索长期记忆，返回历史会话中的相关信息",
        "parameters": {
            "type": "object",
            "required": ["query"],
            "properties": {
                "query": {
                    "type": "string",
                    "description": "要检索的关键词",
                }
            },
        },
    },
]
