"""真机控制面（网页）与部署脚本的入口。

    python -m quadruped.web.app --iface en5

这里只有进程管理和一个静态页面，不 import 控制栈、不碰 DDS —— 理由写在
`app.py` 顶部：lowcmd 只能有一个写者，而 `LinkBusy` 是进程内锁。
"""
