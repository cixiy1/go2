"""虚拟控制器：跑在仿真器上的控制程序。

「虚拟」指机器人是 MuJoCo 里的那台（`lo0` + 仿真器 domain）。这里的东西
发的是同一套 `rt/lowcmd`，换个网卡就对着真机 —— 区别是虚拟侧只用仿真器
具备的条件（没有原生运动服务、没有真实打滑与噪声）。

每个模块都是一个**能独立运行**的控制器，并在模块级暴露 `META` 字典：

    META = dict(title="显示名", group="virtual", desc="一句话说明",
                owns_cmd=True, owns_keys=False)

项目根 `main.py` 扫描本目录，读 `META` 列菜单，选中后以
`python -m quadruped.control.virtual.<文件名>` 拉起来。加控制器 = 加文件。
"""
