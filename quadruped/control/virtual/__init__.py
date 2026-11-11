"""虚拟控制器 = 控制器抽象层。这里的「虚拟」是虚函数那个虚拟，不是仿真。

这层只回答一个问题：**一个模块要长成什么样，才算一个控制器。** 不实现控制
逻辑，不连硬件，不发 DDS。具体的控制器在 `physical/`（直接驱动机器人）和
`embodied/`（决策层）里。

    protocol.py    ControllerModule 协议（给 IDE / mypy）
                   META_KEYS、normalize_meta、check_module（给启动器运行时校验）

## 协议是什么

模块级，不是基类：

    META = dict(title="键盘遥控", desc="一句话说明", group="physical",
                owns_cmd=True, owns_keys=True)

    def main(argv=None) -> int: ...

写成 `class Controller(ABC)` 会逼着每个控制器把主循环塞进一个方法里。
`physical/keyboard.py` 一千三百行（按键源、限幅、收工兜底都在模块级），改成
类的形状收益不大。协议留在这里，是为了让「形状」有个地方写清楚、有地方校验。

字段：

    title       必填。菜单里显示的名字
    desc        一句话说明，菜单里显示在名字下面
    group       所在目录名。写错了启动器报错，不静默改 —— 搬文件忘改最容易发生
    owns_cmd    最终会让机器狗动。两个同时选会互相覆盖 rt/lowcmd，启动器会拦
    owns_keys   接管键盘。两个同时选抢按键，启动器也会拦

## 为什么要有这一层

三个目录里躺着的不是同一类东西：抽象层、具体控制器、决策层。没有这层，
「控制器该有哪些字段」会散落在启动器的 if 里、每个脚本的注释里，以及新写
一个控制器时靠抄隔壁文件。
"""

from quadruped.control.virtual.protocol import (
    META_KEYS,
    REQUIRED_KEYS,
    ControllerModule,
    check_module,
    normalize_meta,
)

__all__ = [
    "META_KEYS",
    "REQUIRED_KEYS",
    "ControllerModule",
    "check_module",
    "normalize_meta",
]
