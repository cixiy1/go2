"""虚拟控制器 = 控制器抽象层。这里的「虚拟」是虚函数那个虚拟，不是仿真。

这层回答两个问题，都不允许控制器自己回答：

1. **一个模块要长成什么样，才算一个控制器**（`protocol.py`）
2. **怎么找到并接上机器人**（`robot.py` / `backends.py`）

具体的控制器在 `physical/`（直接驱动机器人）和 `embodied/`（决策层）里。

    protocol.py    ControllerModule / ControllerGroup 协议（给 IDE / mypy）
                   META_KEYS、normalize_meta、check_module（给启动器运行时校验）
    robot.py       Endpoint 解析、网口与控制端选择、connect() 工厂、Backend 协议
    backends.py    两个控制端实现 —— 全项目唯一 import DDS 与 Go2Robot 的地方

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

## 为什么「接机器人」也属于这一层

控制器只该关心「往前走 0.3 m/s」，不该关心回环口对应 DDS 域 1、仿真器绑的是
哪块网卡、原生运控服务在不在。这些是**环境**的知识，写进控制器就意味着每写
一个控制器抄一遍；抄错了不报错，只是安静地收不到数据。

而且它和 META 校验是同一类事：都在控制器开始干活之前，把「你面对的是什么」
确定下来。所以放在同一层，控制器只调四个函数：

    ep    = choose_endpoint(preset=..., iface=..., domain=...)
    kind  = choose_client(ep, args.client)
    robot = connect(ep, kind, kp=..., kd=..., tau=...)     # 已经 BalanceStand
    robot.close()

## 为什么要有这一层

四个目录里躺着的不是同一类东西：抽象层、具体控制器、决策层、仿真资产。没有
这层，「控制器该有哪些字段」会散落在启动器的 if 里、每个脚本的注释里，以及
新写一个控制器时靠抄隔壁文件。
"""

from quadruped.control.virtual.protocol import (
    GROUP_KEYS,
    META_KEYS,
    REQUIRED_KEYS,
    ControllerGroup,
    ControllerModule,
    check_module,
    normalize_group,
    normalize_meta,
)

__all__ = [
    "GROUP_KEYS",
    "META_KEYS",
    "REQUIRED_KEYS",
    "ControllerGroup",
    "ControllerModule",
    "check_module",
    "normalize_group",
    "normalize_meta",
]

# `robot.py` 故意不在这里 re-export：它 import 了机器人本体，会把 DDS 和整个
# 控制器模块拖进 `main.py --list` 的启动路径。控制器要连机器人时直接
#     from quadruped.control.virtual.robot import connect, choose_endpoint
# 这里是「控制器长什么样」，那边是「机器人怎么接」，两条路分开走。
