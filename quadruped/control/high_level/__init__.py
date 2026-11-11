"""高层控制器：把「往前走」「跳一下」翻译成对机器人的具体指令。

两份实现，同名同义，运行时二选一：

    ours.py       HighLevelController（别名 OursHighLevel）
                  自研。只排动作序列，最终落到 200 Hz 回路的 rt/lowcmd 上。
                  算法在我们这边：关节角、步态相位、力矩分配。

    official.py   OfficialHighLevel
                  官方。走 rt/api/sport/request 调机器上的运控服务。
                  算法在机器那边：宇树的 MPC + WBC，我们只发意图。

## 为什么同名

`physical/keyboard.py` 这类控制器只认一套名字 —— `Move` / `StopMove` /
`BalanceStand` / `jump_async` / `crouch_async` / `park`。两份都满足，所以
换控制端不用改控制器：切换点在 `control/virtual/robot.py` 的 `connect(ep, kind)`，
返回的 `Backend` 把 `client` 指到其中一份。

`ours` 的名字本来就照着 `SportClient` 取（它是「离线复刻一份官方接口」），
所以同名不是巧合，是这两份能与彼此互换的前提。差别在**语义落点**：

    自研   crouch_async(0.08)   把机身降 8 cm 的关节序列排进回路，之后还能站起来
    官方   crouch_async(0.08)   发 BodyHeight(-0.08)，机器自己平滑下降

## 为什么不在这里 import 两份

`official.py` 顶层 import `unitree_sdk2py`，`ours.py` import 控制器（同样拖 DDS）。
`__init__` 一 import 就会把两者都拉进「只想看菜单」的路径。要哪份就写哪份：

    from quadruped.control.high_level.ours import HighLevelController
    from quadruped.control.high_level.official import OfficialHighLevel

## 谁该用哪份

- 真机日常走动、要稳定：官方。它跑的是宇树调好的 MPC + WBC。
- 要改步态、改跳跃、做实验：自研。官方的参数在机器里，改不了。
- 仿真：只有自研。仿真桥没有 rt/api/sport/request 的服务端。
"""
