"""物理控制器：直接驱动机器人的那一类。这里的「物理」对应抽象层的「虚拟」。

`physical/` 里的东西会真的让机器狗动起来 —— 发速度指令、占键盘、开摄像头。
对它来说仿真器只是「换块网卡」：`lo0` 对着仿真，`en5` 对着真机，同一份代码。

现在的成员：

    keyboard.py   键盘遥控，带加速度限幅与按键保持。两个控制端
                  （官方 SportClient / 自研 HighLevelController）

## 控制器不碰连接

这里**不许 import `Go2Robot`、不许 import `unitree_sdk2py`、不许碰 socket**。
网口枚举、域号推断、控制端构造、BalanceStand 都在 `../virtual/robot.py`：

    ep    = choose_endpoint(preset=args.preset, iface=args.iface, domain=args.domain)
    kind  = choose_client(ep, args.client)
    robot = connect(ep, kind, kp=..., kd=..., tau=...)
    robot.close()

理由不是洁癖：连错了不会报错，只会安静地收不到数据（域号错 = 收不到
`rt/lowstate`；控制端选错 = 请求等到超时）。把它收在一处，出错只有一处可能。

## 什么该放进来

- 有人参与的操作接口：键盘、手柄、摇杆、网页控制台
- 按固定动作序列跑的自动化脚本（走路、跳舞、巡检一遍）
- 需要真实接触才能验的东西

## 什么不该放进来

- 抽象、协议、机器人对接 → `../virtual/`
- 大模型决策、视觉理解 → `../embodied/`
- 纯算法（运动学、步态、力矩分配）→ `../controller.py`，那层不碰 IO
- 一次性排查脚本 → `test/`，它们不常驻菜单

约定见 `../virtual/protocol.py`：模块级 `META`（`group` 写 `"physical"`）+
`main(argv=None)`，能 `python -m quadruped.control.physical.<文件名>` 独立跑。
"""

# 菜单登记：声明 GROUP 才会出现在项目入口里，标题与顺序都归本目录自己管。
GROUP = dict(title="物理控制器 · 直接驱动机器人（仿真或真机）", order=10)
