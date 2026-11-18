"""机器人通讯层：让外部程序在另一个进程里指挥这台狗。

## 为什么必须分进程

200 Hz 控制回路独占 DDS：`ChannelFactoryInitialize` 一个进程只能调一次，
`rt/lowcmd` 同一时刻只能有一个发送者。两个进程同时发，电机会收到交替的目标，
表现是「腿一跳一跳的」。所以机器人必须跑在一个常驻进程里，外部程序通过本层调用。

## 文件分工

    protocol.py    报文格式、命令表、参数校验。两侧都 import 这一份
    client.py      客户端 + 命令行
    server.py      常驻服务端：连机器人、收命令、转给 Backend
    ws.py          最小 WebSocket 服务端（给 App 用）
    child.py       子进程外壳（comm/ 里的 Popen 只在这里）
    robot_link.py  到 server 的连接 + 那个服务端的生死
    vla_host.py    VLA 子进程的生死 + 它的 stdin
    cam_host.py    相机馈送子进程的生死
    gateway.py     App 网关：会话、操作权、模式闸门、编排上面三个 host

前三行是「外部程序怎么指挥一台狗」的旧路径，后六行是「App 怎么指挥一台狗」。
两条路径共用同一份 protocol 和同一个 server —— 不是两套控制，是两种入口。

## 谁能写 rt/lowcmd

    comm.server（持 Backend）    手动 / 脚本 / 网关转发      ← 能写
    VLA 子进程（PolicyBackend）  自主                        ← 能写
    percept.feed                 只看相机                    ← 只读
    gateway / Flutter 客户端     不碰 DDS                    ← 只读

谁能写这件事互斥：`rt/lowcmd` 同一时刻一个发送者。网关的「切模式」就是交接，
细节见 `docs/control-ownership.md`。

## 谁该用哪种连接

    Unix 域套接字（默认 /tmp/go2_robot.sock）   同一台机器，权限由文件系统控制
    TCP（--host 127.0.0.1 --port 8765）          机器人在另一台机器上时

默认走 Unix 套接字：不用挑端口、不会和别的服务撞、进程崩了文件也能被清掉。

## 和 `control/virtual/` 的关系

`virtual/` 回答「控制器怎么接上机器人」，本层回答「外部程序怎么调用那个控制器」。
服务端内部用的就是 `virtual.robot.connect()`，所以网卡、域号、控制端这些东西
在客户端一行都看不到 —— 客户端只知道一个套接字路径。

## 谁能直接碰抽象层

分界线不是「进程内 / 进程外」，是两个条件：代码在不在这个仓库里，进程能不能常驻。

    quadruped/ 内部、Yuki 的常驻宿主    直接 import virtual.robot，拿 Backend 用
    短命进程（工具子进程、脚本）        走本层 —— 谁持 rt/lowcmd 由常驻方说了算
    仓库外的项目                        只能走本层：套接字 + JSON 行，不 import quadruped

常驻这条是硬的：`ChannelFactoryInitialize` 一进程只能调一次，`rt/lowcmd` 同一时刻
只有一个发送者。短命进程持 `Backend`，等于每次调用重新起站、退出即停发指令 ——
在仿真里就是调用间隙狗趴着。

内部直接耦合的意思是可以调 `connect()` 拿到 `Backend`，和控制器共享同一个对象、
共享 `pos / rpy / height`。走本层则每次都得付一次序列化和一次往返，写高频逻辑
（比如 20 Hz 的策略）时这笔开销和延迟都是白给的。

外部反过来：它不该知道 `Backend` 长什么样，也不该被 `quadruped` 的依赖拖着走。
本层就是那道边界 —— 只依赖标准库，装不装 DDS 都能 import 客户端。

这条边界由 `test/arch_check.py` 检查（内部 import 本层、本层模块级 import 重依赖
都会判失败），不是靠自觉。
"""

from quadruped.comm.protocol import (
    COMMANDS,
    DEFAULT_PORT,
    DEFAULT_SOCKET,
    MAX_LINE,
    ProtocolError,
    encode,
    error,
    ok,
    parse_request,
)

__all__ = [
    "COMMANDS",
    "DEFAULT_PORT",
    "DEFAULT_SOCKET",
    "MAX_LINE",
    "ProtocolError",
    "encode",
    "error",
    "ok",
    "parse_request",
]
