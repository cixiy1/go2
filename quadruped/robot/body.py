#!/usr/bin/env python3
"""Go2 机器人本体（第 1 层）：唯一对外入口。

只持有一个子对象 —— 控制器，于是三层在这一行里就看得见：

    robot.controller               Go2Controller
    robot.controller.high_level    HighLevelController

本层职责仅限：按网卡选环境 → 造控制器 → 转发连接与传感器读数 → 收工停机。
控制算法一概不在这里。

    robot = Go2Robot()              # 不给参数 = 仿真（按仿真器 config.py 的网卡与域号）
    robot = Go2Robot(iface="lo0")   # 回环 = 仿真
    robot = Go2Robot(iface="en5")   # 非回环网卡 = 真机：domain 0
    robot.connect()
    robot.controller.high_level.forward(2.0)
    robot.print_tree()              # 打印三层对象树并校验嵌套
    robot.shutdown()                # 先蹲下再阻尼

直接运行本文件只打印对象树，不连 DDS。
"""

from __future__ import annotations

import os
import re
import socket
import subprocess
import sys

if __package__ in (None, ""):           # python quadruped/robot/body.py：把项目根加进来
    sys.path.insert(0, os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))

from quadruped.control.controller import GaitParams, Go2Controller, NOMINAL_HEIGHT
from quadruped.control.high_level.ours import HighLevelController

# 仿真器的配置目录（unitree_mujoco 子模块），用来读它绑的网卡与域号。
# 位置：quadruped/sim/unitree_mujoco/simulate_python，相对本文件的上一级包根。
_SIM_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "sim", "unitree_mujoco", "simulate_python")
_SIM_NET: dict | None = None            # sim_network() 的缓存


class Go2Robot:
    """机器人本体：最外层，只持有一个子对象 —— 控制器。

        robot = Go2Robot()                 # 默认仿真（lo0 / domain 1）
        robot = Go2Robot(iface="en5")      # 真机（非回环 / domain 0）

    环境由**网卡**决定，但这个决定有依据：仿真器绑哪块网卡、发布在哪个 DDS 域，
    都写在 quadruped/sim/unitree_mujoco/simulate_python/config.py 里（INTERFACE / DOMAIN_ID），
    本类直接读那两个值。网卡是入口，域号才是事实 —— 真机固定在 domain 0，
    仿真器在它自己配置的域上，两者的网卡通常是 lo0 和 en5。

    同一台电脑同时连着真机和仿真器时，域号不同才保证两边不串台。

    层级是字面意义上的包含关系，不是别名：

        robot.controller                  Go2Controller
        robot.controller.high_level       HighLevelController
    """

    PRESETS = {
        "sim": dict(domain=1, iface="lo0"),      # 读不到 config.py 时的兜底
        "real": dict(domain=0, iface="en5"),
    }

    # 各环境能用的控制端。仿真桥只收发 rt/lowcmd 与 rt/sportmodestate，
    # 没有 rt/api/sport/request 的服务端，官方 SportClient 发过去只会等到超时，
    # 所以仿真只有自研一个，真机两个都有。
    CLIENTS = {
        "sim": ("ours",),
        "real": ("official", "ours"),
    }

    # -- 网卡 → 环境 ----------------------------------------------------

    # 这个环境能用的控制端，顺序即菜单顺序。
    @classmethod
    def clients_for(cls, preset: str) -> tuple[str, ...]:
        return cls.CLIENTS[preset]

    # 回环网卡：macOS 叫 lo0，Linux 叫 lo。
    @staticmethod
    def is_loopback(iface: str) -> bool:
        return iface == "lo" or iface.startswith("lo")

    @classmethod
    def preset_for_iface(cls, iface: str) -> str:
        return "sim" if cls.is_loopback(iface) else "real"

    # 本机是否真有这块网卡（拔了网卡、名字写错都会 False）。
    @staticmethod
    def interface_exists(iface: str) -> bool:
        return iface in {name for _, name in socket.if_nameindex()}

    # 只保留可能接机器人的网卡，其它（虚拟隧道、无线直连、网桥）都是噪音。
    _IFACE_SKIP = ("awdl", "llw", "utun", "gif", "stf", "bridge",
                   "ap", "anpi", "vmenet", "p2p")

    # 本机的物理网卡清单，含 IPv4 与是否激活。
    #
    # 用 ifconfig 而不是 ioctl：macOS 的 ifreq 布局和 Linux 不一样，
    # ioctl 拿地址得写两套；ifconfig 两边输出格式足够接近，解析一遍就够。
    # 拿不到就返回空表，调用方退回预设值。
    @classmethod
    def list_ifaces(cls) -> list[dict]:
        try:
            text = subprocess.check_output(
                ["ifconfig", "-a"], text=True, stderr=subprocess.DEVNULL,
                timeout=3)
        except (OSError, subprocess.SubprocessError):
            return []

        blocks: list[dict] = []
        for line in text.splitlines():
            if line and not line[0].isspace():
                name = line.split(":", 1)[0]
                blocks.append(dict(name=name, ip="", up=False, active=False))
            elif blocks:
                cur = blocks[-1]
                s = line.strip()
                if s.startswith("inet "):
                    parts = s.split()
                    if len(parts) > 1:
                        cur["ip"] = parts[1]
                elif s.startswith("status:"):
                    cur["up"] = True
                    cur["active"] = s.split(":", 1)[1].strip() == "active"
        return [b for b in blocks
                if not b["name"].startswith(cls._IFACE_SKIP)]

    # Unitree 机器人的网段是 192.168.123.x，选网口时把它排前面。
    @staticmethod
    def is_real_robot_net(ip: str) -> bool:
        return ip.startswith("192.168.123.")

    # 仿真器实际绑的网卡与域号 —— 直接读它的 config.py，不靠约定猜。
    #
    # 仿真器是 unitree_mujoco 那一套，配置在 simulate_python/config.py 的
    # DOMAIN_ID / INTERFACE 两行。有人把 INTERFACE 改成 en0、或把 DOMAIN_ID
    # 换成 3，这里读到的就是新的值，控制端不用跟着改代码。
    @classmethod
    def sim_network(cls) -> dict:
        global _SIM_NET
        if _SIM_NET is not None:
            return _SIM_NET
        cfg = dict(cls.PRESETS["sim"])
        path = os.path.join(_SIM_DIR, "config.py")
        try:
            text = open(path, encoding="utf-8").read()
        except OSError:
            text = ""
        m = re.search(r"^\s*DOMAIN_ID\s*=\s*(\d+)", text, re.M)
        if m:
            cfg["domain"] = int(m.group(1))
        m = re.search(r"""^\s*INTERFACE\s*=\s*["']([^"']+)["']""", text, re.M)
        if m:
            cfg["iface"] = m.group(1)
        _SIM_NET = cfg                         # 缓存，别每次都读盘
        return _SIM_NET

    # 把「网卡 / 预设 / 域号」三者归一到具体连接参数。
    #
    # 判定顺序：显式 domain 最高，其次显式 preset，最后由网卡推断。
    #
    # - 网卡等于仿真器配置里的 INTERFACE → 仿真，域号取它的 DOMAIN_ID
    # - 网卡是回环（lo0 / lo）→ 仿真，哪怕配置被改过（回环上没有真狗）
    # - 其它网卡 → 真机，域号 0
    # - 什么都不给 → 仿真器配置里的那套
    #
    # 「仿真与否」的事实依据其实是 域号：仿真器发布在 DOMAIN_ID 上，真机
    # 发布在 0 上。网卡只是用来选，所以这里优先按仿真器自己的配置去认，认不
    # 出来才退回回环这条例外规则。
    @classmethod
    def resolve(cls, preset: str | None = None, *, domain: int | None = None,
                iface: str | None = None) -> dict:
        sim = cls.sim_network()
        if preset is None:
            if iface is None:
                preset = "sim"
            elif iface == sim["iface"] or cls.is_loopback(iface):
                preset = "sim"
            else:
                preset = "real"

        if iface is None:
            iface = sim["iface"] if preset == "sim" else cls.PRESETS["real"]["iface"]
        if domain is None:
            domain = sim["domain"] if preset == "sim" else cls.PRESETS["real"]["domain"]
        return dict(preset=preset, domain=domain, iface=iface)

    def __init__(self, preset: str | None = None, *, domain: int | None = None,
                 iface: str | None = None, height: float = NOMINAL_HEIGHT,
                 kp: float | None = None, kd: float | None = None,
                 kp_calf: float | None = None, gait: GaitParams | None = None,
                 tau_ff: float | None = None,
                 real_params: bool | None = None):
        cfg = self.resolve(preset, domain=domain, iface=iface)
        self.preset = cfg["preset"]
        self.clients = self.clients_for(self.preset)     # 本环境可用的控制端
        # 唯一子对象。控制参数默认两边同一套（真机那一套），不按环境分叉：
        # 仿真器那边已经把 Go2 真机的关节摩擦、力带宽、编码器噪声、指令延迟
        # 都建进去了（见 simulate_python/config.py 的 REALISM 与 go2.xml 的
        # 关节参数）。再给仿真单独调一套软的，等于对着一个不存在的理想模型
        # 调参 —— 之前「仿真调好一上机就抖」「仿真里 0.35、真机一步都不走」
        # 都是这么来的。
        #
        # 想对照理想模型（力矩当帧到位、传感器无噪声）时显式传
        # `real_params=False`，那是调试用的旁路，不是日常默认。
        if real_params is None:
            real_params = True
        self.real_params = real_params
        self.controller = Go2Controller(
            domain=cfg["domain"], iface=cfg["iface"],
            height=height, kp=kp, kd=kd, kp_calf=kp_calf, gait=gait,
            real=real_params, tau_ff=tau_ff)

        # MPC / WBC 在这里就建好，不等第一次行走。
        #
        # 两个理由：
        #   1. 构造要编译两份 QP，实测百毫秒级。放在 200 Hz 回路里就是几十帧
        #      超时，放在 `start_velocity` 里就是起步卡一下。
        #   2. 它们是常设部件，属于机器人本身，不该跟着步态起停。
        #
        # 只在 mpc 力路下建：vmc 一辈子用不到它们，而 MPC 线程常驻会抢主回路
        # 的 CPU —— 实测回路周期 p95 从 5.7 ms 涨到 9.5 ms。
        if self.controller.gait.mode == "mpc":
            self.controller._ensure_mpc()

    # -- 连接与状态 ----------------------------------------------------

    def connect(self, timeout: float = 3.0) -> bool:
        ok = self.controller.connect(timeout)
        # 连上才起 MPC 线程：`_mpc_solve_once` 要读 q / rpy / 机身高度，没收到
        # lowstate 之前这些全是 None，解出来是垃圾。
        if ok and self.controller.gait.mode == "mpc":
            self.controller._mpc_start()
        return ok

    # 收工。park=True 先蹲下再阻尼，避免从站立高度直接掉下来。
    def shutdown(self, park: bool = True) -> None:
        self.controller.shutdown(park=park)

    def telemetry(self, duration: float = 5.0) -> int:
        return self.controller.telemetry(duration)

    def selftest(self) -> bool:
        return self.controller.selftest()

    # -- 对象树 --------------------------------------------------------

    # 返回三层的实际嵌套关系，同时校验它没被写成别名。
    #
    # 校验项：controller 是 Go2Controller、high_level 是 HighLevelController、
    # high_level 挂在 controller 的 __dict__ 里、且 high_level 指回同一个 controller。
    # 任何一项不成立就抛 AssertionError —— 免得哪天又被人「拍平」成兄弟属性。
    def hierarchy(self) -> dict:
        c = self.controller
        hl = getattr(c, "high_level", None)
        assert isinstance(c, Go2Controller), "第 2 层不是 Go2Controller"
        assert isinstance(hl, HighLevelController), "第 3 层不是 HighLevelController"
        assert c.__dict__.get("high_level") is hl, "high_level 不在 controller 上"
        assert hl.controller is c, "high_level 没有指回它的父对象"
        return {
            "Go2Robot": dict(preset=self.preset, id=id(self),
                             children=["controller"]),
            "Go2Controller": dict(id=id(c), parent=id(self),
                                  domain=c.domain, iface=c.iface,
                                  children=["high_level"]),
            "HighLevelController": dict(id=id(hl), parent=id(c),
                                        children=[]),
        }

    # 在终端打印对象树，用来肉眼确认三层是真的嵌套。
    def print_tree(self) -> None:
        c = self.controller
        hl = c.high_level
        print(f"Go2Robot            {self.preset:4s} id=0x{id(self):x}")
        print(f"└─ controller       Go2Controller        id=0x{id(c):x}  "
              f"domain={c.domain} iface={c.iface}")
        print(f"   └─ high_level    HighLevelController  id=0x{id(hl):x}  "
              f"→ 父对象 {'同一实例' if hl.controller is c else '不是同一实例'}")

    @property
    def connected(self) -> bool:
        return self.controller.connected

    @property
    def q(self):
        return self.controller.q

    @property
    def dq(self):
        return self.controller.dq

    @property
    def rpy(self):
        return self.controller.rpy

    @property
    def pos(self):
        return self.controller.pos

    def state(self) -> dict:
        st = self.controller.state()
        st["preset"] = self.preset
        st["domain"] = self.controller.domain
        st["iface"] = self.controller.iface
        st["height"] = self.controller.height
        st["gait"] = dict(period=self.controller.gait.period,
                          duty=self.controller.gait.duty,
                          swing_height=self.controller.gait.swing_height)
        return st

    def __enter__(self):
        if not self.connect():
            raise RuntimeError("连不上：检查仿真器是否在跑、domain/网卡是否一致")
        return self

    def __exit__(self, *exc):
        self.shutdown()


if __name__ == "__main__":
    Go2Robot().print_tree()
