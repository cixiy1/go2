"""机器人对接层：控制器只说「我要一台机器人」，剩下的事在这里。

物理控制器（`control/physical/*`）需要的东西只有四样，全出自本模块：

    ep   = choose_endpoint(preset=args.preset, iface=args.iface,
                           domain=args.domain)     # 接哪一台
    kind = choose_client(ep, args.client)           # 用哪个控制端
    r    = connect(ep, kind, kp=..., kd=..., tau=...)   # 已经 BalanceStand 过
    r.close()                                       # 停车 → 蹲下 → 阻尼

控制器不 import `Go2Robot`、不 import `unitree_sdk2py`、不知道域号怎么定，
也不知道「lo0 是仿真」这件事从哪读来。

## 为什么端点解析必须放在这一层

「回环口 = 仿真、en5 = 真机、域号 1 还是 0、仿真器的配置改过没有」这些事实
来自机器人本体（`robot/body.py`）和仿真器配置，不是控制器的知识。放在控制器
里，每写一个控制器就要抄一遍网卡枚举与域号推断，抄错一次的表现是连不上或者
连错地方 —— 而这两种错都不会报错，只会安静地收不到数据。

## 分层

    protocol.py   控制器**长什么样**（META / GROUP 校验），不连硬件
    robot.py      接哪一台、怎么接（本文件），只认端点与协议
    backends.py   真接上之后**怎么驱**，全项目唯一 import SDK 与 Go2Robot 的地方

`connect()` 才延迟导入 backends：`main.py --list` 只读菜单时不必等 SDK 加载。
"""

import sys
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from quadruped.robot.body import Go2Robot

#: 控制端名字。菜单和错误消息共用，别在控制器里另写一份。
CLIENT_NAMES = {"official": "官方控制器", "ours": "自研控制器"}

#: 菜单里给每个控制端的一行说明。
CLIENT_DESC = {
    "official": "官方控制器    SportClient —— 真机上的原生运动服务",
    "ours": "自研控制器    HighLevelController —— 本仓库实现",
}


@dataclass(frozen=True)
class Endpoint:
    """一台具体的机器人：网卡 + DDS 域号 + 它是仿真还是真机。

    `preset` 是从网卡推出来的结论，不是用户输入的原样转发 —— 决定能用哪些
    控制端（仿真没有原生运动服务）以及菜单默认值。
    """

    preset: str          # "sim" | "real"
    iface: str
    domain: int

    @property
    def sim(self) -> bool:
        return self.preset == "sim"

    @property
    def label(self) -> str:
        return (f"网卡 {self.iface}，DDS 域 {self.domain}"
                f"（{'仿真' if self.sim else '真机'}）")


@runtime_checkable
class Backend(Protocol):
    """接上之后控制器看到的东西。方法名沿用 SportClient，两个实现都齐。"""

    kind: str
    label: str
    target: str
    client: object          # 往下发指令的那个对象（Move / StopMove / 动作接口）

    def start(self) -> int:
        """SetTimeout → Init → BalanceStand，返回码非 0 = 对端没响应。"""

    @property
    def pos(self): ...

    @property
    def rpy(self): ...

    @property
    def height(self): ...

    @property
    def busy(self) -> bool: ...

    @property
    def loop_text(self) -> str: ...

    def jump(self) -> int: ...
    def crouch(self) -> int: ...
    def uncrouch(self) -> int: ...
    def stand(self) -> int: ...
    def stop_motion(self) -> int: ...
    def freeze(self) -> int: ...
    def close(self) -> None: ...


# ---- 事实：网卡与端点 ------------------------------------------------------

def sim_network() -> dict:
    """仿真器实际绑的网卡与域号（读它的 config.py，不靠约定猜）。"""
    return Go2Robot.sim_network()


def clients_for(preset: str) -> tuple[str, ...]:
    """这个环境能用的控制端，顺序即菜单顺序。"""
    return Go2Robot.clients_for(preset)


def resolve_endpoint(preset=None, *, iface=None, domain=None) -> Endpoint:
    """显式参数 → Endpoint。规则见 Go2Robot.resolve。"""
    cfg = Go2Robot.resolve(preset, domain=domain, iface=iface)
    return Endpoint(cfg["preset"], cfg["iface"], int(cfg["domain"]))


def _interfaces() -> list[dict]:
    """候选网卡：仿真器那块在最前，接机器人的网段其次，其余垫后。

    本机真有的网卡都列出来，加上真机默认网卡 —— 那块没插的时候也列，标一句
    「未检测到」，省得人以为程序没认出来。
    """
    sim = sim_network()
    real_pre = Go2Robot.PRESETS["real"]["iface"]

    items = [dict(name=sim["iface"], ip="",
                  note=f"仿真器（domain {sim['domain']}）")]
    seen = {sim["iface"]}

    for it in Go2Robot.list_ifaces():
        n = it["name"]
        if n in seen or n.startswith("lo"):
            continue
        # 没激活又没地址的虚拟网卡（Thunderbolt/USB 口的占位名）不列
        if not it["active"] and not it["ip"] and n != real_pre:
            continue
        if it["ip"]:
            off = "" if Go2Robot.is_real_robot_net(it["ip"]) else "，非 192.168.123 网段"
            note = f"{it['ip']:<15s} 真机（domain 0{off}）"
        else:
            note = "真机默认网卡，当前未检测到"
        items.append(dict(name=n, ip=it["ip"], note=note))
        seen.add(n)

    if real_pre not in seen:
        items.append(dict(name=real_pre, ip="", note="真机默认网卡，当前未检测到"))

    head, tail = items[:1], items[1:]
    head += [x for x in tail if Go2Robot.is_real_robot_net(x["ip"])]
    others = [x for x in tail if not Go2Robot.is_real_robot_net(x["ip"])]
    return head + others


def _pick(options: list[dict], default_no: int, prompt: str) -> int:
    """编号选一项，回车取默认。非交互终端直接返回默认。

    做成通用的一小段：网口和控制端两个菜单都是这个形状。
    """
    if not sys.stdin.isatty():
        return default_no
    while True:
        try:
            ans = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return default_no
        if not ans:
            return default_no
        try:
            n = int(ans)
        except ValueError:
            n = 0
        if 1 <= n <= len(options):
            return n
        print(f"  只认 1 到 {len(options)}。")


def choose_endpoint(preset=None, *, iface=None, domain=None) -> Endpoint:
    """定下接哪一台。没给网卡/预设就先列出来问一句，回车 = 仿真。"""
    if preset is None and iface is None:
        cands = _interfaces()
        print("\n网口：")
        for i, it in enumerate(cands, 1):
            print(f"  {i}) {it['name']:<6s} {it['note']}")
        n = _pick(cands,
                  default_no=1,
                  prompt=(f"  选择 1-{len(cands)}，回车 = 1："
                          if len(cands) > 1 else "  回车继续："))
        iface = cands[n - 1]["name"]
        if not sys.stdin.isatty():
            print(f"  非交互终端，用 {iface}")
    return resolve_endpoint(preset, iface=iface, domain=domain)


def choose_client(endpoint: Endpoint, want: str | None = None) -> str:
    """定下用哪个控制端。`want` 给了就校验，没给就问一句。

    只有一个候选时也停下来问：让人看清自己连的是哪个控制器，回车即可，
    比「程序自己决定了」清楚。`--client` 写错或环境不支持时抛 ValueError，
    消息里说明这个环境到底有哪几个。
    """
    options = clients_for(endpoint.preset)
    where = "仿真" if endpoint.sim else "真机"

    if want:
        if want not in CLIENT_NAMES:
            raise ValueError(f"没有叫 {want!r} 的控制端，可选：{', '.join(CLIENT_NAMES)}")
        if want not in options:
            avail = " / ".join(CLIENT_NAMES[k] for k in options)
            raise ValueError(f"{where}只能用{avail}，没有{CLIENT_NAMES[want]}。")
        return want

    print("\n控制端：")
    for i, kind in enumerate(options, 1):
        print(f"  {i}) {CLIENT_DESC[kind]}")
    if len(options) == 1 and endpoint.sim:
        print("  仿真桥只有 rt/lowcmd 和 rt/sportmodestate，没有运动服务，"
              "官方控制器在这边连不上，所以只给这一个。")

    default = "ours" if endpoint.sim else "official"
    n = _pick(list(options),
              default_no=options.index(default) + 1,
              prompt=(f"  选择 1-{len(options)}，回车 = {options.index(default) + 1}："
                      if len(options) > 1 else "  回车继续："))
    if not sys.stdin.isatty():
        print(f"  非交互终端，按默认选：{CLIENT_NAMES[default]}")
    return options[n - 1]


def connect(endpoint: Endpoint, kind: str = "ours", **gains) -> Backend:
    """连上去。返回时机器人已经 BalanceStand。

    `gains`（kp / kd / tau）只对自研控制端有意义，官方那份跑在机器上，
    给了也不会用到 —— 传下去由实现自己忽略。
    """
    from quadruped.control.virtual.backends import BACKENDS   # 延迟：见模块说明

    try:
        cls = BACKENDS[kind]
    except KeyError:
        raise ValueError(f"没有叫 {kind!r} 的控制端") from None
    return cls(endpoint, **gains)


# ---- 命令行参数 ------------------------------------------------------------
# 每个控制器都要这几个参数，抄一遍就会长得不一样（有人叫 --net，有人叫
# --interface），所以在这里注册。

def add_endpoint_args(parser) -> None:
    """给控制器的 ArgumentParser 加上 --iface / --preset / --domain / --client。"""
    g = parser.add_argument_group("机器人")
    g.add_argument("--iface", default=None,
                   help="网卡名：lo0 = 仿真（域号读仿真器 config.py），其它 = 真机 domain 0")
    g.add_argument("--preset", choices=("sim", "real"), default=None,
                   help="快捷方式：sim=lo0/domain1，real=en5/domain0")
    g.add_argument("--domain", type=int, default=None,
                   help="直接指定 DDS 域号，最高优先级")
    g.add_argument("--client", choices=tuple(CLIENT_NAMES), default=None,
                   help="跳过询问，直接用指定控制端（仿真只有 ours 可用）")
