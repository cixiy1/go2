"""官方动作的两张表：能执行的、只在界面上出现的。

单独一个模块，是因为两张表被两个进程读：`comm/server.py` 只认 `ACTS`（能不能
调用），`comm/gateway.py` 把 `act_catalog()` 塞进 welcome 发给客户端（界面怎么
画）。放一处，服务端和界面就不会各抄一份、各漏一个。

## 为什么界面上要出现不能点的动作

后空翻、倒立这些，落地姿态不可控，场地不够就是摔机。但界面上只有十二个「站着
扭两下」的动作、一个高难度的都没有，看起来像是客户端没做全 —— 实际是刻意关的。
所以留着灰色的条目，点了弹一句「为什么不能点」，比直接不显示诚实。

要真跑这些：用官方 App，那边默认你有场地和护具。
"""
from __future__ import annotations

#: `act` 允许调用的官方动作（`SportClient` 的同名方法）。
#:
#: 全部是「腿还在下面撑着」的动作，做完还能接着遥控。刻意排除的：
#:   FrontFlip / BackFlip / LeftFlip  空翻，落地姿态不可控
#:   HandStand                        倒立，倒了很难自己起来
#:   WalkUpright / CrossStep / StaticWalk  单腿或少腿支撑，对地面要求高
#:   FreeJump / FreeBound / FreeAvoid  自由模式，参数不对会乱蹦
#: 要玩这些请用官方 App —— 那边有场地和护具的前提假设。
# `Hello`（api 1016）在 Go2 上实测返回 3104（DDS 超时，对端不响应）——
# 那是 Go1 的动作。`Sit` / `RiseSit` 实测可用，其余待逐个验。
ACTS = (
    "Stretch", "Heart", "Content", "Pose", "Scrape",
    "Sit", "RiseSit",
    "StandUp", "StandDown", "BalanceStand", "RecoveryStand", "StopMove",
)


#: 动作目录。客户端拿它渲染「更多动作」，不在客户端硬编码一份。
#:
#: 每项：名字、中文标签、分组、能不能执行、一句备注。
#: 分组只有两个：`pose` 姿态（腿都撑在地上）、`show` 表演（原地扭几下）、
#: `risk` 高风险（默认关，只展示）。
#: `enabled=False` 的按钮是灰的、点了只弹一句说明 —— 界面上看得见这些能力，
#: 但不能给一个按一下就真摔的按钮。
ACTS_CATALOG = (
    ("StandUp", "站起", "pose", True, ""),
    ("StandDown", "趴下", "pose", True, ""),
    ("BalanceStand", "平衡站立", "pose", True, ""),
    ("RecoveryStand", "恢复站立", "pose", True, "倒了之后自己爬起来"),
    ("Sit", "坐下", "pose", True, ""),
    ("RiseSit", "从坐姿起立", "pose", True, ""),
    ("StopMove", "停住", "pose", True, ""),
    ("Stretch", "伸懒腰", "show", True, ""),
    ("Heart", "比心", "show", True, ""),
    ("Content", "开心", "show", True, ""),
    ("Pose", "摆姿势", "show", True, ""),
    ("Scrape", "刨地", "show", True, "姿态不好会被拒（4206）"),
    ("FrontFlip", "前空翻", "risk", False, "前后各要 3 米空地，落地姿态不可控"),
    ("BackFlip", "后空翻", "risk", False, "起跳后翻 360°，高度不够会砸到背"),
    ("LeftFlip", "左空翻", "risk", False, "侧向空间不够会撞"),
    ("HandStand", "倒立", "risk", False, "两腿支撑，倒了很难自己起来"),
    ("WalkUpright", "直立行走", "risk", False, "只有后腿撑着，地面不平就倒"),
    ("CrossStep", "交叉步", "risk", False, "单腿交替支撑，地面滑会劈叉"),
    ("FreeJump", "自由跳跃", "risk", False, "高度和距离不可控"),
    ("FreeBound", "并腿跑", "risk", False, "四脚同时腾空，地面滑会摔"),
    ("Dance1", "舞一", "risk", False, "要 2 米见方的场地，没验过"),
    ("Dance2", "舞二", "risk", False, "要 2 米见方的场地，没验过"),
)


def act_catalog() -> list:
    """动作目录，给客户端。字段名短一点，这帧每两秒就要发一遍。"""
    return [
        {"name": n, "label": l, "group": g, "enabled": e, "why": w}
        for (n, l, g, e, w) in ACTS_CATALOG
    ]


# 可执行的目录项必须和白名单一致。两张表分开写是为了让「能点的」和「能看的」
# 各自读得懂，但漏一个就会变成「界面能点、服务端拒绝」—— 导入时就炸掉。
if {n for (n, _, _, e, _) in ACTS_CATALOG if e} != set(ACTS):
    raise AssertionError("ACTS_CATALOG 里可执行的动作和 ACTS 白名单对不上")
