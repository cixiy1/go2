"""官方高层控制器：直接调机器狗上的 sport service，一行也不经我们的控制回路。

和自研那份（`ours.py`）同名同义，二选一：

    自研  200 Hz 回路，往 `rt/lowcmd` 发关节目标 —— 算法在我们这边
    官方  `rt/api/sport/request` 发请求 —— 算法在机器上（宇树的 MPC + WBC）

`physical/keyboard.py`、`embodied/*` 只按这套名字调，不关心连的是哪个；
换的地方在 `control/virtual/robot.py` 的 `connect(ep, kind)`。

## 服务清单

按《运控服务接口》整理，括号里是 api_id。

基础（1001~1010）
    Damp、BalanceStand、StopMove、StandUp、StandDown、RecoveryStand、
    Euler(1007)、Move(1008)、Sit(1009)、RiseSit(1010)

姿态与步幅（1013~1015）
    BodyHeight(1013)、FootRaiseHeight(1014)、SpeedLevel(1015)

步态与模式（1011、1018、1019、1061、1062、2045~2051）
    SwitchGait、ContinuousGait、TrajectoryFollow、
    TrotRun、StaticWalk、FreeWalk、ClassicWalk、FreeBound、FreeJump、
    FreeAvoid、WalkUpright、CrossStep、HandStand、SwitchAvoidMode

特技（1016、1017、1020、1022、1023、1029~1032、1036、2041、2043）
    Hello、Stretch、Content、Dance1、Dance2、Scrape、
    FrontFlip、FrontJump、FrontPounce、Heart、LeftFlip、BackFlip

配置（1027、2054、2055）
    SwitchJoystick、AutoRecoverySet、AutoRecoveryGet

## 两档风格与自研对齐

自研那份的 `mech` / `bio` 在官方这里落到两个运动模式上，名字照抄，键盘的
Tab 因此换后端也不用改：

    mech  标准小跑（对角腿交替，稳）  →  FreeWalk    灵动
    bio   横向序列疾驰（快，姿态低）  →  FreeBound   并腿跑

两者不是同一种步法：官方的并腿跑是前后两对腿交替蹬地，自研的疾驰是四条腿
按左后→右后→左前→右前错开。对齐的是角色 —— 都代表「比标准小跑快的那一档」。

## 这份 SDK 缺的五条

`unitree_sdk2py` 的 Go2 `SportClient` 没注册 SwitchGait(1011)、
BodyHeight(1013)、FootRaiseHeight(1014)、TrajectoryFollow(1018)、
ContinuousGait(1019)，而服务端认这些 id。这里在 `Init()` 里补注册，
方法直接走 `_Call(api_id, json)`。参数格式照
`unitree_sdk2py/b2/sport/sport_client.py` 里同 id 的那几个 —— 同一份 SDK 的
B2 版有它们，api_id 也对得上。

## 不实现的四条

文档列了但拿不到 Go2 的 api_id，所以留着 `NotImplementedError`：

    MoveToPos   B2 是 1036，而 Go2 的 1036 已经给了 Heart，两个撞车
    GetState    文档只给了 C++ 签名（vector + map），没给 id
    Wallow      无据可查
    WalkStair   无据可查

不猜 id 是有理由的：请求发错 id 不会报「未知指令」，而是可能触发一个空翻。
"""
from __future__ import annotations

import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SPORT_PATH_POINT_SIZE, SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

# Go2 版 SportClient 缺、但服务端认的 api_id（来源：B2 版 sport_api.py）
API_SWITCH_GAIT = 1011
API_BODY_HEIGHT = 1013
API_FOOT_RAISE_HEIGHT = 1014
API_TRAJECTORY_FOLLOW = 1018
API_CONTINUOUS_GAIT = 1019

#: 返回码 → 人话。0 是成功。3xxx 来自 SDK 的 RPC 层，4xxx 来自文档。
ERRORS = {
    3001: "未知错误",
    3102: "请求发不出去（这个域里没有 sport 服务，仿真就是这样）",
    3103: "api 没注册（先调 Init，或者这条 id 服务端不认）",
    3104: "DDS 超时（对端没响应，先确认运动服务在跑）",
    3105: "api 版本不匹配（SDK 与机器固件差太远）",
    4101: "轨迹点数错误（TrajectoryFollow 必须正好 30 个点）",
    4201: "动作超时（期望时间内没做完）",
    4205: "状态机未初始化结束",
    4206: "姿态不佳，拒绝执行挥手/拜年类动作",
}

#: 运动模式（SportModeState.mode）
MODES = {
    0: "idle", 1: "balanceStand", 2: "pose", 3: "locomotion", 5: "lieDown",
    6: "jointLock", 7: "damping", 8: "recoveryStand", 10: "sit",
    11: "frontFlip", 12: "frontJump", 13: "frontPounce",
}

#: 步态（SportModeState.gait_type）
GAITS = {0: "idle", 1: "trot", 2: "run", 3: "climbStair", 4: "downStair",
         9: "adjust"}

#: SwitchGait(int d) 的取值
GAIT_SWITCH = {0: "idle", 1: "trot", 2: "trot_running",
               3: "climb_stair", 4: "back_down_stair"}


# 把返回码翻成人话，查不到就原样返回。日志里别只打数字。
def explain(code: int) -> str:
    if code == 0:
        return "ok"
    return ERRORS.get(code, f"错误码 {code}")


class _SportClient(SportClient):
    """补注册 Go2 版缺的那五条 api。

    `Client._Call` 会先查注册表，没注册的 id 直接返回
    `RPC_ERR_CLIENT_API_NOT_REG`，连请求都不会发。所以「服务端支持但我们没注册」
    和「服务端不支持」在代码里是两件事，前者补一下就通。
    """

    EXTRA_APIS = (API_SWITCH_GAIT, API_BODY_HEIGHT, API_FOOT_RAISE_HEIGHT,
                  API_TRAJECTORY_FOLLOW, API_CONTINUOUS_GAIT)

    def Init(self):
        super().Init()
        for api_id in self.EXTRA_APIS:
            self._RegistApi(api_id, 0)


class OfficialHighLevel:
    """官方高层控制器：把机器上的运动服务全量包一层。

    构造即连接：`ChannelFactoryInitialize` + 订阅 `rt/sportmodestate` +
    `SportClient.Init()`。一个进程只能 `ChannelFactoryInitialize` 一次，
    所以同进程里不能再建第二个 `OfficialHighLevel`，也不能和自研控制器混用。
    """

    API_OK = 0
    PATH_SIZE = SPORT_PATH_POINT_SIZE

    # 两档风格，和自研那份的 mech / bio 一一对应，落到官方的两个运动模式上：
    #   mech  灵动（FreeWalk）  —— 默认小跑，稳，日常走动
    #   bio   并腿跑（FreeBound）—— 前腿一对、后腿一对交替蹬地，快，姿态低
    # 名字对齐是为了让键盘的 Tab 在两种后端上都能用；模式是机器上的，不是参数。
    GAIT_STYLES = ("mech", "bio")
    GAIT_STYLE_NAMES = {"mech": "灵动（FreeWalk）", "bio": "并腿跑（FreeBound）"}
    GAIT_MODES = {"mech": "FreeWalk", "bio": "FreeBound"}

    def __init__(self, iface: str, domain: int, *, timeout: float = 3.0):
        self.iface = iface
        self.domain = domain
        self.state = None                      # 最近一帧 rt/sportmodestate
        self.gait_style = "mech"               # 当前档，见 GAIT_STYLES
        ChannelFactoryInitialize(domain, iface)
        self._suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self._suber.Init(self._on_state, 10)
        self.client = _SportClient()
        self.client.SetTimeout(timeout)
        self.client.Init()

    def _on_state(self, msg):
        self.state = msg

    # 给缺方法的那几条 api 用。参数统一 `{"data": ...}` 或散装键值。
    def _call(self, api_id: int, **data) -> int:
        code, _ = self.client._Call(api_id, json.dumps(data))
        return code

    # -- 与自研同名同义：连接与基础姿态 ---------------------------------

    # 通道在构造时就建好了，这里重发一次 `SportClient.Init()` 无害。
    def Init(self) -> int:
        return self.client.Init() or self.API_OK

    def SetTimeout(self, seconds: float) -> int:
        self.client.SetTimeout(seconds)
        return self.API_OK

    # 阻尼状态。电机关节全部失力，优先级最高，用于急停。
    def Damp(self) -> int:
        return self.client.Damp()

    # 平衡站立。姿态与高度不受地形影响，可配合 Euler / BodyHeight 用。
    def BalanceStand(self) -> int:
        return self.client.BalanceStand()

    # 停下当前动作，把内部运动参数恢复默认值。
    def StopMove(self) -> int:
        return self.client.StopMove()

    # 关节锁定站高，默认 0.33 m。姿态不再自适应地形。
    def StandUp(self) -> int:
        return self.client.StandUp()

    # 关节锁定趴下。
    def StandDown(self) -> int:
        return self.client.StandDown()

    # 从翻倒或趴下恢复到平衡站立。
    def RecoveryStand(self) -> int:
        return self.client.RecoveryStand()

    # 坐下。特殊动作，必须等上一个动作做完。
    def Sit(self) -> int:
        return self.client.Sit()

    # 从坐下恢复到平衡站立。
    def RiseSit(self) -> int:
        return self.client.RiseSit()

    # -- 速度与姿态 ------------------------------------------------------

    # 机体坐标系速度。vx ∈ [-2.5, 3.8]、vy ∈ [-1, 1]、vyaw ∈ [-4, 4]。
    #
    # 无回复调用：机器会一直保持这个速度，直到下一条指令或 StopMove。
    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        return self.client.Move(vx, vy, vyaw)

    # 机体姿态角，单位 rad。roll/pitch ∈ [-0.75, 0.75]、yaw ∈ [-0.6, 0.6]。
    def Euler(self, roll: float, pitch: float, yaw: float) -> int:
        return self.client.Euler(roll, pitch, yaw)

    # 机身高度相对默认值的偏移，∈ [-0.18, 0.03] m（默认 0.33 m）。
    #
    # `BodyHeight(-0.1)` = 降到 0.23 m。这是自研那份 `crouch_async` 的落点。
    def BodyHeight(self, height: float) -> int:
        return self._call(API_BODY_HEIGHT, data=height)

    # 抬足高度相对偏移，∈ [-0.06, 0.03] m（默认 0.09 m）。
    def FootRaiseHeight(self, height: float) -> int:
        return self._call(API_FOOT_RAISE_HEIGHT, data=height)

    # 速度档位：-1 慢 / 0 正常 / 1 快。
    def SpeedLevel(self, level: int) -> int:
        return self.client.SpeedLevel(level)

    # -- 步态与运动模式 --------------------------------------------------

    # 切步态：0 idle / 1 trot / 2 trot running / 3 正向爬楼 / 4 逆向爬楼。
    def SwitchGait(self, gait: int) -> int:
        return self._call(API_SWITCH_GAIT, data=gait)

    # 持续步态：速度为 0 时仍原地踏步（要「一直动」的场合用）。
    def ContinuousGait(self, enable: bool = True) -> int:
        return self._call(API_CONTINUOUS_GAIT, data=enable)

    # 小跑。
    def TrotRun(self) -> int:
        return self.client.TrotRun()

    # 静态步行（慢，但稳）。
    def StaticWalk(self) -> int:
        return self.client.StaticWalk()

    # 经典步行。false 退出回灵动。
    def ClassicWalk(self, enable: bool = True) -> int:
        return self.client.ClassicWalk(enable)

    # 灵动模式。跳跃/空翻结束会自动落到这个模式。
    def FreeWalk(self) -> int:
        return self.client.FreeWalk()

    # 并腿跑。
    def FreeBound(self, enable: bool = True) -> int:
        return self.client.FreeBound(enable)

    # 跳跃模式。
    def FreeJump(self, enable: bool = True) -> int:
        return self.client.FreeJump(enable)

    # 闪避模式：移动时避障，静止时对前方物体闪避。
    def FreeAvoid(self, enable: bool = True) -> int:
        return self.client.FreeAvoid(enable)

    # 后腿直立。
    def WalkUpright(self, enable: bool = True) -> int:
        return self.client.WalkUpright(enable)

    # 交叉步。
    def CrossStep(self, enable: bool = True) -> int:
        return self.client.CrossStep(enable)

    # 倒立行走。需要先处于可移动状态，建议先 BalanceStand。
    def HandStand(self, enable: bool = True) -> int:
        return self.client.HandStand(enable)

    # 切换避障模式。
    def SwitchAvoidMode(self) -> int:
        return self.client.SwitchAvoidMode()

    # -- 特技 ------------------------------------------------------------

    def Hello(self) -> int:
        return self.client.Hello()

    def Stretch(self) -> int:
        return self.client.Stretch()

    def Content(self) -> int:
        return self.client.Content()

    def Heart(self) -> int:
        return self.client.Heart()

    # 拜年作揖。要求当前姿态良好，否则返回 4206。
    def Scrape(self) -> int:
        return self.client.Scrape()

    def Dance1(self) -> int:
        return self.client.Dance1()

    def Dance2(self) -> int:
        return self.client.Dance2()

    def FrontFlip(self) -> int:
        return self.client.FrontFlip()

    def FrontJump(self) -> int:
        return self.client.FrontJump()

    def FrontPounce(self) -> int:
        return self.client.FrontPounce()

    # 左空翻。要求非运动状态，结束后自动进灵动模式。
    def LeftFlip(self) -> int:
        return self.client.LeftFlip()

    # 后空翻。要求非运动状态，结束后自动进灵动模式。
    def BackFlip(self) -> int:
        return self.client.BackFlip()

    # -- 轨迹与配置 ------------------------------------------------------

    # 跟一条轨迹。`path` 必须是 30 个 `PathPoint`（tFromStart/x/y/yaw/vx/vy/vyaw）。
    #
    # 参考系是机器人的绝对坐标系；点数不对直接返回 4101，不发请求。
    def TrajectoryFollow(self, path: list) -> int:
        if len(path) != self.PATH_SIZE:
            return 4101

        points = []
        for p in path:
            points.append(dict(t_from_start=p.timeFromStart, x=p.x, y=p.y,
                               yaw=p.yaw, vx=p.vx, vy=p.vy, vyaw=p.vyaw))
        code = self.client._CallNoReply(API_TRAJECTORY_FOLLOW, json.dumps(points))
        return code

    # 原生遥控器是否响应。关掉之后遥控器不干扰程序。
    def SwitchJoystick(self, enable: bool) -> int:
        return self.client.SwitchJoystick(enable)

    # 自动恢复开关（翻倒了自己爬起来）。
    def AutoRecoverySet(self, enable: bool) -> int:
        return self.client.AutoRecoverySet(enable)

    def AutoRecoveryGet(self) -> tuple[int, bool | None]:
        return self.client.AutoRecoveryGet()

    # -- 文档里有、拿不到 id 的 ------------------------------------------
    # 这四条一律抛 NotImplementedError：发错 api_id 不会报错，会直接触发一个动作。

    # 走到指定点。B2 上是 1036，Go2 的 1036 已经给了 Heart，撞车，不猜。
    def MoveToPos(self, x: float, y: float, yaw: float) -> int:
        raise NotImplementedError(
            "MoveToPos 在 B2 上是 api 1036，而 Go2 的 1036 给了 Heart，两个撞车。"
            "要走到指定点，用自研那份的 forward/strafe/turn（带里程计闭环）。")

    # 取内部状态表。文档只给了 C++ 签名，没给 api_id；状态读 rt/sportmodestate 就够。
    def GetState(self) -> int:
        raise NotImplementedError(
            "GetState 的 C++ 签名是 (vector<string>, map<string,string>)，"
            "没有 api_id。要状态就直接读 rt/sportmodestate，见本类的 mode / "
            "gait_type / height / velocity。")

    # 打滚。任何文档都没列 Go2 的 id。
    def Wallow(self) -> int:
        raise NotImplementedError("Wallow（打滚）查不到 Go2 的 api_id，不猜。")

    # 上楼梯。同样没有 id；要爬楼用 SwitchGait(3) / SwitchGait(4)。
    def WalkStair(self, enable: bool = True) -> int:
        raise NotImplementedError(
            "WalkStair 查不到 Go2 的 api_id。爬楼用 SwitchGait(3) / SwitchGait(4)。")

    # -- 兼容自研那份的异步名字 ------------------------------------------
    # 官方服务是同步 RPC，本来就不阻塞腿，所以这些只是名字对齐。

    def stand_async(self) -> int:
        return self.BalanceStand()

    def stop_async(self) -> int:
        return self.StopMove()

    # 停步并站定。
    def freeze(self) -> int:
        self.StopMove()
        return self.BalanceStand()

    # 前跳。自研那份可以调下蹲深度与蹬腿幅度，官方只有固定的一套。
    def jump_async(self, crouch: float = 0.0, push: float = 0.0) -> int:
        return self.FrontJump()

    # 蹲低 `depth` 米。官方没有关节级的蹲，用 BodyHeight 降机身。
    #
    # 想彻底坐下用 `Sit()`，那是另一个姿势（后腿折叠、前腿撑地）。
    def crouch_async(self, depth: float = 0.08) -> int:
        return self.BodyHeight(-abs(depth))

    def uncrouch_async(self) -> int:
        return self.BodyHeight(0.0)

    # 收工：停步。原生运动服务自己有平衡环，停住就行，不需要 Damp 趴下。
    def park(self) -> int:
        return self.StopMove()

    # -- 风格：与自研同名，落到官方的运动模式上 --------------------------

    # 当前档的名字，键盘状态行取「（」前面那段，所以括号里放英文模式名。
    @property
    def gait_style_label(self) -> str:
        return self.GAIT_STYLE_NAMES[self.gait_style]

    # 换档：mech 发 FreeWalk，bio 发 FreeBound。返回换完之后的档位键。
    #
    # 官方的这两档是运动模式，不是参数：切过去机器会重新规划步态。请求没送到
    # （仿真里没有 sport 服务，回 3102）就不改本地档位 —— 状态行显示一个机器
    # 实际没进去的模式，比不显示更误导。
    def set_gait_style(self, style: str) -> str:
        if style not in self.GAIT_STYLES:
            raise ValueError(f"没有这种运动步态：{style}，只有 {self.GAIT_STYLES}")
        code = self.FreeWalk() if style == "mech" else self.FreeBound(True)
        if code != self.API_OK:
            print(f"切 {self.GAIT_MODES[style]} 失败：{explain(code)}", flush=True)
            return self.gait_style
        self.gait_style = style
        return style

    # 在 mech / bio 之间来回切，返回切完之后的标签，调用方直接拿去显示。
    def toggle_gait_style(self) -> str:
        i = self.GAIT_STYLES.index(self.gait_style)
        self.set_gait_style(self.GAIT_STYLES[(i + 1) % len(self.GAIT_STYLES)])
        return self.gait_style_label

    # 跳跃只有 FrontJump 一种，没有档可切。留着同名属性返回 None，
    # 调用方按「没有标签」处理，不用给后端分支。
    @property
    def jump_style_label(self):
        return None

    def toggle_jump_style(self):
        return None

    # -- 状态 ------------------------------------------------------------

    @property
    def pos(self):
        return tuple(self.state.position) if self.state is not None else None

    @property
    def rpy(self):
        return tuple(self.state.imu_state.rpy) if self.state is not None else None

    # 机体高度，直接读状态里的 body_height。
    #
    # 仿真桥不填这个字段（一直是 0），所以 0 当作「没有」返回 None ——
    # 拿 position[2] 冒充会差一个 base_link 的常数，宁可说没有。
    @property
    def height(self):
        if self.state is None or self.state.body_height <= 0.01:
            return None
        return self.state.body_height

    @property
    def velocity(self):
        return tuple(self.state.velocity) if self.state is not None else None

    @property
    def mode(self) -> str:
        return MODES.get(self.state.mode, "?") if self.state is not None else "?"

    @property
    def gait_type(self) -> str:
        return GAITS.get(self.state.gait_type, "?") if self.state is not None else "?"

    @property
    def foot_force(self):
        return tuple(self.state.foot_force) if self.state is not None else None

    # 动作进行中（progress=1）。站着的狗一直 false，和自研那份语义一致。
    @property
    def busy(self) -> bool:
        return bool(self.state is not None and self.state.progress)

    # 一行状态，日志和调试用。
    def status_text(self) -> str:
        if self.state is None:
            return "还没有 rt/sportmodestate（服务没起或域号不对）"
        rpy = ", ".join(f"{v:+.2f}" for v in self.state.imu_state.rpy)
        v = ", ".join(f"{x:+.2f}" for x in self.state.velocity)
        h = "?" if self.height is None else f"{self.height:.3f}m"
        return (f"{self.mode}/{self.gait_type} 高 {h} "
                f"rpy=({rpy}) v=({v}) 动作中={self.busy}")
