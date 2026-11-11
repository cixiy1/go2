#!/usr/bin/env python3
"""Go2 控制器（第 2 层）：DDS 通信、PD、解析 IK、足端轨迹、唯一控制回路。

三个文件对应三层，父对象在各自的 __init__ 里构造子对象，没有别名：

    go2_robot.py        Go2Robot              .controller →  本体：预设 / 连接 / 传感器 / 收工
    go2_controller.py   Go2Controller         .high_level →  底层：DDS / PD / IK / 轨迹 / 控制回路
    go2_high_level.py   HighLevelController   .controller →  高层：基础动作 + 距离角度闭环

本文件是第 2 层，也是运动学与 DDS 通信的唯一实现处：

    Go2Controller.fk_leg("FL", (0.0, 0.9, -1.8))        # 正解，静态方法，不碰 DDS
    Go2Controller.ik_leg("FL", (0.1934, 0.142, -0.248))

    go2 = Go2Controller(domain=1, iface="lo0")          # 仿真
    go2.connect(); go2.stand(duration=5)

直接运行本文件 = 跑自检（MuJoCo 当裁判，不需要仿真器）：

    .venv/bin/python quadruped/control/controller.py

===========================================================================
运动学
===========================================================================

坐标系：机体系 base —— x 前、y 左、z 上，原点在 base_link。
足端参考点取 *_foot body 的原点（接触球心在它下方 0.022 m，见 FOOT_RADIUS）。

关节约定与 MJCF（unitree_robots/go2/go2.xml）逐位对应：

    q1  abduction   绕 +x
    q2  thigh       绕 +y，正角度把足端甩向 -x（向后）
    q3  knee        绕 +y，负角度屈膝

运动链（以左前腿为例）：

    base_link
      └─ FL_hip     pos=(+0.1934, +0.0465, 0)     ← 髋部原点 H
          │  [q1 绕 +x]
          └─ FL_thigh  pos=(0, +0.0955, 0)        ← 髋俯仰关节 T0
              │  [q2 绕 +y]
              └─ FL_calf  pos=(0, 0, -0.213)      ← 膝关节
                  │  [q3 绕 +y]
                  └─ FL_foot  pos=(0, 0, -0.213)  ← 足端

abduction 不是「绕 x 转一下再往下走」，它把后面整条腿的俯仰平面一起带偏，
所以 q1 必须从足端相对髋部原点的 y-z 投影解出来：

    Rx(q1) @ (0, s·d, vz) = (py, pz)          d = 0.0955, s = ±1
  ⇒ |(py, pz)|² = d² + vz²  ⇒  vz = -√(py²+pz²-d²)
  ⇒ q1 = atan2(pz, py) - atan2(vz, s·d)

俯仰平面内再解两连杆，膝关节用余弦定理，大腿角用相位差：

    cos q3 = (L² - l1² - l2²) / (2·l1·l2)     L = 足端到髋俯仰关节的距离
    q2 = φ_thigh_frame - φ_target

===========================================================================
足端轨迹
===========================================================================

仿真是离散的、控制器是定频的，所以轨迹必须是时间的光滑函数，而不是一串
关键帧插值 —— 后者在切换点速度会跳变，关节会挨一次冲击。

两段式：
  支撑相 stance  足端相对机身以 -v 匀速后移，脚不离地
  摆动相 swing   从离地点抬脚跨到下一个落地点

摆动相为什么不能用摆线：摆线 x(s)=s-sin(2πs)/2π 两端速度都是 0，但支撑相里
足端相对机身是以 -v 匀速后退的。落地瞬间摆动末速度 0、支撑初速度 -v，差一个 v，
物理含义就是足端带着 +v 的相对速度砸向地面 —— 落地就打滑，速度越快越明显。

对水平方向用四次多项式，约束：

    f(0)=0  f(1)=1  f'(0)=0  f''(0)=0  f'(1)=u      u = -(1-duty)/duty

解得 f(s)=5s³-4s⁴（duty=0.5 特例）。f'(s)=15s²-16s³ 在 s=0.9375 过零，
所以 f 会先冲过头 3% 再退回 —— 这正是「伸腿迎地、触地前回收」的动作，
7.5cm 步幅下过冲只有 2mm。

竖直方向用升余弦，两端速度为 0：足端垂直离地、垂直落地，没有冲击。
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread

# --- 连杆参数与整机质量，来源是 model.py（它自己对着 go2.xml 校验）----------
# 别在这里另写一份：几何错 1 mm，IK 差几度；质量错 1 kg，重力前馈差 10 N。
from .model import (CALF_LEN, FOOT_RADIUS, HIP_X, HIP_Y, LEG_GEOM,  # noqa: F401
                    THIGH_LEN, THIGH_Y, TOTAL_MASS)

NUM_MOTOR = 12

# 仿真里的执行器顺序！go2.xml 的 actuator 与 sensor 都是 FR, FL, RR, RL，
# 不是宇树习惯的 FL, FR, RL, RR。搞错的话四条腿会乱蹬。
CTRL_LEGS = ("FR", "FL", "RR", "RL")

# 腿部顺序（几何侧），与 LEG_GEOM（来自 model.py）一致
LEGS = ("FL", "FR", "RL", "RR")

LIMITS = {
    "abduction": (-1.0472, 1.0472),
    "thigh": (-1.5708, 3.4907),        # 前腿
    "knee": (-2.7227, -0.83776),
}
LIMITS_REAR_THIGH = (-0.5236, 4.5379)  # 后腿前限更宽松

# 站立姿态，与 go2.xml 的 <key name="home"> 一致
STAND_Q = (0.0, 0.9, -1.8)

# 标称机身高度（足底贴地）。MuJoCo 静止实测 0.276，取 0.27。
NOMINAL_HEIGHT = 0.27

# 蹲到底、还能保持四脚着地的最低机身高度。再低小腿会顶到机身。
MIN_HEIGHT = 0.15

# rt/sportmodestate 的 position 不是 base_link 原点，而是仿真里多抬了约 3.2 cm 的
# frame 点：站立指令 0.270 时读到 0.302，蹲到 0.190 时读到 0.220。
# 读高度、比高度的时候一律减掉它，否则会以为腿短了 3 cm。
SPORT_Z_BIAS = 0.032

MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),   # quadruped/
    "sim", "unitree_mujoco", "unitree_robots", "go2", "go2.xml")


# ===========================================================================
# 步态参数
# ===========================================================================

# 抬脚曲线 0→1→0，快抬慢落。
#
# 原来是 (1−cos2πs)/2：两端导数都是 0，也就是起飞最慢、落地最慢。
# 仿真里没问题（脚是理想刚体），真机上这一条很致命：Go2 足端是橡胶球，
# 支承相还主动压地几毫米，压进去之后和地面是「粘」住的。起飞那一下的
# 垂直速度接近 0，腿拉不动这块橡胶，位置环只能硬抗 —— 关节顶着 10 N·m
# 以上出不了位移，机身被反作用撞得一震一震，同时脚在地面干磨，就是
# 「硬摩擦声 + 高频震动 + 走不动」这三件事。
#
# 改成前 40% 抬到顶（正弦上升，起飞那一刻斜率最大）、后 60% 落下
# （余弦下降，落地仍然柔和）。峰值处两段导数都是 0，接得上。
def _raise(s: float) -> float:
    if s < 0.4:
        return math.sin(math.pi * s / 0.8)
    return math.cos(math.pi * (s - 0.4) / 1.2)


# 摆动相水平四次多项式 f(s)=a3·s³ + a4·s⁴ 的两个系数。
#
# 约束 f(0)=f'(0)=f''(0)=0, f(1)=1, f'(1)=u, u=-(1-duty)/duty
# 解出 a3 = 4-u, a4 = u-3。
def _swing_poly_coeffs(duty: float) -> tuple[float, float]:
    u = -(1.0 - duty) / duty
    return 4.0 - u, u - 3.0


class GaitParams:
    """步态参数。

    两档风格，和跳跃风格同名同义，`offsets` 决定谁跟谁同相：

    - `mech` 机械：对角腿一组（FL+RR / FR+RL），占空比 0.5，任何时刻都有两条
      腿着地。就是最初那套标准小跑：稳、省事，但看着像机器。对应官方的灵动
      （FreeWalk）。
    - `bio`  仿生：前腿一组、后腿一组（FL+FR / RL+RR），两组差半个周期，占空比
      0.55，两组之间各留 5% 的四脚支撑、不腾空。对应官方的并腿跑（FreeBound）。

    另有一套 `gallop`（四腿按左后→右后→左前→右前错开）留做对照，不进切换序列。
    """

    def __init__(self, period=0.5, duty=0.5, swing_height=0.06,
                 offsets=None, neutral=None, name="trot",
                 pitch_amp=0.0, pitch_phase=0.0, bounce=0.0,
                 pitch_gain=0.0, roll_gain=0.0, pitch_rate=0.0,
                 roll_rate=0.0, stride_gain=1.0, lean=0.0,
                 stride_max=0.075, speed_range=None,
                 reach=0.0, reach_err=0.0, stance_sink=0.0,
                 stand_raise=0.0, damp_z=0.0, wrench=1.0,
                 speed_loop=0.0, stance_spread=0.0, mode="vmc"):
        if not 0.05 < duty < 0.95:
            raise ValueError(f"duty={duty} 不合理，应在 (0.05, 0.95)")
        self.period = period            # 一个完整步态周期，秒
        self.duty = duty                # 支撑相占比，trot 取 0.5
        self.swing_height = swing_height
        # 相位偏移：FL 与 RR 同相，FR 与 RL 同相，两组差半个周期
        self.offsets = offsets if offsets is not None else {
            "FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5,
        }
        self.neutral = (neutral if neutral is not None
                        else _neutral_feet(NOMINAL_HEIGHT, stance_spread))
        # 站宽外扩（米）：左右足端再各往外撑多少。
        #
        # trot 的支撑多边形横向就是两条对角腿的连线，重心正好落在这条线上
        # —— 横向稳定裕度是**零**。侧翻不是「撞到了」，是扰动没有衰减机制。
        # 把站宽拉开是最省事的一招：支撑基线变宽，倾覆临界角随之变大。
        #
        # 代价在 q1：`neutral_y` 那个位置正好是髋侧摆 q1=0 的构型，往外一撑
        # q1 就持续非零，髋侧摆电机得一直出力。所以不能贪，3.5 cm 实测最划算：
        # 5° 横坡上 roll 均值 −4.8°→−1.9°、峰峰 15.7°→4.9°，速度一点没掉。
        self.stance_spread = stance_spread
        self.name = name
        # 机身俯仰摆动幅度（rad）与相位。只有跳跑用：机身跟着前腿落地压下去、
        # 后腿蹬起来抬起来，狗跑起来就是这副样子。脚踩地时这个指令才有真效果，
        # 幅度给大了会把支撑腿顶穿，0.05 左右就够看得出来。
        self.pitch_amp = pitch_amp
        self.pitch_phase = pitch_phase
        # 竖直弹跳幅度（米）。支撑相里机身先压下去、再蹬起来，每组腿蹬一次，
        # 一个步态周期弹两下。没有这个，四脚永远贴地、腾空相名存实亡，
        # 「跳跑」就退化成膝盖弯曲幅度大一点的小跑。
        self.bounce = bounce
        # 俯仰反馈增益（米/弧度）。并腿跑时只有一端着地，机身会绕着那两条腿
        # 点头，反作用全花在抬头/低头上，一点都不剩给前进。量到低头就压前脚、
        # 量到抬头就压后脚，把机身按平了，步幅才推得动机身。
        self.pitch_gain = pitch_gain
        # 滚转反馈增益（米/弧度）。并腿跑一组腿着地时，机身除了点头还会向一侧
        # 滚 —— 四足不同步落地，或者路面稍有不平，滚转就会越晃越大。量到往哪边
        # 歪就压哪边的脚，把机身按平。左右是反对称给的，不改变支撑总高度。
        self.roll_gain = roll_gain
        # 角速度阻尼（米/(弧度/秒)）。只有角度反馈时，机身像挂在弹簧上：
        # 压过头、回弹、再压过头，落地一颤就荡起来。角速度项是阻尼，
        # 姿态变化越快给的反向修正越大，把摆动收住，这是压制抖动的主力。
        self.pitch_rate = pitch_rate
        self.roll_rate = roll_rate
        # 步幅增益：支撑相里脚该往回扫多少。打滑厉害时把这一项放大，脚在地上
        # 蹭得更快，机身前移才跟得上指令速度。
        self.stride_gain = stride_gain
        # 转弯靠侧倾：生物狗和猎豹跑起来转弯不是原地扭头，是整条身子朝弯内倾、
        # 外侧腿多蹬。这个系数就是「每 1 rad/s 的偏航指令，机身侧倾多少弧
        # 度」，足端落点不动、机身歪过去，重心自然被甩到弯内侧。
        self.lean = lean
        # 单腿步幅上限（米）。放大步幅能补打滑，但腿的水平工作空间就这么大，
        # 超了 IK 会削顶、脚在地上拖，反而翻 —— 高速时自动收到这个上限，
        # 剩下的差靠打滑吃掉，至少姿态是稳的。
        self.stride_max = stride_max
        # 这套步态「跑得起来」的速度窗口 (下限, 上限)。跳跑是给高速准备的：
        # 低于下限步幅太小，脚在地上蹭不出推力，实测 0.3 m/s 会直接前栽；
        # 高于上限腿的水平行程不够，会翻。所以把指令速度夹进这个窗口。
        self.speed_range = speed_range
        # 落点前移（秒）。整个足端轨迹沿 x 平移 reach × 速度：跑得越快，脚越往
        # 机身前方踩。狗和猎豹跑起来就是这样 —— 脚落在质心前方，身体越过脚的
        # 过程中支撑力把机身往前带，而不是落在重心底下原地蹭。平移是「落点」和
        # 「离地点」一起移，不改变行程长度，所以不会吃掉 stride_max 的余量。
        self.reach = reach
        # 速度误差反馈（秒）。实测比指令快就往更前面踩（支撑力制动 + 抬头），
        # 比指令慢就少踩一点（不停刹车）。这一项是 Raibert 那套落脚点自稳：
        # 速度偏差直接换成落点偏差，跑起来不会越跑越快、也不会前后晃。
        self.reach_err = reach_err
        # 支撑相压地量（米）。支撑腿不是「刚好贴地」而是主动往下压一点，
        # 保证足底有稳定的法向力 —— 没有法向力就没有摩擦，脚只会在地上打滑
        # 蹭出噪声，机身一步都推不动（后退时尤其明显：姿态稍有前倾，重力分量
        # 刚好抵消那点蹭出来的推力）。压 8 mm 换来稳定支撑，代价是机身低一点点。
        self.stance_sink = stance_sink
        # 跑起来时把机身再抬高多少（米）。腿伸得直一点，步幅才有余量，
        # 四腿错相那套（gallop）也不至于在腿弯到底的时候被顶翻，它取 1.5 cm。
        self.stand_raise = stand_raise
        # 支撑相足端速度的闭环增益。**这一项决定往前跑还是往后跑。**
        #
        # 原来支撑相脚的后扫速度按「指令速度」开环算：脚世界速度 = 实测机身
        # 速度 - 指令速度 × 增益。机身一旦被扰动（哪怕很小），这个差就变号 ——
        # 脚改成往前蹭，摩擦把机身往后推，机身更慢、差更大，正反馈直接把狗
        # 推着倒退（实测指令 +0.45 跑出 -0.34，后腿接触时脚在地上是往前滑的）。
        #
        # 改成闭环：脚该以多快后扫，看的是实测机身速度 v_meas 加上一个速度
        # 误差项 k·(v_cmd - v_meas)。机身被扰动慢了，误差项自动变大、脚扫得
        # 更狠把机身推回来；跑到指令速度，误差项归零、脚几乎不打滑。这样摩擦
        # 方向始终朝前，也就不存在「莫名其妙往后走」。
        self.speed_loop = speed_loop
        # 机身竖直速度阻尼（米 / (米/秒)）。四条腿撑在地面上其实是一个弹簧：
        # kp 110、单腿竖直刚度约 2500 N/m，两条腿撑着 15 kg，固有频率
        # sqrt(5000/15) ≈ 2.9 Hz。步频是 2.2 Hz —— 每一步落地都在这个共振点
        # 附近激一下，而腿上只有 kd 1.5 那点阻尼，机身就一直在 ±1.5 cm 地弹。
        # 实测 0.55 指令下机身高度峰峰 83 mm、俯仰峰峰 36°，看着就是「极其不稳」。
        # 这一项按机身竖直速度反向挪支撑脚：机身往上走就把脚往回收，把上升的
        # 那一下吃掉。0.03~0.05 相当于给这个模态 0.3~0.5 的阻尼比。
        self.damp_z = damp_z
        # 机身层控制权重：0 = 老路子（位置伺服 + 逐腿竖直前馈），
        # 1 = 全部交给 `body_wrench_tau` 分配合力/合力矩。中间值线性过渡，
        # 方便在同一套步态上做 A/B，也能在过渡期留一点位置伺服的兜底。
        #
        # 为什么要有这一层：位置伺服只能指定「脚在哪」，指定不了「出多大力」。
        # 承重和姿态都靠 kp 的静差凑，凑出来多少取决于地面刚度 —— 仿真地面
        # 硬，凑得出来；真机地面软、还不平，凑不出来就打滑。机身层直接把
        # 「要多少力」算清楚再下发，地面软硬只影响脚陷多深，不影响出力大小。
        self.wrench = wrench
        # 力路的实现，`"vmc" | "mpc"`。见 `_mpc_feet_publish` 上那段：
        #
        # - `vmc` 现役路径：`_wrench_cmd` 只按**当前这一帧**的姿态/高度/速度误差
        #   直接给出 6 维 wrench，再经 `body_wrench_tau` 逐腿分 Jᵀf。无预测、
        #   无摩擦锥、无腿自重与惯性项。
        # - `mpc` 自研路径：50 Hz 的 SRBD-MPC 出地面反力序列 → 200 Hz 的 WBC
        #   把它连同足端轨迹一起解成关节力矩。
        #
        # 默认 vmc：自研那一路还在 A/B 阶段，出问题要能一秒钟退回去。
        if mode not in ("vmc", "mpc"):
            raise ValueError(f"mode={mode!r} 不认识，只有 'vmc' 与 'mpc'")
        self.mode = mode
        self.a3, self.a4 = _swing_poly_coeffs(duty)

    # 一个周期里四脚全离地的时间。两组支撑不重叠就是腾空相。
    @property
    def aerial_time(self) -> float:
        return max(0.0, (1.0 - 2.0 * self.duty) * self.period)

    @property
    def stance_time(self) -> float:
        return self.period * self.duty

    @property
    def swing_time(self) -> float:
        return self.period * (1.0 - self.duty)

    # 一个支撑相内足端相对机身走过的距离 = 机身速度 × 支撑时间。
    def stride(self, vx: float, vy: float) -> tuple[float, float]:
        return vx * self.stance_time, vy * self.stance_time


# 旧名字映射到新的两档：trot 走 mech，bound 走 bio。gallop 没有别名，
# 它是留做对照的第三套（四腿错相），不进切换序列。
_GAIT_ALIASES = {"trot": "mech", "bound": "bio"}


# 按风格造一套步态参数。
#
# 对外两档，和跳跃风格 mech/bio 同名同义，两边始终一致。两档都照着官方的
# 两个运动模式取，名字能对上，调参才有参照：
#
# - `mech` 机械＝官方灵动（FreeWalk）：高层运动一直用的那套标准小跑（对角腿
#   交替、0.5 s 周期、抬脚 6 cm、步幅 7.5 cm）。原本的参数，没有任何反馈和
#   速度闭环。慢，但每一步都一样，不会走样。
# - `bio`  仿生＝官方并腿跑（FreeBound）：前腿一组、后腿一组，两组差半个
#   周期，占空比 0.55 —— 两组之间各有 5% 的四脚着地重叠，不会腾空。腿上有
#   速度闭环和俯仰反馈。
#
# 另留一套 `gallop`（横向序列 RL→RR→FL→FR 错相）做对照，不进切换序列，
# 要试的时候 `make_gait("gallop")` 直接给。
#
# `real=True` 只影响 `gallop`：步幅收小（仿真里打滑是「损失」，真机打滑是
# 「整条腿在地上刨，一步都走不动」）、抬脚抬高（真机足端有橡胶、地面软，
# 6 cm 的余量经常蹭地）。`mech` / `bio` 不分叉，真机与仿真同一套参数。
#
# `mode` 选力路的实现（`"vmc"` 现役 / `"mpc"` 自研），直接透传给 GaitParams。
# 它不属于步态形态，只是方便「同一套步态参数换一条力路做 A/B」。
#
# 不给就落到环境变量 `GO2_MOTION`，那是启动器（`quadruped/cli/launcher.py`）
# 设的。走环境变量而不是给每个控制器加一个 argparse 参数，是因为所有入口的
# 步态最后都过这里 —— `main.py` 的子命令、键盘遥控经 `OursBackend` 到
# `Go2Robot` 的兜底、`sim_check`，一处改就全覆盖。显式传参仍然优先。
def make_gait(style: str, height: float = NOMINAL_HEIGHT,
              real: bool = False, mode: str | None = None) -> GaitParams:
    if mode is None:
        # `or` 而不是 `get(..., "vmc")`：`GO2_MOTION= cmd` 这种空赋值很常见，
        # 空串要当成没设，否则会一路走到 GaitParams 里去报「mode='' 不认识」。
        mode = os.environ.get("GO2_MOTION") or "vmc"
    neutral = _neutral_feet(height)
    style = _GAIT_ALIASES.get(style, style)

    def _gp(**kw):
        kw.setdefault("mode", mode)
        return GaitParams(**kw)

    if style == "mech":
        # 标准小跑，原样参数：对角腿、0.5 s 周期、duty 0.5、抬脚 6 cm、
        # 步幅上限 7.5 cm。零反馈、零速度闭环、不压地、不外扩站宽。
        #
        # 2026-11-10 17:25 之后在这档上叠过一串「补稳定性 / 补速度」的改动
        # （stride_max 0.12、speed_loop 1.5、stance_sink 6 mm、duty 0.65、
        # stance_spread 3.5 cm、pitch_gain −0.05、真机 swing 0.055 等）。
        # 2026-11-11 按用户要求全部撤回，回到最初那个「慢，但每一步都一样」
        # 的版本 —— 那串改动的实测收益（横坡 roll 15.7°→3.5°、速度 0.209→
        # 0.224）是结构性的，但代价是步态形态被改，且这些量级的问题本来就
        # 该由上层 MPC+WBC 解决，不该靠改步态参数凑。
        #
        # 撤掉之后的已知边界，别当成新 bug 查：
        # - 双支撑时间为 0，横向稳定裕度是零，横坡/侧向扰动会一直累积到翻；
        # - 单按 w 最快约 0.35 m/s（步幅上限削顶，物理上到不了 0.9）；
        # - 支撑腿不主动压地，法向力全靠 kp 静差，软地面容易打滑。
        #
        # 真机与仿真**同一套**，不再分叉（真机档曾单独给 swing 0.055 /
        # stride 0.10 / speed_range 0.22，一并撤掉）。
        return _gp(period=0.5, duty=0.5, neutral=_neutral_feet(height),
                          name="mech")
    if style == "gallop":
        # 横向序列疾驰（rotary gallop），猫科和犬科真正跑起来用的就是这一套：
        # 左后 → 右后 → 左前 → 右前，相邻两条腿只错开 12% 周期，前后两组
        # 之间错开 38%。占空比 0.55 > 相邻相位差 0.38，所以任何时刻都至少
        # 有两条腿在地上，不会四条腿一起腾空。
        #
        # 这套不进切换序列，只做对照：支撑力连续、落地单腿轻接，是这几套里
        # 最像活物的一套；代价是四腿错相，反馈修正会落到正在摆动的腿上，
        # 姿态项只能全关（开了反而翻）。
        if real:
            # 真机档。步幅 0.09（原来给 0.07，实测根本走不动：四腿错相时任意
            # 时刻只有 1~2 条腿支撑，0.07 的行程蹬不破静摩擦，指令 0.6 只跑出
            # 0.0 m/s；0.09 立刻回到 0.41）。抬脚 9.5 cm，落脚点反馈减半 ——
            # 0.15 的 reach_err 在仿真里是稳定器，真机上地面不平会变成来回抢。
            #
            # 速度闭环 1.5 而不是 1.2：四腿错相时支撑力在 1~2 条腿之间
            # 来回交班，闭环弱一点就跟不上，实测同一个配置反复跑会在 0.14 和
            # 0.41 两个吸引子之间跳。1.5 之后四次都落在 0.35~0.40。
            return _gp(period=0.44, duty=0.55, swing_height=0.095,
                              offsets={"RL": 0.00, "RR": 0.12,
                                       "FL": 0.50, "FR": 0.62},
                              neutral=_neutral_feet(height + 0.015), name="gallop",
                              stand_raise=0.015,
                              stride_gain=1.15,
                              stride_max=0.090,
                              speed_loop=1.5,
                              pitch_gain=0.0, pitch_rate=0.0,
                              roll_gain=0.0, roll_rate=0.0,
                              bounce=0.0, lean=0.10,
                              reach=0.02, reach_err=0.08,
                              stance_sink=0.004,
                              speed_range=(0.0, 0.45))
        return _gp(period=0.44, duty=0.55, swing_height=0.075,
                          offsets={"RL": 0.00, "RR": 0.12,
                                   "FL": 0.50, "FR": 0.62},
                          neutral=_neutral_feet(height + 0.015), name="gallop",
                          # 抬起来跑。四腿错相时左右腿落地差 0.05 s，机身被
                          # 交替横向推；腿弯着跑（低机身）一被推就翻。抬高
                          # 1.5 cm 腿更直、支撑余量大，实测 12 次里 11 次站住，
                          # 速度也从 0.20 提到 0.25~0.39 m/s。
                          stand_raise=0.015,
                          # 步幅倍率只留一点推力余量：脚该扫多快由下面那条
                          # 速度闭环算，不再靠放大倍率硬打滑推。
                          stride_gain=1.3,
                          stride_max=0.105,
                          # 支撑相足端速度闭环。开环时后腿会往前蹭、把机身
                          # 推着倒退（见 GaitParams.speed_loop）。
                          speed_loop=1.5,
                          # 姿态反馈全关。四腿错相时，俯仰/滚转修正会落到
                          # 正在摆动的那条腿上，落地砸得更重；实测加上去反而
                          # 从「能跑」变成「翻」（roll 180°）。
                          pitch_gain=0.0, pitch_rate=0.0,
                          roll_gain=0.0, roll_rate=0.0,
                          bounce=0.0,
                          lean=0.12,
                          # 落脚点：基础前移一点，再按速度误差 -0.15 前后调。
                          # 这一项是这套步态的稳定器 —— 机身快了就往前多踩一点
                          # 制动，慢了就少踩，实测 4/4 不翻、速度从 0.20 提到
                          # 0.35 m/s；给到 0.25 就过头，四脚一起刨地直接翻。
                          reach=0.02,
                          reach_err=0.15,
                          # 支撑腿主动压地 6 mm，撑出稳定的法向力；没有它脚
                          # 在「刚好贴地」附近时接时离，摩擦时有时无。
                          stance_sink=0.006,
                          # 这套实际交付 0.40 左右，指令封到 0.45。见
                          # _gait_feet_publish 顶部那段注释。
                          speed_range=(0.0, 0.45))
    if style == "bio":
        # 并腿跑，对应官方的 FreeBound：前腿一组、后腿一组，两组差半个周期。
        #
        # 占空比 0.55，两组各有 5% 时间四脚着地重叠，换腿那一下不会出现
        # 「只剩一组腿撑全机」——duty 正好 0.5 时前腿单独支撑半个周期，质心
        # 永远在支撑点后方 19 cm，机身后仰到 90°。两组首尾相接也不留腾空，
        # 所以落地是两条腿一起接、不是砸。
        return _gp(period=0.40, duty=0.55, swing_height=0.080,
                          offsets={"FL": 0.5, "FR": 0.5, "RL": 0.0, "RR": 0.0},
                          neutral=neutral, name="bio",
                          # 步幅倍率。有了下面那条速度闭环，脚该扫多快是算出来的，
                          # 不再需要靠放大倍率去打滑硬推；1.2 只是让步幅比「刚好
                          # 追上机身」再大一点，留下一点推力余量。
                          stride_gain=1.2,
                          # 单腿水平行程上限。0.105 是一次落地前后共扫 0.21 m，
                          # 腿高 0.248 m、水平余量够，IK 不削顶。
                          stride_max=0.105,
                          # 支撑相足端速度闭环。见 GaitParams.speed_loop 的注释：
                          # 开环时后腿会往前蹭、把机身推着倒退，闭环才跑得起来。
                          speed_loop=2.0,
                          # 只有一端着地时，反作用全花在点头上。角度项把机身
                          # 按平，角速度项（阻尼）把它收住 —— 只有角度项的话
                          # 机身会在平衡点两侧一路荡到 ±45°。
                          pitch_gain=0.30, pitch_rate=0.035,
                          # 滚转反馈先关：两组腿落地不同步时，左右两侧的修正
                          # 会互相打架，实测直接把机身滚翻（roll −180°）。
                          # 等速度闭环把前后方向稳住，再单独调这一项。
                          roll_gain=0.0, roll_rate=0.0,
                          # 竖直弹跳：先别开。步幅已经放大，再叠弹跳两个耦合
                          # 起来会翻（实测滚转 −159°）。机身本来就会被两组腿
                          # 交替顶起来，弹跳是锦上添花。
                          bounce=0.0,
                          lean=0.12,
                          # 落脚点前移：跑得越快脚越往前踩，支撑力把机身往前带，
                          # 而不是落在重心底下空转。
                          reach=0.05,
                          # 速度误差反馈（Raibert 落脚点）。速度闭环已经管住了
                          # 「该扫多快」，这里不再重复叠一层，免得两个反馈互相
                          # 抵消、出现低频摆动。
                          reach_err=0.0,
                          # 支撑腿主动压地 1 cm：撑出稳定的法向力。没有它，脚在
                          # 「刚好贴地」附近时接时离，摩擦时有时无，往前跑不动、
                          # 有时还被摆动的脚反过来拽着走 —— 就是「莫名其妙往后跑」。
                          stance_sink=0.010,
                          speed_range=None)
    raise ValueError(
        f"没有这种运动步态：{style}，只有 ('mech', 'bio')"
        f"（旧名 trot→mech、bound→bio 仍可用；gallop 是对照用的第三套）")


# 四足对称站位（模块级版本，GaitParams 初始化时用）。
#
# 侧向位置必须走 `Go2Controller.neutral_y`，不能再自己算一遍。
# 原先这里是 `LEG_GEOM[leg][1] + s*THIGH_Y`，和 `neutral_y` 数学上等价，
# 但两处各写一份的后果是：只要改站宽而不知道有两个来源，就会只改一边。
# `foot_target` 用 `gait.neutral`，`p_nom` 用 `stand_feet`（走 neutral_y），
# 两者差一个站宽的量，轨迹基准和支撑权重闸门的基准就对不上。
#
# spread 是每侧再往外撑的距离（米），左腿 +、右腿 −。
def _neutral_feet(height: float, spread: float = 0.0) -> dict:
    return {leg: (LEG_GEOM[leg][0],
                  Go2Controller.neutral_y(leg) + LEG_GEOM[leg][2] * spread,
                  FOOT_RADIUS - height) for leg in LEGS}


# ===========================================================================
# 几何工具（旋转、角度归一、转置乘）
# ===========================================================================

# 绕 x 轴旋转（右手系）。
def _rot_x(v, q):
    c, s = math.cos(q), math.sin(q)
    x, y, z = v
    return (x, y * c - z * s, y * s + z * c)


# 绕 y 轴旋转（右手系）。注意返回值里 x 的符号，别照抄教科书矩阵。
def _rot_y(v, q):
    c, s = math.cos(q), math.sin(q)
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


# 把角度归一到 (-π, π]。
#
# 右腿的 abduction 是两个第三象限 atan2 相减，不归一就会得到 6.12 rad
# 这种值，看着像越限，其实是同一个角。
def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ZYX 顺序的旋转矩阵（机体系 → 世界系），与 IMU 的 rpy 同一套约定。
def _rpy_to_R(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


# Rᵀ·v：把世界系的位移换到机体系。
def _mat_t_vec(R, v):
    return (R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
            R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
            R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2])


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


# ===========================================================================
# 逆运动学结果
# ===========================================================================

class IKResult:
    """IK 结果。ok=False 时 q 是「夹到可达边界」的近似解，别直接下发给电机。"""

    __slots__ = ("q", "ok", "reason", "clamped")

    def __init__(self, q, ok, reason="", clamped=None):
        self.q = q
        self.ok = ok
        self.reason = reason
        self.clamped = clamped or {}

    def __repr__(self):
        tag = "ok" if self.ok else f"FAIL({self.reason})"
        return f"IKResult(q=[{self.q[0]:+.3f}, {self.q[1]:+.3f}, {self.q[2]:+.3f}], {tag})"


# ===========================================================================
# 控制器（底层）
# ===========================================================================

class Go2Controller:
    """Go2 底层控制器：运动学、轨迹、DDS，以及**唯一的 200 Hz 控制回路**。

    整个对象生命周期只跑一个 RecurrentThread，靠 self._mode 切换：

        idle  不发布（默认，防止没连上就乱写）
        hold  每帧发同一个目标（站立、蹲姿等静态姿态靠它撑住）
        pose  按序列播放姿势插值：一段接一段，播完转 hold。
              队列由 pose_sequence() 排，**排完立即返回**，不占调用方线程 ——
              所以跳跃、蹲伏这类动作能叠加、能打断、能和步态交替
        gait  起立 → 小跑 → 幅度收到 0 → 转 hold

    这样动作可以串起来：`forward()` 结束时腿还是站着的，接着调 `turn()` 不会掉。

    运动学方法是静态的，不构造对象也能用：
        Go2Controller.fk_leg("FL", (0.0, 0.9, -1.8))
    """

    CTRL_HZ = 200.0          # 与仿真 bridge 的步进频率一致（真机可提到 500）
    MODE_SERVO = 0x01        # 真机 PM_Servo，仿真里被忽略但保持一致

    # 关节 PD 刚度。仿真的电机是理想力矩源，kp 一大就振；真机有减速器摩擦
    # 死区和几十毫秒响应延迟，kp 小了腿就是软的（顶不住体重、抬不起脚）。
    #
    # 真机这两个数被「抖动」反向咬过一口：一开始按实物经验把 kd 提到 5.0、
    # kp 提到 140，结果电机啸叫、整条腿在颤 —— 真机的 dq 来自编码器差分，
    # 噪声比仿真大一个量级，kd 乘上去就是高频力矩噪声，200 Hz 下根本压不住。
    # 所以真机走「低 kp + 小 kd + 力矩前馈」：刚度靠前馈给，PD 只负责纠偏。
    # 备用的一套「理想力矩源」参数。现在 Go2Robot 默认两边都用真机参数
    # （KP_REAL/KD_REAL + 真机步态），这组只在显式 real=False、要对着理想模型
    # 做对照实验时才用到 —— 别拿它当仿真默认，见 go2_robot.Go2Robot.__init__。
    KP_SIM, KD_SIM = 120.0, 2.5
    # 100 是实测出来的下界：70 的时候指令膝角峰峰 24°、实际只有 7°，跟踪
    # 误差常年 6~10° —— 腿抬不到目标高度，摆动相蹭着地面走，摩擦阻力又把
    # 腿拖得更抬不起来，正反馈卡死。刚度不够这件事不能靠调力矩补：抬腿是
    # 位置跟踪问题，前馈只能帮它抗重力，帮不了它跟指令。
    # 110 时的噪声不是 kp 本身的问题，是指令里有阶跃（姿态修正、步幅、
    # 力矩三处），那三处现在都加了限速，所以 100 不会再把噪声带回来。
    KP_REAL, KD_REAL = 100.0, 1.5

    # 重力前馈强度：1.0 = 完全补偿支撑腿该顶的体重。真机必须开 —— 纯位置控制
    # 靠 kp 的静差撑体重，机身会一直往下沉、腿一直压在地上，看着就是「没力气」。
    TAU_FF_REAL = 1.0
    TAU_FF_SIM = 0.0
    # 前馈力矩的一阶低通时间常数。步态里 q_target 每帧都在变，直接算出来的
    # τ 也跟着跳，抖。40 ms 足够跟住支撑相的变化，又抹掉帧间跳变。
    TAU_TAU = 0.04
    # 单关节前馈力矩上限（N·m）。Go2 关节额定约 23.7，留一半给 PD。
    TAU_MAX = 12.0
    # 机身层力矩指令的平滑（见 _slew_tau）。分配器每帧独立解一遍，输入里带着
    # 陀螺噪声（WRENCH_KD 乘上去）、机身竖直速度噪声（WRENCH_DZ 乘上去），
    # 还有支撑权重跨过 0.12 那一帧的阶跃。这些高频成分直接变成关节力矩，
    # 真机上就是减速器在零速附近高频换向 —— 听起来是硬摩擦声，看着是整条腿
    # 在颤，走起来还费劲。仿真里 IMU 干净，这个现象几乎看不到。
    # 低通 0.03 s：转折约 5 Hz，比步频 2 Hz 高、比噪声低。
    # slew 2.5 N·m/帧（200 Hz 下 500 N·m/s）：从 0 到 20 要 8 帧 40 ms，
    # 够快，但足以把阶跃抹成斜坡。
    TAU_W_TAU = 0.03
    TAU_W_SLEW = 2.5
    # 实际关节角与目标差超过这个值就不给前馈（弧度）。趴着往起站的头几帧
    # 差值有三四弧度，必须挡住，见 _tau_ff。
    TAU_ARM_ERR = 0.45

    # 速度伺服的加速度上限。原生 Move 直接把速度甩给运动服务，由它内部限幅；
    # 我们的步态是开环的，不限幅会一步跨太大、脚下打滑。
    ACCEL_LIN = 1.0          # m/s²
    ACCEL_ANG = 2.5          # rad/s²

    # 姿态/速度反馈的滤波时间常数（秒）。原始 IMU 与运动服务速度都带高频噪声，
    # 直接乘增益喂给足端就是让机身跟着噪声抖。50 ms 一阶低通比步态节律
    # （2.5 Hz，周期 400 ms）快一个量级，既滤掉噪声又不拖慢真实姿态变化。
    STAB_TAU = 0.05
    VEL_TAU = 0.06
    # 速度环输出（evx/evy）的平滑。**这一项直接就是步幅**，它每帧一跳，
    # 足端就在地上前后猛蹭。真机上 v_filt 的峰峰能到 0.5 m/s —— sportmodestate
    # 停发以后速度估计全靠 IMU 积分和关节里程计融合，脚一打滑两者就打架。
    # 乘 1.5 的闭环增益再乘支撑时间，dx 会在 ±stride_max 之间反复变号：
    # 实测单帧关节角跳 27.8°，9.2% 的关节·帧跳超过 2.9°，落地就是硬碰声，
    # 而且这些来回摆动全耗在摩擦上，机身根本不前进。
    # 低通 0.08 s 把噪声压掉一个量级，slew 0.05 m/s/帧兜住大跳。
    EVX_TAU = 0.08
    EVX_SLEW = 0.05
    # 反馈施加在足端竖直方向的最大量（米）。超了就等于用一条腿去顶整机重量，
    # IK 会削顶、脚在地上打滑，反而抖得更凶。
    STAB_MAX = 0.045
    # 姿态修正量的逐帧限速（米/帧）。STAB_MAX 是 45 mm，但落地瞬间陀螺尖峰
    # 乘 pitch_rate 完全能顶到限幅，而限幅本身又把它变成方波：那一帧足端 z
    # 目标直接跳 45 mm，膝角跟着跳 20°，打出来的就是撞击声，机身被反作用
    # 撞得高频震动。实测 9.2% 的关节·帧跳超过 2.9%，全部落在落地前后。
    # 姿态修正量是**位置**，必须连续：3 mm/帧 = 0.6 m/s，45 mm 用 15 帧（75 ms）
    # 走完，比步态周期 500 ms 快得多，不会拖慢姿态响应。
    STAB_SLEW = 0.003

    # 支撑权重的实测闸门：足端相对中立位抬到 HI 才开始削，到 HI−W 归零。
    # 原来 (0.035, 0.020) —— 抬 3.5 cm 就判离地。而步态的抬脚是 5.5~6 cm，
    # 支撑相中途的机身起伏、俯仰把足端在机体系里推高一两厘米，闸门就会
    # 误伤正在承重的腿：那条腿的法向力被撤掉，摩擦没了，脚在地上滑，
    # 机身失速。失速又让速度环放大步幅、脚扫得更快，滑得更狠 —— 实测
    # 就是这个链条，vx 从 +0.3 一路掉到 −0.5，然后姿态崩。
    # 放到 6 cm / 4 cm 之后，只有真正抬到最高点的摆动腿才会被削。
    STANCE_GATE_HI = 0.060
    STANCE_GATE_W = 0.040

    # ---- 重力前馈 ----
    # 纯位置控制靠 kp 的稳态误差撑体重：误差 e 处产生的力矩 = kp·e，要凑够
    # 一条腿该顶的力，kp 小的时候机身会一直往下沉、腿一直是软的。真机上这个
    # 现象尤其明显（减速器有摩擦死区，仿真里没有），表现就是「腿没力气、
    # 抬不起来、一直在打滑」。所以按 τ = Jᵀ·f 直接把支撑力算成力矩前馈。
    # 整机质量取 model.py 的实测值 15.206408 kg（机 6.921 + 四条腿各 2.071）。
    # 原来写 15.0 是估的，重力前馈常年少给 1.4%，单腿支撑差 2 N，落地那一下
    # 全靠 kp 静差补 —— 这也是 kp 一直降不下去的原因之一。
    BODY_MASS = TOTAL_MASS
    GRAVITY = 9.81
    # 四条腿均分时的单腿体重（N）。注意 gravity_tau 用的是「总和 = 体重、
    # 按支撑权重分配」，不是每条腿固定这么多 —— 步态里只有两条腿在地上时，
    # 固定 mg/4 会凑不够总重量。
    LEG_WEIGHT_N = BODY_MASS * GRAVITY / 4.0

    # ------------------------------------------------------------------
    # 机身层（合力 / 合力矩）控制增益
    # ------------------------------------------------------------------
    # 量纲都按「物理量 → 牛顿 / 牛米」，不做归一化，这样参数换到真机上
    # 不用重新标定 —— 质量、重力、腿长都是真机实测值。

    # 高度：机身低 kz 米就补 kz*KZ 牛的额外支撑力。
    # 四条腿撑地等效刚度约 2500 N/m/腿（kp 110 时），两条腿 5000 N/m，
    # 机身 15 kg 时固有频率 2.9 Hz。这里给 1500 N/m 的主动刚度，把等效
    # 刚度压到约 1.25 倍、频率抬到 3.2 Hz，同时靠 DZ 把阻尼比拉到 0.6 左右。
    WRENCH_KZ = 1500.0        # N/m
    WRENCH_DZ = 180.0         # N/(m/s)：机身下沉速度 0.1 m/s 补 18 N
    # 速度前馈推力：指令与实测速度差 0.1 m/s 就给 0.1*KV 牛的推力。
    # 15 kg 机身要 0.5 m/s² 的加速度需要 7.5 N，KV=90 时 0.08 m/s 的
    # 误差就够 —— 相当于一个时间常数 0.17 s 的速度环，不会跟步态共振。
    WRENCH_KV = 90.0          # N/(m/s)
    # 姿态：角度项给弹簧，角速度项给阻尼。
    # 15 kg 机身绕 y 轴转动惯量约 0.2 kg·m²，Kp=160 时固有频率
    # sqrt(160/0.2)/2π ≈ 4.5 Hz（高于步频，不会被步态激起来），
    # Kd=16 时阻尼比约 0.45。
    WRENCH_KP = 160.0         # N·m/rad
    WRENCH_KD = 16.0          # N·m/(rad/s)
    # 支撑腿的 kp 缩放。位置伺服留在支撑腿上会跟力指令抢：脚被钉在轨迹上，
    # 力矩加进去只是让「钉得更紧」，地面反力还是由 kp 静差决定。降下来之后
    # 腿变成「有一定刚度的力源」，力由分配器说了算。
    # 0.18 是实测扫出来的：0.35→0.18 前进 5 秒，高度纹波 33/60/38 → 17/16/19
    # mm，俯仰峰峰 18/28/22 → 14/17/18°，末段速度 0.16 → 0.19~0.23 m/s
    # （三次交替重复，每项都赢）。下界也测了：0.10 勉强站住，0.05 和 0 直接
    # 翻 —— 再往下让，就没有东西约束腿的姿态了，见 body_wrench_tau 上面
    # 那段关于「缺零空间姿态项」的说明。
    # 偏航：足端轨迹只能"按指令转"，管不了"不想转却被地面拽着转"。实测
    # 走直线 5 秒后偏航从 0 漂到 −8°，然后每秒 30° 加速，20 秒时整条狗
    # 横过来、侧向摩擦把滚转也拽翻（roll 40° → 180°）。所以必须有一路
    # 真正的偏航力矩。这里给角度 + 角速度 PD，目标角由 yaw 指令积分得到。
    WRENCH_KP_YAW = 15.0      # N·m/rad
    WRENCH_KD_YAW = 2.0       # N·m/(rad/s)
    WRENCH_KP_STANCE = 0.18
    # 真机的支撑腿刚度缩放。0.18 是仿真里扫出来的最优点，但那个前提是模型
    # 精确、脚的高度、雅可比、腿长都对得上。真机上模型差好几厘米（实测站姿
    # 足端 −0.283、模型 −0.2475），位置环是唯一可靠的刚度来源：撑到 12.6
    # 的时候腿被体重压塌，轨迹里的水平行程跟着塌掉，机身纹丝不动。
    # 0.5 = 真机 kp 70 时的支撑腿等效刚度 35，比纯位置伺服软一档，但还撑得住。
    WRENCH_KP_STANCE_REAL = 0.5
    # 机身层单独一套力矩限幅。不能用 TAU_MAX=12 —— 那是给"位置伺服为主、
    # 前馈为辅"配的；现在力矩是主力，单腿支撑时光承重就要 24.6 N·m。
    # 20 是仿真里能跑通的值，但真机上实测峰值常年贴着它跑，而实际总力矩还要
    # 叠上 kp·(q_des−q)（驱动器算的，软件管不住）—— 两项加起来会越过额定
    # 23.7，靠驱动器过流保护兜底，一旦触发就突然卸力、直接摔。
    # 14 留出位置环那一份：kp 70 时 0.1 rad 的跟踪误差是 7 N·m，合计 21。
    WRENCH_TAU_MAX = 14.0
    # 分配器用的摩擦系数。go2.xml 足底给的是 0.9，真机橡胶脚垫在瓷砖上实测
    # 0.6~0.8、在地胶上更低，按 0.6 算 —— 宁可少给力也不要给一个地面提供
    # 不出来的水平力，那种情况下腿会滑、力矩白烧，还会把偏航拽歪。
    WRENCH_MU = 0.6
    # 横移指令上限，0 = 完全禁止。见 _gait_feet_publish 顶部那段：现在横着
    # 走会侧翻，真机上摔一次比走不动代价大得多。横移步态做出来之后再放开。
    MAX_VY = 0.0
    # 单腿竖直力上限。单腿支撑时理论上要扛 mg = 147 N，落地冲击会更高，
    # 220 给 1.5 倍余量。再高就没意义了 —— 电机额定才 23.7 N·m。
    WRENCH_FZ_MAX = 220.0
    # 俯仰/滚转力矩上限。几何上限：左右腿竖直力差最大 147 N、力臂 0.142 m，
    # 也就是 20.9 N·m（那已经是「一条腿扛全部、另一条腿悬空」的极限）。
    #
    # 原来给 6，注释里写「2.5 m/s² 够纠偏了」—— 那个判断是错的，漏了重力：
    # 机身抬头 θ 时重心前移 0.27·sinθ，要抵消的是 m·g·0.27·sinθ 这个持续
    # 力矩，不是角加速度。θ=10° 就是 6.9 N·m，正好在 6 的限幅之上 ——
    # 于是姿态环一旦偏到 10° 就饱和，再也拉不回来。实测长距离行走 8 s 后
    # pitch 稳定停在 −10°、速度掉一半，就是这个饱和。
    # 14 覆盖到 20°（13.6 N·m），给分配器留的余量也还在摩擦锥之内。
    WRENCH_M_MAX = 14.0

    # ------------------------------------------------------------------
    # 自研力路：MPC + WBC（gait.mode == "mpc"）
    # ------------------------------------------------------------------
    # 两层的时间尺度不一样：MPC 看 240 ms 的 horizon，50 Hz 就够，而且它一帧
    # 4.9 ms（144 维稠密 QP），塞进 200 Hz 的 5 ms 预算里必然超时；WBC 只有
    # 24 维、0.6 ms，必须每帧跑（它输出的是这一帧的力矩）。所以 MPC 起独立
    # 线程，WBC 留在主回路。
    MPC_HZ = 50.0
    MPC_HORIZON = 12          # knot 数，dt×N = 0.24 s
    MPC_DT = 0.02
    # 计划过期判据（秒）。MPC 线程被 GC 或调度卡住超过这个时间，就退回重力
    # 前馈并把 `_mpc_miss` 记上 —— 拿一份 0.2 s 前的力当现在的力，机身会被
    # 已经消失的扰动继续推。
    MPC_STALE = 0.2
    # WBC 的关节力矩上限。**不能沿用 WRENCH_TAU_MAX=14**：那是给「位置伺服
    # 为主 + 前馈为辅」配的；WBC 是力矩主力，双支撑小跑静态承重就要 17.2 N·m，
    # 压到 14 会把正常承重削掉、机身直接塌（见 wbc.py 顶部）。20 留 3.7 给
    # 叠在上面的那圈低刚度 PD。
    WBC_TAU_MAX = 20.0
    # 送 WBC 的关节速度低通时间常数（秒）。理由见 `_dq_lp`。
    DQ_TAU = 0.02
    # MPC 的代价权重。**这一层是外环的慢修正 + 预测前馈，不是快环。**
    #
    # 从「测到状态」到「力矩生效」有 20~50 ms（50 Hz 计划 + 5 ms 链路），
    # 高度/姿态这种双积分被控对象的带宽上限只有 2 Hz 左右。把它当快环用
    # （w_height 4e3、w_rpy 1e2）实测直接进极限环：高度峰峰 30~45 mm、
    # roll 峰峰 45°、Fz 在 110~340 N 之间摆、Mx 每 80 ms 翻一次符号。
    #
    # 现在快环归 `_mpc_trim`（200 Hz，与 `_wrench_cmd` 同一套律），这里只留
    # 低增益的慢修正 —— 它的主要作用是把「接下来 240 ms 要抬哪条腿」这件事
    # 提前算进力里，外加防止 QP 因为代价全零而退化成「什么都不出力」。
    MPC_W_HEIGHT = 1e2
    MPC_W_RPY = 1e1
    MPC_W_OMEGA = 5e-1
    MPC_W_FORCE = 5e-3
    # 交给 WBC 之前的 wrench 限幅（相对体重的倍数）。理由见 `_clamp_wrench`。
    MPC_FZ_MAX = 1.6
    MPC_FXY_MAX = 0.35
    # WBC 里足端 PD 的笛卡尔刚度/阻尼。`wbc.py` 的默认值（3000 / 120）是按
    # 「关节 kp 120」折算的，只对**摆动腿**成立 —— 摆动腿在空中，追轨迹就是
    # 它的全部工作，刚度高一点没有副作用。
    WBC_KP_FOOT = 3000.0
    WBC_KD_FOOT = 120.0
    # 支撑腿保留多少足端位置任务。**给 0**：支撑脚踩在地上、机身系的目标在
    # 往后扫，机身没跟上时误差等于「机身落后了多少」，没有物理上限。实测
    # 5 cm × 3000 = 150 m/s² 的加速度需求，QP 里唯一的天花板是 τ ≤ 20，于是
    # 12 个关节每帧都顶在限幅上、WBC 退化成 bang-bang，机身被顶得乱跳、打滑
    # 200%、roll 峰峰 50°。支撑腿的任务只有出力，位置交给地面。
    WBC_STANCE_FOOT = 0.0
    # WBC 的时序正则权重（x 靠近上一帧的解）。
    #
    # **不开这一项根本没有稳定解。** 同一个 wrench 在摩擦锥上有很多等价顶点
    # 解，OSQP 逐帧跳着选：实测 Mx 每 80 ms 翻一次符号（±40 N·m）、τ 帧间差
    # p95 36 N·m / 峰值 90，机身被这些台阶抖得 roll 峰峰 44°。开 1e3 之后
    # 同一段测试 roll 降到 14.7°、τ 峰值 40；再加外层 `_slew_tau` 到 4.4°。
    #
    # 数值上没有更好的选择：约束是线性锥、解集是凸多面体，顶点解不唯一是结构
    # 性的，不是求解器精度问题，只能靠正则挑一个连续的解。
    WBC_W_SLEW = 1e4

    # ------------------------------------------------------------------
    # 基础几何
    # ------------------------------------------------------------------

    # 髋部原点在机体系的位置。
    @staticmethod
    def hip_origin(leg: str) -> tuple[float, float, float]:
        x, y, _ = LEG_GEOM[leg]
        return (x, y, 0.0)

    # abduction = 0 时足端所在的 y。
    #
    # 这不是髋部原点的 y。abduction 绕 x 转，只有当整个侧向偏移
    # (HIP_Y + THIGH_Y = 0.142) 都转回水平位置时，足端才落在髋俯仰关节
    # 正下方，此时 q1 = 0 —— 也就是 home 关键帧那个姿态。
    #
    # 用 HIP_Y 会得到 q1 ≈ -0.35 rad 的外八字站位，白白吃掉 20° 的
    # abduction 行程，而且四条腿受力不对称。
    @staticmethod
    def neutral_y(leg: str) -> float:
        _, y, s = LEG_GEOM[leg]
        return y + s * THIGH_Y

    # 髋俯仰关节（thigh joint）在机体系的位置，随 abduction 一起偏转。
    @classmethod
    def hip_pitch_joint(cls, leg: str, q1: float) -> tuple[float, float, float]:
        _, _, s = LEG_GEOM[leg]
        return _add(cls.hip_origin(leg), _rot_x((0.0, s * THIGH_Y, 0.0), q1))

    # 四足对称站位的足端坐标（足端 body 原点），直接喂给 ik_all。
    @classmethod
    def stand_feet(cls, height: float = NOMINAL_HEIGHT,
                   spread: float = 0.0) -> dict:
        return {leg: (LEG_GEOM[leg][0],
                      cls.neutral_y(leg) + LEG_GEOM[leg][2] * spread,
                      FOOT_RADIUS - height) for leg in LEGS}

    # 用「机身高度 + 足端相对中立位的水平偏移」描述足端，调试更顺手。
    @classmethod
    def foot_from_body(cls, leg: str, body_z: float,
                       xy_offset=(0.0, 0.0)) -> tuple:
        return (LEG_GEOM[leg][0] + xy_offset[0],
                cls.neutral_y(leg) + xy_offset[1],
                FOOT_RADIUS - body_z)

    # 让机身相对世界系倾斜 / 平移，足端落点不动 → 机体系下的足端坐标。
    #
    # 这是做「俯仰、横滚、重心平移」这类静态动作的唯一入口：脚踩在原地，
    # 机身按 R(roll,pitch,yaw) 转过去，机体系里看到的足端位置就是 Rᵀ·(p - base)。
    #
    # pitch 正为低头（x 轴指向下方），roll 正为向左滚，yaw 正为逆时针。
    @classmethod
    def feet_for_body_pose(cls, roll: float = 0.0, pitch: float = 0.0,
                           yaw: float = 0.0, shift=(0.0, 0.0),
                           height: float | None = None) -> dict:
        h = NOMINAL_HEIGHT if height is None else height
        R = _rpy_to_R(roll, pitch, yaw)
        out = {}
        for leg in LEGS:
            x, y, s = LEG_GEOM[leg]
            # 世界系落点：脚踩地，足端 body 原点离地正好 FOOT_RADIUS
            p_world = (x - shift[0], y + s * THIGH_Y - shift[1], FOOT_RADIUS)
            out[leg] = _mat_t_vec(R, (p_world[0], p_world[1], p_world[2] - h))
        return out

    # ------------------------------------------------------------------
    # 正运动学
    # ------------------------------------------------------------------

    # 三个关节角 → 足端在机体系的位置。q = (q1, q2, q3)，弧度。
    @staticmethod
    def fk_leg(leg: str, q) -> tuple[float, float, float]:
        q1, q2, q3 = q
        _, _, s = LEG_GEOM[leg]

        calf_vec = _rot_y((0.0, 0.0, -CALF_LEN), q3)
        foot_in_thigh = _add((0.0, 0.0, -THIGH_LEN), calf_vec)

        foot_in_hip = _rot_y(foot_in_thigh, q2)
        t0 = _rot_x((0.0, s * THIGH_Y, 0.0), q1)
        return _add(_add(Go2Controller.hip_origin(leg), t0), _rot_x(foot_in_hip, q1))

    # q12 = 12 个关节角，顺序与 LowState / LowCmd 一致：FR,FL,RR,RL。
    #
    # 这里必须用 `CTRL_LEGS` 而不是 `LEGS`。两个常量的顺序不一样
    # （LEGS 是 FL,FR,RL,RR），用错了会把左右腿的正解对调 —— 表现就是
    # 「目标站立的腿实际在飞、目标摆动的腿实际踩在地上」，看起来像相位
    # 整体错半周期，很容易带偏诊断方向。
    @classmethod
    def fk_all(cls, q12) -> dict:
        return {leg: cls.fk_leg(leg, q12[3 * i: 3 * i + 3])
                for i, leg in enumerate(CTRL_LEGS)}

    # 足端位置对三个关节角的雅可比，返回 Jᵀ（3×3）。
    #
    # Jᵀ 的第 j 行 = 第 j 个关节对应的足端位移方向，于是 τ = Jᵀ·f 一次算完。
    # 数值微分就够：fk_leg 是纯三角运算，1e-6 的扰动能把误差压到 1e-9，
    # 比手推解析式不容易写错，而这个量只用来算重力前馈，差一点无所谓。
    @staticmethod
    def jacobian_leg(leg: str, q, eps: float = 1e-6) -> tuple:
        eps = 1e-6 if not eps else eps
        p0 = Go2Controller.fk_leg(leg, q)
        cols = []
        for k in range(3):
            qq = list(q)
            qq[k] += eps
            p1 = Go2Controller.fk_leg(leg, qq)
            cols.append(tuple((p1[i] - p0[i]) / eps for i in range(3)))
        # cols[k][i] = ∂p_i/∂q_k；转置成 [i][k] 便于 tau_j = Σ_i J[i][j]·f_i
        return tuple((cols[0][i], cols[1][i], cols[2][i]) for i in range(3))

    # 按支撑腿分权重给出重力前馈力矩（执行器顺序）。
    #
    # `weights` 是 {腿名: 0~1}，0 表示这条腿在空中、不该顶体重。
    # 返回 12 维力矩，直接塞进 LowCmd.motor_cmd[i].tau。
    #
    # 竖直力按权重归一，总和恒等于体重。 这里是踩过坑的地方：原来每条
    # 支撑腿固定给 mg/4，四条腿都在地上的站姿没问题，可一进步态就出事 ——
    # 小跑任意时刻只有两条腿支撑，总前馈只剩 mg/2，另一半体重全靠 kp 的
    # 静差硬顶。腿被压弯到工作空间底、足端在地上拖，支撑相根本扫不动，
    # 表现出来就是「腿一直打滑、力气过小」。真机化仿真实测：指令 0.9 只
    # 跑出 0.10 m/s，关掉前馈反而有 0.29。
    #
    # 归一之后，两条腿支撑时各扛 mg/2，单腿支撑时独扛 mg。总竖直力不再随
    # 支撑腿数量变化，机身在换腿时不会被抽掉一半支撑而往下掉。
    #
    # 但分母要夹一个下限 1.0，不能直接除 Σw。 换腿那一瞬间，旧腿的
    # 权重已经在往 0 掉、新腿的刚从 0 起，Σw 会掉到 0.1 以下 —— 除下去等于
    # 把 mg 全压给一条刚沾地的腿，力矩瞬间顶满限幅，下一帧又掉回来。实测
    # 就是这个：步态里 9.6% 的关节帧贴在 12 N·m 的限幅上，机身高度峰峰
    # 24 mm、角速度 rms 0.65 rad/s，看着就是「一直在颤」。
    # 夹到 1.0 之后，Σw<1 的时段前馈按比例少给（不足的部分 PD 补），
    # 不给尖峰。物理上也对：权重 0.15 的腿就是只沾了 15% 的地，凭什么扛
    # 整个体重。
    #
    # 符号是负的，别改成正的。 这里给的是「关节要平衡的足端外力」，
    # 取负对应"关节在抵抗足端被压下去"。静站实测：tau_ff=+1.0 机身沉
    # 12 mm、-1.0 升 12 mm —— 正的等于四条腿一起把机身往地里按，PD 的
    # 静差全花在对抗这个力上，一步一步都在刨地。
    #
    # 注意 fk_leg 的 z 是向上为正（站立位 p_z = −0.265），负号不是
    # 坐标约定的产物，是 τ = −Jᵀ·f 里那个负号。`body_wrench_tau` 是同一
    # 套约定的通用版本，`gravity_tau` 等于它「只给竖直力、不要求矩」的特例。
    @classmethod
    def gravity_tau(cls, q12, weights: dict) -> np.ndarray:
        tau = np.zeros(NUM_MOTOR)
        ws = [max(0.0, float(weights.get(leg, 0.0))) for leg in CTRL_LEGS]
        wsum = sum(ws)
        if wsum <= 1e-3:
            return tau
        f_total = cls.LEG_WEIGHT_N * len(CTRL_LEGS)   # mg
        wden = max(wsum, 1.0)          # 见上：不让换腿瞬间除出一个尖峰
        for i, leg in enumerate(CTRL_LEGS):
            w = ws[i]
            if w <= 1e-3:
                continue
            jt = cls.jacobian_leg(leg, q12[3 * i: 3 * i + 3])
            fz = -f_total * w / wden       # 机体系向上（p_z 向下为正，故取负）
            for j in range(3):
                tau[3 * i + j] = fz * jt[2][j]
        return tau

    # 机身层的力分配：期望合力 F 与绕机体质心的合力矩 M → 关节力矩。
    #
    # `F`、`M` 是地面反力在机体系里的表达（x 前、y 左、z 上，单位 N / N·m）。
    # 返回 12 维力矩，直接塞 LowCmd.motor_cmd[i].tau。
    #
    # ## 为什么要有这一层
    #
    # 原来的做法是「位置伺服 + 逐腿竖直力前馈」：承重靠前馈，姿态靠挪
    # 足端 z 目标。两套指令作用在同一条腿上 —— 位置伺服想把脚钉在轨迹上，
    # 前馈想把机身顶起来，于是实际出力等于两者之和，谁也不知道地面反力
    # 到底是多少。腿在这种状态下的刚度是「kp 与地面刚度串联」，真实地面
    # 软一点就打滑、硬一点就震。真机上「打滑、抬不开腿」有一半来自这里。
    #
    # 真机原生的 sportmode 本来就是力矩级的 MPC + WBC，我们这份纯位置伺服
    # 在控制层级上还不如它。这一层就是把它补回来：先在机身层面定「要多大
    # 的力、多大的力矩」，再分配到每条腿，最后换成关节力矩。
    #
    # ## 分配怎么算
    #
    # 不能把「力」和「力矩」合成一个 6×3n 的方程一起解。n=2（对角小跑）
    # 时那个方程秩只有 5、条件数 2×10⁴ —— 实测解出来的单腿力能到 100 N
    # 以上、正负乱跳，乘上雅可比就是 80 N·m 的力矩，全被限幅削平。
    # 根因是它没有单边约束：地面只能推不能拉，而最小二乘会给出「某条腿
    # 往下拽地面」的解，那在物理上不存在。
    #
    # 改成按任务优先级分两步，各自良态：
    #
    # 1. 竖直力管承重和俯仰/滚转。总和严格等于 F_z（先按支撑权重分，
    #    再在 Σdf=0 的零空间里加力矩差额），然后夹到 [0, FZ_MAX]。
    #    Mx = Σ py·fz、My = −Σ px·fz，力臂只有十几厘米，2 N·m 的力矩
    #    就对应几十牛的差 —— 姿态环给的力矩必须按这个能力限住，见
    #    `_wrench_cmd` 里的 `WRENCH_M_MAX`。
    # 2. 水平力管推进和偏航，逐腿限在摩擦锥 μ·fz 内。
    #
    # 这就是四足常用的「虚拟模型控制」：姿态和承重由竖直力，速度由水平力，
    # 两套解耦，不会互相拽。
    #
    # ## 还差什么（决定这套能不能算「力矩控制」）
    #
    # 这一层只闭环了 6 维合力和合力矩，力度是 τ = −Jᵀ·F。完整的计算力矩
    # 控制是 τ = M(q)·q̈_ref + C(q,q̇) + g(q) + Jᵀ·F_ext，缺的两项在这里：
    #
    # - 零空间姿态项。−Jᵀ·F 只在支撑腿的「能出力」方向上有定义，腿在
    #   自己的零空间（膝角与髋角同向转、足端不动的组合）里没人管，会慢慢
    #   漂。现在的做法是靠外面那圈位置环（`WRENCH_KP_STANCE`）兜住，所以
    #   kp 不能真的降到 0：实测 0.10 勉强站住，0.05 和 0 前进 5 秒直接翻。
    #   补 N·τ_posture（N 是接触雅可比的零空间投影）之后，kp 才能继续往下让。
    # - 逆动力学前馈 M·q̈ + C。现在摆动腿纯靠位置环跟踪，没有惯性/科氏
    #   补偿，只有重力的静态部分。
    #
    # 注意两边都不是「纯开环力矩」：宇树的 WBC 出来也是关节力矩 + 关节级
    # PD 兜底，真机上模型不准，留一圈小刚度位置环是必须的，不是妥协。
    #
    # ## 力矩的符号
    #
    # τ = −Jᵀ·f。负号来自平衡关系：地面给足端 +z 的推力，关节必须输出
    # 反向力矩才不会被压弯。`gravity_tau` 是 M=0、F=(0,0,mg) 的特例，
    # 两者算出来的数一致（有对拍测试）。
    @classmethod
    def body_wrench_tau(cls, q12, weights: dict, F, M,
                        reg: float = 1e-4) -> np.ndarray:
        # 只认真正着地的腿。阈值 0.12 而不是 1e-3：支撑权重的窗口前后各有
        # 一段线性过渡，权重 0.01 的腿其实已经离地（或者还没落地），把它算进
        # 分配有两个坏处 —— 一是让空中的腿被要求出力（等于往下踹，落地就是
        # 一记冲击），二是极小权重会让 Aw 接近奇异，解出来的力矩能到 100 N·m
        # 以上（实测 113）。
        idx = [i for i, leg in enumerate(CTRL_LEGS)
               if float(weights.get(leg, 0.0)) > 0.12]
        n = len(idx)
        tau = np.zeros(NUM_MOTOR)
        if n == 0:
            return tau

        px = np.zeros(n)
        py = np.zeros(n)
        for k, i in enumerate(idx):
            p = cls.fk_leg(CTRL_LEGS[i], q12[3 * i:3 * i + 3])
            px[k], py[k], _ = float(p[0]), float(p[1]), float(p[2])
        wgt = np.array([max(0.12, float(weights.get(CTRL_LEGS[i], 0.0)))
                        for i in idx])

        # ---------- 第一步：竖直力（承重 + 俯仰/滚转） ----------
        # 竖直力总和必须**精确**等于目标，否则机身高度环的积分会把误差越积越大。
        # 所以先按权重严格分摊，再在「总和不变」的零空间里加力矩所需的差额
        # （Σdf = 0），最后做单边约束。
        fz = F[2] * wgt / wgt.sum()
        need_m = np.array([M[0], M[1]], dtype=float)
        if n == 1:
            # 单腿：竖直力改不了任何力矩，只能承重。硬解只会得到一个错的数。
            pass
        else:
            # Mx = Σ py·fz，My = −Σ px·fz → 差额 df 要满足
            #   [pyᵀ; −pxᵀ]·df = [Mx, My],  Σdf = 0
            # 用「投影到零空间」的加权最小范数解：先解 3×n 的加权最小二乘，
            # 再减去均值恢复 Σdf=0。
            G = np.vstack([py, -px])                     # 2×n
            Gw = G * wgt
            lam = Gw.T @ np.linalg.solve(Gw @ Gw.T + reg * np.eye(2), need_m)
            df = wgt * lam
            df -= df.mean()                              # 恢复 Σdf = 0
            fz = fz + df
        # 单边约束：地面只能推不能拉。没有这一条，分配器会给出「某条腿往下
        # 拽」的解 —— 实测对角支撑时 fz 能到 −60 N，再乘上雅可比就是 80 N·m
        # 的力矩，被限幅削成方波，机身就被一步步抽起来。
        fz = np.clip(fz, 0.0, cls.WRENCH_FZ_MAX)
        # 削完可能总量不够（某条腿被拉到 0），把缺口补给还有余量的腿。
        # 两轮足够收敛：一轮补缺口，一轮处理补完又顶上限的情况。
        for _ in range(2):
            gap = F[2] - fz.sum()
            if abs(gap) < 0.5:
                break
            room = (cls.WRENCH_FZ_MAX - fz) if gap > 0 else fz
            if room.sum() < 1e-6:
                break
            fz = fz + gap * room / room.sum()
            fz = np.clip(fz, 0.0, cls.WRENCH_FZ_MAX)

        # ---------- 第二步：水平力（推进 + 偏航） ----------
        # 和竖直力**分开解**。合在一起解就是 6×3n 那个方程：n=2 时秩只有 5、
        # 条件数 2×10⁴，最小二乘给出的水平分量能到几十牛，四条腿方向互相
        # 打架。分开之后，竖直力只负责承重和俯仰滚转，水平力只负责推进和
        # 偏航，两套各自良态 —— 这也是四足机器人常用的「虚拟模型控制」。
        fx = np.zeros(n)
        fy = np.zeros(n)
        if n >= 2:
            H = np.zeros((3, 2 * n))
            for k in range(n):
                H[0, 2 * k] = 1.0
                H[1, 2 * k + 1] = 1.0
                H[2, 2 * k] = -py[k]        # Mz = Σ(px·fy − py·fx)
                H[2, 2 * k + 1] = px[k]
            hw = np.array([max(0.12, wgt[k]) for k in range(n) for _ in range(2)])
            Hw = H * hw
            fh = hw * (Hw.T @ np.linalg.solve(Hw @ Hw.T + reg * np.eye(3),
                                              np.array([F[0], F[1], M[2]])))
            fx, fy = fh[0::2], fh[1::2]
            # 摩擦锥：水平力不能超过 μ·竖直力。超出等于要求地面提供不存在
            # 的摩擦力，实际结果是打滑 + 力矩白给。
            mag = np.hypot(fx, fy)
            lim = cls.WRENCH_MU * fz
            scale = np.ones(n)
            over = mag > lim + 1e-6
            scale[over] = lim[over] / np.maximum(mag[over], 1e-6)
            fx, fy = fx * scale, fy * scale

        # ---------- 转成关节力矩 ----------
        for k, i in enumerate(idx):
            jt = cls.jacobian_leg(CTRL_LEGS[i], q12[3 * i:3 * i + 3])
            for j in range(3):
                tau[3 * i + j] = -(jt[0][j] * fx[k]
                                   + jt[1][j] * fy[k]
                                   + jt[2][j] * fz[k])
        return tau

    # ------------------------------------------------------------------
    # 逆运动学
    # ------------------------------------------------------------------

    # 足端在机体系的位置 → 三个关节角。
    #
    # p_body 是足端「body 原点」的位置，不是足底接触点。足底贴地时取
    # p_body = (x, y, FOOT_RADIUS)。
    #
    # 解不出来（超出腿长范围）时，clamp=True 会把目标缩到可达边界并返回
    # ok=False，让你能画图但仍知道自己越界了。
    @staticmethod
    def ik_leg(leg: str, p_body, clamp=True, check_limits=True) -> IKResult:
        hx, hy, s = LEG_GEOM[leg]
        px = p_body[0] - hx
        py = p_body[1] - hy
        pz = p_body[2]

        # --- 第一步：abduction。约束是俯仰关节在 y-z 投影里必须等于 s·d ---
        ryz2 = py * py + pz * pz
        d = THIGH_Y
        clamped = {}
        if ryz2 < d * d:
            # 足端比髋部原点的侧向偏移还近，几何上无解（腿被要求「穿过」机身）
            return IKResult(STAND_Q, False,
                            f"足端距离过近 |yz|={math.sqrt(ryz2):.4f} < {d:.4f}")

        vz = -math.sqrt(ryz2 - d * d)          # 取向下的一支
        q1 = _wrap_pi(math.atan2(pz, py) - math.atan2(vz, s * d))

        # --- 第二步：俯仰平面内解两连杆 ---
        vx = px
        L = math.hypot(vx, vz)
        l1, l2 = THIGH_LEN, CALF_LEN
        Lmax, Lmin = l1 + l2, abs(l1 - l2)

        ok, reason = True, ""
        if L > Lmax or L < Lmin:
            ok = False
            target = min(max(L, Lmin), Lmax)
            reason = f"腿长越界 L={L:.4f} ∉ [{Lmin:.4f}, {Lmax:.4f}]"
            if not clamp:
                return IKResult((q1, 0.0, 0.0), False, reason)
            k = target / L if L > 1e-9 else 1.0
            vx, vz = vx * k, vz * k
            L = target
            clamped["L"] = target

        cos_q3 = (L * L - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = -math.acos(cos_q3)               # 屈膝，取负分支（range 全在负半轴）

        px2 = -l2 * math.sin(q3)
        pz2 = -l1 - l2 * math.cos(q3)
        phi_thigh = math.atan2(px2, -pz2)     # 这两个 atan2 里 -pz 不能写成 pz
        phi_target = math.atan2(vx, -vz)
        q2 = phi_thigh - phi_target

        q = (q1, q2, q3)

        if check_limits:
            lo1, hi1 = LIMITS["abduction"]
            lo2, hi2 = LIMITS_REAR_THIGH if leg in ("RL", "RR") else LIMITS["thigh"]
            lo3, hi3 = LIMITS["knee"]
            bad = []
            if q1 < lo1 or q1 > hi1:
                bad.append(f"abd {q1:+.3f} ∉ [{lo1:+.3f},{hi1:+.3f}]")
            if q2 < lo2 or q2 > hi2:
                bad.append(f"thigh {q2:+.3f} ∉ [{lo2:+.3f},{hi2:+.3f}]")
            if q3 < lo3 or q3 > hi3:
                bad.append(f"knee {q3:+.3f} ∉ [{lo3:+.3f},{hi3:+.3f}]")
            if bad:
                ok = False
                reason = "; ".join(bad) if not reason else reason + "; " + "; ".join(bad)

        return IKResult(q, ok, reason, clamped)

    # feet_body = {'FL': (x,y,z), ...} → {'FL': IKResult, ...}
    @classmethod
    def ik_all(cls, feet_body: dict, **kw) -> dict:
        return {leg: cls.ik_leg(leg, feet_body[leg], **kw) for leg in LEGS}

    # 四足落点 → 12 个关节角，按执行器顺序（FR, FL, RR, RL）排列。
    def joint_from_feet(self, feet: dict, strict: bool = True) -> np.ndarray:
        q = np.zeros(NUM_MOTOR)
        for i, leg in enumerate(CTRL_LEGS):
            r = self.ik_leg(leg, feet[leg])
            if not r.ok and strict:
                raise RuntimeError(f"IK 失败 {leg} ({feet[leg]}): {r.reason}")
            q[3 * i: 3 * i + 3] = r.q
        return q

    # 站立姿态的 12 个关节角，按执行器顺序（FR, FL, RR, RL）排列。
    def stand_joint_target(self, height: float | None = None) -> np.ndarray:
        return self.joint_from_feet(
            self.stand_feet(self.height if height is None else height,
                            self.gait.stance_spread))

    # ------------------------------------------------------------------
    # 足端轨迹
    # ------------------------------------------------------------------

    # 支撑相里足端相对机身的速度，也就是落地瞬间必须匹配到的目标。
    #
    # 线速度贡献 -v；偏航给一个绕 z 的切向速度 v = ω × r，r 是足端到机身
    # 中心的水平位移，所以左前腿和右后腿的切向速度方向相反。
    def stance_velocity(self, leg: str, vx: float, vy: float,
                        yaw_rate: float) -> tuple[float, float]:
        nx, ny, _ = self.gait.neutral[leg]
        wx = -yaw_rate * ny if abs(yaw_rate) > 1e-12 else 0.0
        wy = yaw_rate * nx if abs(yaw_rate) > 1e-12 else 0.0
        return (-(vx + wx), -(vy + wy))

    # 足端在机体系的水平速度（后向差分，采样点全落在同一相位段内）。
    #
    # 只取 x、y —— 竖直方向在落地瞬间本来就是 0，和支撑相的 0 天然一致，
    # 真正会打滑的是水平分量。
    def foot_velocity(self, leg: str, t: float, vx: float, vy: float,
                      yaw_rate: float, h: float = 1e-7) -> tuple[float, float]:
        p0 = self.foot_target(leg, t - h, vx, vy, yaw_rate)
        p1 = self.foot_target(leg, t, vx, vy, yaw_rate)
        return ((p1[0] - p0[0]) / h, (p1[1] - p0[1]) / h)

    # 在时刻 t，返回该腿足端在机体系的期望位置。
    #
    # vx, vy: 机身期望线速度（m/s），vy 正为向左
    # yaw_rate: 期望偏航角速度（rad/s），正为逆时针
    def foot_target(self, leg: str, t: float, vx: float = 0.0, vy: float = 0.0,
                    yaw_rate: float = 0.0) -> tuple[float, float, float]:
        gp = self.gait
        nx, ny, _ = gp.neutral[leg]
        ph = (t / gp.period + gp.offsets[leg]) % 1.0
        nz = FOOT_RADIUS - self.height

        # 速度窗口的夹紧**不在这里做**。夹在 foot_target 里只能削轨迹，削不到
        # 速度闭环算出来的 evx —— 于是闭环继续按「还没跑到 0.9」满步幅地刨，
        # 轨迹却被削窄，两边打架。夹在 _gait_feet_publish 入口，一处生效。

        # 支撑相内足端相对机身的净位移：线速度 + 偏航切向速度
        stx, sty = gp.stride(vx, vy)
        if abs(yaw_rate) > 1e-12:
            wx = -yaw_rate * ny
            wy = yaw_rate * nx
        else:
            wx = wy = 0.0
        dx = (stx + wx * gp.stance_time) * 0.5 * gp.stride_gain
        dy = (sty + wy * gp.stance_time) * 0.5 * gp.stride_gain
        lim = gp.stride_max
        dx = max(-lim, min(lim, dx))
        dy = max(-lim, min(lim, dy))

        if ph < gp.duty:
            # --- 支撑相：从「前半个行程」匀速退到「后半个行程」 ---
            s = ph / gp.duty                      # 0→1
            f = 0.5 - s                           # +0.5 → -0.5
            x = nx + 2.0 * dx * f
            y = ny + 2.0 * dy * f
            # 竖直弹簧：触地时腿是自然长度 → 中段压下去（机身下沉）→ 离地前蹬到
            # 比自然长度还长（机身被顶起来）。没有这一段，四脚全程贴地，机身只能
            # 靠腿的角度上下蹭两厘米，腾空相是假的，「跳跑」看着跟快走一样。
            z = nz + gp.bounce * (math.sin(math.pi * s) - s) if gp.bounce else nz
            # 主动压地：把脚再按下去 stance_sink，撑出一条稳定的法向力。
            # 「刚好贴地」在位置控制下就等于「时接时离」—— 机身一抬头脚就悬空，
            # 没有法向力就没有摩擦，脚只会在地上打滑，机身一点都不往前走。
            # 两端各留 15% 收回去：落地不硬砸、离地不拽着地面走。
            if gp.stance_sink:
                z -= gp.stance_sink * min(1.0, s / 0.15, (1.0 - s) / 0.15)
        else:
            # --- 摆动相：从支撑相终点跨到下一个支撑相起点 ---
            s = (ph - gp.duty) / (1.0 - gp.duty)   # 0→1
            c = gp.a3 * s ** 3 + gp.a4 * s ** 4
            x = (nx - dx) + 2.0 * dx * c
            y = (ny - dy) + 2.0 * dy * c
            z = nz + gp.swing_height * _raise(s)

        return (x, y, z)

    def all_feet(self, t: float, vx: float = 0.0, vy: float = 0.0,
                 yaw_rate: float = 0.0) -> dict:
        return {leg: self.foot_target(leg, t, vx, vy, yaw_rate) for leg in LEGS}

    # ------------------------------------------------------------------
    # 构造与通信
    # ------------------------------------------------------------------

    def __init__(self, domain: int = 1, iface: str = "lo0",
                 height: float = NOMINAL_HEIGHT, kp: float | None = None,
                 kd: float | None = None, kp_calf: float | None = None,
                 gait: GaitParams | None = None, real: bool = False,
                 tau_ff: float | None = None):
        self.domain = domain
        self.iface = iface
        # 真机：刚度按实物经验值给（官方调试框架的站立段就是 kp/kd 乘一个
        # pd_ratio）。仿真里 kp 给低一些反而更稳，因为仿真没有减速器摩擦死区、
        # 也没有电机响应延迟，kp 一大就振。
        self.real = real
        # 支撑腿的位置环缩放在真机 / 仿真上不一样，理由见 WRENCH_KP_STANCE_REAL。
        self.wrench_kp_stance = (self.WRENCH_KP_STANCE_REAL if real
                                 else self.WRENCH_KP_STANCE)
        # 高度环的增益在真机上要收。反馈量是四条支撑腿正解出来的高度，
        # 落地/离地换腿、关节噪声都会进到里面，实测纹波 28 mm —— 乘 KZ 1500
        # 就是 ±42 N，乘 DZ 180 再叠一层。法向力这样抖，摩擦是断的，支撑相
        # 推不动，而摆动相前摆的反作用反而把机身往后顶（实测里程计 −0.015）。
        # 真机只留高度的低频分量：KZ 打四折，DZ 直接关（那一路的差分噪声
        # 比有用信号还大）。
        self.wrench_kz = self.WRENCH_KZ * (0.4 if real else 1.0)
        self.wrench_dz = self.WRENCH_DZ * (0.0 if real else 1.0)
        if kp is None:
            kp = self.KP_REAL if real else self.KP_SIM
        if kd is None:
            kd = self.KD_REAL if real else self.KD_SIM
        self.height = height
        # 默认机械档：就是高层运动原来那套标准小跑，最稳，开机先别跑花活。
        self.gait = gait or make_gait("mech", height, real)
        # 中立位必须跟着机身高度走，否则改 height 后站姿和轨迹会用两套基准
        self.gait.neutral = _neutral_feet(height + self.gait.stand_raise,
                                          self.gait.stance_spread)
        self.gait_style = (self.gait.name if self.gait.name in self.GAIT_STYLES
                           else "mech")

        self.kp = np.full(NUM_MOTOR, float(kp))
        if kp_calf is not None:
            self.kp[[2, 5, 8, 11]] = float(kp_calf)
        self.kd = np.full(NUM_MOTOR, float(kd))

        # 重力前馈强度（0 关，1 全补偿）。真机默认开，仿真默认关。
        if tau_ff is None:
            tau_ff = self.TAU_FF_REAL if real else self.TAU_FF_SIM
        self.tau_ff = float(tau_ff)
        self._tau = np.zeros(NUM_MOTOR)
        # 机身层力矩指令的滤波状态，见 _slew_tau。None 表示还没开始，第一帧
        # 直接取输入（否则会从 0 爬上去，起歩那一下腿是软的）。
        self._tau_w = None
        # 速度环输出的平滑状态（见 _smooth_ev）。f 是滤波值，p 是上一帧输出，
        # slew 限的是「这一帧相对上一帧」，两个都要留。
        self._evx_f = self._evy_f = 0.0
        self._evx_p = self._evy_p = 0.0
        # 姿态修正量的上一帧值（见 _slew_stab）。
        self._dz_p = self._dr_p = 0.0
        # 足端 z 的实测偏置（见 _gait_feet_publish 里那段）。None = 还没测。
        self._z_off = None
        # 运动学机身高度（真机用，见 _update_kin_height）。
        self._h_kin = None
        # 起步时实测的站姿高度，作为高度环的目标（见 _wrench_cmd）。
        self._h_des = None

        # 遥测状态
        self.q: np.ndarray | None = None
        self.dq: np.ndarray | None = None
        # 电机侧估计力矩（只读诊断，见 on_low）。仿真桥也会填。
        self.tau_est: np.ndarray | None = None
        self.motor_temp: list[float] | None = None
        self.motor_lost: list[int] | None = None
        self.rpy: list[float] = []
        # IMU 角速度 (rad/s)。只用来做姿态阻尼 —— 光有角度反馈是「弹簧」，
        # 机身在平衡点两端来回荡；加上角速度项才把它收住。
        self.gyro: list[float] = []
        # 机体系比力（m/s²）。真机 imu_state.accelerometer 的语义：静止平放
        # 时 (0,0,+9.81)，即「加速度 − 重力加速度」。
        self.acc: list[float] = []
        self.pos: np.ndarray | None = None
        self.vel: np.ndarray | None = None
        self.frames = 0
        self.peak_z = 0.0

        # rt/sportmodestate 最后一次更新的时刻。真机一旦被 rt/lowcmd 接管，
        # 内置运动服务就退出了，这个 topic 停止更新 —— 而我们的速度闭环要拿
        # 它当真值。不判新鲜度的话，self.vel 会一直是上一帧（通常 0），
        # 闭环算出来就是「实测 0、指令 0.5」，脚按 1.5 倍速度往后刨，真机上
        # 就是这个「一直在打滑、跑不动」。
        self._sport_t = 0.0
        self.SPORT_TIMEOUT = 0.25

        # 关节里程计：sportmodestate 停发时的速度后备。用关节编码器的实际角度
        # 做正解、按支撑腿反推机身速度（见 _step_odom）。
        self._odom_p: dict | None = None
        self._odom_t = 0.0
        self._odom_v = np.zeros(3)
        self._odom_ok = False
        # 每条腿的支撑权重（1=踩实，0=在空中），由步态相位每帧更新，
        # 里程计和重力前馈都用它。
        self._stance_w = {leg: 1.0 for leg in CTRL_LEGS}

        # 控制回路实际周期。这条回路决定关节力矩，周期一抖就是力矩一抖；
        # 真机上「腿在颤、电机啸叫」有一半出在这里（详见 loop_stats）。
        self._tick_t = 0.0
        self._tick_dts: deque[float] = deque(maxlen=400)

        # 控制回路状态
        self._pub = None
        self._cmd = None
        self._crc = CRC()
        self._thread = None
        self._mode = "idle"
        self._hold_q: np.ndarray | None = None
        # 释放前官方运控的模式名，关机时按原样交还。空串表示没释放过。
        self._mcf_prev_mode = ""
        self._last_pub: np.ndarray | None = None
        self._pose: dict | None = None
        self._gait: dict | None = None
        self._vel_target = [0.0, 0.0, 0.0]
        # 速度伺服最后一帧的实际速度。跳跃这类动作会把步态顶掉，落地后要按
        # 这个速度起步 —— 否则 0.3 m/s 走着的狗，跳完得从 0 重新爬，手感上
        # 就是「跳跃把速度吃掉了」。
        self._last_vel = [0.0, 0.0, 0.0]
        # 反馈滤波状态：俯仰、滚转（rad）、前进速度（m/s）。每次速度伺服起步
        # 都清零 —— 上一段跑的残留值会让第一步就踩偏。
        self._pit_filt = 0.0
        self._rol_filt = 0.0
        self._v_filt = 0.0
        # 横移速度同样要滤：真机 sportmodestate 的 vy 毛刺比 vx 还大，
        # 直接进推力环会左右来回推，机身在地上搓。
        self._vy_filt = 0.0
        # 机身速度估计是否可用。用 False 起手：步态第一帧还没读到里程计。
        self._v_ok = False
        # IMU 积分速度（世界系），以及积分时间戳。加速度计有零偏，纯积分
        # 会慢慢漂，所以只当「短时基准」，靠关节里程计慢修正。
        self._v_imu = np.zeros(3)
        self._imu_t = 0.0
        self._imu_ok = False
        # 偏航保持的参考角：起步时锁在当前角，之后按 yaw 指令积分往前推。
        self._yaw_ref = None
        # --- 自研力路（gait.mode == "mpc"）的状态 ---
        # mpc / wbc 惰性构造：构造一次要建 144×144 的 QP 结构，进步态时才建，
        # 站着用 VMC 的时候一个字节都不占。
        self.mpc = None
        self.wbc = None
        # MPC 在 50 Hz 独立线程里跑（见 `_mpc_worker`）。`_mpc_plan` 是它交给
        # 200 Hz 回路的交接点，只在锁内读写。
        self._mpc_th = None
        self._mpc_stop = None
        self._mpc_lock = threading.Lock()
        self._mpc_plan = None
        self._mpc_err = None
        self._mpc_com = np.zeros(3)      # 整机质心（机体系），_ensure_mpc 里算
        self._mpc_miss = 0               # 200 Hz 回路没拿到计划的帧数
        self._dq_f = None                # 送 WBC 的关节速度（低通后），见 _dq_lp
        self._dq_t = 0.0
        self._mpc_yaw = None             # MPC 的参考航向（起步时锁住）
        self._mpc_n = 0                  # MPC 累计求解次数，量实际频率用
        # 竖直阻尼的滤波状态：机身高度差分出来的竖直速度，以及上一帧的高度
        # 采样点。见 _gait_feet_publish 里的 dz_vdamp。
        self._vz_filt = 0.0
        self._h_prev = None
        self._h_t = 0.0
        self._t0 = 0.0

        # 姿态序列：一次排好几段插值，交给控制回路一段段播完。
        # 有它动作就不用 sleep 等做完 —— 排完立即返回，主循环照常跑。
        self._steps: list[tuple] = []
        self._step_i = 0
        self._seq_label = ""
        self._seq_z0: float | None = None
        # 排序列在调用方线程、播序列在 200 Hz 回路，_steps 与 _step_i 必须一起改
        self._seq_lock = threading.RLock()

        # 子对象：高层运行控制器。控制器只负责把它造出来并挂在自己身上，
        # 控制回路里从不调用它的方法（依赖是单向的：高层 → 控制器）。
        # 延迟导入：go2_high_level 反过来要用本模块的 MIN_HEIGHT / _wrap_pi，
        # 顶层互相 import 会成环，所以放在构造时导入（此时本模块已加载完）。
        from quadruped.control.high_level.ours import HighLevelController

        self.high_level = HighLevelController(self)

    # -- 连接 ----------------------------------------------------------

    # 关掉宇树官方运控（mcf / sport_mode），把 rt/lowcmd 让给我们。
    #
    # 这是真机上所有异常的共同前提。 Go2 开机就起 mcf，它自己也在往
    # `rt/lowcmd` 发关节指令。我们不发 MotionSwitcher 释放，两条指令就在
    # 同一个 topic 上交替覆盖：下发的 kp=100 能否落地取决于这一帧是谁的，
    # 实际关节刚度在两者之间来回跳。
    #
    # 表现完全对得上实测：膝静差 10° 而 tau_est 只有 6.8 N·m（等效刚度
    # 只有下发的四成）、站立时 1.2 Hz 的持续抖动（两个控制器的拍频）、
    # tau_est 帧间跳 4.2 N·m、抬腿跟踪只有三成、怎么调增益都没用。
    # 仿真里永远复现不了 —— 仿真桥只收发 lowcmd，没有第二个源。
    #
    # 返回是否真的释放成功（再查一次模式确认为空）。
    def _release_motion_service(self) -> bool:
        if not self.real:
            return False
        # `real` 只是「用真机那套增益」，仿真是 lo0 也会置真。真正的硬件
        # 看网卡：回环口上没有 mcf，去调只会白等 5 秒超时。
        if self.iface.startswith("lo"):
            return False
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
                import MotionSwitcherClient
            sc = MotionSwitcherClient()
            sc.SetTimeout(5.0)
            sc.Init()
            code, data = sc.CheckMode()
            cur = (data or {}).get("name", "") if code == 0 else ""
            self._mcf_prev_mode = cur
            if not cur:
                return False
            sc.ReleaseMode()
            time.sleep(0.3)
            code, data = sc.CheckMode()
            left = (data or {}).get("name", "") if code == 0 else "?"
            if left:
                print(f"[运控] 释放失败，当前仍是 {left}", flush=True)
                return False
            print(f"[运控] 已从 {cur} 释放，lowcmd 归我们独占", flush=True)
            return True
        except Exception as e:                       # 老固件没这个服务
            print(f"[运控] 释放异常：{e}", flush=True)
            return False

    # 把运控还给官方，否则拔掉我们之后狗一直没人管。
    def _restore_motion_service(self):
        if not self._mcf_prev_mode:
            return
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
                import MotionSwitcherClient
            sc = MotionSwitcherClient()
            sc.SetTimeout(5.0)
            sc.Init()
            sc.SelectMode(self._mcf_prev_mode)
            print(f"[运控] 已交还 {self._mcf_prev_mode}", flush=True)
        except Exception as e:
            print(f"[运控] 交还失败：{e}", flush=True)
        self._mcf_prev_mode = ""

    # 初始化 DDS、挂订阅、建发布器，并等第一帧 rt/lowstate。
    #
    # 一个进程只能调用一次 ChannelFactoryInitialize；重复调用会抛异常。
    def connect(self, timeout: float = 3.0) -> bool:
        ChannelFactoryInitialize(self.domain, self.iface)

        def on_low(msg):
            self.q = np.array([msg.motor_state[i].q for i in range(NUM_MOTOR)])
            self.dq = np.array([msg.motor_state[i].dq for i in range(NUM_MOTOR)])
            # 电机侧估计力矩。纯读数，不参与任何控制。
            # 用途：分辨「关节不转」是驱动器把力矩削顶了（tau_est 卡在
            # 持续力矩附近），还是根本不需要那么大力（tau_est 却很大 =
            # 有外力顶着，比如脚粘在地上）。顺带记温度和通信丢失计数：
            # 前者是电机过载的直接证据，后者能排除掉线导致的假故障。
            self.tau_est = np.array([msg.motor_state[i].tau_est
                                     for i in range(NUM_MOTOR)])
            self.motor_temp = [float(msg.motor_state[i].temperature)
                               for i in range(NUM_MOTOR)]
            self.motor_lost = [int(msg.motor_state[i].lost)
                               for i in range(NUM_MOTOR)]
            self.rpy = [float(v) for v in msg.imu_state.rpy]
            self.gyro = [float(v) for v in msg.imu_state.gyroscope]
            # 机体系比力（含重力，静止时 z≈+9.81）。真机和仿真桥的语义一致，
            # 见 simulate_python/unitree_sdk2py_bridge.py 里那段换算。
            self.acc = [float(v) for v in msg.imu_state.accelerometer]
            self.frames += 1

        def on_sport(msg):
            self.pos = np.array([float(msg.position[i]) for i in range(3)])
            self.vel = np.array([float(msg.velocity[i]) for i in range(3)])
            self._sport_t = time.time()

        ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)
        ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(on_sport, 10)

        # 等反馈。没有反馈的 PD 是开环，直接发指令会把狗抽飞。
        t_wait = time.time()
        while self.frames == 0 and time.time() - t_wait < timeout:
            time.sleep(0.05)
        if self.frames == 0:
            return False

        self._pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._pub.Init()
        self._cmd = unitree_go_msg_dds__LowCmd_()
        self._cmd.head[0] = 0xFE
        self._cmd.head[1] = 0xEF
        for i in range(NUM_MOTOR):
            self._cmd.motor_cmd[i].mode = self.MODE_SERVO

        # 真机：先把官方运控让出去，再开始发我们自己的关节指令。
        # 这一步不做，后面的所有参数调整都是在跟另一个控制源拔河。
        if self.real and self._release_motion_service():
            # 释放的那一刻 mcf 停止发 lowcmd，狗会立刻失力。用当前姿态起一个
            # hold 回路攥住它，避免从站姿直接软下去。
            self._hold_q = self.q.copy()
            self._mode = "hold"
            self._ensure_loop()
        return True

    @property
    def connected(self) -> bool:
        return self._pub is not None and self.q is not None

    # 用支撑腿正解反推机身离地高度，写进 `_h_kin`。
    #
    # 真机上必须用这一路。sportmodestate 的 z 在被 rt/lowcmd 接管之后就不
    # 反映真实高度了：实测狗站着 0.303 m，它报 0.248 m，差 5.5 cm。高度环
    # 拿它当反馈，等于一直认为「还没到目标高度」，于是持续往外给 30 多牛
    # 的推力 —— 机身被顶得比站姿还高，四条腿在竖直方向上和地面较劲，水平
    # 方向的推力全被这个静态冲突吃掉，看起来就是脚踩着地、腿在动、机身
    # 不走，还一直响。
    #
    # 只用支撑腿（权重 > 0.12）：摆动腿悬空，它的正解量的是抬腿高度，不是
    # 机身高度。没有支撑腿就保持上一次的值。
    def _update_kin_height(self):
        if self.q is None:
            return
        hs = []
        for i, leg in enumerate(CTRL_LEGS):
            if float(self._stance_w.get(leg, 0.0)) > 0.12:
                p = self.fk_leg(leg, self.q[3 * i:3 * i + 3])
                hs.append(FOOT_RADIUS - float(p[2]))
        if not hs:
            return
        h = sum(hs) / len(hs)
        # 一阶低通：正解本身不抖，但落地/离地换腿那一帧参与计算的腿会变，
        # 直接切会让高度环看到一个阶跃。
        if self._h_kin is None:
            self._h_kin = h
        else:
            k = 1.0 / (1.0 + self.CTRL_HZ * 0.04)
            self._h_kin += k * (h - self._h_kin)

    # 机身高度（离地），米。真机用运动学，仿真用状态估计。
    #
    # 真机那一路的原因见 `_update_kin_height` 的文档。仿真的 pos[2] 是
    # 真值，没必要绕。
    def body_height(self) -> float | None:
        if self.real and self._h_kin is not None:
            return self._h_kin
        return None if self.pos is None else float(self.pos[2]) - SPORT_Z_BIAS

    # IMU 加速度积分得到的机体系速度，m/s。没有加速度数据返回 None。
    #
    # 为什么需要它：关节里程计算出来的不是机身速度，而是足端在机体系里
    # 移动多快。不打滑时两者相等；打滑时足端按指令在地上刨，里程计就把
    # 「指令扫动速度」当成了机身速度 —— 于是机身越滑、估计越快，速度闭环
    # 以为已经跑够，反而把扫动收小，狗就慢慢停下来。实测 14 秒里估计值比
    # 真实值高 0.6 m/s，位移从 0.25 m/s 掉到 0。IMU 不受打滑影响。
    #
    # 积分必然漂（加速度计零偏），所以它只当「短时基准」，长时的零偏靠
    # 关节里程计慢修正，见 `body_velocity`。
    #
    # a_world = R·f_body + (0,0,−g)（f_body 是比力），积分后转回机体系。
    def _imu_vel(self) -> np.ndarray | None:
        if len(self.acc) < 3 or not self.rpy:
            return None
        now = time.perf_counter()
        dt = now - self._imu_t
        self._imu_t = now
        # 同一帧被调用多次就直接给缓存：否则第二次的 dt 只有几十微秒，
        # 积进去的是一个几乎为零的增量，但时间戳被推后了，下一帧的 dt
        # 就少了一截 —— 长期下来积分出来的速度系统性偏小。
        if dt < 0.5 / self.CTRL_HZ:
            return self._v_imu if self._imu_ok else None
        if not (0.0 < dt < 0.05):
            return self._v_imu if self._imu_ok else None
        R = np.array(_rpy_to_R(self.rpy[0], self.rpy[1], self.rpy[2]),
                     dtype=float)
        a_w = R @ np.array(self.acc, dtype=float)
        a_w[2] -= self.GRAVITY
        self._v_imu += a_w * dt
        self._imu_ok = True
        # 积分炸了（超出狗能跑的量级）就重置，不然误差会一路带下去
        if float(np.abs(self._v_imu).max()) > 3.0:
            self._v_imu[:] = 0.0
        return R.T @ self._v_imu

    # 实测速度，机体系 (vx, vy, vz)，m/s；拿不到返回 None。
    #
    # 三个来源，按可信度从高到低：
    #
    # 1. `rt/sportmodestate`（新鲜时）。真机原生的 UKF 状态估计，最准。
    #    但 lowcmd 接管后这个 topic 就不更新了，只有在 sportmode 还没被顶掉
    #    时才用得上。注意它是世界系的，按当前 yaw 转回机体系 —— 横移、原地
    #    转、跳跑的时候，世界系的 vx 和「机身往前跑多快」不是一回事。
    # 2. IMU 积分。短时可信，打滑也不受影响，缺点是会漂。
    # 3. 关节里程计。长时无偏，但打滑时测到的其实是指令扫动速度，
    #    打滑越狠高估越多。
    #
    # 所以融合方式是以 IMU 为主、里程计为辅：两者差得不远（< 0.25 m/s）
    # 时用里程计把 IMU 的漂移慢慢拉回来；差得远说明正在打滑，这时里程计
    # 是错的，直接不用它。
    def body_velocity(self) -> tuple[float, float, float] | None:
        if self.vel is not None and time.time() - self._sport_t < self.SPORT_TIMEOUT:
            yaw = float(self.rpy[2]) if self.rpy else 0.0
            c, s = math.cos(yaw), math.sin(yaw)
            return (float(c * self.vel[0] + s * self.vel[1]),
                    float(-s * self.vel[0] + c * self.vel[1]),
                    float(self.vel[2]))
        v_imu = self._imu_vel()
        if v_imu is not None:
            if self._odom_ok:
                d = self._odom_v[:2] - v_imu[:2]
                if float(np.hypot(*d)) < 0.25:
                    # 200 Hz 下 0.02 的修正率 = 时间常数 0.25 s，只抹零偏
                    v_imu[:2] += 0.02 * d
            return (float(v_imu[0]), float(v_imu[1]), float(v_imu[2]))
        if self._odom_ok:
            return (float(self._odom_v[0]), float(self._odom_v[1]), 0.0)
        return None

    # 关节里程计：支撑腿足端在世界系静止 → 反推机身速度（机体系）。
    #
    # 推导：p_world = b + R·p_body，脚不打滑时左边求导为 0，
    # 忽略 Ṙ·p_body（低速转动、腿短，这一项比平动小一个量级），
    # 得 v_body ≈ -Rᵀ·(dp_world/dt)，换到机体系就是 -ṗ_body。
    #
    # 关键性质：打滑时足端相对机身几乎不动，ṗ_body 很小，估计出来的速度
    # 也接近 0 —— 这比「拿指令速度当真值」安全得多，不会越滑越猛。
    def _step_odom(self) -> None:
        if self.q is None:
            return
        now = time.time()
        p = {leg: self.fk_leg(leg, self.q[3 * i:3 * i + 3])
             for i, leg in enumerate(CTRL_LEGS)}
        prev, self._odom_p = self._odom_p, p
        t0, self._odom_t = self._odom_t, now
        dt = now - t0
        if prev is None or not (1e-4 < dt < 0.1):
            self._odom_ok = False
            return

        vx = vy = wsum = 0.0
        for leg in CTRL_LEGS:
            w = float(self._stance_w.get(leg, 0.0))
            if w <= 1e-3:
                continue
            vx += w * -(p[leg][0] - prev[leg][0]) / dt
            vy += w * -(p[leg][1] - prev[leg][1]) / dt
            wsum += w
        if wsum <= 1e-3:
            self._odom_ok = False
            return

        # 差分噪声大（5 ms 的 dt），一阶低通到 50 ms 量级再用。
        raw = np.array([vx / wsum, vy / wsum, 0.0])
        a = min(1.0, dt / self.STAB_TAU)
        self._odom_v += a * (raw - self._odom_v)
        self._odom_ok = True

    def require_connection(self):
        if not self.connected:
            raise RuntimeError("还没连上：先调用 connect()，并确认仿真器在跑")

    # -- 单一控制回路 --------------------------------------------------

    def _ensure_loop(self):
        if self._thread is None:
            self._thread = RecurrentThread(interval=1.0 / self.CTRL_HZ,
                                           target=self._tick, name="go2")
            self._thread.Start()

    def _tick(self):
        # 回路节奏。抖动排查第一件事就是看这个：标称 5 ms，实际要是一直
        # 6~9 ms 上下跳，PD 的增益就等于在随机变化，什么参数都调不稳。
        now = time.perf_counter()
        if self._tick_t:
            dt = now - self._tick_t
            if dt < 0.5:                    # 跨越大停顿（暂停过）就不算
                self._tick_dts.append(dt)
        self._tick_t = now

        # 里程计每帧算一次（差分要连续），由 body_velocity() 读取。
        self._step_odom()
        self._update_kin_height()
        mode = self._mode
        # 这里**不停** MPC 线程。线程跟着 `Go2Robot` 的 connect / shutdown 走，
        # 不跟步态走：MPC 是常设部件，收步时停掉、再起步时重启，等于每次切换
        # 都赌一次线程重新同步。
        #
        # 代价是站姿期间它还在解一份没人用的计划（`_mpc_stand_publish` 用的是
        # 固定 wrench，不读 plan），白烧一份 CPU。这是有意的：宁可多算，也不要
        # 在 hold↔gait 的边界上出现「线程刚起、计划还是旧的那份」。旧计划交给
        # `MPC_STALE` 拦，不会真被拿去用。
        if mode in ("hold", "pose"):
            # 站着不动、或者摆姿态，四条腿都是踩实的，权重一律 1。**必须每帧
            # 重置**：步态中途收步时，最后一帧可能有一条腿正处在摆动相（权重
            # 0），这个 0 会一直留到姿态模式里 —— 那条腿不带重力前馈，腿是软
            # 的，而按权重归一又会把它的份额分给另外三条，机身被顶得前后歪。
            for leg in CTRL_LEGS:
                self._stance_w[leg] = 1.0
        if mode == "hold":
            # 站着不动也要给前馈 —— 真机上不给我前馈，站姿全靠 kp 的静差撑着，
            # 腿是软的、机身下沉、一推就走。
            if self.gait.mode == "mpc" and self._hold_q is not None:
                self._mpc_stand_publish(self._hold_q)
            else:
                self._publish(self._hold_q, tau=self._tau_ff(self._hold_q))
        elif mode == "pose":
            self._pose_tick()
        elif mode == "gait":
            self._gait_tick()
        if mode != "idle":                  # 回路在跑就顺手记峰值，跳跃靠它量高度
            h = self.body_height()
            if h is not None and h > self.peak_z:
                self.peak_z = h

    # 按当前支撑权重算重力前馈力矩；没开前馈返回 None。
    #
    # 算出来的值先过一阶低通再限幅：步态里 q_target 每帧都在动，原始 τ 会
    # 跟着逐帧跳，叠加到 PD 上就是可听的抖动。限幅是防一条腿被判成全支撑
    # 时给出远超关节额定的力矩。
    def _tau_ff(self, q_target):
        if not self.tau_ff or q_target is None:
            return None
        # 腿还没到目标附近就先收力。从趴着/阻尼状态往站姿插值的头几帧，
        # 实际关节角和目标差着上百度，这时候给「支撑体重」的力矩，腿会朝
        # 莫名其妙的方向蹬一下 —— 真机上这一下很吓人，也可能顶坏限位。
        #
        # **斜降，不能硬切。** 原来是超过阈值直接 return None，力矩从满值
        # 一步掉到 0。步态跑快了以后这个阈值会被反复穿进穿出（实测 0.9 指令
        # 下 200 帧里掐断 68 次），于是前馈变成 12 N·m 的方波，机身被一推一放
        # 地激起来，俯仰峰峰 54° —— 这是「抖」和「弹飞」的直接来源之一。
        # 现在按「差多少」线性退：阈值内全给，1.5 倍阈值外完全不给，中间连续。
        arm = 1.0
        if self.q is not None:
            err = float(np.abs(self.q - q_target).max())
            lim = self.TAU_ARM_ERR
            if err > 1.5 * lim:
                self._tau *= 0.85          # 退力，别留残值
                return None
            if err > lim:
                arm = (1.5 * lim - err) / (0.5 * lim)
        # 雅可比用**实测**关节角算，不用目标角：kp 110 下两者差 0.05 rad，
        # 雅可比跟着偏 3~5°，36.8 N 的竖直力就分出 2 N 的水平分量，四条腿
        # 方向相反、互相拉扯 —— 实测正好把前进速度从 0.24 拖到 0.06 m/s。
        q_ref = self.q if self.q is not None else q_target
        raw = self.gravity_tau(q_ref, self._stance_w) * self.tau_ff * arm
        k = 1.0 / (1.0 + self.CTRL_HZ * self.TAU_TAU)
        self._tau += k * (raw - self._tau)
        return np.clip(self._tau, -self.TAU_MAX, self.TAU_MAX)

    # 机身层力矩指令的平滑：一阶低通 + 每帧变化率限制。
    #
    # 分配器的输出是「这一帧该出多大力」，它每帧独立解一遍。输入里带着
    # 陀螺噪声（乘 WRENCH_KD）、竖直速度噪声（乘 WRENCH_DZ）、还有支撑
    # 权重跨过 0.12 时那条腿从「不参与」变成「参与」的阶跃。这些都不该
    # 原样进关节 —— 关节后面是减速器，零速附近高频换向就是齿隙撞击，
    # 听起来是硬摩擦声，而且这些往返力矩全部耗在摩擦上，地面反力反而
    # 变小，走起来就是「抖着挪、迈不开」。
    #
    # 低通管带宽，slew 管单帧幅度，两个都要：低通对阶跃会留一个很陡的
    # 上升沿，slew 把它压成斜坡。
    def _slew_tau(self, tau):
        tau = np.asarray(tau, dtype=float)
        if self._tau_w is None or self._tau_w.shape != tau.shape:
            self._tau_w = tau.copy()
            return tau
        dt = 1.0 / self.CTRL_HZ
        a = dt / (self.TAU_W_TAU + dt)
        out = self._tau_w + a * (tau - self._tau_w)
        step = self.TAU_W_SLEW
        out = np.clip(out, self._tau_w - step, self._tau_w + step)
        self._tau_w = out
        return out

    # 速度环输出 → 支撑相步幅，先让它连续。见 EVX_TAU。
    #
    # 两段：一阶低通压噪声，逐帧 slew 兜大跳（低通对阶跃会留一个陡沿）。
    def _smooth_ev(self, evx, evy):
        k = 1.0 / (1.0 + self.CTRL_HZ * self.EVX_TAU)
        step = self.EVX_SLEW
        out = []
        for cur, f, prev in ((evx, self._evx_f, self._evx_p),
                             (evy, self._evy_f, self._evy_p)):
            f += k * (cur - f)
            if abs(f - prev) > step:
                f = prev + math.copysign(step, f - prev)
            out.append(f)
        self._evx_f, self._evy_f = out
        self._evx_p, self._evy_p = out
        return out[0], out[1]

    # 姿态修正量的逐帧限速，见 STAB_SLEW。
    #
    # 俯仰和滚转各存一份上一帧的值。`w_st` 在落地那一帧还会从 0 跳到 1，
    # 这一层管不住那一下（它是权重不是修正量），但把修正量本身压到 3 mm/帧
    # 之后，两者相乘的阶跃从 45 mm 降到 3 mm，膝角跳变从 20° 降到 1.5°。
    def _slew_stab(self, val: float, axis: str) -> float:
        prev = self._dz_p if axis == "z" else self._dr_p
        step = self.STAB_SLEW
        if abs(val - prev) > step:
            val = prev + math.copysign(step, val - prev)
        if axis == "z":
            self._dz_p = val
        else:
            self._dr_p = val
        return val

    # 机身竖直速度，米/秒，向上为正。拿不到高度就返回 0。
    #
    # 用高度差分而不是 IMU 的 vz：真机 sportmodestate 的速度没有可靠的 z
    # 分量，而高度我们每帧都在算（`pos[2]` 减去固定偏置）。差分后必须低通，
    # 200 Hz 下原始差值全是编码器量化噪声。
    def _body_vz(self) -> float:
        h = self.body_height()
        if h is None:
            return 0.0
        now = time.perf_counter()
        dt = now - self._h_t
        # 同一帧内被调用两次（damp_z 一次、机身层一次）就直接给缓存值。
        # 否则第二次的 dt 是微秒级，除下去机身竖直速度会瞬间暴涨几百倍，
        # 高度环据此给一个巨大的力 —— 表现为「偶尔莫名弹飞」。
        if dt < 0.5 / self.CTRL_HZ:
            return self._vz_filt
        if 0.0 < dt < 0.05:            # 跨越大停顿（GC、卡过）就不算这一帧
            vz = (h - self._h_prev) / dt
            kz = 1.0 / (1.0 + self.CTRL_HZ * 0.04)
            self._vz_filt += kz * (vz - self._vz_filt)
        self._h_prev, self._h_t = h, now
        return self._vz_filt

    # ==================================================================
    # 自研力路：MPC（50 Hz 独立线程）+ WBC（活在 200 Hz 回路里）
    # ==================================================================
    #
    # 为什么分两层、为什么偏偏这样分工：
    #
    # VMC 那一层每帧独立算一次「要多大的合力/合力矩」，然后不管三七二十一
    # 按 Jᵀ 摊到四条腿上。它有三个结构性缺陷，靠调增益都补不上：
    #
    # 1. **没有预测**。机身这一帧低了 1 cm，它就补 15 N，下一帧到 0 了就撤。
    #    而脚落地要 20 ms 才见效，于是补过头、撤过头，环一直在追自己的延迟。
    # 2. **没有摩擦锥**。Jᵀ 摊出来的水平分量如果超过 μ·fz，地面给不出来，
    #    腿就在地上滑；分配器并不知道这件事，还在按「给得出」算。
    # 3. **没有腿的动力学**。τ = −Jᵀf 只有负载项，腿自重与摆动腿惯性全靠
    #    kp 兜 —— 这就是 kp 停在 100 降不下去的直接原因。
    #
    # MPC 解决 1 和 2（horizon 内把力摊开、显式带摩擦锥），WBC 解决 3
    # （τ = M·a + h − Jᵀ·f，把逆动力学算全）。
    #
    # 分工的分界线是**时间尺度**：MPC 的 horizon 240 ms、50 Hz 足够，而且它
    # 一帧 4.9 ms，塞进 200 Hz 的 5 ms 预算里必然超时；WBC 24 维、0.6 ms，
    # 输出就是这一帧的力矩，必须每帧跑。所以 MPC 起线程、WBC 留主回路。
    #
    # 两条路都不改 `_gait_feet_publish` 里相位/落脚点那一段：步态是上层，
    # 力路是下层，混在一起就没法做 A/B。

    # 惰性构造 MPC / WBC。第一次进 mpc 模式时花这一笔，之后一直复用。
    def _ensure_mpc(self):
        if self.mpc is not None:
            return
        from .model import whole_body_com, whole_body_inertia
        from .mpc import SRBDMPC
        from .wbc import LEGS as WBC_LEGS
        from .wbc import WBC

        # **腿序断言**。WBC 的 τ 是 12 维向量、直接进 motor_cmd，顺序错了就是
        # 四条腿互相接错；而 `wbc.LEGS` 与 `CTRL_LEGS` 分处两个模块，谁改了
        # 另一边都不会报错，只会跑起来就翻。第一次构造就把这件事掐死。
        if tuple(WBC_LEGS) != tuple(CTRL_LEGS):
            raise RuntimeError(f"腿序不一致：wbc.LEGS={WBC_LEGS} "
                               f"controller.CTRL_LEGS={CTRL_LEGS}")

        q_stand = self.stand_joint_target()
        self._mpc_com = whole_body_com(q_stand)
        self.mpc = SRBDMPC(mass=self.BODY_MASS,
                           inertia=whole_body_inertia(q_stand),
                           com_body=self._mpc_com,
                           mu=self.WRENCH_MU, fz_max=self.WRENCH_FZ_MAX,
                           horizon=self.MPC_HORIZON, dt=self.MPC_DT,
                           # 水平速度**不闭环**，见 mpc.py 里 Q 的那段：速度
                           # 环留在外层步态（speed_loop 调步幅），两层同时
                           # 管同一个自由度会互相抢。
                           w_vxy=0.0,
                           w_height=self.MPC_W_HEIGHT,
                           w_rpy=self.MPC_W_RPY,
                           w_omega=self.MPC_W_OMEGA,
                           w_force=self.MPC_W_FORCE)
        self.wbc = WBC(mu=self.WRENCH_MU, fz_max=self.WRENCH_FZ_MAX,
                       tau_max=self.WBC_TAU_MAX, com=self._mpc_com,
                       kp_foot=self.WBC_KP_FOOT, kd_foot=self.WBC_KD_FOOT,
                       stance_foot=self.WBC_STANCE_FOOT,
                       w_slew=self.WBC_W_SLEW)

    # 丢掉手上那份 MPC 计划。起步时调。
    #
    # 线程现在常驻（跟着 `Go2Robot.connect` / `shutdown`，不跟步态），所以进
    # gait 的那一刻 `_mpc_plan` 里躺着的是**上一段**的解：站姿时它是「原地不动的
    # 零速参考」，接着上一段走时它是旧的 vx。不清掉，`_mpc_feet_publish` 一进
    # gait 就照旧参考出力，实测起步被推着往旧方向走 —— walk 平均速度从 0.194
    # 掉到 0.146。
    #
    # 清掉之后 `age > MPC_STALE` 会先退化几帧到 `_tau_ff`，等新计划落下来再接上。
    # 那几帧是干净的，比拿一份语义不对的计划强。
    def _mpc_flush_plan(self):
        with self._mpc_lock:
            self._mpc_plan = None

    # 起 MPC 线程。幂等：已经活着就直接返回。正常路径由 `Go2Robot.connect()`
    # 调一次，`_mpc_feet_publish` 里那次是兜底（vmc 起步、中途切到 mpc）。
    def _mpc_start(self):
        self._ensure_mpc()
        if self._mpc_th is not None and self._mpc_th.is_alive():
            return
        self._mpc_stop = threading.Event()
        self._mpc_th = threading.Thread(target=self._mpc_worker,
                                        args=(self._mpc_stop,),
                                        name="go2-mpc", daemon=True)
        self._mpc_th.start()

    # 停 MPC 线程。**必须 join**：不 join 的话旧线程可能正卡在 `_mpc_solve_once`
    # 里（4.8 ms），而这里已经把 `_mpc_th` 清空，下一次 `_mpc_start` 就会再起
    # 一个 —— 两个线程同时调同一个 `SRBDMPC`，warm start 与迭代计数互相踩，
    # 而且不报任何错。
    #
    # 只在 connect 失败与收工时走，最多等一轮（`_mpc_worker` 里 sleep 上限
    # 20 ms），不在 200 Hz 回路里调用。
    def _mpc_stop_thread(self):
        th, ev = self._mpc_th, self._mpc_stop
        if ev is not None:
            ev.set()
        if (th is not None and th.is_alive()
                and th is not threading.current_thread()):
            th.join(timeout=0.2)
        self._mpc_stop = None
        self._mpc_th = None
        with self._mpc_lock:
            self._mpc_plan = None

    # MPC 线程主体。周期靠绝对时刻累加，不靠 sleep(period)：后者会把求解
    # 耗时也加进去，实际频率变成 1/(period+5ms) = 40 Hz，horizon 就越算越偏。
    def _mpc_worker(self, stop):
        period = 1.0 / self.MPC_HZ
        nxt = time.perf_counter() + period
        while not stop.is_set():
            try:
                plan = self._mpc_solve_once()
                self._mpc_n += 1
                if plan is not None:
                    with self._mpc_lock:
                        self._mpc_plan = plan
                        self._mpc_err = None
            except Exception as e:                     # noqa: BLE001
                # 线程里抛异常会静默死掉（daemon 线程没人收尸），200 Hz 回路
                # 只能看到「计划一直是旧的」。把异常存下来，让回路那边报。
                with self._mpc_lock:
                    self._mpc_err = f"{type(e).__name__}: {e}"
            nxt += period
            d = nxt - time.perf_counter()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.perf_counter()               # 追不上就重新对齐

    # 采一帧状态、解一次 MPC，产出一份交给 200 Hz 回路的计划。
    #
    # 两份都拿不到就返回 None（还没连上、或者不在步态里）—— 回路那边据此
    # 退回重力前馈，不会把「上一段跑剩的力」继续发出去。
    def _mpc_solve_once(self):
        from .mpc import LEGS as MPC_LEGS
        from .mpc import SRBDRef, SRBDState, contact_from_gait

        g = self._gait
        q = None if self.q is None else self.q.copy()
        if g is None or q is None or not self.rpy:
            return None
        rpy = np.array([float(self.rpy[0]), float(self.rpy[1]),
                        float(self.rpy[2])])
        h = self.body_height()
        if h is None:
            return None

        R = np.array(_rpy_to_R(*rpy), dtype=float)
        com_w = R @ self._mpc_com
        # 质心位置（世界系）。x/y 在 MPC 的代价里权重是 0（走多远是步态的事），
        # 只有 z 参与高度闭环，所以 x/y 给个粗值就够 —— 但**不能不给**：它还是
        # 参考状态里的一维，给 0 会让 `e` 里带一个巨大的常数。
        p = np.array([0.0, 0.0, h]) + com_w
        if self.pos is not None:
            p[0], p[1] = float(self.pos[0]), float(self.pos[1])
        bv = self.body_velocity()
        v = np.zeros(3) if bv is None else R @ np.array(bv, dtype=float)
        # 陀螺是机体系的，而 MPC 的模型写的是 ṙpy = ω（小角近似），所以直接用
        # 机体系角速度、不再转到世界系 —— 转过去反而和「Euler 角导数」差一个
        # 1/cos(pitch) 的因子。
        omega = (np.zeros(3) if not self.gyro
                 else np.array(self.gyro, dtype=float))
        st = SRBDState(p=p, v=v, rpy=rpy, omega=omega)

        if g.get("stream"):
            vx, vy, yaw_cmd = (float(x) for x in g["vel"])
        else:
            vx, vy, yaw_cmd = (float(g["vx"]), float(g["vy"]),
                               float(g["yaw"]))
        z_des = (self._h_des if self._h_des is not None
                 else FOOT_RADIUS - self.gait.neutral[CTRL_LEGS[0]][2])
        # 参考高度是**质心**高度，不是机身高度。差一个 R·com 的 z 分量
        # （站姿约 −2 cm），不补的话高度环会一直认为机身低了两厘米、持续
        # 多加 30 N。
        ref = SRBDRef(vx=vx, vy=vy, yaw_rate=yaw_cmd,
                      height=z_des + float(com_w[2]))

        # 参考航向：起步时锁在当前角，之后按 yaw 指令积分。**不能用实测航向
        # 当参考** —— 那样误差恒为 0，机身被地面拽着转也没人管（实测 6 秒
        # 漂 −7°）。这和 `_wrench_cmd` 里 `_yaw_ref` 的逻辑是同一套。
        if self._mpc_yaw is None:
            self._mpc_yaw = float(rpy[2])
        else:
            self._mpc_yaw += yaw_cmd * self.mpc.dt

        feet = {}
        for i, leg in enumerate(CTRL_LEGS):
            feet[leg] = self.fk_leg(leg, q[3 * i:3 * i + 3])
        tg = float(g.get("t_acc", 0.0)) - float(g.get("ramp", 0.0))
        contacts = contact_from_gait(self.gait, tg, self.mpc.N, self.mpc.dt)

        sol = self.mpc.solve(st, ref, feet, contacts, yaw_ref=self._mpc_yaw)
        F, M = sol.wrench_body()
        if not (np.isfinite(F).all() and np.isfinite(M).all()):
            raise RuntimeError("MPC 的 wrench 非有限值")
        # 头一个 knot 的逐腿力，转回机体系 —— WBC 里力的变量是机体系的。
        # 它是 WBC 的「力分配正则」目标：同样能满足 6 维 wrench，腿往内夹
        # 还是往外撑由这一项定。
        force = {name: sol.R.T @ sol.forces[0][i]
                 for i, name in enumerate(MPC_LEGS)}
        return dict(F=F, M=M, force=force, t=time.perf_counter(),
                    n_contact=int(contacts[0].sum()), info=sol.info)

    # 送进 WBC 之前把关节速度滤一下。
    #
    # 真机的 `dq` 是编码器差分出来的，1σ 有 0.15 rad/s（仿真桥里同样加了这一
    # 条，见 `unitree_sdk2py_bridge.PublishLowState`）。VMC 那边 `kd` 只有 1.5，
    # 乘上去是 0.2 N·m，看不出来；WBC 的 `kd_foot` 是 120（它在做「足端笛卡尔
    # 空间」的阻尼，不是关节空间），同一个噪声乘上去就是 18 m/s² 的任务加速度，
    # 经 M 变成每秒 1~2 N·m 的力矩抖动 —— 站着不动都在抖。
    #
    # 所以这一层不是「额外加的滤波」，是把 WBC 的输入恢复到和 `kd` 同一个噪声
    # 水平：低通 20 ms，转折 8 Hz，比摆动腿最快的运动（2~3 Hz）高得多。
    def _dq_lp(self) -> np.ndarray:
        if self.dq is None:
            return np.zeros(NUM_MOTOR)
        now = time.perf_counter()
        dt = now - self._dq_t
        self._dq_t = now
        # 同一帧被调两次（stand 与 gait 不会同时走，但保险）就给缓存
        if not (0.0 < dt < 0.05):
            if self._dq_f is None:
                self._dq_f = self.dq.copy()
            return self._dq_f
        if self._dq_f is None:
            self._dq_f = self.dq.copy()
        else:
            k = dt / (self.DQ_TAU + dt)
            self._dq_f += k * (self.dq - self._dq_f)
        return self._dq_f

    # 站姿（`hold`）走 WBC：四个脚全着地、只要求「顶住体重」。
    #
    # 为什么站姿也要走这一层：`hold` 原来发的是 `gravity_tau` —— 那个函数只
    # 算负载项 Jᵀf，没有腿自重（站姿膝力矩 0.27 N·m，是负载的 4%），也没有
    # 关节加速度项。所以站着的时候机身高度其实靠 kp 的静差撑着：kp 一降，
    # 机身就下沉，这是「kp 停在 100 降不下去」的另一半原因。
    #
    # **不能抛异常**：这条路径在 200 Hz 回路里，抛一次整条线程就死。出错就
    # 退回重力前馈 —— 那本来就是没有 WBC 时的站姿，能站住。
    def _mpc_stand_publish(self, q_out):
        try:
            self._ensure_mpc()
            q_meas = self.q if self.q is not None else q_out
            dq_meas = self._dq_lp()
            feet = {leg: np.asarray(self.fk_leg(leg, q_out[3 * i:3 * i + 3]),
                                    dtype=float)
                    for i, leg in enumerate(CTRL_LEGS)}
            from .wbc import WBCTarget

            # 力的分配目标是「四腿均分体重」。不光是物理上合理 —— 它在数值上
            # 也必需：不给 force 目标时，f 那 12 维只剩 w_force=1e-2 的正则撑着，
            # 而 τ 那 12 维只剩 w_tau=1e-4，H 的条件数跨到 1e12 量级，OSQP 每次
            # 都要迭代到上限（实测站姿一帧 3.7 ms，给了均分之后 0.4 ms）。
            mg4 = self.BODY_MASS * self.GRAVITY / 4.0
            feed = {leg: np.array([0.0, 0.0, mg4]) for leg in CTRL_LEGS}
            tgt = WBCTarget(feet=feet,
                            feet_vel={leg: np.zeros(3) for leg in CTRL_LEGS},
                            weights={leg: 1.0 for leg in CTRL_LEGS},
                            F=np.array([0.0, 0.0, self.BODY_MASS * self.GRAVITY]),
                            M=np.zeros(3), force=feed)
            res = self.wbc.solve(q_meas, dq_meas, tgt)
            self._publish(q_out, tau=res.tau)
        except Exception as e:                          # noqa: BLE001
            with self._mpc_lock:
                self._mpc_err = f"{type(e).__name__}: {e}"
            self._mpc_miss += 1
            self._publish(q_out, tau=self._tau_ff(q_out))

    # 把 MPC 给的 wrench 限到腿实际能给出来的范围。
    #
    # MPC 的 QP 只在**它自己假设的接触序列**上满足 fz_max / 摩擦锥，而那个
    # 序列是按相位预测的、而且允许四脚同时着地。真实权重可能是两条腿，可行
    # 的合力矩就小得多。实测 MPC 输出 Mx ±60 N·m，而两条腿的几何上限是
    # 22 N·m（`WRENCH_M_MAX` 取 14 是留余量的值）。
    #
    # 追一个不存在的力矩，WBC 只会把旋转那一项（权重还是平移的 10 倍）当成
    # 硬约束，牺牲摩擦锥和足端任务去凑，四条腿互相拉扯、机身被拧得乱滚。
    # VMC 那一路一直有这个限幅，接 MPC 时漏掉了。
    def _clamp_wrench(self, F, M):
        mg = self.BODY_MASS * self.GRAVITY
        F = np.asarray(F, dtype=float).copy()
        M = np.asarray(M, dtype=float).copy()
        M[0] = _clamp(float(M[0]), -self.WRENCH_M_MAX, self.WRENCH_M_MAX)
        M[1] = _clamp(float(M[1]), -self.WRENCH_M_MAX, self.WRENCH_M_MAX)
        M[2] = _clamp(float(M[2]), -2.0 * self.WRENCH_M_MAX,
                      2.0 * self.WRENCH_M_MAX)
        F[2] = _clamp(float(F[2]), 0.2 * mg, self.MPC_FZ_MAX * mg)
        lim = math.hypot(float(F[0]), float(F[1]))
        if lim > self.MPC_FXY_MAX * mg:
            s = self.MPC_FXY_MAX * mg / lim
            F[0], F[1] = F[0] * s, F[1] * s
        return F, M

    # 200 Hz 快环给出的 wrench 修正量。
    #
    # **高度与姿态的快环必须留在 200 Hz。** 实测（仿真，walk 0.2 m/s）：
    # 让 MPC 自己在 50 Hz 上闭这两路，无论权重怎么给（w_rpy 从 100 扫到 1、
    # w_height 从 2e2 扫到 2e3）roll 峰峰都停在 43~47°，Mx 每 80 ms 翻一次
    # 符号（±40 N·m），高度误差 +15 mm 时 Fz 反而给到 217 N（mg = 149）。
    # 原因是延迟：50 Hz 计划 + 5 ms 链路 = 20~50 ms，这两个环的带宽被延迟
    # 卡死在 2 Hz 以下，再往上调就是极限环。
    #
    # 同样两条反馈律放到 200 Hz（就是 `_wrench_cmd` 那套）roll 是 3.1°。
    # 所以分工改成：**200 Hz 负责「现在偏了，纠回来」**，MPC 负责
    # **「接下来 240 ms 里要抬哪条腿，力该怎么挪」** —— 后者是预测，前者是
    # 反馈，本来就不该由同一个环兼任。
    def _mpc_trim(self, vx, vy, yaw, a):
        wr = self._wrench_cmd(vx, vy, yaw, a)
        if wr is None:
            return np.zeros(3), np.zeros(3)
        F, M = wr
        mg = self.BODY_MASS * self.GRAVITY
        # `_wrench_cmd` 的 Fz 里含承重 mg，MPC 那份也含 —— 只取超出承重的
        # 那一部分作为修正量，否则体重算两遍。
        return (np.array([F[0], F[1], F[2] - mg], dtype=float),
                np.asarray(M, dtype=float))

    # 200 Hz 回路的 mpc 分支：拿最新计划 + 这一帧的足端目标，解 WBC 下发力矩。
    #
    # **无论出什么错都要发一帧合法的力矩**（退回重力前馈）。这条回路是
    # RecurrentThread，抛出去就等于整条线程死掉，狗当场瘫。
    def _mpc_feet_publish(self, g, q_out, feet_tgt, feet_vel, a, vx, vy, yaw):
        tau = None
        try:
            if self._mpc_th is None or not self._mpc_th.is_alive():
                self._mpc_start()
            with self._mpc_lock:
                plan, err = self._mpc_plan, self._mpc_err
            age = 1e9 if plan is None else time.perf_counter() - plan["t"]
            if a < 0.05 or age > self.MPC_STALE:
                # 起停段（a 还没铺开）或计划过期：退回重力前馈。
                self._mpc_miss += 1
                self._publish(q_out, tau=self._tau_ff(q_out))
                return
            from .wbc import WBCTarget

            q_meas = self.q if self.q is not None else q_out
            dq_meas = self._dq_lp()
            # 合成顺序：MPC 的前馈 + 200 Hz 的修正，**最后一起限幅**。先限再加
            # 会把修正量整个丢掉（MPC 那份常年贴着上限，加完再限就是原地不动）。
            dF, dM = self._mpc_trim(vx, vy, yaw, a)
            F, M = self._clamp_wrench(np.asarray(plan["F"], float) + dF,
                                      np.asarray(plan["M"], float) + dM)
            tgt = WBCTarget(feet=feet_tgt, feet_vel=feet_vel,
                            weights=dict(self._stance_w),
                            F=F, M=M, force=plan["force"])
            res = self.wbc.solve(q_meas, dq_meas, tgt)
            # 和 VMC 那一路同样过一道时序滤波。QP 的解在摩擦锥的顶点之间会
            # 逐帧跳（同一个 wrench 有多个等价顶点解），实测帧间差 p95 36 N·m、
            # 峰值 90 —— 减速器感受到的就是每 5 ms 一次的力矩台阶，机身被抖得
            # 乱滚。WBC 里的 `w_slew` 在内层压同一件事，这里是外层兜底。
            tau = self._slew_tau(res.tau)
        except Exception as e:                          # noqa: BLE001
            with self._mpc_lock:
                self._mpc_err = f"{type(e).__name__}: {e}"
            self._mpc_miss += 1
            self._publish(q_out, tau=self._tau_ff(q_out))
            return

        # 支撑腿降 kp：理由和 VMC 那一路完全一样（位置伺服会把脚钉死在轨迹上，
        # 力矩加进去只是「钉得更紧」）。WBC 已经把逆动力学算全了，这里降得
        # 更狠也不怕 —— kp 到 40 还能站住就是 Step 3 的验收项。
        kp = self.kp.copy()
        for i, leg in enumerate(CTRL_LEGS):
            w = float(self._stance_w.get(leg, 0.0))
            if w > 1e-3:
                kp[3 * i:3 * i + 3] *= (1.0 - w * (1.0 - self.wrench_kp_stance)
                                        * self.gait.wrench)
        self._publish(q_out, kp=kp, tau=tau)

    # 自研力路的诊断读数：MPC/WBC 各自的耗时与失败帧数。
    def mpc_stats(self) -> dict:
        out = dict(legs=len(CTRL_LEGS), miss=self._mpc_miss, err=self._mpc_err,
                   n=self._mpc_n,
                   alive=bool(self._mpc_th is not None
                              and self._mpc_th.is_alive()))
        if self.mpc is not None:
            out["mpc"] = self.mpc.stats()
        if self.wbc is not None:
            out["wbc"] = self.wbc.stats()
        with self._mpc_lock:
            p = self._mpc_plan
        if p is not None:
            out["age_ms"] = 1e3 * (time.perf_counter() - p["t"])
            out["n_contact"] = p["n_contact"]
            out["F"] = [round(float(x), 1) for x in p["F"]]
            out["M"] = [round(float(x), 2) for x in p["M"]]
        return out


    #
    # `a` 是步态幅度（0~1）。起停时它把反馈项一起收掉，避免站着的时候
    # 姿态反馈还在推脚 —— 那时候脚在地上，推了就是滑。
    #
    # ## F 的三个分量
    #
    # - z：体重 + 高度闭环。这一项是「承重」的唯一来源，原来靠 kp
    #   静差凑，现在直接给。
    # - x / y：速度误差乘一个刚度当推力。这就是「往前走」的力，原来靠
    #   足端后扫、靠摩擦传递；有了这一项，脚打滑也不会立刻失去推力。
    #
    # ## M 的三个分量
    #
    # 角度项给弹簧、角速度项给阻尼，两者都是负反馈：
    # 低头（pitch>0）要抬头 → 后腿多出力 → M_y > 0 → 取 −Kp·pitch。
    # 往左滚（roll>0）要右滚 → 左腿多出力 → M_x > 0 → 取 −Kp·roll。
    # （符号是按 `M = Σ p × f` 推出来的，不是试出来的：前腿 p_x>0，
    # M_y = −p_x·f_z，前腿出力大则 M_y 为负。）
    #
    # 偏航：足端轨迹只能「按指令转」，管不了「不想转却被地面拽着转」。实测
    # 走直线 5 秒后偏航从 0 漂到 −8°，之后每秒 30° 加速，20 秒时整条狗
    # 横过来、侧向摩擦把滚转也拽翻。所以另给一路真正的偏航力矩。
    def _wrench_cmd(self, vx: float, vy: float, yaw_cmd: float, a: float):
        gp = self.gait
        if not gp.wrench:
            return None
        mg = self.BODY_MASS * self.GRAVITY
        weight = max(0.0, min(1.0, float(gp.wrench))) * a
        if weight <= 1e-3:
            return None

        Fx = Fy = 0.0
        Fz = mg
        Mx = My = Mz = 0.0

        # ---- 高度闭环 ----
        h = self.body_height()
        if h is not None:
            # 目标高度用**起步时的实测站姿**，不是模型算的 height。真机的腿
            # 比模型长（实测 0.303 vs 设定 0.270），拿模型值当目标，环会一直
            # 认为「还没到」并持续往下压/往上顶，和地面较劲，水平推力全被
            # 这个静态冲突吃掉。
            z_des = (self._h_des if self._h_des is not None
                     else FOOT_RADIUS - gp.neutral[CTRL_LEGS[0]][2])
            vz = self._body_vz()
            dz = _clamp(z_des - h, -0.04, 0.04)     # 差 4 cm 以上先不管，慢慢来
            Fz += self.wrench_kz * dz - self.wrench_dz * vz

        # ---- 姿态闭环 ----
        if self.rpy:
            k = 1.0 / (1.0 + self.CTRL_HZ * self.STAB_TAU)
            self._rol_filt += k * (float(self.rpy[0]) - self._rol_filt)
            self._pit_filt += k * (float(self.rpy[1]) - self._pit_filt)
            gx = float(self.gyro[0]) if self.gyro else 0.0
            gy = float(self.gyro[1]) if self.gyro else 0.0
            Mx = -self.WRENCH_KP * self._rol_filt - self.WRENCH_KD * gx
            My = -self.WRENCH_KP * self._pit_filt - self.WRENCH_KD * gy
            # 偏航：参考角按指令积分。a<0.1 时（起停、幅度还没铺开）直接把
            # 参考角钉到当前角 —— 这时候狗基本在原地，谈不上"保持航向"。
            yaw_now = float(self.rpy[2])
            if self._yaw_ref is None or a < 0.1:
                self._yaw_ref = yaw_now
            self._yaw_ref += yaw_cmd / self.CTRL_HZ
            e = math.atan2(math.sin(yaw_now - self._yaw_ref),
                           math.cos(yaw_now - self._yaw_ref))
            wz = float(self.gyro[2]) if self.gyro else 0.0
            Mz = -self.WRENCH_KP_YAW * e - self.WRENCH_KD_YAW * wz

        # 力矩要按腿能给出来的量限住。力臂只有 9 cm，147 N 全压到一侧也
        # 只有 13 N·m；姿态环在 10° 误差下会要 28 N·m，分配器只能靠「一条
        # 腿拉到 0、另一条腿顶满」去凑，凑出来的合力方向完全失真。
        if self.WRENCH_M_MAX > 0:
            Mx = _clamp(Mx, -self.WRENCH_M_MAX, self.WRENCH_M_MAX)
            My = _clamp(My, -self.WRENCH_M_MAX, self.WRENCH_M_MAX)
            Mz = _clamp(Mz, -2.0 * self.WRENCH_M_MAX, 2.0 * self.WRENCH_M_MAX)

        # ---- 速度推力 ----
        if self._v_ok:
            Fx = self.WRENCH_KV * (vx - self._v_filt)
            Fy = self.WRENCH_KV * (vy - self._vy_filt)
            lim = 0.35 * mg          # 推重比封顶，不让速度环把狗推着打滑
            s = math.hypot(Fx, Fy)
            if s > lim:
                Fx, Fy = Fx * lim / s, Fy * lim / s

        # 起停时 a 把反馈项一起收掉，只剩纯承重（站姿不该被姿态环推着走）
        return ((Fx * weight, Fy * weight, mg + (Fz - mg) * weight),
                (Mx * weight, My * weight, Mz * weight))

    # 控制回路的节奏，单位毫秒：均值 / p95 / 最大 / 超过 1.5 倍周期的次数。
    #
    # 标称周期是 1000/CTRL_HZ（200 Hz 就是 5 ms）。真机「腿在颤、电机啸叫」
    # 先看这里：均值明显大于标称说明回路跑不动，max 一大截说明有 GC 或
    # 调度暂停，这两种都是参数调不好的，得先修时序。
    def loop_stats(self) -> dict:
        if not self._tick_dts:
            return dict(n=0, mean=0.0, p95=0.0, max=0.0, over=0,
                        nominal=1000.0 / self.CTRL_HZ)
        d = sorted(self._tick_dts)
        n = len(d)
        lim = 1.5 / self.CTRL_HZ
        return dict(n=n,
                    mean=1000.0 * sum(d) / n,
                    p95=1000.0 * d[min(n - 1, int(n * 0.95))],
                    max=1000.0 * d[-1],
                    over=sum(1 for x in d if x > lim),
                    nominal=1000.0 / self.CTRL_HZ)

    # 播放姿态序列的一段：插值 → 保持 → 下一段，全在回路里做，不占调用方。
    def _pose_tick(self):
        with self._seq_lock:
            p = self._pose
            t = time.time() - p["t0"]
            d = p["dur"]
            if t < d:
                u = t / d
                s = 0.5 * (1.0 - math.cos(math.pi * u))     # 两端角速度为 0
                self._publish(p["from_"] + s * (p["to"] - p["from_"]))
                return

            self._publish(p["to"])
            self._hold_q = p["to"].copy()
            if t >= d + p["hold"] and not self._next_step():
                self._mode = "hold"
                self._on_seq_done()

    def _next_step(self) -> bool:
        if self._step_i >= len(self._steps):
            return False
        to, dur, hold = self._steps[self._step_i]
        self._step_i += 1
        self._pose = dict(from_=self._hold_q.copy(),
                          to=np.asarray(to, dtype=float),
                          t0=time.time(), dur=max(0.0, float(dur)),
                          hold=max(0.0, float(hold)))
        return True

    def _on_seq_done(self):
        if self._seq_z0 is None:
            return
        print(f"\n  {self._seq_label}完成：峰值 z={self.peak_z:.3f}，"
              f"离地 {self.peak_z - self._seq_z0:+.3f} m", flush=True)
        self._seq_z0 = None

    def _publish(self, q_target, kp=None, kd=None, tau=None):
        if self._pub is None or q_target is None:
            return
        kp = self.kp if kp is None else kp
        kd = self.kd if kd is None else kd
        for i in range(NUM_MOTOR):
            cmd = self._cmd.motor_cmd[i]
            cmd.q = float(q_target[i])
            cmd.dq = 0.0
            cmd.kp = float(kp[i])
            cmd.kd = float(kd[i])
            # 力矩前馈：真机靠它顶住体重，仿真里同样有好处（PD 不用靠静差出力）。
            cmd.tau = 0.0 if tau is None else float(tau[i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)
        self._last_pub = np.array(q_target, dtype=float)

    # 把腿变成阻尼器。直接停发会让 ctrl 停在最后一帧，腿是僵的。
    def damping(self, kd: float = 4.0, repeats: int = 10):
        if self._pub is None:
            return
        self._mode = "idle"
        for i in range(NUM_MOTOR):
            self._cmd.motor_cmd[i].kp = 0.0
            self._cmd.motor_cmd[i].kd = kd
            self._cmd.motor_cmd[i].tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        for _ in range(repeats):
            self._pub.Write(self._cmd)
            time.sleep(0.01)

    # 停回路并阻尼。可重复调用；之后还能再调动作，回路会自己重启。
    #
    # park=True 时先让子对象（高层）执行 StandDown 蹲到底，再从低处松腿。
    # 仿真桥把 rt/lowcmd 直接算成力矩，不发就没有力矩，从 27 cm 站立高度
    # 直接松腿会把腿甩到限位。
    def shutdown(self, park: bool = False):
        if park and self.connected and self._mode != "idle":
            try:
                self.high_level.StandDown()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.Wait(1.0)
            self._thread = None
        self._mpc_stop_thread()
        self.damping()
        # 阻尼发完、狗已经趴稳，再把运控交还官方。顺序不能反：先交还的话
        # mcf 接手时我们还在发指令，又是一轮双源冲突。
        self._restore_motion_service()

    # -- 姿态 ----------------------------------------------------------

    # 保持一个关节角目标不动（默认保持当前实测姿态）。
    def hold(self, q: np.ndarray | None = None):
        self.require_connection()
        self._ensure_loop()
        self._hold_q = (self.q.copy() if q is None
                        else np.asarray(q, dtype=float).copy())
        self._mode = "hold"
        self._publish(self._hold_q)

    # 排一段姿态序列，排完立即返回，由 200 Hz 回路一段段播。
    #
    # steps 是 [(关节目标, 插值时长, 保持时长), ...]。这是「动作连贯」的关键：
    # 动作不再用 sleep 等自己演完，主循环照常跑（按键、遥测都不卡）。新序列
    # 直接顶掉正在播的旧序列，并且从当前实测关节角接着走，所以随时能打断、
    # 接上另一个动作，中间不会跳。
    #
    # z0 给了就在序列播完时打印峰值高度，跳跃靠它量离地多少。
    def pose_sequence(self, steps, label: str = "",
                      z0: float | None = None) -> None:
        self.require_connection()
        self._ensure_loop()
        with self._seq_lock:
            self._hold_q = self.q.copy()    # 起点用实测姿态，不用上一段的目标
            self._steps = list(steps)
            self._step_i = 0
            self._seq_label = label
            self._seq_z0 = z0
            if z0 is not None:
                self.peak_z = 0.0
            self._mode = "pose"
            self._next_step()

    # 腿正被一段姿态序列占用（跳跃、蹲伏这类）。此时别往下发速度指令。
    @property
    def busy(self) -> bool:
        return self._mode == "pose"

    # 等当前序列播完。阻塞版动作用它，键盘那条路不用。
    def wait_idle(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while self._mode == "pose" and time.time() - t0 < timeout:
            time.sleep(0.01)
        return self._mode != "pose"

    # 把 12 个关节角平滑移过去，移到位才返回（阻塞版）。
    #
    # duration 不给就按最快关节的行程自动算（默认限 2.5 rad/s）。
    # 要不阻塞的，用 apply_pose_async()。
    def apply_pose(self, q_target, duration: float | None = None,
                   hold_s: float = 0.3, qdot_max: float = 2.5) -> float:
        duration = self.apply_pose_async(q_target, duration=duration,
                                         hold_s=hold_s, qdot_max=qdot_max)
        self.wait_idle(timeout=duration + hold_s + 5.0)
        return duration

    # 排一段插值就返回。返回这段的时长。
    def apply_pose_async(self, q_target, duration: float | None = None,
                         hold_s: float = 0.0, qdot_max: float = 2.5) -> float:
        to = np.asarray(q_target, dtype=float)
        if duration is None:
            duration = max(0.45, float(np.abs(to - self.q).max()) / qdot_max)
        self.pose_sequence([(to, duration, hold_s)])
        return duration

    # -- 监控打印 ------------------------------------------------------

    def _state_line(self, t: float, p0=None) -> str:
        line = f"{t:5.1f}s"
        h = self.body_height()
        if h is not None:
            line += f"  z={h:.3f}"
        if p0 is not None and self.pos is not None:
            d = self.pos - p0
            yaw = self.rpy[2] if self.rpy else 0.0
            line += f"  前向 {d[0] * math.cos(yaw) + d[1] * math.sin(yaw):+.3f}m"
        if self.rpy:
            line += (f"  rpy=[{self.rpy[0]:+.2f},{self.rpy[1]:+.2f},"
                     f"{self.rpy[2]:+.2f}]")
        return line

    # 保持当前模式，按秒打印，到点返回。
    def _watch(self, duration: float, period: float = 1.0) -> None:
        p0 = self.pos.copy() if self.pos is not None else None
        next_print = 0.0
        while True:
            time.sleep(0.05)
            t = time.time() - self._t0
            h = self.body_height()
            if h is not None:
                self.peak_z = max(self.peak_z, h)
            if t >= next_print:
                next_print = t + period
                print(self._state_line(t, p0), flush=True)
            if t >= duration:
                break

    # ------------------------------------------------------------------
    # 站立与步态
    # ------------------------------------------------------------------

    # 从当前（通常瘫倒的）姿态插到站立姿态并保持，不阻尼。
    def stand(self, duration: float = 30.0, ramp: float = 0.8) -> bool:
        self.require_connection()
        q_stand = self.stand_joint_target()
        print(f"起立：首帧 q={np.round(self.q, 3).tolist()}")
        print(f"      目标 q={np.round(q_stand, 3).tolist()}")
        print(f"      机身高度 {self.height:.3f} m，kp={self.kp[0]:.0f} "
              f"kd={self.kd[0]:.1f}，插值 {ramp:.1f}s\n", flush=True)

        self.peak_z = 0.0
        self._t0 = time.time()
        self.apply_pose(q_stand, duration=ramp, hold_s=0.0)
        try:
            self._watch(duration)
        except KeyboardInterrupt:
            print("\n中断")

        up = self.peak_z >= 0.20
        print(f"\n峰值机身高度 {self.peak_z:.3f} m")
        if not up:
            print("没站起来。加大 kp，或确认仿真器真的收到了 rt/lowcmd。")
        return up

    # 按固定时长开环小跑。
    #
    # 开环意味着打滑 30~45%，实际位移约是「指令速度 × 时间」的 55~70%。
    # 要「走两米」这种准确结果，用 high_level.forward()，它拿里程计闭环。
    def trot(self, vx: float = 0.3, vy: float = 0.0, yaw: float = 0.0,
             duration: float = 8.0, ramp: float = 0.8,
             gait_ramp: float = 0.6) -> dict:
        return self._run_gait(vx=vx, vy=vy, yaw=yaw, ramp=ramp,
                              gait_ramp=gait_ramp, timeout=ramp + duration)

    # 跑到 done() 返回 True 就收步，最多 timeout 秒。
    def trot_until(self, done, timeout: float, vx: float = 0.3,
                   vy: float = 0.0, yaw: float = 0.0, ramp: float = 0.8,
                   gait_ramp: float = 0.6) -> dict:
        return self._run_gait(vx=vx, vy=vy, yaw=yaw, ramp=ramp,
                              gait_ramp=gait_ramp, done=done, timeout=timeout)

    # -- 运动步态风格 --------------------------------------------------

    # 两档，名字跟官方的两个运动模式对得上，调参才有参照：
    #   mech  标准小跑 → 官方灵动（FreeWalk）
    #   bio   并腿跑   → 官方并腿跑（FreeBound）
    GAIT_STYLES = ("mech", "bio")
    GAIT_STYLE_NAMES = {
        "mech": "机械（标准小跑，同官方灵动）",
        "bio": "仿生（并腿跑）",
    }

    # 换运动步态（`mech` 机械 / `bio` 仿生），返回换完之后的那个。
    #
    # 跑着也能换：`foot_target()` 每帧都读 `self.gait`，下一帧就按新相位走。
    # 相位会跳一下（两条腿的相位对上/对不上变了），换的时候会顿一顿，正常。
    def set_gait_style(self, style: str) -> str:
        style = _GAIT_ALIASES.get(style, style)
        if style not in self.GAIT_STYLES:
            raise ValueError(f"没有这种运动步态：{style}，只有 {self.GAIT_STYLES}")
        # 力路跟着走。**不能靠默认值**：`make_gait` 的 mode 默认是 vmc，切一次
        # 档就把 mpc 静默换成 vmc，而 MPC 线程还在跑、白烧 CPU，力矩却是 vmc
        # 算的，从任何指标上都看不出发生了什么。
        self.gait = make_gait(style, self.height, self.real, self.gait.mode)
        self.gait_style = style
        # 反馈滤波状态跟着步态一起清：上一套步态跑出来的俯仰/速度残值，喂给
        # 新步态的第一个周期就是一脚踩偏，切完要顿一下甚至打滑。
        self._pit_filt = self._rol_filt = self._v_filt = 0.0
        return style

    def toggle_gait_style(self) -> str:
        i = self.GAIT_STYLES.index(self.gait_style)
        return self.set_gait_style(self.GAIT_STYLES[(i + 1) % len(self.GAIT_STYLES)])

    @property
    def gait_style_label(self) -> str:
        return self.GAIT_STYLE_NAMES[self.gait_style]

    # 步态主循环：起立 → 跑 → 幅度收到 0 → 站定在 hold。
    #
    # 收步不是「停发指令」，而是把轨迹幅度 a 从 1 渐收 0：足端回到中立位，
    # 机身高度自然回到站立值。直接停发会掉在地上。
    def _run_gait(self, vx=0.0, vy=0.0, yaw=0.0, ramp=0.8, gait_ramp=0.6,
                  stop_ramp=0.45, hold_s=0.35, done=None, timeout=None,
                  report=1.0) -> dict:
        self.require_connection()
        gp = self.gait
        print(f"步态 周期 {gp.period:.2f}s 支撑 {gp.stance_time:.2f}s "
              f"抬脚 {gp.swing_height * 1000:.0f}mm   指令 "
              f"vx={vx:.2f} vy={vy:.2f} yaw={yaw:.2f}")
        print(f"kp={self.kp[0]:.0f} kd={self.kd[0]:.1f}\n", flush=True)

        q_stand = self.stand_joint_target()
        g = dict(vx=vx, vy=vy, yaw=yaw, ramp=ramp, gait_ramp=gait_ramp,
                 stop_ramp=stop_ramp, hold_s=hold_s, t0=time.time(),
                 q_init=self.q.copy(), q_stand=q_stand,
                 p_nom=self.stand_feet(self.height, self.gait.stance_spread),
                 last=q_stand.copy(), stop_at=None, why="", n=0, fail=0,
                 stream=False)
        self._gait = g
        self.peak_z = 0.0
        self._mpc_yaw = None
        self._mpc_flush_plan()
        self._t0 = g["t0"]
        self._ensure_loop()
        self._mode = "gait"

        p0 = self.pos.copy() if self.pos is not None else None
        next_print = 0.0
        try:
            while True:
                time.sleep(0.05)
                t = time.time() - g["t0"]
                h = self.body_height()
                if h is not None:
                    self.peak_z = max(self.peak_z, h)

                if t >= next_print:
                    next_print = t + report
                    print(self._state_line(t, p0), flush=True)

                if g["stop_at"] is None:
                    if done is not None and done():
                        g["stop_at"], g["why"] = t, "任务完成"
                    elif timeout is not None and t >= timeout:
                        g["stop_at"], g["why"] = t, "到时"
                elif t >= g["stop_at"] + stop_ramp + hold_s:
                    break
        except KeyboardInterrupt:
            print("\n中断")
            g["why"] = "中断"

        # 收步结束，回路已经是 hold 状态（_gait_tick 里切的）
        if self._mode == "gait":
            self.hold(q_stand)

        moved = 0.0
        if p0 is not None and self.pos is not None:
            moved = float(np.linalg.norm((self.pos - p0)[:2]))
        print(f"\n步态结束（{g['why'] or '到时'}）  水平位移 {moved:.3f} m  "
              f"峰值高度 {self.peak_z:.3f} m")
        if g["n"]:
            print(f"IK 调用 {g['n']} 次，失败 {g['fail']} 次")
        return dict(moved=moved, peak_z=self.peak_z, ik_calls=g["n"],
                    ik_fail=g["fail"], reason=g["why"], vx=vx, vy=vy, yaw=yaw)

    def _gait_tick(self):
        g = self._gait
        # 相位时钟用**限幅累加**，不用绝对时间差。
        #
        # `tg` 直接进 `foot_target` 的相位（`ph = (tg/period + offset) % 1`），
        # 所以它必须是连续、单调、步长有界的。原先 `t = time.time() - t0`：
        # 只要某一帧卡一下（GC、窗口重绘、调度），tg 就跳一大步，相位跟着跳
        # 0.1~0.2 个周期，足端目标瞬移。运气不好会正好跨过支撑/摆动边界，
        # 于是一条腿从前半个行程直接跳到后半个行程 —— 支撑相从「往后扫」
        # 变成「往前扫」，地面反力方向整个反过来，机身被踹得倒退。
        # 实测崩溃瞬间 vx 到 −0.51 m/s、机身抬 3 cm、pitch 冲到 −22°。
        now = time.time()
        w = g.get("t_wall")
        if w is None:
            g["t_wall"] = now
            g["t_acc"] = 0.0
            dtw = 0.0
        else:
            dtw = now - w
            g["t_wall"] = now
        if dtw < 0.0:
            dtw = 0.0
        # 200 Hz 正常步长 5 ms；上限取 20 ms，超出部分直接丢掉（宁可步态
        # 慢一点，也不能让它跳相）。
        g["t_acc"] = g.get("t_acc", 0.0) + min(dtw, 0.020)
        t = g["t_acc"]

        # 第一段：从瘫倒姿态插值到站立
        if t < g["ramp"]:
            a = t / g["ramp"] if g["ramp"] > 0 else 1.0
            self._publish(g["q_init"] + a * (g["q_stand"] - g["q_init"]))
            return

        tg = t - g["ramp"]
        if g.get("stream"):
            self._vel_tick(g, tg)
            return

        a_in = min(1.0, tg / g["gait_ramp"]) if g["gait_ramp"] > 0 else 1.0

        if g["stop_at"] is None:
            a = a_in
        else:
            u = (t - g["stop_at"]) / g["stop_ramp"]
            if u >= 1.0:
                # 幅度收到 0：四脚都回到中立位，转成纯 hold
                self._hold_q = g["q_stand"].copy()
                self._mode = "hold"
                self._publish(self._hold_q)
                return
            a = a_in * (1.0 - u)

        self._gait_feet_publish(g, tg, a, g["vx"], g["vy"], g["yaw"])

    # 速度伺服：目标速度来自 set_velocity()，加速度在这里限幅。
    #
    # 原生 SportClient.Move 是「发一次就保持」的语义，所以这里没有终点：
    # 速度归零后幅度 a 收到 0，才切回 hold。
    def _vel_tick(self, g, tg):
        dt = (tg - g["last_t"]) if g["last_t"] else 1.0 / self.CTRL_HZ
        dt = min(max(dt, 1e-3), 0.05)
        g["last_t"] = tg

        for i, lim in enumerate((self.ACCEL_LIN, self.ACCEL_LIN, self.ACCEL_ANG)):
            cur, tgt = g["vel"][i], self._vel_target[i]
            step = lim * dt
            nxt = cur + max(-step, min(step, tgt - cur))
            g["accel"][i] = (nxt - cur) / dt          # 实际走的加速度，含限幅
            g["vel"][i] = nxt
        self._last_vel = list(g["vel"])

        moving = any(abs(v) > 1e-3 for v in g["vel"])
        if moving:
            g["started"] = True          # 记一下真的动过，否则一进来就退 hold
        goal = 1.0 if moving else 0.0
        step_a = dt / max(1e-3, g["gait_ramp"])
        g["a"] += max(-step_a, min(step_a, goal - g["a"]))

        if g["started"] and g["a"] <= 0.0 and not moving:
            # 脚都回到中立位了，交回 hold
            self._hold_q = g["q_stand"].copy()
            self._mode = "hold"
            self._publish(self._hold_q)
            return
        if g["a"] <= 0.0:
            # 速度还没起作用，先维持站姿，避免零幅度步态抖动
            if self.gait.mode == "mpc":
                self._mpc_stand_publish(g["q_stand"])
            else:
                self._publish(g["q_stand"])
            return

        self._gait_feet_publish(g, tg, g["a"], *g["vel"])

    # 把「足端轨迹 × 幅度 a + 中立位」解成关节角并下发。
    #
    # a=0 时足端就在中立位，等价于站姿，所以起停都靠插值 a 完成，不切换模式。
    #
    # 跳跑（bound）还要在这里叠一个机身俯仰摆动：脚落点不动、机身跟着节奏点
    # 头/抬头，狗跑起来就是这个样子。俯仰按足端做旋转等价于机身转过去，与
    # `feet_for_body_pose()` 同一套约定（正=低头）。
    def _gait_feet_publish(self, g, tg, a: float, vx: float, vy: float,
                           yaw: float):
        gp = self.gait
        if a < 0.1:
            # 起停阶段机身基本没动，把 IMU 积分的漂移清掉，别带进下一段。
            self._v_imu[:] = 0.0
        # 指令先按这套步态真正跑得出来的速度封顶。键盘按住 W 时指令能到 0.9，
        # 而机械档满步幅、零打滑的理论上限也只有 0.89，实际交付 0.5 左右 ——
        # 不封顶的话速度闭环会一直认定「还没跑到 0.9」，把步幅常年顶在
        # stride_max 上，全程打滑，机身被刨得一颠一颠（实测 0.9 指令下俯仰
        # 峰峰 54°、滚转 34°，就是「极其不稳」的样子）。封顶之后指令 0.9 与
        # 0.55 等价，跑出来是同一份稳定步态，只是不再试图更快。
        if gp.speed_range:
            lo, hi = gp.speed_range
            spd = math.hypot(vx, vy)
            if spd > 0.02:
                tgt = max(lo, min(hi, spd))
                vx, vy = vx * tgt / spd, vy * tgt / spd
        # 横移封顶。**默认 0 = 关闭横移。** 现在这两档步态横着走都会侧翻：
        # 实测指令 0.12 m/s 也在 6 秒内滚转 360°，位置伺服和机身层都一样，
        # 换 duty、换周期、加滚转反馈都不救。根因是小跑的支撑多边形在横向很窄
        # （只有两条对角腿），横移时重心很快就出了支撑边界，而抬脚高度 9 cm
        # 不允许靠拖脚补救。真机上侧翻比走近更糟，所以先关掉，等横移的步态
        # 单独设计（要降低抬脚、加大 duty、并引入落点侧的横移落脚点反馈）
        # 之后再打开。
        vy = _clamp(vy, -self.MAX_VY, self.MAX_VY)
        cp = sp = None
        if gp.pitch_amp:
            ph = 2.0 * math.pi * (tg / gp.period + gp.pitch_phase)
            pitch = gp.pitch_amp * math.sin(ph) * a
            cp, sp = math.cos(pitch), math.sin(pitch)

        # ---- 姿态反馈：角度按平 + 角速度阻尼 ----
        # **只在机身层关闭时才走这一路。** 机身层（wrench=1）自己就有一套
        # 姿态闭环：`_wrench_cmd` 按 roll/pitch 误差算出合力矩 M，再经
        # `body_wrench_tau` 分配到四条腿的力矩。那一路是「按力矩纠姿态」，
        # 这一路是「按腿长纠姿态」，两条同时开着就是两个控制器抢同一个
        # 自由度。
        #
        # 实测（仿真，8 s，机身层开）：
        #   pitch_gain −0.05（原值）：4.8 s 起 pitch 单调恶化到 −26°，不再恢复
        #   pitch_gain +0.05        ：±3° 与 −30° 之间来回振荡，最后锁在 −28°
        #   pitch_gain  0（关掉）   ：4.8 s 同样被扰动，但 6.9 s 自己回到 −2°
        #
        # 三条的差别只在「扰动之后能不能恢复」。也就是说这一路在机身层之上
        # 是净正反馈：它和力矩环的相位对不上，越修越偏。关掉之后步态本身的
        # 姿态由重力前馈 + 合力矩分配维持，反而干净。
        #
        # 老路子（wrench=0，位置伺服 + 重力前馈）没有力矩环，这一路是唯一的
        # 姿态修正手段，必须保留。
        if (gp.pitch_gain or gp.roll_gain) and self.rpy:
            k = 1.0 / (1.0 + self.CTRL_HZ * self.STAB_TAU)
            self._pit_filt += k * (float(self.rpy[1]) - self._pit_filt)
            self._rol_filt += k * (float(self.rpy[0]) - self._rol_filt)
        dz_stab = 0.0
        if (gp.pitch_gain or gp.pitch_rate) and gp.wrench < 1.0:
            # 角度项：低头（rpy[1]>0）要让前腿缩短、后腿伸长。
            # 注意这里的符号跟直觉相反：前脚踩在机身前面，它承重产生的力矩
            # 是「把机身压低头」。所以想抬头不能去伸长前腿，那只会让前腿吃更多
            # 力、机身更低 —— 之前就是符号写反了，低头时前腿越伸越长，正反馈
            # 把机身一路顶到抬头 40°，前腿彻底翘在空中，四条腿都给不出向前的
            # 摩擦。正确方向：低头 → 前脚抬高（缩短）、后脚下压（伸长）。
            # 角速度项：点头越快反向修正越大，把摆动收住（光有角度项是弹簧）。
            gx = float(self.gyro[1]) if self.gyro else 0.0
            dz_stab = _clamp((gp.pitch_gain * self._pit_filt
                              + gp.pitch_rate * gx) * a,
                             -self.STAB_MAX, self.STAB_MAX)
        dz_stab = self._slew_stab(dz_stab, "z")
        dr_stab = 0.0
        if (gp.roll_gain or gp.roll_rate) and gp.wrench < 1.0:
            # 往左滚（rpy[0]>0）就压左脚、抬右脚，把机身按平。
            gr = float(self.gyro[0]) if self.gyro else 0.0
            dr_stab = _clamp((gp.roll_gain * self._rol_filt
                              + gp.roll_rate * gr) * a,
                             -self.STAB_MAX, self.STAB_MAX)
        dr_stab = self._slew_stab(dr_stab, "r")

        # ---- 机身竖直速度阻尼 ----
        # 四条腿撑在地上就是一个弹簧，固有频率 2.9 Hz，恰好贴着步频 2.2 Hz。
        # 每步落地都在激它，而腿上只有 kd 1.5 那点阻尼，机身就一直弹。
        # 这里按机身竖直速度反向挪支撑脚：机身往上冲就把脚往回收（缩短腿），
        # 把上升的那一下吃掉。机身往下掉就反向送脚，接住它。
        #
        # 竖直速度用机身高度差出来，不用 IMU 的 vz：真机上 sportmodestate 的
        # 速度就没有可靠的 z 分量，而高度我们本来每帧都在算。
        dz_vdamp = 0.0
        if gp.damp_z and gp.wrench < 1.0:
            # 竖直速度走 `_body_vz` 唯一入口：它内部做了同帧去重，和机身层
            # 共用同一个差分状态也不会互相踩。
            dz_vdamp = _clamp(-gp.damp_z * self._body_vz() * a
                              , -0.02, 0.02)

        # ---- 支撑相足端速度闭环（摩擦方向不反号的关键）----
        # 脚在支撑相该以多快后扫，不能只看指令：要看实测机身速度，再加一个
        # 正比于速度误差的项。机身慢了误差变大、脚扫得更狠把机身推回指令速度；
        # 机身够快误差归零、脚几乎不打滑。开环算的话，机身一被扰动，脚在地面
        # 的滑动方向就反过来，摩擦立刻变成刹车，狗会自己倒退着走。
        evx, evy = vx, vy
        bv = self.body_velocity()
        self._v_ok = bv is not None
        if bv is not None:
            # 实测前进速度先低通：运动服务给的速度带毛刺，直接进轨迹会变成抖
            v_m = _clamp(float(bv[0]), -1.0, 1.0)
            kb = 1.0 / (1.0 + self.CTRL_HZ * self.VEL_TAU)
            self._v_filt += kb * (v_m - self._v_filt)
            self._vy_filt += kb * (_clamp(float(bv[1]), -1.0, 1.0)
                                   - self._vy_filt)
        if gp.speed_loop and bv is not None:
            v_m = self._v_filt
            evx = v_m + gp.speed_loop * (vx - v_m)
            # 同样闭环横移。横滑是侧翻的前奏，不修的话原地跑都会往一边偏。
            vm_y = self._vy_filt
            evy = vm_y + gp.speed_loop * (vy - vm_y)
            # 限幅：速度误差大（起步、被撞）时这一项会算得很大，脚猛扫一下
            # 就刨穿地面。放到 1.6 倍指令之内，加速慢一点，姿态稳得多。
            lim_v = max(0.10, 1.15 * math.hypot(vx, vy))
            s = math.hypot(evx, evy)
            if s > lim_v:
                evx, evy = evx * lim_v / s, evy * lim_v / s

        # 平滑在闭环判决之外做：speed_loop 关掉时 evx 就是指令，指令本身也
        # 可能是阶跃（键盘一按到底），照样要走这条路，否则切档那一下会跳。
        evx, evy = self._smooth_ev(evx, evy)

        # ---- 落点前移 + 速度误差反馈（Raibert 落脚点）----
        # 只在跑起来之后生效；原地站着时 vx=0、误差也不该动脚，全靠 a 收尾。
        x_shift = 0.0
        if gp.reach or gp.reach_err:
            # reach 项跟指令方向走（后退就是把脚往前踩）。
            # reach_err 是「跑快了就多踩一点制动」，它是按前进设计的：后退时
            # vx 为负，误差项算出来是往前踩，等于一直把狗往回顶 —— 实测后退
            # 直接走不动。所以只在往前走时才叠这一项。
            x_shift = gp.reach * vx
            if vx > 0.02:
                x_shift += gp.reach_err * (self._v_filt - vx)
            x_shift = _clamp(x_shift * a, -gp.stride_max, gp.stride_max)

        # 转弯侧倾：机身朝弯内歪过去，脚落点不动 —— 等于重心被甩到弯内侧，
        # 跟狗和猎豹跑弯一样，靠压弯而不是原地扭头。
        R_lean = None
        if gp.lean and abs(yaw) > 1e-3:
            R_lean = _rpy_to_R(gp.lean * yaw * a, 0.0, 0.0)
        # 自研力路要的两份东西：足端在机体系的目标位置与目标速度。
        # 位置就是下面 IK 的输入（连同姿态修正、压地、落脚点前移一起带上），
        # 速度取 `foot_target` 的导数乘幅度 —— 支撑相的腿本来就在以 −v 后扫，
        # 摆动相则沿着那条四次多项式走。只给位置不给速度的话，WBC 里那个
        # kd_foot 会把「轨迹本来就该有的速度」当成误差去纠，摆动腿起步那
        # 一下会明显拖一拍。
        feet_tgt: dict = {}
        feet_vel: dict = {}
        for i, leg in enumerate(CTRL_LEGS):
            h_body = float(self.body_height() or 0.0)
            p_traj = self.foot_target(leg, tg, evx, evy, yaw)
            # ---- 足端 z 的实测偏置 ----
            # 轨迹的高度是按模型算的（nz = FOOT_RADIUS − height），但真机站姿
            # 的实际足端比模型低得多：实测 −0.283 vs 模型 −0.2475，差 3.5 cm
            # ——承重之后腿部有静差，模型里没有这一项。这个差值不补，走路时
            # 支撑相的脚就一直悬在地面上方 3.5 cm：腿在空中划水，机身纹丝
            # 不动，而里程计也读不出速度，看起来就是「怎么调都走不了、还一直
            # 响」。仿真里没有静差，所以从来不会暴露。
            #
            # 偏置在开始跑的时候测一次（那时四脚都在站姿、最有代表性），
            # 之后整段轨迹平移。四条腿用同一个值：单腿的差异是几毫米，
            # 而这里要补的是几厘米的系统偏差。
            if self._z_off is None and self.q is not None:
                offs = []
                for k, lg in enumerate(CTRL_LEGS):
                    pk = self.fk_leg(lg, self.q[3 * k:3 * k + 3])
                    offs.append(float(pk[2]) - (FOOT_RADIUS - self.height))
                self._z_off = float(np.mean(offs))
            if self._z_off:
                p_traj = (p_traj[0], p_traj[1], p_traj[2] + self._z_off)
            p = tuple(n + a * (t2 - n) for n, t2 in zip(g["p_nom"][leg], p_traj))
            if x_shift:
                p = (p[0] + x_shift, p[1], p[2])
            # 支撑权重：支撑相中段 1，两端过渡，摆动相 0。
            # 里程计按它加权反推机身速度，重力前馈按它分配支撑力 ——
            # 给摆动腿加前馈等于在空中往下踹，落地更重。
            #
            # **过渡窗口要跨过落地/离地那一帧，不能全放在支撑相里面。**
            # 原来是 min(1, ph/0.2, (duty-ph)/0.2)：落地后才从 0 爬、离地前
            # 就掉回 0。duty=0.5 两条腿的支撑期首尾相接，于是一条腿已经掉到 0、
            # 另一条还没爬起来 —— 换腿那一瞬间**两条腿的权重同时是 0**，重力
            # 前馈整段消失，机身直接自由落体，下一秒又被接住。每个周期沉两下，
            # 实测机身高度峰峰 50~80 mm、俯仰峰峰 27°，就是「极其不稳」。
            # 现在的窗口是 [−pre, duty]：**落地前 pre 就开始给力**（预载，落地
            # 更实），**离地那一帧正好撤完**。撤力点不能晚于离地 —— 腿已经在
            # 空中了还要求它扛体重，等于在空中往下踹一脚，落地再来一记冲击，
            # 狗会被这一脚踹得弹起来（实测机身能冲到 0.33 m）。
            # pre=ramp=0.1 时两条腿的权重之和恒为 1，机身没有支撑真空期。
            pre = 0.1
            ph = (tg / gp.period + gp.offsets[leg]) % 1.0
            ps = (ph + pre) % 1.0
            win = gp.duty + pre
            if ps < win:
                w_st = min(1.0, ps / pre, (win - ps) / pre)
            else:
                w_st = 0.0
            # 相位的预载窗口会在腿还没落地时就给出 0.5 的权重。分配器据此
            # 向一条离地 4 cm 的腿要竖直力 —— 它只能在空中往下蹬，机体被
            # 反作用顶起来，落地再来一记冲击。实测这一步就是俯仰 23° 峰峰
            # 的主要来源。按实测足端**相对中立位抬高了多少**再掐一道。
            #
            # 判据必须是相对的，不能是「离地高度」：那要用 h_body，而真机上
            # h_body 取的是 sportmodestate 的 z，被 lowcmd 接管之后它不再反映
            # 真实高度，运动中能漂 20 mm。20 mm 在这个 20 mm 宽的闸门里就是
            # 0 到 1 来回翻 —— 支撑力断断续续，脚打滑、机身被顶得一震一震，
            # 而且怎么调增益都没用。p_now − p_nom 只跟关节角有关，编码器给多少
            # 就是多少。
            if w_st > 0.0 and self.q is not None:
                p_now = self.fk_leg(leg, self.q[3 * i:3 * i + 3])
                pn = g["p_nom"][leg]
                # **中立位必须跟着机身姿态转。** `p_now` 是机体系里的实测足端，
                # `p_nom` 是机身水平时的中立位。机身一抬头，后腿足端在机体系
                # 里就「升高」了 —— 并不是腿抬起来了，而是机身转了。拿没转的
                # p_nom 去减，后腿会被判成离地、权重被闸门削掉，分配器不再给
                # 它竖直力，机身后部塌下去，抬头更大 —— 正反馈。
                #
                # 实测这条正反馈的完整过程（仿真，指令 0.3）：t≈4.6 s 起步
                # 前栽，pitch 从 −3° 单调走到 −26°，速度归零，之后再也回不来。
                # 把这一路和 dz_stab 都关掉只能让它「崩了能自愈」，治不了根。
                # 用 Rᵀ 把中立位转到当前机体系之后，同一段测试不再出现前栽。
                if self.rpy:
                    R = _rpy_to_R(float(self.rpy[0]), float(self.rpy[1]), 0.0)
                    pn = _mat_t_vec(R, pn)
                lift = float(p_now[2]) - float(pn[2])
                w_st *= _clamp((self.STANCE_GATE_HI - lift)
                               / self.STANCE_GATE_W, 0.0, 1.0)
            self._stance_w[leg] = w_st
            if dz_vdamp and w_st:
                p = (p[0], p[1], p[2] + dz_vdamp * w_st)
            if dz_stab and w_st:
                # 前脚抬高（缩短）、后脚下压（伸长）：把低头的机身抬正。
                # 修正量只给支撑腿：摆动腿在空中，改它的高度只会改抬脚轨迹，
                # 落地那一帧砸得更重。两端 20% 过渡见上面的 w_st。
                dz = (dz_stab if leg in ("FL", "FR") else -dz_stab) * w_st
                p = (p[0], p[1], p[2] + dz)
            if dr_stab and w_st:
                # **符号和俯仰相反：这里是 +。** 轨迹里 p_z 向上为正，往左滚
                # （rpy[0]>0）要压左脚、抬右脚 —— 左脚 z 减小、右脚 z 增大，
                # 也就是 dr>0 时左腿取 −dr、右腿取 +dr。原来写成 (p[2] - dr)，
                # 左右对调了，滚转反馈变成正反馈：机身一歪就往歪的那边推，
                # 实测一加油门就滚翻（roll 360°）。改回 + 之后同一个增益下
                # 滚转峰峰从 38° 降到 16°、机身纹波从 92 降到 65 mm。
                dr = (dr_stab if leg in ("FL", "RL") else -dr_stab) * w_st
                p = (p[0], p[1], p[2] + dr)
            if R_lean is not None:
                p = _mat_t_vec(R_lean, p)
            if cp is not None:
                x, y, z = p
                p = (cp * x - sp * z, y, sp * x + cp * z)
            feet_tgt[leg] = np.array(p, dtype=float)
            fv = self.foot_velocity(leg, tg, evx, evy, yaw)
            feet_vel[leg] = np.array([a * fv[0], a * fv[1], 0.0])
            r = self.ik_leg(leg, p)
            g["n"] += 1
            if r.ok:
                g["last"][3 * i: 3 * i + 3] = r.q
            else:
                g["fail"] += 1
        q_out = g["last"]
        if gp.mode == "mpc":
            self._mpc_feet_publish(g, q_out, feet_tgt, feet_vel, a, vx, vy, yaw)
            return
        wr = self._wrench_cmd(vx, vy, yaw, a)
        if wr is None:
            self._publish(q_out, tau=self._tau_ff(q_out))
        else:
            F, M = wr
            # 雅可比用实测角：kp 110 下目标与实际差 0.05 rad，雅可比偏 3~5°，
            # 上百牛的竖直力就分出几牛的水平分量，四条腿方向相反互相拉扯。
            q_ref = self.q if self.q is not None else q_out
            tau = self.body_wrench_tau(q_ref, self._stance_w, F, M)
            tau = np.clip(tau, -self.WRENCH_TAU_MAX, self.WRENCH_TAU_MAX)
            tau = self._slew_tau(tau)
            # 支撑腿降 kp：位置伺服留在上面会把脚钉死在轨迹上，力矩加进去
            # 只是"钉得更紧"，真正的出力还是 kp 静差说了算。按支撑权重线性
            # 过渡，落地/离地那几帧不会突然变软。
            kp = self.kp.copy()
            for i, leg in enumerate(CTRL_LEGS):
                w = float(self._stance_w.get(leg, 0.0))
                if w > 1e-3:
                    kp[3 * i:3 * i + 3] *= (
                        1.0 - w * (1.0 - self.wrench_kp_stance) * gp.wrench)
            self._publish(q_out, kp=kp, tau=tau)

    # ------------------------------------------------------------------
    # 速度伺服（对应原生 Move / StopMove）
    # ------------------------------------------------------------------

    # 进入速度伺服模式：没有终点，一直跑，直到 stop_velocity()。
    #
    # trot() 是「跑固定时长」，这个是「听命令跑」，对应原生 SportClient。
    #
    # ramp 是「从当前姿态爬到站姿」的时间。已经站着的时候按最大关节偏差算，
    # 差值小就直接跳过 —— 否则每次动作完再起步都要白等 0.8 秒，看起来像卡住。
    #
    # vel0 是起步速度，不给就用上次速度伺服的实际值：跳跃把步态顶掉之后，
    # 落地那一帧要接着原来的速度跑，而不是从 0 重新加速。
    def start_velocity(self, ramp: float = 0.8, gait_ramp: float = 0.35,
                       vel0=None) -> None:
        self.require_connection()
        q_stand = self.stand_joint_target()
        if self.q is not None:
            err = float(np.abs(self.q - q_stand).max())
            ramp = min(ramp, max(0.02, err / 2.5))

        v0 = list(self._last_vel if vel0 is None else vel0)
        moving = any(abs(v) > 1e-3 for v in v0)
        self._pit_filt = self._rol_filt = self._v_filt = 0.0
        # 参考航向重新锁：上一段跑的残留值会让第一帧就要把机身掰回去。
        self._mpc_yaw = None
        self._mpc_flush_plan()
        self._gait = dict(stream=True, ramp=ramp, gait_ramp=gait_ramp,
                          stop_ramp=0.45, t0=time.time(), last_t=0.0,
                          q_init=self.q.copy(), q_stand=q_stand,
                          p_nom=self.stand_feet(self.height, self.gait.stance_spread),
                          last=q_stand.copy(), vel=v0,
                          # 幅度还是从 0 爬：带速度进来只跳过「速度从 0 加速」
                          # 那一段，步态本身仍要渐起，否则落地会窜一下。
                          a=0.0,
                          accel=[0.0, 0.0, 0.0],
                          started=moving, n=0, fail=0)
        self._vel_target = list(v0)
        # 力矩滤波重新起头：上一轮停机前的值留着会让第一步从旧力矩爬。
        self._tau_w = None
        # 足端 z 偏置每次起步重测：上次落地可能把站姿压低了。
        self._z_off = None
        # 高度环的目标也取这一刻的实测值，别去追模型算出来的 height。
        self._h_des = self.body_height() or self._h_des
        self._mode = "gait"
        # MPC / WBC **在进模式时就构造**，不在 200 Hz 回路里现场造。构造要编译
        # 两份 QP（12N 维的 MPC、24 维的 WBC），实测第一帧在回路里花掉 320 ms
        # —— 那一帧的力矩晚 0.3 秒发出去，狗已经趴下了。线程仍然是惰性启动的
        # （`_mpc_start`），那时构造已经完成，只是起一个 thread 对象。
        if self.gait.mode == "mpc":
            self._ensure_mpc()
        self._ensure_loop()

    # 只改目标速度，不改相位；实际加速度由 200 Hz 回路限住。
    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     yaw: float = 0.0) -> None:
        if not (self._gait and self._gait.get("stream")):
            raise RuntimeError("先 start_velocity()")
        self._vel_target = [float(vx), float(vy), float(yaw)]

    # 目标速度归零。收步是渐进的，本调用立即返回。
    def stop_velocity(self) -> None:
        self._vel_target = [0.0, 0.0, 0.0]

    def velocity_active(self) -> bool:
        g = self._gait
        return bool(g and g.get("stream") and self._mode == "gait")

    # 当前实际速度（已过加速度限幅）。不在速度伺服里就是 0。
    def velocity_command(self) -> tuple[float, float, float]:
        g = self._gait
        if not (g and g.get("stream")) or self._mode != "gait":
            return (0.0, 0.0, 0.0)
        return tuple(g["vel"])

    # 当前实际加速度 m/s²、rad/s²。跳跃继承速度时要用它算前飞距离。
    def velocity_accel(self) -> tuple[float, float, float]:
        g = self._gait
        if not (g and g.get("stream")) or self._mode != "gait":
            return (0.0, 0.0, 0.0)
        return tuple(g.get("accel", (0.0, 0.0, 0.0)))

    # ------------------------------------------------------------------
    # 遥测
    # ------------------------------------------------------------------

    # 只读模式：不发任何指令，确认 DDS 通不通、频率对不对。
    def telemetry(self, duration: float = 5.0) -> int:
        self.require_connection()
        f0, t0 = self.frames, time.time()
        time.sleep(duration)
        hz = (self.frames - f0) / max(1e-6, time.time() - t0)

        print(f"rt/lowstate  {hz:6.1f} Hz（真机 500，仿真 200）")
        if self.q is not None:
            print(f"  关节 q   {np.round(self.q, 3).tolist()}")
            print(f"  关节 dq  {np.round(self.dq, 3).tolist()}")
        if self.rpy:
            print(f"  姿态 rpy {[round(v, 3) for v in self.rpy]}")
        if self.pos is not None:
            print(f"  机身位置 {np.round(self.pos, 3).tolist()}")
            if self.body_height() < 0.2:
                print("  ⚠️  机身高度偏低，狗已经趴了（没有控制器时正常）")
        return 0

    def state(self) -> dict:
        return dict(connected=self.connected, mode=self._mode, frames=self.frames,
                    z=self.body_height(),
                    rpy=list(self.rpy),
                    pos=self.pos.tolist() if self.pos is not None else None)

    # ==================================================================
    # 自检：不碰 DDS，不需要仿真器
    # ==================================================================

    # 用 MuJoCo 当第三方裁判校验运动学，再校验步态。
    #
    # FK 和 IK 都是我写的两份实现，它们自洽不代表正确 —— 如果两个公式共用
    # 同一个错误假设（比如关节轴方向搞反），往返测试照样全过。所以必须拿
    # mj_kinematics 的 xpos 做独立参照。
    def selftest(self) -> bool:
        import mujoco

        m = mujoco.MjModel.from_xml_path(MODEL_PATH)
        d = mujoco.MjData(m)
        feet = {leg: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_foot")
                for leg in LEGS}
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        print(f"模型 nq={m.nq} nv={m.nv} nu={m.nu}\n")

        def pose(q12):
            d.qpos[:] = 0.0
            d.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)   # 姿态取单位四元数 → 机体系 = 世界系
            d.qpos[7:19] = q12
            mujoco.mj_kinematics(m, d)
            return d.xpos[base].copy()

        e_home = self._check_home()
        e_fk = self._check_fk(m, d, feet, pose)
        e_ik = self._check_ik(m, d, feet, pose)
        e_pose = self._check_body_pose()
        gait_ok = self._check_gait()

        print("\n=== 结论 ===")
        print(f"  home 往返 {e_home:.2e} rad   FK 偏差 {e_fk * 1e6:.3f} μm   "
              f"IK 闭环 {e_ik * 1e6:.3f} μm   姿态动作可达 {e_pose}")
        kin_ok = e_home < 1e-9 and e_fk < 1e-6 and e_ik < 1e-6
        print("  运动学可用" if kin_ok else "  ✗ 运动学误差偏大，检查连杆参数")
        return kin_ok and gait_ok

    # 最容易手算的一步：home 关键帧的足端位置，IK 必须解回同一组关节角。
    def _check_home(self) -> float:
        print("=== 0. home 关键帧自洽（q1=0 才算对）===")
        worst = 0.0
        for leg in LEGS:
            p = self.fk_leg(leg, STAND_Q)
            r = self.ik_leg(leg, p)
            worst = max(worst, max(abs(a - b) for a, b in zip(STAND_Q, r.q)))
            print(f"  {leg}  foot=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})  "
                  f"IK→({r.q[0]:+.4f},{r.q[1]:+.4f},{r.q[2]:+.4f})")
        print(f"  与 STAND_Q 最大偏差 {worst:.2e} rad\n")
        return worst

    def _check_fk(self, m, d, feet, pose, n: int = 500, seed: int = 0) -> float:
        print("=== 1. 解析 FK vs MuJoCo xpos ===")
        rng = np.random.default_rng(seed)
        worst = 0.0
        for _ in range(n):
            q12 = []
            for _leg in LEGS:
                q12 += [rng.uniform(-0.4, 0.4), rng.uniform(-0.2, 1.5),
                        rng.uniform(-2.6, -0.9)]
            origin = pose(q12)
            for i, leg in enumerate(LEGS):
                mine = np.array(self.fk_leg(leg, q12[3 * i: 3 * i + 3]))
                theirs = d.xpos[feet[leg]] - origin
                worst = max(worst, float(np.linalg.norm(mine - theirs)))
        print(f"  {n} 组随机姿态，足端位置最大偏差 {worst * 1000:.4f} mm")
        return worst

    def _check_ik(self, m, d, feet, pose) -> float:
        print("\n=== 2. IK 实际精度（MuJoCo 裁判）===")
        stand = self.stand_feet(0.27, self.gait.stance_spread)
        worst, n_ok, n_fail = 0.0, 0, 0
        for dx in (-0.10, -0.05, 0.0, 0.05, 0.10):
            for dy in (-0.05, 0.0, 0.05):
                for dz in (-0.06, -0.03, 0.0, 0.03):
                    q12, target = [], {}
                    for leg in LEGS:
                        tgt = np.array(stand[leg]) + np.array([dx, dy, dz])
                        r = self.ik_leg(leg, tgt)
                        q12 += list(r.q)
                        if r.ok:
                            n_ok += 1
                            target[leg] = tgt
                        else:
                            n_fail += 1
                    origin = pose(q12)
                    for leg, tgt in target.items():
                        got = d.xpos[feet[leg]] - origin
                        worst = max(worst, float(np.linalg.norm(got - tgt)))
        total = n_ok + n_fail
        print(f"  {total} 个目标点：成功 {n_ok}，越界 {n_fail}")
        print(f"  足端落点最大误差 {worst * 1000:.4f} mm")
        return worst

    # 静态动作（蹲、俯仰、重心平移）的落点必须都能解出来，且留有余量。
    def _check_body_pose(self) -> bool:
        print("\n=== 3. 姿态动作可达性 ===")
        cases = [("蹲 8cm", dict(height=self.height - 0.08)),
                 ("蹲到底", dict(height=MIN_HEIGHT)),
                 ("低头 10°", dict(pitch=0.175)),
                 ("抬头 10°", dict(pitch=-0.175)),
                 ("左滚 8°", dict(roll=0.14)),
                 ("前移重心 5cm", dict(shift=(0.05, 0.0)))]
        ok_all = True
        for name, kw in cases:
            feet = self.feet_for_body_pose(**kw)
            bad = [leg for leg in LEGS if not self.ik_leg(leg, feet[leg]).ok]
            ok_all &= not bad
            detail = "" if not bad else "  ✗ " + ", ".join(bad)
            print(f"  {name:<14}{'✓' if not bad else '✗'}{detail}")
        return ok_all

    # 落地速度匹配、切相连续性、摆动峰值速度、关节角速度、IK 可行性。
    def _check_gait(self, rate: float = 4000.0) -> bool:
        print("\n=== 4. 步态轨迹校验 ===")
        gp = self.gait
        ok_all = True

        # v=0 应原地踏步：x 不漂，z 只抬 swing_height
        xs, zs = [], []
        for i in range(2000):
            p = self.foot_target("FL", i * gp.period / 2000)
            xs.append(p[0])
            zs.append(p[2])
        drift = (max(xs) - min(xs)) * 1e6
        lift = (max(zs) - min(zs)) * 1000
        print(f"  v=0：x 漂移 {drift:.3f} μm（应为 0），"
              f"抬脚 {lift:.1f} mm（应 = {gp.swing_height * 1000:.0f}）")

        for vx in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            row = self._scan_gait(vx, rate)
            ok_all &= row["td_err"] < 1e-4 and row["ik_fail"] == 0
            print(f"  vx={vx:.1f}  落地速度误差 {row['td_err'] * 1e3:6.3f} mm/s  "
                  f"切相跳变 {row['pos_jump'] * 1e9:5.3f} nm  "
                  f"摆动峰值 {row['peak_foot']:.2f} m/s  "
                  f"关节 {row['peak_joint']:4.1f} rad/s  "
                  f"IK 失败 {row['ik_fail']}")

        print(f"  落地速度匹配与 IK 可行性：{'✓' if ok_all else '✗'}（阈值 0.1 mm/s）")
        print("  离地瞬间速度跳变是有意保留的：匹配它要足端先向后多退 20% 步幅，代价大于收益")
        return ok_all

    def _scan_gait(self, vx: float, rate: float = 4000.0) -> dict:
        gp = self.gait
        worst_td = worst_jump = worst_vel_jump = 0.0
        for leg in LEGS:
            off = gp.offsets[leg]
            t_td = ((1.0 - off) % 1.0) * gp.period        # 落地时刻
            t_lo = ((gp.duty - off) % 1.0) * gp.period    # 离地时刻
            v_expect = self.stance_velocity(leg, vx, 0.0, 0.0)

            # 1) 落地速度：摆动末（无接触）和支撑初（贴地）都要等于支撑相速度
            v_sw = self.foot_velocity(leg, t_td, vx, 0.0, 0.0)
            v_st = self.foot_velocity(leg, t_td + 1e-4, vx, 0.0, 0.0)
            worst_td = max(worst_td, math.dist(v_sw, v_expect),
                           math.dist(v_st, v_expect))

            # 2) 切相点位置连续：两个解析式理论上在边界恒等
            p_b = self.foot_target(leg, t_lo - 1e-9, vx, 0.0, 0.0)
            p_a = self.foot_target(leg, t_lo + 1e-9, vx, 0.0, 0.0)
            worst_jump = max(worst_jump, math.dist(p_b, p_a))

            # 3) 离地速度跳变：支撑末 -v vs 摆动初 0，采样点必须明确在各自相位内
            vlo_b = self.foot_velocity(leg, t_lo - 1e-6, vx, 0.0, 0.0)
            vlo_a = self.foot_velocity(leg, t_lo + 1e-6, vx, 0.0, 0.0)
            worst_vel_jump = max(worst_vel_jump, math.dist(vlo_a, vlo_b))

        # 4) 峰值速度、峰值加速度、IK、关节角速度
        n = int(rate * gp.period)
        dt = 1.0 / rate
        prev_q = {leg: None for leg in LEGS}
        prev_v = {leg: None for leg in LEGS}
        peak_foot = peak_joint = peak_acc = 0.0
        ik_fail = 0

        for i in range(n):
            t0, t1 = i * dt, (i + 1) * dt
            for leg in LEGS:
                p0 = self.foot_target(leg, t0, vx, 0.0, 0.0)
                p1 = self.foot_target(leg, t1, vx, 0.0, 0.0)
                v = tuple((b - a) / dt for a, b in zip(p0, p1))
                peak_foot = max(peak_foot, math.sqrt(sum(c * c for c in v)))
                if prev_v[leg] is not None:
                    peak_acc = max(peak_acc, math.dist(v, prev_v[leg]) / dt)
                prev_v[leg] = v

                r = self.ik_leg(leg, p1)
                if not r.ok:
                    ik_fail += 1
                if prev_q[leg] is not None:
                    peak_joint = max(peak_joint, max(
                        abs(x - y) for x, y in zip(r.q, prev_q[leg])) / dt)
                prev_q[leg] = r.q

        return dict(td_err=worst_td, pos_jump=worst_jump,
                    vel_jump=worst_vel_jump, peak_foot=peak_foot,
                    peak_joint=peak_joint, peak_acc=peak_acc, ik_fail=ik_fail)


if __name__ == "__main__":
    if __package__ in (None, ""):
        # 直接当脚本跑：把项目根塞进 sys.path，并改用包里那一份 Go2Controller，
        # 避免 __main__ 和 quadruped.control.controller 同时存在两份模块
        # （类身份会不一致）。文件在 quadruped/control/，项目根是上三级。
        sys.path.insert(0, os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        from quadruped.control.controller import Go2Controller as _Go2Controller
    else:
        _Go2Controller = Go2Controller
    sys.exit(0 if _Go2Controller().selftest() else 1)
