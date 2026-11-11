#!/usr/bin/env python3
"""Go2 高层运行控制器（第 3 层）：只表达意图，不碰 DDS。

父对象是 controller.Go2Controller，构造时注入，存放于 self.controller；
每条动作最后都落到它的 apply_pose / _run_gait / start_velocity 上。
本层不 import 机器人，也不持有机器人引用 —— 依赖只向上走一层。

    from quadruped.robot.body import Go2Robot

    robot = Go2Robot(); robot.connect()
    hl = robot.controller.high_level
    hl.stand_up(); hl.squat(0.08); hl.forward(2.0); hl.turn(90)

单独构造（测试用）：HighLevelController(Go2Controller(...)) —— 控制器内部就是这么做的。
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

from quadruped.control.controller import MIN_HEIGHT, NOMINAL_HEIGHT, _wrap_pi

if TYPE_CHECKING:                       # 只为类型标注，运行时不引入依赖
    from quadruped.control.controller import Go2Controller


class HighLevelController:
    """高层运行控制器：只表达意图，不碰 DDS。

    父对象是 Go2Controller（构造时注入，存在 self.controller），
    本层的每条动作最终都落到它的 apply_pose / _run_gait / start_velocity 上。
    不 import 机器人，也不持有机器人引用 —— 依赖只向上走一层。

    三组方法：

    原生对齐（与 unitree_sdk2py 的 SportClient 同名同语义，返回码 0=成功）
        Init / SetTimeout / BalanceStand / StandUp / StandDown / Sit / RiseSit /
        RecoveryStand / Damp / Move / StopMove / BodyHeight / Euler /
        Hello / Stretch / SwitchGait
        名字、参数、语义都按原生来，所以 quadruped/control/virtual/keyboard.py 这类现成脚本
        不用改逻辑就能直接驱动（只换连接的那一个对象）。

    基础动作（静态姿态类）
        stand_up / squat / tilt / shift / nod / lie_down   靠插值关节角摆姿态
        step_in_place / walk_time                           靠控制器的开环步态

    基础动作（不阻塞版，键盘这类实时控制用）
        jump_async / crouch_async / uncrouch_async / stand_async / stop_async
        只把序列排给控制回路就返回，调用方不必等动作演完。新动作直接顶掉旧的，
        所以「跳着也能接蹲下」这种连招是通的。同名不带 _async 的版本等做完才返回，
        留给命令行脚本用。

    任务（带反馈）
        forward / strafe / turn                             用 rt/sportmodestate 的
                                                            位置和 IMU 的 yaw 闭环
        sequence                                            按名字串起来执行

    为什么距离要闭环：开环小跑打滑 30~45%，让它「走两米」实际只走 1.2 m。
    高层读到实际位移够了再收步，滑多少都不影响结果。
    """

    API_OK = 0
    API_UNSUPPORTED = 3104     # 原生里表示「这条 api 不支持」

    def __init__(self, controller: Go2Controller):
        self.controller = controller
        self.history: list[tuple[str, dict]] = []
        self.timeout = 3.0
        # 默认机械档：跳跃用同步弹跳、跑用标准小跑，两边一致。
        # 仿生档（后腿发力跳 + 横向序列疾驰）按 Shift / Tab 切过去。
        self.jump_style = "mech"        # 风格，见 JUMP_STYLES

    # -- 原生对齐：连接与姿态 ------------------------------------------

    def Init(self) -> int:
        """原生 SportClient 里 Init() 建立 api 通道；这里的通道在 connect() 就有了。"""
        return self.API_OK

    def SetTimeout(self, seconds: float) -> int:
        self.timeout = float(seconds)
        return self.API_OK

    def _settle(self, timeout: float = 2.5) -> None:
        """正在走的话先收步站定。原生由运动服务内部处理，我们得显式做。"""
        c = self.controller
        if c._mode != "gait":
            return
        if c._gait and not c._gait.get("stream"):
            c.hold(c.stand_joint_target())      # 定时步态直接切站定
            return
        c.stop_velocity()
        t0 = time.time()
        while c._mode == "gait" and time.time() - t0 < timeout:
            time.sleep(0.05)

    def BalanceStand(self) -> int:
        """站好并保持。原生里这一步同时会切到平衡站立模式。"""
        self._settle()
        self.controller.apply_pose(self.controller.stand_joint_target(), hold_s=0.25)
        return self.API_OK

    def StandUp(self) -> int:
        """从趴/蹲立起来。"""
        return self.BalanceStand()

    def RecoveryStand(self) -> int:
        """跌倒恢复。我们只会「立起来」这一种，翻过去的姿态自己解不了。"""
        return self.BalanceStand()

    def StandDown(self) -> int:
        """蹲下保持（原生为切换站立关闭）。"""
        self._settle()
        self.controller.apply_pose(self.controller.joint_from_feet(self.controller.stand_feet(MIN_HEIGHT)),
                          hold_s=0.3)
        return self.API_OK

    def Sit(self) -> int:
        """近似坐姿：蹲到最低保持。真正的坐姿要把后腿折叠、前腿伸直，还没做。"""
        return self.StandDown()

    def RiseSit(self) -> int:
        return self.BalanceStand()

    def Damp(self) -> int:
        """松成阻尼器，趴下。"""
        self.controller.shutdown()
        return self.API_OK

    def park(self) -> int:
        """收工：停步 → 蹲到最低 → 阻尼。

        仿真桥把 rt/lowcmd 直接换算成力矩，**不发就没有力矩**（真机是 1 秒后
        保护停车）。所以进程退出前必须自己蹲好再松腿，否则是从 27 cm 高度
        直接砸下去，腿会甩到限位。
        """
        self._settle()
        self.StopMove()
        self.StandDown()
        return self.Damp()

    # -- 原生对齐：速度 --------------------------------------------------

    def Move(self, vx: float, vy: float, vyaw: float) -> int:
        """设定机身速度，一直保持到 StopMove 或下一次 Move。**不阻塞**。

        vx 前、vy 左、vyaw 逆时针为正，机体系。加速度由控制器限幅（1 m/s²、
        2.5 rad/s²），所以从静止直接给 0.4 m/s 也不会一下窜出去。

        正在跳/蹲这类姿态动作时忽略速度指令 —— 腿此刻被序列占着，插一脚只会
        把动作撕碎。动作播完调用方再发就行，键盘那边就是这么做的。
        """
        c = self.controller
        if not c.connected:
            return 3102                       # 原生「对端未响应」的近似
        if c.busy:
            return self.API_OK
        if not c.velocity_active():
            c.start_velocity()
        c.set_velocity(vx, vy, vyaw)
        return self.API_OK

    @property
    def busy(self) -> bool:
        """腿是否正被一段姿态动作占用（跳跃、蹲伏）。"""
        return self.controller.busy

    def stop_async(self) -> int:
        """停车并站定，**不等收步完成**，返回后腿还在渐收。

        原生是「发完就不管」，我们的回路自己把幅度收到 0 再转 hold，所以调用方
        不必在这里睡着等 —— 键盘 20 Hz 的主循环就靠这个不卡。
        """
        c = self.controller
        if c.velocity_active():
            c.stop_velocity()
        elif c.busy:
            c.apply_pose_async(c.stand_joint_target(), duration=0.2)
        return self.API_OK

    def StopMove(self) -> int:
        """停车：目标速度归零，等步态幅度收到 0、四脚站稳再返回（约 0.5 s）。

        原生是「发完就不管」，因为真机上运动服务一直在跑；我们的控制回路跟着
        进程走，进程一退就不能停在摆动态 —— 那样只剩三条腿撑地，会侧翻。
        所以这里多等一会儿，最多 1.5 s。
        """
        c = self.controller
        if c.velocity_active():
            c.stop_velocity()
            t0 = time.time()
            while c._mode == "gait" and time.time() - t0 < 1.5:
                time.sleep(0.02)
            if c._mode != "gait":
                c.hold(c.stand_joint_target())
        elif c.connected and c._mode != "gait" and not c.busy:
            c.hold()
        return self.API_OK

    def BodyHeight(self, offset: float) -> int:
        """改机身高度。与原生一致，参数是相对默认高度的偏移，范围 -0.12~0.03。"""
        self._settle()
        h = max(MIN_HEIGHT, min(self.controller.height + 0.03, self.controller.height + offset))
        self.controller.apply_pose(self.controller.joint_from_feet(self.controller.stand_feet(h)),
                          hold_s=0.25)
        return self.API_OK

    def Euler(self, roll: float, pitch: float, yaw: float) -> int:
        """机身相对姿态（弧度）。正 pitch 为低头，与原生一致。"""
        self._settle()
        feet = self.controller.feet_for_body_pose(roll=roll, pitch=pitch, yaw=yaw)
        self.controller.apply_pose(self.controller.joint_from_feet(feet), hold_s=0.3)
        return self.API_OK

    def Hello(self) -> int:
        """近似「打招呼」：低头-抬头两下。原生是坐姿抬前爪，我们没做坐姿。"""
        self.ensure_standing()
        self.nod(times=2, pitch=0.14)
        return self.API_OK

    def Stretch(self) -> int:
        """近似「伸懒腰」：蹲 → 站起来 → 前倾。"""
        self.squat(depth=0.07, hold=0.3)
        self.tilt(pitch=0.12, hold=0.5)
        return self.API_OK

    def SwitchGait(self, gait_type: int) -> int:
        """换步态类型。我们只有一种对角小跑，返回原生「不支持」。"""
        return self.API_UNSUPPORTED

    # -- 基础动作：跳跃与蹲伏 ------------------------------------------

    def crouch_async(self, depth: float = 0.08) -> int:
        """蹲下并停住。**立即返回**，腿由控制回路按序列摆好。"""
        c = self.controller
        h = max(MIN_HEIGHT, c.height - depth)
        c.pose_sequence([(c.joint_from_feet(c.stand_feet(h)), 0.22, 0.0)],
                        label="蹲下")
        print(f"蹲下：机身 {c.height:.3f} → {h:.3f} m", flush=True)
        return self.API_OK

    def uncrouch_async(self) -> int:
        """从蹲姿站起来。**立即返回**。"""
        self.controller.apply_pose_async(self.controller.stand_joint_target(),
                                         duration=0.28)
        return self.API_OK

    def stand_async(self) -> int:
        """回到站立并保持，顺便中断正在演的动作。**立即返回**。"""
        self.controller.apply_pose_async(self.controller.stand_joint_target(),
                                         duration=0.28)
        return self.API_OK

    def crouch(self, depth: float = 0.08) -> int:
        """蹲下并保持（squat 是蹲一下就起来，这个是停在低处）。"""
        self._settle()
        code = self.crouch_async(depth)
        self.controller.wait_idle(3.0)
        self._report("蹲底")
        return code

    def uncrouch(self) -> int:
        """从蹲姿站起来。"""
        self._settle()
        code = self.uncrouch_async()
        self.controller.wait_idle(3.0)
        self._report("起立")
        return code

    # 起跳时脚相对机身往后滑的最大距离。腿的水平工作空间就这么多，
    # 再多髋关节会顶到限位、落地时前腿撑不住。
    JUMP_TRAVEL = 0.10
    # 走着跳：脚在支撑段该以多快的速度往后退。1.5 倍实测机身速度 ——
    # 等于 1.0 的话脚只是跟着地面走，一点推力都不给；实测要 1.5 机身才真的
    # 被推着往前跑（和步态里的 stride_gain 一个道理，都是补打滑）。
    JUMP_GROUND_GAIN = 1.5
    # 落地前把脚收到机身前面这么多，落地那一下脚落在重心前面，机身穿过去
    # 才不会一顿。不给的话脚正好落在重心下方，落地就是一脚刹车。
    JUMP_LAND_REACH = 0.03
    # 走着跳时「前腿伸前 / 后腿后拖」保留几成。原地跳靠这个伸展摆出狗的姿势，
    # 但带着速度落地时，脚伸得太靠前等于在重心前面插撑杆，机身会被顶回头。
    JUMP_REACH_FADE = 0.30
    # 跳跃序列的时间轴（秒）。空中那两段是估算的：实际离地/触地看腿的实际情况，
    # 但「落地」那一段必须踩在真的着地时刻上，否则脚还没碰地就被收回中立位，
    # 机身等于从半空砸下来 —— 实测落地瞬间 −0.95 m/s 就是这么来的。
    # 用 pose_sequence 播放时这两段就是纯等待，唯一的作用是对齐触地。
    JUMP_T = dict(crouch=0.16, hold_in=0.02, push1=0.07, push2=0.06,
                  air=0.14, fall=0.28, land=0.14, hold_land=0.08,
                  recover=0.35, hold_end=0.18)

    @classmethod
    def _jump_times(cls) -> tuple[float, float, float]:
        """(离地, 触地, 站定) 三个时刻，与序列本身严格一致。"""
        T = cls.JUMP_T
        t_lift = T["crouch"] + T["hold_in"] + T["push1"] + T["push2"]
        t_touch = t_lift + T["air"] + T["fall"]
        return t_lift, t_touch, t_touch + T["land"] + T["hold_land"]

    # -- 仿生步态 --
    # 蹬伸时后腿比前腿多伸多少。后腿是动力源，蹬到极限把机身顶起来；
    # 前腿也要蹬（`JUMP_FRONT_PUSH`），只是蹬得少 —— 前腿不出力的话，机身
    # 一升高前脚就被动离地，前端没人顶，整条身子立刻前栽（实测 32° 脸朝下）。
    JUMP_REAR_DRIVE = 0.07
    JUMP_FRONT_PUSH = 0.020
    # 前腿在空中往前伸出去等落地，后腿收着拖在后面。这是狗跳的招牌姿态：
    # 前腿够到最前面先着地接住机身，后腿再甩回来撑住。
    JUMP_FRONT_REACH = 0.090
    JUMP_REAR_TRAIL = 0.045
    # 前腿往前送、后腿往后拖，两条腿的摆动会给机身一个低头角动量，脚在空中
    # 又踩不住地面，只能看着机身越飞越低头。这两个角度是反向补偿：空中压一点，
    # 触地那一下让机身低头，把落地的俯仰拉平。实测把俯仰从 -29~+15° 收到 ±4°。
    JUMP_AIR_PITCH = -0.06
    JUMP_LAND_PITCH = 0.10
    # 原地起跳的补偿。前腿往前送、后腿往后拖，净反作用力把机身往后推，
    # 什么都不给的话站着跳会退后 15 cm。这段额外位移让它落回原地。
    JUMP_STAND_BIAS = 0.06

    # -- 机械步态 --
    # 四条腿一起蹬，推力是仿生的两倍，还赶上 Go2 质心偏后 —— 同样蹬 0.07
    # 会直接后空翻（实测离地 +0.29 m、pitch −89°）。所以单独给一个小的。
    JUMP_MECH_DRIVE = 0.025
    # 也不能真的「四条腿完全一样」：质心偏后，前腿负载轻、先离地，四条腿同速
    # 伸等于只用后腿顶，机身照样仰头（实测 −40°）。这点后腿偏置是补重心。
    # 蹬伸时给机身一个低头指令。脚还踩着地，这个指令才有真作用力 —— 正好抵消
    # 「质心偏后、被后腿顶得仰头」那点角速度。空中再改就晚了（四肢扭不动机身）。
    JUMP_MECH_PUSH_PITCH = 0.06
    # 重心补偿：质心偏后，后腿本来就吃更多体重。四条腿同长度同速度伸的话，前腿
    # 先离地、只剩后腿顶，机身立刻仰头（实测 −40°）。给后腿多加这一段延伸量，
    # 前后脚的离地/着地时刻才对得上 —— 四腿同时蹬、同时落。
    JUMP_MECH_BIAS = 0.045
    # 蹬伸高度也要压住：蹬得越高，空中仰头攒得越多。
    JUMP_MECH_PUSH = 0.245
    # 落地俯仰补偿。蹬伸阶段补重心让机身带着仰头角速度出去，脚在空中又改不回来
    # （空中改足端目标基本扭不动机身），只能在触地那一下用低头压住。正值为低头。
    JUMP_MECH_AIR_PITCH = 0.02
    JUMP_MECH_LAND_PITCH = 0.24
    # 机械步态站着跳会往前漂 13 cm。这段位移是落地时滑出来的，不是摆动反作用，
    # 所以位移补偿压不住（-0.05 / -0.08 都试过，反而更大），这里留 0。
    JUMP_MECH_STAND_BIAS = 0.0

    # 两种风格，Shift / Tab 切换。跳跃动作和运动步态共用同一档，切一次两样
    # 一起换 —— 不会出现「仿生跳 + 机械跑」这种对不上的组合：
    #   bio  仿生 —— 跳跃：后腿出力、前腿伸前接落地；跑：横向序列疾驰
    #   mech 机械 —— 跳跃：四条腿一个相位，硬邦邦的弹簧跳；跑：标准小跑
    JUMP_STYLES = ("mech", "bio")
    JUMP_STYLE_NAMES = {
        "mech": "机械（四腿同步跳 / 标准小跑）",
        "bio": "仿生（后腿发力跳 / 横向序列疾驰）",
    }

    def set_jump_style(self, style: str) -> str:
        """切风格：跳跃序列和运动步态一起换，返回切完之后的那个。"""
        if style not in self.JUMP_STYLES:
            raise ValueError(f"没有这种跳跃风格：{style}，只有 {self.JUMP_STYLES}")
        self.jump_style = style
        # 运动步态跟着走。两边同名同义，切风格就是切整套。
        if style in self.controller.GAIT_STYLES:
            self.controller.set_gait_style(style)
        return style

    def toggle_jump_style(self) -> str:
        """在当前风格和下一个之间来回切。"""
        i = self.JUMP_STYLES.index(self.jump_style)
        return self.set_jump_style(self.JUMP_STYLES[(i + 1) % len(self.JUMP_STYLES)])

    @property
    def jump_style_label(self) -> str:
        return self.JUMP_STYLE_NAMES[self.jump_style]

    # -- 运动步态（跑起来怎么迈腿，和跳跃动作共用同一档风格）-------------

    @property
    def gait_style(self) -> str:
        return self.controller.gait_style

    @property
    def gait_style_label(self) -> str:
        return self.controller.gait_style_label

    def set_gait_style(self, style: str) -> str:
        """换风格（`mech` 机械 / `bio` 仿生），跳跃动作一并跟着换。"""
        style = self.controller.set_gait_style(style)
        if style in self.JUMP_STYLES:
            self.jump_style = style
        print(f"运动步态 → {self.controller.gait_style_label}", flush=True)
        return style

    def toggle_gait_style(self) -> str:
        style = self.controller.toggle_gait_style()
        if style in self.JUMP_STYLES:
            self.jump_style = style
        print(f"运动步态 → {self.controller.gait_style_label}", flush=True)
        return style

    def _jump_bio_steps(self, jf, crouch: float, push: float, sweep_at,
                        prep_h: float | None = None,
                        reach_scale: float = 1.0) -> list[tuple]:
        """仿生：后腿蹬得多、前腿伸前等落地。

        `sweep_at(t)` 给出该时刻足端相对中立位要后退多少米 —— 走着跳的时候，
        机身在地面上往前跑，脚在机体系里就必须等速后退，否则脚一踩实就把机身
        拽停。这是「起跳把速度吃掉」的唯一原因，也是整条序列里最容易漏的一环。
        """
        T = self.JUMP_T
        # 走着跳时「前腿伸前」要收着点：那套伸展是给原地跳准备的，机身带着
        # 速度前进时脚再往前伸一大截，落地等于在重心前面插了一根撑杆，
        # 机身被顶得抬头、往回弹（实测落地瞬间 −0.95 m/s）。
        drive = self.JUMP_REAR_DRIVE
        reach = self.JUMP_FRONT_REACH * reach_scale
        trail = self.JUMP_REAR_TRAIL * reach_scale
        fpush = self.JUMP_FRONT_PUSH
        clock = [0.0]
        steps: list[tuple] = []

        def add(make, key, hold=0.0):
            dur = T[key]
            clock[0] += dur + hold
            steps.append((make(sweep_at(clock[0])), dur, hold))

        # 走到跳：先把迈步中途的四条腿收成站姿，起跳才直。0.08 s 够短，
        # 脚一边收一边跟着机身往后退，速度不会掉。
        if prep_h is not None:
            clock[0] += 0.08
            steps.append((jf(prep_h, sweep=sweep_at(clock[0])), 0.08, 0.0))
        add(lambda s: jf(crouch, sweep=s, rear_h=-0.015, front_h=+0.010),
            "crouch", T["hold_in"])
        add(lambda s: jf(crouch + 0.045, sweep=s, rear_h=drive * 0.65,
                         front_h=fpush), "push1")
        add(lambda s: jf(push, sweep=s, rear_h=drive * 0.45, front_h=-0.020,
                         front_x=0.035), "push2")
        # 空中：后腿收着拖在后面，前腿伸到最前面等着接地面。
        add(lambda s: jf(crouch + 0.02, sweep=s, rear_h=+0.020, rear_x=-trail,
                         front_h=+0.030, front_x=reach,
                         pitch=self.JUMP_AIR_PITCH), "air")
        # 下坠段：机身还在半空，腿保持伸前等着的姿势，别提前收回中立位。
        add(lambda s: jf(crouch + 0.02, sweep=s, rear_h=+0.020, rear_x=-trail,
                         front_h=+0.030, front_x=reach), "fall")
        # 触地：前腿先够到地面接住机身（伸得最长），后腿从后面甩回来。
        # 前后差值别拉太大，否则后腿一撑、机身尾部被抬起，落地会仰。
        add(lambda s: jf(MIN_HEIGHT + 0.04, sweep=s, rear_h=-0.025,
                         rear_x=-trail * 0.6, front_h=+0.030,
                         front_x=reach * 0.7, pitch=self.JUMP_LAND_PITCH),
            "land", T["hold_land"])
        # 回正站好，并且多按住一会儿：序列播完时腿就被交还给步态了，
        # 这时候机身如果还在半空，一开步就会翻。
        add(lambda s: jf(NOMINAL_HEIGHT, sweep=s), "recover", T["hold_end"])
        return steps

    def _jump_mech_steps(self, jf, crouch: float, push: float, sweep_at,
                         prep_h: float | None = None) -> list[tuple]:
        """机械：四条腿同一个相位，一起伸、一起收、一起落地。

        踩地时后腿比前腿多伸 `JUMP_MECH_BIAS`，这是重心补偿而不是前后分工：质心
        偏后，不给这点补偿，前腿会先离地、落地时又慢半拍，四腿并跳就成了前后两段。
        空中和落地两段则严格同长度，四只脚才可能同时离地、同时着地。
        """
        T = self.JUMP_T
        lift, bias = self.JUMP_MECH_DRIVE, self.JUMP_MECH_BIAS
        clock = [0.0]
        steps: list[tuple] = []

        def all4(h, sweep=(0.0, 0.0), dh=0.0, pitch=0.0, ground: bool = False):
            """四条腿同一个相位。

            后腿那点偏置只在 `ground=True`（脚踩着地）时给：踩地时它是补重心，
            悬空时它就变成「后腿多伸」的仰头力矩了 —— 带进空中就是落地仰头的根源。
            """
            b = bias if ground else 0.0
            return jf(h, rear_h=dh + b, front_h=dh, pitch=pitch, sweep=sweep)

        def add(make, key, hold=0.0):
            dur = T[key]
            clock[0] += dur + hold
            steps.append((make(sweep_at(clock[0])), dur, hold))

        if prep_h is not None:
            clock[0] += 0.08
            steps.append((jf(prep_h, sweep=sweep_at(clock[0])), 0.08, 0.0))
        add(lambda s: all4(crouch), "crouch", T["hold_in"])
        add(lambda s: all4(crouch + 0.06, s, dh=lift * 0.8, ground=True,
                           pitch=self.JUMP_MECH_PUSH_PITCH * 0.5), "push1")
        add(lambda s: all4(push, s, dh=lift, ground=True,
                           pitch=self.JUMP_MECH_PUSH_PITCH), "push2")
        # 蹬完整个飞行段都伸直，落地那一下才屈腿缓冲。四腿并跳不能在空中收腿：
        # 后腿还踩着地时一折，等于把尾部往下拽，机身立刻仰头（实测 −25°）。
        # 伸直着飞也正好是「机械」该有的样子 —— 像一张平着抛出去的桌子。
        add(lambda s: all4(push - 0.008, s, dh=lift), "air")
        add(lambda s: all4(push - 0.016, s, dh=lift,
                           pitch=self.JUMP_MECH_AIR_PITCH), "fall")
        add(lambda s: all4(MIN_HEIGHT + 0.04, s, dh=-lift * 0.3,
                           pitch=self.JUMP_MECH_LAND_PITCH), "land",
            T["hold_land"])
        add(lambda s: jf(NOMINAL_HEIGHT, sweep=s), "recover", T["hold_end"])
        return steps

    def jump_async(self, crouch: float = 0.185, push: float = 0.265,
                   vx: float | None = None, vy: float | None = None,
                   style: str | None = None) -> int:
        """起跳：蓄力 → 蹬伸 → 空中 → 触地 → 回正。**立即返回**。

        两种步态（`style` 不给就用当前设置，Shift 键在键盘脚本里切换）：

        `bio` 仿生 —— 四条腿分工，机械狗看着像活物：

        - **后腿发力**。蹬伸段后腿压得更深（`+JUMP_REAR_DRIVE`），前腿也蹬、但蹬得
          少（`+JUMP_FRONT_PUSH`）。前腿不能不出力，也不能提前折起来 —— 前端一悬空，
          机身在尾部支撑下立刻前栽（实测 front_h=0 时 pitch 到 +32°，脸朝下）。
        - **前腿伸前等落地**。离地之后前足往机身前方送 `JUMP_FRONT_REACH`，后足往
          后拖 `JUMP_REAR_TRAIL`，落地那一下是前腿先够到地面接住机身，后腿随后
          甩回来撑住。不这么分，四条腿只会一起上下蹬，看着像弹簧不像狗。

        `mech` 机械 —— 四条腿一个相位，一起伸一起收，脚掌始终排成一条线。
        没有前后分工，跳得矮一点、姿态硬一点，好处是俯仰天生是平的。

        位置伺服做跳，靠的是「脚还踩着地面那一下，机身被快速顶高」。蹬伸必须在
        腿伸到极限之前结束，否则冲量全变成撞限位；所以这里拆成两段，第二段更短
        更快，速度是递增的。

        **走着跳**（按住 w 时按空格）：速度用**实测**的，不用指令的 —— 步态本身
        打滑，指令 0.3 时机身实际只有 0.2 出头，按指令算会算过头、落地往前栽。
        足端偏移不再是「飞了多远」的归一化进度，而是**按时间**积分的后退量：
        机身在地上以 v 前进，脚在机体系里就必须以 v 后退，否则脚一踩实就把机身
        拽停 —— 之前「一起跳速度就没了」就是这个原因。空中把脚收到机身前方
        `JUMP_LAND_REACH` 准备接地面，触地那 0.12 s 再按 v 往后扫，落地不打滑。

        整条序列交给控制回路播，调用方拿到就返回 —— 跳的同时你按别的键、调速度、
        甚至再跳一次都行，不用等它落地站稳。

        能不能离地取决于 kp（刚度）和力矩上限，仿真的地面摩擦也会影响。起不来就
        把 push 加 0.01 再试；翻倒了就说明蹬伸太快，把第二段 duration 拉长。
        """
        c = self.controller
        c.require_connection()

        if vx is None or vy is None:
            # 优先用实测速度：步态打滑，指令 0.3 时实际只有 0.2，拿指令去算
            # 足端后退量会算过头，脚在地上蹭过头、落地往前栽。
            mv = c.body_velocity()
            cvx, cvy, _ = c.velocity_command()
            if mv is not None and abs(mv[0]) + abs(mv[1]) > 0.02:
                cvx, cvy = mv[0], mv[1]
            vx = cvx if vx is None else vx
            vy = cvy if vy is None else vy
            ax, ay, _ = c.velocity_accel()
        else:
            ax = ay = 0.0

        style = style or self.jump_style
        z0 = c.body_height()
        spd = abs(vx) + abs(vy)
        moving = spd > 0.04
        prep_h = z0 if (moving and c._mode == "gait") else None

        # 走着跳：整条序列按时间给足端累计一个「后退量」。
        #   地面段  0 → t_lift     以 v·g 后退，机身才真的往前跑
        #   空中段  t_lift → t_touch 收到机身前方 R，落地时脚在前
        #   触地段  再往后扫 v·0.12，落地瞬间脚相对地面几乎不动，不打滑
        # g 取 1.5：实测机身速度只有指令的六成，脚扫快一点才推得动。
        t_lift, t_touch, t_done = self._jump_times()
        if prep_h is not None:
            t_lift, t_touch, t_done = t_lift + 0.08, t_touch + 0.08, t_done + 0.08
        if moving:
            s_lift = min(self.JUMP_TRAVEL * 0.85,
                         spd * t_lift * self.JUMP_GROUND_GAIN)
        else:
            # 原地跳：脚不跟着走，只留一点点前送，抵消摆腿的反作用（原来那点
            # 「跳完退后 15 cm」的补偿，现在按同一条曲线给）。
            s_lift = (self.JUMP_STAND_BIAS if style == "bio"
                      else self.JUMP_MECH_STAND_BIAS)
        r_fwd = self.JUMP_LAND_REACH if moving else 0.0

        def sweep_at(t: float) -> tuple[float, float]:
            """t 时刻足端相对中立位要后退多少米，(前后, 左右)。"""
            if moving:
                knots = [(0.0, 0.0), (t_lift, s_lift), (t_touch, -r_fwd),
                         (t_touch + 0.12, -r_fwd + spd * 0.12), (t_done, 0.0),
                         (1e9, 0.0)]
            else:
                knots = [(0.0, 0.0), (t_lift, s_lift), (t_done, s_lift),
                         (1e9, s_lift)]
            sx = knots[0][1]
            for (t1, v1), (t2, v2) in zip(knots, knots[1:]):
                if t <= t1:
                    sx = v1
                    break
                if t <= t2:
                    sx = v1 + (v2 - v1) * (t - t1) / max(1e-9, t2 - t1)
                    break
            if not moving:
                return (0.0, 0.0)
            if abs(vx) < 0.02:                          # 纯横移
                return (0.0, max(-self.JUMP_TRAVEL,
                                 min(self.JUMP_TRAVEL, sx * math.copysign(1.0, vy))))
            return (sx, sx * vy / vx)

        def jf(h: float, rear_h: float = 0.0, front_h: float = 0.0,
               rear_x: float = 0.0, front_x: float = 0.0, pitch: float = 0.0,
               sweep: tuple[float, float] = (0.0, 0.0)):
            """一组足端目标。

            h       机身高度基准（四个足端 z 都由它定）
            sweep   足端整体后退量（米），机身因此相对地面前进
            rear_h / front_h   后 / 前腿额外伸展（正=蹬得更长，负=折起）
            rear_x / front_x   后 / 前足额外前后偏移（正=往机身前方）
            pitch   机身相对足端低头多少（正=低头，与 Euler() 同一套约定）。
                    空中前后腿前后拉伸会产生低头力矩，靠它反着补回来。
            """
            cp, sp = math.cos(pitch), math.sin(pitch)
            sx, sy = sweep
            feet = {}
            for leg in ("FL", "FR", "RL", "RR"):
                rear = leg in ("RL", "RR")
                x, y, z = c.foot_from_body(
                    leg,
                    h + (rear_h if rear else front_h),
                    xy_offset=(-sx + (rear_x if rear else front_x), -sy))
                if pitch:
                    x, z = cp * x - sp * z, sp * x + cp * z    # Rᵀ·(x,y,z)，只绕 y
                feet[leg] = (x, y, z)
            return c.joint_from_feet(feet)

        if style == "mech":
            push = min(push, self.JUMP_MECH_PUSH)      # 蹬太高空中仰头压不住
            steps = self._jump_mech_steps(jf, crouch, push, sweep_at, prep_h)
        else:
            steps = self._jump_bio_steps(jf, crouch, push, sweep_at, prep_h,
                                         reach_scale=1.0 if not moving
                                         else self.JUMP_REACH_FADE)
        c.pose_sequence(steps, label=f"跳跃·{style}", z0=z0)
        tail = (f"带速度 {vx:+.2f}/{vy:+.2f} m/s（a={ax:+.2f}），"
                f"地面后退量 {s_lift * 100:.0f} cm" if moving else "原地")
        print(f"起跳[{self.JUMP_STYLE_NAMES[style]}]：压到 {crouch:.3f} → "
              f"蹬到 {push:.3f} m，{tail}", flush=True)
        return self.API_OK

    def jump(self, crouch: float = 0.185, push: float = 0.265,
             vx: float | None = None, vy: float | None = None,
             style: str | None = None) -> int:
        """跳跃（阻塞版，给命令行用）：等落地回正再返回。"""
        code = self.jump_async(crouch=crouch, push=push, vx=vx, vy=vy, style=style)
        self.controller.wait_idle(4.0)
        return code

    # -- 基础动作：静态姿态 --------------------------------------------

    def ensure_standing(self, threshold: float = 0.20,
                        hold: float = 0.4) -> bool:
        """趴着就先立正，站着就什么都不做。所有静态动作开头都调它。"""
        z = self.controller.body_height()
        if z is not None and z >= threshold:
            return False
        print(f"当前机身高度 {z if z is None else round(z, 3)} m，先起立", flush=True)
        self.controller.apply_pose(self.controller.stand_joint_target(), hold_s=hold)
        return True

    def _report(self, label: str = "") -> None:
        """打完一个静态姿态后，报告实测高度与姿态，方便和指令对账。"""
        c = self.controller
        h = c.body_height()
        z = f"{h:.3f}" if h is not None else "--"
        rpy = ", ".join(f"{v:+.2f}" for v in c.rpy) if c.rpy else "--"
        print(f"  {label}实测 z={z}  rpy=({rpy})", flush=True)

    def stand_up(self, hold: float = 0.6) -> float:
        """从任意姿态立正站好。"""
        return self.controller.apply_pose(self.controller.stand_joint_target(), hold_s=hold)

    def squat(self, depth: float = 0.08, hold: float = 0.6) -> None:
        """下蹲到 height-depth 再站起来。depth 上限约 0.12（再低小腿顶机身）。"""
        self.ensure_standing()
        h = max(MIN_HEIGHT, self.controller.height - depth)
        q = self.controller.joint_from_feet(self.controller.stand_feet(h))
        print(f"下蹲：机身 {self.controller.height:.3f} → {h:.3f} m", flush=True)
        self.controller.apply_pose(q, hold_s=hold)
        self._report("蹲底")
        self.stand_up(hold=0.3)

    def tilt(self, pitch: float = 0.15, roll: float = 0.0,
             hold: float = 0.8) -> None:
        """脚不动、机身低头/抬头或侧滚，停一下再回正。

        pitch 正为低头（前倾），roll 正为向左滚，单位弧度。
        这是最直观的 IK 演示：机身转了，机体系里看到的足端坐标就变了。
        """
        self.ensure_standing()
        feet = self.controller.feet_for_body_pose(roll=roll, pitch=pitch)
        print(f"倾身：pitch={math.degrees(pitch):+.0f}° "
              f"roll={math.degrees(roll):+.0f}°", flush=True)
        self.controller.apply_pose(self.controller.joint_from_feet(feet), hold_s=hold)
        self._report("倾斜时")
        self.stand_up(hold=0.2)

    def shift(self, dx: float = 0.05, dy: float = 0.0,
              hold: float = 0.6) -> None:
        """重心平移：机身水平移到脚上方偏 dx/dy 的位置（负 dx 为后坐）。"""
        self.ensure_standing()
        feet = self.controller.feet_for_body_pose(shift=(dx, dy))
        print(f"重心平移：dx={dx:+.2f} dy={dy:+.2f} m", flush=True)
        self.controller.apply_pose(self.controller.joint_from_feet(feet), hold_s=hold)
        self._report("平移时")
        self.stand_up(hold=0.2)

    def nod(self, times: int = 2, pitch: float = 0.12) -> None:
        """点头：低头-抬头循环，行程由一个连续的姿态插值完成。"""
        self.ensure_standing()
        print(f"点头 {times} 次，幅度 {math.degrees(pitch):.0f}°", flush=True)
        feet_low = self.controller.joint_from_feet(self.controller.feet_for_body_pose(pitch=pitch))
        feet_up = self.controller.joint_from_feet(self.controller.feet_for_body_pose(pitch=-pitch))
        for _ in range(max(1, times)):
            self.controller.apply_pose(feet_low, hold_s=0.05)
            self.controller.apply_pose(feet_up, hold_s=0.05)
        self.stand_up(hold=0.2)

    def lie_down(self, hold: float = 0.3) -> None:
        """蹲到最低再把腿松成阻尼，趴在地上。之后要动就重新调 stand_up。"""
        q = self.controller.joint_from_feet(self.controller.stand_feet(MIN_HEIGHT))
        self.controller.apply_pose(q, hold_s=hold)
        self.controller.shutdown()

    # -- 基础动作：步态类 ----------------------------------------------

    def step_in_place(self, seconds: float = 3.0) -> dict:
        """原地踏步：四个速度全 0，足端只上下动，用来验证步态相位。"""
        self.ensure_standing()
        print(f"原地踏步 {seconds:.0f}s", flush=True)
        return self.controller._run_gait(vx=0.0, vy=0.0, yaw=0.0,
                                timeout=0.8 + seconds)

    def walk_time(self, vx: float = 0.2, vy: float = 0.0, yaw: float = 0.0,
                  seconds: float = 3.0) -> dict:
        """按时间开环走，不保证距离（打滑、偏航都会累积误差）。"""
        return self.controller._run_gait(vx=vx, vy=vy, yaw=yaw, timeout=0.8 + seconds)

    # -- 任务：闭环 ----------------------------------------------------

    def forward(self, distance: float, vx: float = 0.3,
                tol: float = 0.08) -> dict:
        """向前走 distance 米（负值就是后退）。

        收步过程中还会滑行约 stop_ramp×speed ≈ 8 cm，所以默认留 8 cm 余量。
        """
        return self._translate(distance, 0.0, vx=vx, tol=tol)

    def strafe(self, distance: float, vy: float = 0.2,
               tol: float = 0.08) -> dict:
        """向左平移 distance 米（负值向右）。"""
        return self._translate(0.0, distance, vy=vy, tol=tol)

    def _translate(self, dx: float, dy: float, vx: float = 0.3,
                   vy: float = 0.2, tol: float = 0.08) -> dict:
        c = self.controller
        c.require_connection()
        self.ensure_standing()

        p0 = c.pos.copy() if c.pos is not None else None
        if p0 is None:
            raise RuntimeError("还没有 rt/sportmodestate，拿不到里程计就做不了闭环")

        yaw0 = c.rpy[2] if c.rpy else 0.0
        if abs(dx) >= abs(dy):
            axis, goal, speed = (math.cos(yaw0), math.sin(yaw0)), dx, vx
        else:
            axis, goal, speed = (-math.sin(yaw0), math.cos(yaw0)), dy, vy

        sign = 1.0 if goal >= 0 else -1.0
        along_x = abs(dx) >= abs(dy)
        cmd = dict(vx=sign * speed if along_x else 0.0,
                   vy=0.0 if along_x else sign * speed, yaw=0.0)
        target = abs(goal)

        def progress() -> float:
            d = c.pos - p0
            return d[0] * axis[0] + d[1] * axis[1]

        def done() -> bool:
            return progress() >= target - tol if sign > 0 else progress() <= -target + tol

        timeout = 0.8 + max(6.0, target / max(0.05, speed) * 2.0)
        print(f"任务：{'前进' if sign > 0 else '后退'} {target:.2f} m，"
              f"指令速度 {speed:.2f} m/s，超时 {timeout:.0f}s")
        out = c.trot_until(done, timeout=timeout, **cmd)
        out["progress"] = progress()
        out["target"] = target * sign
        self.history.append(("translate", out))
        return out

    def turn(self, degrees: float, yaw_rate: float = 0.5,
             tol: float = math.radians(10)) -> dict:
        """原地转 degrees 度，正为逆时针。

        开环偏航实测约打七折（指令 0.5 得 0.35 rad/s），闭环按实际 yaw 判断，
        所以慢一点但角度准。同样要留收步滑行的十来度余量。
        """
        c = self.controller
        c.require_connection()
        if not c.rpy:
            raise RuntimeError("还没有 IMU 姿态，转不了")
        self.ensure_standing()

        yaw0 = c.rpy[2]
        target = math.radians(degrees)
        sign = 1.0 if target >= 0 else -1.0

        def done() -> bool:
            delta = _wrap_pi(c.rpy[2] - yaw0)
            return delta * sign >= abs(target) - tol

        timeout = 0.8 + max(6.0, abs(target) / max(0.1, yaw_rate) * 2.5)
        print(f"任务：{'逆' if sign > 0 else '顺'}时针转 {abs(degrees):.0f}°，"
              f"角速度 {sign * yaw_rate:.2f} rad/s，超时 {timeout:.0f}s")
        out = c.trot_until(done, timeout=timeout, vx=0.0, vy=0.0,
                           yaw=sign * yaw_rate)
        out["turned_deg"] = math.degrees(_wrap_pi(c.rpy[2] - yaw0))
        self.history.append(("turn", out))
        return out

    def sequence(self, names) -> list[dict]:
        """按名字依次执行，名字就是本类的方法名。

        sequence(["stand_up", "squat", "forward", "turn", "lie_down"])
        需要参数的动作写 (名字, 字典)：("forward", {"distance": 2.0})
        """
        out = []
        for item in names:
            name, kw = (item if isinstance(item, tuple) else (item, {}))
            fn = getattr(self, name, None)
            if fn is None or not callable(fn):
                raise AttributeError(f"没有这个动作：{name}")
            print(f"\n--- {name} {kw if kw else ''} ---", flush=True)
            r = fn(**kw)
            out.append({name: r} if r is not None else {name: None})
        return out

    def stop(self) -> None:
        """急停：停回路并给腿下阻尼。之后要动就重新调动作，回路会自己重启。"""
        self.controller.shutdown()
