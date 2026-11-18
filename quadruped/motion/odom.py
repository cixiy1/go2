"""腿部里程计：不靠任何位姿话题，用四条腿的运动学反推机身走了多远。

为什么需要
----------
真机被 `lowcmd` 接管后，位姿话题会**同时**停发（2026-11-17 用
`test/pose_probe.py` 与一次真实接管并行实测）：

    rt/sportmodestate      298 Hz → 0
    rt/utlidar/robot_pose   19 Hz → 0
    rt/utlidar/robot_odom  150 Hz → 0
    rt/lf/sportmodestate    20 Hz → 0

不是某一个话题的问题，是整个 SLAM / 里程计服务认为「运动控制交出去了」。于是
`DirectLink.pos` 永远停在接管前那一帧，`memory` 只记得下起点那几个格子，VLA
每一拍都在「绕到墙后面那个点」—— 转十圈也走不出去。

原理
----
支撑腿的足端在世界系里是静止的（不打滑时），所以机身相对脚在动：

    v_body ≈ -d(足端在机体系的位置)/dt        （对支撑腿取平均）

再按 yaw 转到世界系积分。yaw 用 IMU 的 gyro z 积分（`rt/lowstate` 不受接管影响，
500 Hz 一直有）。`controller.py` 的 `_step_odom` 是同一个式子，区别是那边有步态
相位可以直接知道哪条腿在支撑，这里只能按足端高度猜。

打滑时足端相对机身几乎不动，估出来的速度也接近 0 —— 这比「拿指令速度当真值」
安全得多，不会越滑越猛。

为什么自带一份正运动学
----------------------
`quadruped/motion/` 不许 import `control/`（`test/arch_check.py` 规则 4，推理层
只依赖 LowLink）。所以腿长和髋位置在这里再写一遍，值取自 `control/model.py`，
那边是 MuJoCo 模型的唯一真源；改模型时两处要一起改，`test/odom.py` 拿两边对表。
"""

from __future__ import annotations

import math

# 连杆几何，与 `quadruped/control/model.py` 一致（MuJoCo go2.xml 的 body pos）。
HIP_X = 0.1934        # 髋部原点 |x|
HIP_Y = 0.0465        # 髋部原点 |y|
THIGH_Y = 0.0955      # 髋俯仰关节相对髋部原点的侧向偏移
THIGH_LEN = 0.213
CALF_LEN = 0.213

# 腿名 → (髋 x, 髋 y, 左右镜像符号)。顺序和 `LowState` 的 12 维一致：FR/FL/RR/RL。
LEGS = ("FR", "FL", "RR", "RL")
GEOM = {
    "FR": (+HIP_X, -HIP_Y, -1.0),
    "FL": (+HIP_X, +HIP_Y, +1.0),
    "RR": (-HIP_X, -HIP_Y, -1.0),
    "RL": (-HIP_X, +HIP_Y, +1.0),
}


def _rot_x(v, q):
    c, s = math.cos(q), math.sin(q)
    x, y, z = v
    return (x, y * c - z * s, y * s + z * c)


def _rot_y(v, q):
    c, s = math.cos(q), math.sin(q)
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)


def foot_position(leg: str, q) -> tuple[float, float, float]:
    """足端在机体系的位置。q = (abduction, thigh, calf)。"""
    q1, q2, q3 = q
    hx, hy, s = GEOM[leg]

    calf = _rot_y((0.0, 0.0, -CALF_LEN), q3)
    in_thigh = (calf[0], calf[1], -THIGH_LEN + calf[2])

    in_hip = _rot_y(in_thigh, q2)
    lateral = _rot_x((0.0, s * THIGH_Y, 0.0), q1)
    foot = _rot_x(in_hip, q1)
    return (hx + lateral[0] + foot[0],
            hy + lateral[1] + foot[1],
            lateral[2] + foot[2])


def knee_position(leg: str, q) -> tuple[float, float, float]:
    """膝关节在机体系的位置。腿的上下两段在这里拐弯，剔自身点要用它。"""
    q1, q2, _ = q
    hx, hy, s = GEOM[leg]
    thigh = _rot_y((0.0, 0.0, -THIGH_LEN), q2)
    lateral = _rot_x((0.0, s * THIGH_Y, 0.0), q1)
    bent = _rot_x(thigh, q1)
    return (hx + lateral[0] + bent[0],
            hy + lateral[1] + bent[1],
            lateral[2] + bent[2])


# 四条腿的线段：髋 → 膝 → 足。剔自身点时按「点离任何一段多近」判。
def leg_segments(q12) -> list[tuple[tuple, tuple]]:
    q12 = list(q12)
    out = []
    for i, leg in enumerate(LEGS):
        q = q12[3 * i:3 * i + 3]
        hx, hy, _ = GEOM[leg]
        hip = (hx, hy, 0.0)
        knee = knee_position(leg, q)
        foot = foot_position(leg, q)
        out.append((hip, knee))
        out.append((knee, foot))
    return out


class FootOdometer:
    """把 12 个关节角和 gyro z 积分成 (x, y, yaw)。

    `update()` 要按固定节奏调（50 Hz 合适）：差分对噪声敏感，频率太低会把
    一步之内的往返抵消掉，太高则放大量化噪声。
    """

    # 判定支撑：足端离「本帧最低那只脚」高出不到这么多，就算踩在地上。
    #
    # 用相对量而不是绝对值，是因为机身高度会变（站 0.30、走起来沉 1 cm），
    # 绝对阈值一改高度就失效。3.5 cm 小于摆动相的抬腿高度（6 cm）的一半，
    # 又大于地面起伏和关节噪声 —— 两条腿同时踩地的过渡相也能覆盖到。
    STANCE_EPS = 0.035

    # 速度一阶低通的时间常数（秒）。5 ms 的差分会把关节编码器的量化噪声
    # 放大成十几 cm/s 的抖动，平滑到 50 ms 量级才像速度。
    TAU = 0.05

    # 单帧 dt 超过这个值就跳过：中间断了（线程卡住、DDS 丢包），差分会
    # 变成一次巨大的假位移。
    MAX_DT = 0.10

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0                   # 机体系速度，只用来算 dt 内的位移
        self.vy = 0.0
        self.ok = False
        self.frames = 0
        self._prev = None               # 上一帧的足端位置
        self._t = 0.0

    def update(self, q12, gyro_z: float, now: float) -> None:
        q12 = list(q12)
        if len(q12) < 12:
            return
        feet = {leg: foot_position(leg, q12[3 * i:3 * i + 3])
                for i, leg in enumerate(LEGS)}

        prev, self._prev = self._prev, feet
        t0, self._t = self._t, now
        dt = now - t0
        if prev is None or not (1e-4 < dt < self.MAX_DT):
            self.ok = False
            return

        # 支撑腿：足端最低的那几只。走起来总有一只在地上；四脚腾空的瞬间
        # （小跑）这里会退化成「全部算支撑」，速度被平均掉一部分，可接受。
        lows = [feet[leg][2] for leg in LEGS]
        floor = min(lows)
        stance = [leg for leg in LEGS if feet[leg][2] <= floor + self.STANCE_EPS]
        if not stance:
            self.ok = False
            return

        sx = sy = 0.0
        for leg in stance:
            sx += feet[leg][0] - prev[leg][0]
            sy += feet[leg][1] - prev[leg][1]
        n = float(len(stance))
        # 脚相对机身往后走 = 机身往前走。
        raw_vx = -sx / n / dt
        raw_vy = -sy / n / dt

        a = min(1.0, dt / self.TAU)
        self.vx += a * (raw_vx - self.vx)
        self.vy += a * (raw_vy - self.vy)

        # yaw 用 gyro 积分：腿部运动学算不出机身绕 z 的转动（四只脚对称转，
        # 足端相对机身的位置几乎不变），这一项必须靠 IMU。
        self.yaw += float(gyro_z) * dt

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (self.vx * c - self.vy * s) * dt
        self.y += (self.vx * s + self.vy * c) * dt
        self.ok = True
        self.frames += 1

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)
