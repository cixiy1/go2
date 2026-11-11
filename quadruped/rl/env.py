"""进程内 MuJoCo 环境：不经 DDS，不开窗口，快于实时。

为什么不复用 `Go2Controller` 的回路：它的 200 Hz 线程里挂着 MPC（50 Hz 线程）
与 WBC（QP 求解），跑在快于实时的仿真里会自己追自己。所以这里单独实现一条
200 Hz 控制链：相位 → 足端目标 → IK → PD + 重力前馈 → 力矩。

物理 400 Hz（dt 2.5 ms）、控制 200 Hz（每 2 步一次）。部署侧是 500 Hz 物理，
差异在接触硬度上，5° 坡这类工况要回 DDS 仿真复核。
"""
from __future__ import annotations

import functools
import os

import mujoco
import numpy as np

from ..control.controller import (CTRL_LEGS, LEGS, NOMINAL_HEIGHT,
                                  Go2Controller, MODEL_PATH)
from .gait import FootPlanner, GaitSpec

CTRL_HZ = 200.0
PHYS_DT = 0.0025
DECIM = 2                      # 每个控制步推进的物理步数 → 控制频率 1/(2*2.5ms)

KP = 100.0                     # 与 KP_REAL 一致：搜索与部署同一套刚度
KD = 1.5
TAU_FF_GAIN = 1.0              # 与 TAU_FF_REAL 一致
TAU_MAX = 12.0
TAU_TAU = 0.04                 # 前馈一阶低通，同 controller.py
FALL_HEIGHT = 0.15
FALL_TILT = 0.90               # 约 52°


# 生成场景：go2.xml 里只有机器人本体，没有地面，直接拿它跑会自由落体。
#
# 不用写临时 xml —— `<include>` 会把 asset 的相对路径解析到临时文件所在目录，
# 补个地面就得把 base_0.obj 一起搬过去。MjSpec 在编译前加 geom 没这个问题。
#
# 横坡靠给平面加绕 x 的旋转，平地与坡面因此共用同一份机器人模型。
#
# 模型编译约 0.2 s，一次搜索要评估上千个 episode，所以按 (坡度, 路径) 缓存；
# MjModel 只读，可以被多个 MjData 共享。
@functools.lru_cache(maxsize=8)
def build_model(slope_deg: float = 0.0, model_path: str = MODEL_PATH,
                rough: float = 0.0, rough_seed: int = 0):
    spec = mujoco.MjSpec.from_file(model_path)
    g = spec.worldbody.add_geom()
    g.name = "floor"
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [20.0, 20.0, 0.05]
    g.friction = [1.0, 0.005, 0.0001]
    if slope_deg:
        a = np.deg2rad(slope_deg) * 0.5
        g.quat = [float(np.cos(a)), float(np.sin(a)), 0.0, 0.0]
    # 地面起伏：铺一层高度随机的方块。平地不需要离地间隙，所以优化器会把
    # swing_height 压到 5 cm 也不付代价 —— 那在真实地面上是绊脚的直接原因。
    # 铺上起伏之后，抬不够就会踢到台面，代价才出现。
    if rough > 0.0:
        rng = np.random.default_rng(rough_seed)
        tile = 0.20
        # 起步区留平（x<0.6）：初始足端在 z≈0.022，方块最高 2 cm，
        # 铺到脚下会让四只脚开局就插进方块的边里，一步都走不动。
        for i in range(3, 16):             # x：0.6 ~ 4.0 m
            for j in range(-3, 4):         # y：-0.7 ~ 0.7 m
                h = float(rng.uniform(0.0, rough))
                b = spec.worldbody.add_geom()
                b.name = f"tile_{i}_{j}"
                b.type = mujoco.mjtGeom.mjGEOM_BOX
                b.size = [tile / 2, tile / 2, max(h, 1e-3) / 2]
                b.pos = [i * tile, j * tile, max(h, 1e-3) / 2]
                b.friction = [1.0, 0.005, 0.0001]
    return spec.compile()


# 四元数 (w,x,y,z) → 旋转矩阵，避免为这一个函数引入 scipy
def quat_to_R(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# 欧拉角 (roll, pitch)，只取判断摔倒需要的两个角
def roll_pitch(q) -> tuple[float, float]:
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return float(roll), float(pitch)


class GaitEnv:
    """一套步态跑一个固定时长，回收指标。

        env = GaitEnv(vx=0.25, seconds=6.0)
        m = env.run(GaitSpec.from_vector(x))
        m["score"]      # 供优化器用的标量
    """

    def __init__(self, vx: float = 0.25, seconds: float = 6.0,
                 height: float = NOMINAL_HEIGHT, settle: float = 0.4,
                 model_path: str = MODEL_PATH, seed: int = 0,
                 slope_deg: float = 0.0, stance_gate: bool = True,
                 motor_bw: float = 0.0, kp: float = KP, kd: float = KD,
                 w_speed: float = 0.45, w_stab: float = 0.30,
                 w_yaw: float = 0.0, push: float = 0.0,
                 rough: float = 0.0, rough_seed: int = 0,
                 slip_asym: float = 0.0):
        # 接触不对称：每次落地给一个随机偏航冲量（N·m·s）。真机的四个脚垫
        # 不会完全一样，落地瞬间的偏航力矩也就不会对称；仿真里完全对称，
        # 所以 yaw 恒为 0，优化器学不到「这套步态经不经得住偏航扰动」。
        self.slip_asym = float(slip_asym)
        self.rough = float(rough)
        self._prev_stance: dict = {}
        # 侧向冲量（牛顿·秒）。仿真左右对称、地面均匀，不推它的话 yaw 永远
        # 是 0 —— 优化器看不到「这套步态一被侧推就打转」，搜出来的参数搬到
        # 真实接触上就原地转圈。推一下，让 yaw 项有东西可惩罚。
        self.push = float(push)
        self.w_yaw = float(w_yaw)
        # kp/kd 必须与部署侧同源。搜索时定多少，跑的时候就得给多少 ——
        # 中途换一套，搜出来的参数对着的是另一个被控对象。
        self.kp = float(kp)
        self.kd = float(kd)
        self.w_speed = float(w_speed)
        self.w_stab = float(w_stab)
        self.motor_bw = float(motor_bw)     # 力矩一阶滞后带宽 Hz，0 = 理想
        self.vx = float(vx)
        self.seconds = float(seconds)
        self.height = float(height)
        self.settle = float(settle)      # 起立过渡时间，不计指标
        self.rng = np.random.default_rng(seed)
        self.slope_deg = float(slope_deg)
        # 复刻 controller.py 里那道「足端抬高就削支撑权重」的闸门。
        # 关掉它就是纯相位判支撑 —— 两者的差别足以让搜出来的参数搬回控
        # 制器后表现不同，所以默认开着，和部署侧一致。
        self.stance_gate = bool(stance_gate)

        self.model = build_model(slope_deg, model_path, self.rough, rough_seed)
        self.model.opt.timestep = PHYS_DT
        self.data = mujoco.MjData(self.model)
        self._build_index()

    # 执行器 → 关节 → qpos/dof 的下标映射。
    #
    # 不能按下标顺序硬套：actuator 是 FR/FL/RR/RL，而 xml 里 joint 的定义
    # 顺序是 FL/FR/RL/RR，两边错一位就是左右腿对调。
    def _build_index(self):
        m = self.model
        self._bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.qadr = np.zeros(12, dtype=int)
        self.vadr = np.zeros(12, dtype=int)
        for i in range(12):
            jid = int(m.actuator_trnid[i, 0])
            self.qadr[i] = int(m.jnt_qposadr[jid])
            self.vadr[i] = int(m.jnt_dofadr[jid])

    # 回到 home 关键帧，站姿 0.27 m、零速度、零力矩。
    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.qpos[0:3] = [0.0, 0.0, self.height]
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.xfrc_applied[:] = 0.0
        self._prev_stance = {}
        mujoco.mj_forward(self.model, self.data)
        self.t = 0.0
        self._tau_ff = np.zeros(12)
        self._tau_now = np.zeros(12)

    # 当前 12 个关节角/角速度，按 CTRL_LEGS 顺序。
    def joints(self) -> tuple[np.ndarray, np.ndarray]:
        d = self.data
        q = np.array([d.qpos[a] for a in self.qadr])
        dq = np.array([d.qvel[a] for a in self.vadr])
        return q, dq

    # 机体系线速度（x 前 y 左）。
    def body_vel(self) -> tuple[float, float]:
        R = quat_to_R(self.data.qpos[3:7])
        v = R.T @ self.data.qvel[0:3]
        return float(v[0]), float(v[1])

    # 偏航角，直接由四元数算，避免欧拉角在 ±π 附近跳变。
    def _yaw(self) -> float:
        w, x, y, z = self.data.qpos[3:7]
        return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    # 一个控制步宽的侧向冲量 + 偏航冲量，加在 base_link 上。
    #
    # 冲量大小按牛顿·秒给，除以这一步的时长换成力，所以 `push=2.0` 就是
    # 2 N·s —— 15 kg 机身被横向踢出 0.13 m/s，量级上相当于撞一下桌角。
    def _kick(self):
        dt = DECIM * PHYS_DT
        f = self.push / dt
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.data.xfrc_applied[bid, :] = [0.0, f, 0.0, 0.0, 0.0, f * 0.04]

    def _unkick(self):
        self.data.xfrc_applied[:] = 0.0

    # 一个控制步：算足端 → IK → 力矩 → 推进 DECIM 个物理步。
    def tick(self, planner: FootPlanner):
        q, dq = self.joints()
        vx_meas, _ = self.body_vel()
        feet, stance = planner.feet(self.t, self.vx, 0.0, vx_meas)

        # 落地瞬间的偏航冲量。只在这一帧给，下一帧清掉 —— 等于一次冲量。
        if self.slip_asym:
            self.data.xfrc_applied[:] = 0.0
            for leg in LEGS:
                down = bool(stance[leg])
                if down and not self._prev_stance.get(leg, False):
                    # slip_asym 按 N·m·s 给，除以这一帧的时长换成力矩
                    self.data.xfrc_applied[self._bid, 5] += float(
                        self.rng.normal(0.0, self.slip_asym)) / (DECIM * PHYS_DT)
                self._prev_stance[leg] = down

        q_des = np.zeros(12)
        for i, leg in enumerate(CTRL_LEGS):
            r = Go2Controller.ik_leg(leg, feet[leg])
            q_des[3 * i:3 * i + 3] = r.q if r.ok else q[3 * i:3 * i + 3]

        # 支撑权重，与 controller.py:_gait_feet_publish 同一套：
        #   窗口 [-pre, duty]，落地前 pre 开始预载，离地那一帧撤完；
        #   再按实测足端抬高量掐一道闸门。
        # 二值相位判支撑会少掉「预载」这一段 —— 少了它，脚是落地后才开始
        # 给力，控制器是落地前 0.1 周期就在往下压，两者的竖直力剖面不同。
        pre = 0.1
        spec = planner.spec
        win = spec.duty + pre
        w = {}
        for i, leg in enumerate(CTRL_LEGS):
            ph = (self.t / spec.period + planner.offsets[leg]) % 1.0
            ps = (ph + pre) % 1.0
            wst = min(1.0, ps / pre, (win - ps) / pre) if ps < win else 0.0
            if wst > 0.0 and self.stance_gate:
                p_now = Go2Controller.fk_leg(leg, q[3 * i:3 * i + 3])
                lift = float(p_now[2]) - float(planner.neutral[leg][2])
                wst *= float(np.clip(
                    (Go2Controller.STANCE_GATE_HI - lift)
                    / Go2Controller.STANCE_GATE_W, 0.0, 1.0))
            w[leg] = wst

        raw = Go2Controller.gravity_tau(q, w) * TAU_FF_GAIN
        k = 1.0 / (1.0 + CTRL_HZ * TAU_TAU)
        self._tau_ff += k * (raw - self._tau_ff)
        tau_ff = np.clip(self._tau_ff, -TAU_MAX, TAU_MAX)

        tau = self.kp * (q_des - q) + self.kd * (0.0 - dq) + tau_ff

        # 电机力矩带宽。真机驱动器跟不上 200 Hz 的指令跳变，一阶滞后是最小
        # 可信模型；不加这一层，仿真里的力矩当帧到位，任何步态都显得稳。
        if self.motor_bw > 0.0:
            a = 1.0 / (1.0 + CTRL_HZ / self.motor_bw)
            tau = self._tau_now + a * (tau - self._tau_now)
            self._tau_now = tau

        self.data.ctrl[:] = np.clip(tau, -23.7, 23.7)

        for _ in range(DECIM):
            mujoco.mj_step(self.model, self.data)
        self.t += DECIM * PHYS_DT

    # 跑完一个 episode，返回指标字典（键名与 test/gait_speed.py 对齐）。
    def run(self, spec: GaitSpec) -> dict:
        self.reset()
        planner = FootPlanner(spec, self.height)

        n_total = int(self.seconds * CTRL_HZ)
        n_settle = int(self.settle * CTRL_HZ)
        x0 = float(self.data.qpos[0])

        rolls, pitches, heights, taus = [], [], [], []
        vx_tail, dev_tail = [], []
        fell = False
        yaw0 = self._yaw()
        yaw_tail = []
        push_at = n_settle + int(0.8 * CTRL_HZ)

        for i in range(n_total):
            if self.push and i == push_at:
                self._kick()
            self.tick(planner)
            if self.push and i == push_at:
                self._unkick()
            d = self.data
            r, p = roll_pitch(d.qpos[3:7])
            h = float(d.qpos[2])

            if i >= n_settle:
                rolls.append(r)
                pitches.append(p)
                heights.append(h)
                taus.append(float(np.abs(self.data.ctrl).max()))
                vxm, _ = self.body_vel()
                vx_tail.append(vxm)
                dev_tail.append(abs(vxm - self.vx))
                yaw_tail.append(abs(self._yaw() - yaw0))

            if h < FALL_HEIGHT or abs(r) > FALL_TILT or abs(p) > FALL_TILT:
                fell = True
                break

        dx = float(self.data.qpos[0]) - x0
        dur = max(self.t - self.settle, 1e-6)
        return self._metrics(spec, dx, dur, rolls, pitches, heights, taus,
                             vx_tail, dev_tail, yaw_tail, fell, n_total)

    # 指标汇总。评分项全部归一到 [0,1]，权重在 score() 里给。
    def _metrics(self, spec, dx, dur, rolls, pitches, heights, taus,
                 vx_tail, dev_tail, yaw_tail, fell, n_total) -> dict:
        def pp(x):
            return float(max(x) - min(x)) if x else 9.9

        vx_mean = float(np.mean(vx_tail)) if vx_tail else 0.0
        m = dict(
            period=spec.period, duty=spec.duty,             vx_cmd=self.vx,
            vx=vx_mean,
            vx_ratio=float(np.clip(vx_mean / self.vx, 0.0, 1.2)) if self.vx else 0.0,
            dx=dx, dur=dur,
            roll_pp=pp(rolls), pitch_pp=pp(pitches),
            h_mean=float(np.mean(heights)) if heights else 0.0,
            h_min=float(np.min(heights)) if heights else 0.0,
            tau_p95=float(np.percentile(taus, 95)) if taus else 99.0,
            slip=float(np.mean(dev_tail) / self.vx) if (dev_tail and self.vx) else 9.9,
            yaw_drift=float(max(yaw_tail)) if yaw_tail else 0.0,
            fell=bool(fell), t_end=round(self.t, 3),
            done_ratio=len(rolls) / max(n_total - int(self.settle * CTRL_HZ), 1),
        )
        m["score"] = score(m, w_speed=self.w_speed, w_stab=self.w_stab,
                           w_yaw=self.w_yaw)
        return m


# 评分：把四个指标压成一个标量。权重决定搜出来是什么风格。
#
# 速度占主导（0.45），因为「走不动」是最常见的失败；姿态与高度是惩罚项，
# 摔倒直接压到 0.05 —— 不给负分，避免优化器学会「用摔跤换速度」。
#
# 想换风格只改这里：要更稳就抬 w_stab，要更快就抬 w_speed。
def score(m: dict, w_speed=0.45, w_stab=0.30, w_torque=0.10, w_height=0.15,
          w_yaw: float = 0.0) -> float:
    if m["fell"]:
        return 0.05
    s_speed = m["vx_ratio"] if m["vx_ratio"] < 1.0 else 1.0 - 0.5 * (m["vx_ratio"] - 1.0)
    s_stab = float(np.exp(-m["roll_pp"] / 0.12) * np.exp(-m["pitch_pp"] / 0.20))
    s_torque = float(np.exp(-max(m["tau_p95"] - 4.0, 0.0) / 6.0))
    s_height = float(np.exp(-abs(m["h_mean"] - NOMINAL_HEIGHT) / 0.03))
    # 偏航保持。0.20 rad 的偏航漂移（11°）就给 0.37 分，够狠才压得住
    # 「原地打转也算前进」这种解。
    s_yaw = float(np.exp(-m.get("yaw_drift", 0.0) / 0.20))
    # 开了 yaw 项就按比例压缩其余四项，总分范围不变，避免权重语义漂移。
    k = 1.0 - w_yaw
    return float(k * (w_speed * s_speed + w_stab * s_stab
                      + w_torque * s_torque + w_height * s_height)
                 + w_yaw * s_yaw)


# 命令行自检：跑一套基准步态，打印指标。
def _main(argv=None) -> int:
    import argparse
    from .gait import BASELINE

    ap = argparse.ArgumentParser(description="单个 episode 自检")
    ap.add_argument("--vx", type=float, default=0.25)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--spec", default="baseline",
                    choices=("baseline", "mech", "bio"))
    a = ap.parse_args(argv)

    if a.spec == "mech":
        from ..control.controller import make_gait
        g = make_gait("mech")
        spec = GaitSpec(period=g.period, duty=g.duty,
                        swing_height=g.swing_height, stride_max=g.stride_max)
    elif a.spec == "bio":
        from ..control.controller import make_gait
        g = make_gait("bio")
        spec = GaitSpec(period=g.period, duty=g.duty,
                        swing_height=g.swing_height, stride_max=g.stride_max,
                        stride_gain=g.stride_gain, speed_loop=g.speed_loop,
                        stance_sink=g.stance_sink)
    else:
        spec = GaitSpec(**BASELINE)

    env = GaitEnv(vx=a.vx, seconds=a.seconds)
    m = env.run(spec)
    print(f"{os.path.basename(__file__)} vx={a.vx:.2f} {a.seconds:.1f}s")
    for k in ("vx", "vx_ratio", "dx", "roll_pp", "pitch_pp", "h_mean",
              "tau_p95", "slip", "fell", "score"):
        v = m[k]
        print(f"  {k:9s} {v:.4f}" if isinstance(v, float) else f"  {k:9s} {v}")
    return 0 if not m["fell"] else 1


if __name__ == "__main__":
    raise SystemExit(_main())
