#!/usr/bin/env python3
"""无界面闭环仿真：把**真实的** `Go2Controller` 接到 MuJoCo 上，不走 DDS。

为什么要有它：`run_sim.sh` 走 mjpython + 图形窗口（macOS 要求 GUI 在主线程），
脚本和 CI 里都跑不了；而力路的验收（站 / 走 / 侧坡 / 侧推、力矩帧间差、
`loop_stats()`）必须闭环才测得出。这里复用同一个 `go2.xml`、同一套桥的 PD
与真机化处理（5 ms 延迟、150 Hz 力控带宽、编码器噪声），只把 DDS 收发换成
直接读写 `mjData` —— 控制器那一侧**一行代码都不改**，`_tick()` 照常 200 Hz。

    uv run python test/sim_check.py --mode vmc --action walk
    uv run python test/sim_check.py --mode mpc --action walk
    uv run python test/sim_check.py --ab --action walk      # 一次跑两条力路

几处刻意的做法：

- **物理步长 5 ms**，和 `config.SIMULATE_DT` 一致，一帧一步；控制器 200 Hz。
- **障碍课程关掉**。`scene.xml` 从 x=1.2 起是一串台阶（给爬台阶用的），
  0.3 m/s 走 6 秒正好撞上去，A/B 就变成比谁先撞台阶了。只关 worldbody 上
  那几个 box 的碰撞，机器人自己的碰撞一个不动。
- **侧坡用「地面绕 x 转 + 机身同步转」**，重力保持竖直。等价于把整个世界
  绕坡轴转回去，物理上和「站在 5° 斜坡上」完全一样，但省掉一套坡面几何。
- 时间按墙钟走：`_gait_tick` 的相位时钟本来就是 `time.time()`，而 MPC 跑在
  独立线程里 —— 加速跑会同时改变两者的时间尺度，测出来的东西不可信。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "quadruped", "sim", "unitree_mujoco",
                     "unitree_robots", "go2", "scene.xml")

NUM = 12
DT = 0.005          # 物理步长 = 控制周期，与 config.SIMULATE_DT 一致


# ---------------------------------------------------------------------------
# 姿态换算（与 unitree_sdk2py_bridge.py 里那段逐字一致）
# ---------------------------------------------------------------------------

def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def rpy_from_quat(qw, qx, qy, qz):
    return (math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)),
            math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))),
            math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


# ---------------------------------------------------------------------------
# 仿真本体
# ---------------------------------------------------------------------------

class Sim:
    """一个 MuJoCo 实例 + 一份「和仿真桥一样的」执行器模型。"""

    def __init__(self, slope: float = 0.0, realism: bool = True):
        """`slope` 是**度**（和命令行一致）。"""
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = DT
        self.n = self.m.nu

        mj = mujoco
        self.floor = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_GEOM, "floor")
        # 只关 worldbody 上的装饰性 box（scene.xml 里那串台阶）。机器人的 geom
        # 全挂在 base_link 与各连杆下面，bodyid != 0，一个都不碰。
        for gid in range(self.m.ngeom):
            if self.m.geom_bodyid[gid] != 0 or gid == self.floor:
                continue
            self.m.geom_contype[gid] = 0
            self.m.geom_conaffinity[gid] = 0
        self.base = mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_BODY, "base_link")

        mj.mj_resetDataKeyframe(self.m, self.d, 0)
        # 命令行给的是度，几何用弧度。**这里必须换**：直接把 5 度当弧度用，
        # 地面就绕 x 转了 286°，反过来了 —— 四条腿踩在背面上，狗当场滚翻，
        # A/B 两条力路都 roll 360°、位移 0。
        self.slope = math.radians(float(slope))
        if self.slope:
            # 地面绕 x 转 slope，机身同步转，重力仍竖直 —— 这就是一个侧坡。
            q = quat_from_rpy(self.slope, 0.0, 0.0)
            self.m.geom_quat[self.floor] = q
            self.d.qpos[3:7] = q
            self.d.qpos[2] = 0.27 + self._slope_lift(q)
        mj.mj_forward(self.m, self.d)

        # 传感器地址按名字查，不靠顺序 —— 少一条 sensor 就会全错位。
        adr = lambda nm: self.m.sensor_adr[
            mj.mj_name2id(self.m, mj.mjtObj.mjOBJ_SENSOR, nm)]
        self.a_q = adr("FR_hip_pos")
        self.a_dq = adr("FR_hip_vel")
        self.a_tau = adr("FR_hip_torque")
        self.a_quat = adr("imu_quat")
        self.a_gyro = adr("imu_gyro")
        self.a_acc = adr("imu_acc")
        self.a_pos = adr("frame_pos")
        self.a_vel = adr("frame_vel")

        # 真机化：5 ms 延迟（正好一帧）+ 150 Hz 一阶滞后 + 编码器噪声。
        self.realism = realism
        self.lat = max(1, int(round(0.005 / DT))) if realism else 0
        self.alpha = (1.0 - math.exp(-2.0 * math.pi * 150.0 * DT)
                      if realism else 1.0)
        self.rng = np.random.default_rng(7)
        self.sig_q = 0.0015 if realism else 0.0
        self.sig_dq = 0.15 if realism else 0.0
        self.pend: list = []
        self.tau_f = np.zeros(self.n)

    # 坡面：法线 n = R·ẑ 绕原点转出来的平面，过原点时 z = −(n_x x + n_y y)/n_z。
    # 先把机身放在原点算出「最低那只脚」相对坡面的间隙，再整体抬到 +1 cm。
    def _slope_lift(self, quat):
        n = np.zeros(3)
        self.mj.mju_rotVecQuat(n, np.array([0.0, 0.0, 1.0]), quat)
        R = np.zeros(9)
        self.mj.mju_quat2Mat(R, quat)
        R = R.reshape(3, 3)
        gap = []
        for p in _home_feet().values():
            pw = R @ p
            gap.append(pw[2] - (-(n[0] * pw[0] + n[1] * pw[1]) / n[2]))
        return -min(gap) + 0.01

    # 把机身抬到「没有任何 geom 穿透地面」的位置。**必须按真实碰撞算，不能按
    # 正解估**：`_slope_lift` 用的是 xml 关键帧姿态 (0, 0.9, −1.8) 的足端高度，
    # 而实际下发的是 `stand_joint_target()` (0, 0.949, −1.899) —— 腿更直、足端
    # 更低 3 cm，于是四条腿整个扎进坡面。求解器一上来就给几百牛的接触力，狗
    # 当场滚翻（实测 VMC / MPC 都 roll 360°、位移 0，谁也没跑起来 —— 这种
    # 「两边一样烂」的结果先怀疑测试台，不要怀疑控制器）。
    def settle(self, clearance: float = 0.005) -> None:
        mj, m, d = self.mj, self.m, self.d
        for _ in range(8):
            mj.mj_forward(m, d)
            if d.ncon == 0:
                break
            worst = min(float(d.contact[i].dist) for i in range(d.ncon))
            if worst >= clearance:
                break
            d.qpos[2] += (clearance - worst)
        mj.mj_forward(m, d)

    def state(self):
        sd = self.d.sensordata
        n = self.n
        q = np.array(sd[self.a_q:self.a_q + n])
        dq = np.array(sd[self.a_dq:self.a_dq + n])
        tau = np.array(sd[self.a_tau:self.a_tau + n])
        quat = np.array(sd[self.a_quat:self.a_quat + 4])
        gyro = np.array(sd[self.a_gyro:self.a_gyro + 3])
        acc = np.array(sd[self.a_acc:self.a_acc + 3])
        pos = np.array(sd[self.a_pos:self.a_pos + 3])
        vel = np.array(sd[self.a_vel:self.a_vel + 3])
        # 桥只把噪声加在**控制器看到的**读数上，PD 自己用的是干净值（见
        # unitree_sdk2py_bridge.LowCmdHandler / PublishLowState）。这里照抄。
        qn = q + self.rng.normal(0.0, self.sig_q, n)
        dqn = dq + self.rng.normal(0.0, self.sig_dq, n)
        # MuJoCo 的 accelerometer 是「世界系、不含重力」，真机是「机体系、含
        # 重力」的比力。桥里做过这一步换算，这里同样做。
        R = _R(*rpy_from_quat(*quat))
        acc_b = R.T @ (acc + np.array([0.0, 0.0, 9.81]))
        return dict(q=q, dq=dq, q_n=qn, dq_n=dqn, tau=tau, quat=quat,
                    rpy=rpy_from_quat(*quat), gyro=gyro, acc=acc_b,
                    pos=pos, vel=vel)

    def apply(self, q_des, kp, kd, tau):
        sd = self.d.sensordata
        n = self.n
        q = np.array(sd[self.a_q:self.a_q + n])
        dq = np.array(sd[self.a_dq:self.a_dq + n])
        raw = tau + kp * (q_des - q) + kd * (0.0 - dq)
        if not self.realism:
            self.tau_f = raw
            self.d.ctrl[:] = raw
            return
        self.pend.append(raw)
        out = self.pend.pop(0) if len(self.pend) > self.lat else self.pend[0]
        self.tau_f += self.alpha * (out - self.tau_f)
        self.d.ctrl[:] = self.tau_f

    def step(self):
        self.mj.mj_step(self.m, self.d)

    def push(self, fy: float):
        self.d.xfrc_applied[self.base, :] = (0.0, fy, 0.0, 0.0, 0.0, 0.0)


# 按绝对时刻节拍。**这里刻意用纯 sleep，不自旋**：自旋能消掉宿主定时器的
# 2~5 ms 合并延迟（实测超时帧 92 → 23），但它会把 CPU 占满，反过来让 MPC
# 线程抢 `_tick()` 的时间（单帧计算 p95 从 0.9 涨到 5.8 ms）。两个数都想看
# 干净，就得二选一；算力预算是主线，所以保留纯 sleep，回路周期的抖动单独
# 在 `show()` 里报，并在验收记录里写明成因。
def _pace(nxt: float) -> None:
    rest = nxt - time.perf_counter()
    if rest > 0:
        time.sleep(rest)


def _R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _home_feet():
    from quadruped.control.controller import Go2Controller, CTRL_LEGS
    q = np.array([0.0, 0.9, -1.8] * 4)
    return {leg: np.array(Go2Controller.fk_leg(leg, q[3 * i:3 * i + 3]))
            for i, leg in enumerate(CTRL_LEGS)}


# ---------------------------------------------------------------------------
# 跑一次
# ---------------------------------------------------------------------------

def run(mode: str, action: str, vx: float, dur: float, slope: float,
        push_n: float, kp: float | None, style: str, gait_over: dict,
        realism: bool = True, wrench: str = "mpc",
        burst: bool = False) -> dict:
    from quadruped.control.controller import (CTRL_LEGS, SPORT_Z_BIAS,
                                              Go2Controller, make_gait)

    gait = make_gait(style, mode=mode)
    for k, v in gait_over.items():
        setattr(gait, k, v)
    if "duty" in gait_over or "period" in gait_over:
        from quadruped.control.controller import _swing_poly_coeffs
        gait.a3, gait.a4 = _swing_poly_coeffs(gait.duty)

    # `real=True` 而不是 False：`Go2Robot` 的默认就是 `real_params=True`
    # （见 body.py），仿真器里跑的也是这一套 —— kp 100 / kd 1.5 / 前馈 1.0 /
    # 支撑腿 kp×0.5 / 高度用运动学而不是 sportmodestate。用 `real=False`
    # 那套（kp 120 / 无前馈）测出来的东西和实际部署不是一回事。
    c = Go2Controller(domain=1, iface="lo0", gait=gait, real=True,
                      **({} if kp is None else {"kp": kp}))
    sim = Sim(slope, realism)

    # 站姿用控制器自己算的那个（IK 出来的），不是 xml 的 home 关键帧 ——
    # 两者差着足端半径和站宽，闭环里的高度基准完全是另一套。
    q0 = c.stand_joint_target()
    sim.d.qpos[7:7 + NUM] = q0
    sim.d.qvel[:] = 0.0
    sim.mj.mj_forward(sim.m, sim.d)
    # 平地上正解和真值对得上，坡上不一定 —— 一律按真实穿透抬一遍。
    sim.settle()
    c.q = q0.copy()
    c.dq = np.zeros(NUM)
    c.tau_est = np.zeros(NUM)
    c._mode = "gait"
    c._gait = dict(stream=True, ramp=0.25, gait_ramp=0.5, stop_ramp=0.45,
                   t0=time.time(), last_t=0.0, q_init=q0.copy(),
                   q_stand=q0.copy(),
                   p_nom=c.stand_feet(c.height, gait.stance_spread),
                   last=q0.copy(), vel=[0.0, 0.0, 0.0], a=0.0,
                   accel=[0.0, 0.0, 0.0], started=False, n=0, fail=0)

    cmd = dict(q=q0.copy(), kp=c.kp.copy(), kd=c.kd.copy(),
               tau=np.zeros(NUM))

    def fake_publish(q_target, kp=None, kd=None, tau=None):
        cmd["q"] = np.array(q_target, dtype=float)
        cmd["kp"] = c.kp.copy() if kp is None else np.array(kp, dtype=float)
        cmd["kd"] = c.kd.copy() if kd is None else np.array(kd, dtype=float)
        cmd["tau"] = (np.zeros(NUM) if tau is None
                      else np.array(tau, dtype=float))

    c._publish = fake_publish       # 实例属性覆盖方法，DDS 完全不参与

    # 和 `start_velocity` 一样：构造放在进循环之前。否则第一帧在 200 Hz 回路
    # 里现场编译 QP，那一帧要 300 ms 以上。
    if mode == "mpc":
        c._ensure_mpc()

    if mode == "mpc" and wrench == "fixed":
        # 隔离用：MPC 不参与了，直接给常数 wrench（只承重、不纠姿态）。
        # 它把「WBC 能不能跟住一个已知的力」和「MPC 给得对不对」分开 ——
        # 闭环出问题时先跑这一档，就知道该查哪一层。
        mg = c.BODY_MASS * c.GRAVITY
        c._mpc_solve_once = lambda: dict(
            F=np.array([0.0, 0.0, mg]), M=np.zeros(3), force=None,
            t=time.perf_counter(), n_contact=4, info={})

    st = sim.state()
    c.rpy, c.gyro, c.acc = list(st["rpy"]), list(st["gyro"]), list(st["acc"])
    c.pos, c.vel = st["pos"], st["vel"]
    # `_h_des` **不能在这里取**：`real=True` 时 `body_height()` 走运动学
    # （`_h_kin`），而它要等 `_tick()` 跑过一帧才有值。这时候取会落到
    # `pos[2] − SPORT_Z_BIAS` 那条兜底上（imu 站点比 base 高 4 cm），拿到
    # 一个偏高 1 cm 的目标 —— MPC 就会一直多推 22% 的体重，机身被顶起来。
    # 真机/仿真里这个值是 `start_velocity()` 取的，那时已经站好了。

    rec = dict(t=[], pos=[], rpy=[], tau=[], h=[], work=[])
    nxt = time.perf_counter()
    t = 0.0
    n_step = int(round(dur / DT))
    for k in range(n_step):
        st = sim.state()
        c.q, c.dq = st["q_n"], st["dq_n"]
        c.rpy = list(st["rpy"])
        c.gyro = list(st["gyro"])
        c.acc = list(st["acc"])
        c.pos, c.vel = st["pos"], st["vel"]
        c._sport_t = time.time()          # sportmodestate 是新鲜的

        moving = action != "stand" and t > 0.3
        if moving and c._h_des is None:
            c._h_des = c.body_height()
        c._vel_target = [vx if moving else 0.0, 0.0, 0.0]
        sim.push(push_n if (action == "push"
                            and 0.55 * dur < t < 0.55 * dur + 0.3) else 0.0)
        if burst:
            # burst 下墙钟跑得比仿真时间快，而步态的相位是拿 `time.time()`
            # 累加的限幅增量算的 —— 不接管它，整段都停在「起立插值」那一步，
            # `_tick` 直接 return，量出来 0.01 ms 毫无意义。这里把相位时钟
            # 钉到仿真时间上，节拍仍然是 200 Hz，只是不等真实时间。
            c._gait["t_wall"] = time.time() - DT
            c._gait["t_acc"] = t

        w0 = time.perf_counter()
        c._tick()
        rec["work"].append(time.perf_counter() - w0)
        sim.apply(cmd["q"], cmd["kp"], cmd["kd"], cmd["tau"])
        sim.step()
        # 记**实际下发到电机的那一份**（前馈 + PD，过完延迟与带宽），不是
        # 控制器给的前馈。帧间差要量的是减速器感受到的东西。
        rec["tau"].append(sim.tau_f.copy())
        rec["t"].append(t)
        rec["pos"].append(st["pos"].copy())
        rec["rpy"].append(np.array(st["rpy"]))
        rec["h"].append(float(st["pos"][2]) - SPORT_Z_BIAS)

        t += DT
        nxt += DT
        if not burst:
            _pace(nxt)

    # burst 模式：不睡、按自己的节奏连跑，量到的才是「一帧算多久」。
    # paced 循环里 `time.sleep` 会让 macOS 把进程降到低 QoS，量出来的耗时
    # 是降频后的（实测同一段 WBC 站姿：paced 3.6 ms、burst 0.6 ms）。
    res = _metrics(rec, vx, action, dur, burst)
    res["mode"] = mode
    res["action"] = action
    res["loop"] = c.loop_stats()
    res["force"] = c.mpc_stats() if mode == "mpc" else {}
    res["burst"] = burst
    c._mpc_stop_thread()
    return res


def _metrics(rec: dict, vx: float, action: str, dur: float,
             burst: bool = False) -> dict:
    t = np.array(rec["t"])
    p = np.array(rec["pos"])
    rpy = np.array(rec["rpy"])
    tau = np.array(rec["tau"])
    h = np.array(rec["h"])
    # 前 1 秒留给起立和幅度爬升，指标从 1 s 起算
    w = t >= 1.0
    tw = t[w]
    roll, pitch = rpy[w, 0], rpy[w, 1]
    out = dict(
        h_mean=1000.0 * float(h[w].mean()), h_min=1000.0 * float(h[w].min()),
        h_max=1000.0 * float(h[w].max()),
        roll_pp=float(np.degrees(roll.max() - roll.min())),
        pitch_pp=float(np.degrees(pitch.max() - pitch.min())),
        roll_mean=float(np.degrees(roll.mean())),
        pitch_mean=float(np.degrees(pitch.mean())),
        yaw_end=float(np.degrees(rpy[w, 2][-1] - rpy[w, 2][0])),
        dx=float(p[w, 0][-1] - p[w, 0][0]),
        dy=float(p[w, 1][-1] - p[w, 1][0]),
    )
    span = max(1e-6, tw[-1] - tw[0])
    out["v_mean"] = float(math.hypot(out["dx"], out["dy"]) / span)
    out["slip"] = (float(1.0 - out["dx"] / (vx * span))
                   if action != "stand" and vx else 0.0)
    d = np.abs(np.diff(tau[w], axis=0)).max(axis=1)
    out["dtau_p95"] = float(np.percentile(d, 95))
    out["dtau_max"] = float(d.max()) if d.size else 0.0
    out["tau_peak"] = float(np.abs(tau[w]).max())
    # burst 里前 0.5 s 还在起立插值，那几帧什么都不算，别拉低 p50
    wk = np.array(rec["work"][100 if burst else 2:]) * 1000.0
    out["work_p50"] = float(np.percentile(wk, 50))
    out["work_p95"] = float(np.percentile(wk, 95))
    out["work_max"] = float(wk.max())
    return out


# ---------------------------------------------------------------------------
# 输出
# ---------------------------------------------------------------------------

def show(r: dict) -> None:
    if r.get("burst"):
        # burst 里物理时间是压缩的（步态相位用的是墙钟），位置/姿态这些物理
        # 指标没有意义 —— 只能看单帧计算耗时。
        print(f"\n=== {r['mode']:>3} / {r['action']:<5}  [burst 只量算力]"
              f"   单帧计算 p50 {r['work_p50']:.2f}"
              f" p95 {r['work_p95']:.2f} max {r['work_max']:.2f} ms")
        f = r.get("force") or {}
        if f:
            m, w = f.get("mpc", {}), f.get("wbc", {})
            print(f"    MPC p95 {m.get('p95', 0):.2f} ms"
                  f"（迭代 均值 {m.get('it_mean', 0):.0f}）"
                  f"  WBC p95 {w.get('p95', 0):.2f} ms"
                  f"  退化帧 {f.get('miss', 0)}")
            if f.get("err"):
                print(f"    !! {f['err']}")
        return
    print(f"\n=== {r['mode']:>3} / {r['action']:<5}"
          f"  平均速度 {r['v_mean']:.3f} m/s  位移 {r['dx']:+.3f}/"
          f"{r['dy']:+.3f} m  打滑 {100 * r['slip']:+.0f}%")
    print(f"    高度 均值 {r['h_mean']:.1f} min {r['h_min']:.1f}"
          f" max {r['h_max']:.1f} mm"
          f"    roll 峰峰 {r['roll_pp']:.1f}° 均值 {r['roll_mean']:+.1f}°"
          f"    pitch 峰峰 {r['pitch_pp']:.1f}° 均值 {r['pitch_mean']:+.1f}°"
          f"    yaw 漂 {r['yaw_end']:+.1f}°")
    print(f"    τ 帧间差 p95 {r['dtau_p95']:.3f} 峰 {r['dtau_max']:.3f}"
          f"    τ 峰值 {r['tau_peak']:.2f} N·m")
    lp = r["loop"]
    print(f"    回路 均值 {lp['mean']:.2f} p95 {lp['p95']:.2f}"
          f" max {lp['max']:.2f} ms（标称 {lp['nominal']:.1f}）"
          f" 超时 {lp['over']} 帧"
          f"  单帧计算 p50 {r['work_p50']:.2f} p95 {r['work_p95']:.2f}"
          f" max {r['work_max']:.2f} ms")
    f = r.get("force") or {}
    if f:
        m, w = f.get("mpc", {}), f.get("wbc", {})
        print(f"    MPC p95 {m.get('p95', 0):.2f} ms"
              f"（迭代 均值 {m.get('it_mean', 0):.0f}"
              f" p95 {m.get('it_p95', 0):.0f}）"
              f"  WBC p95 {w.get('p95', 0):.2f} ms"
              f"  退化帧 {f.get('miss', 0)}"
              f"  接触 {f.get('n_contact', '-')}"
              f"  计划滞后 {f.get('age_ms', 0):.1f} ms")
        if f.get("err"):
            print(f"    !! {f['err']}")


def compare(a: dict, b: dict) -> None:
    print("\n" + "=" * 76)
    print(f"{a['action']}  A/B：{a['mode']} vs {b['mode']}（↓ 表示越小越好）")
    print(f"{'指标':<20}{a['mode']:>12}{b['mode']:>12}   谁小")
    rows = [
        ("平均速度 m/s", "v_mean", False, 3),
        ("横向漂移 m", "dy", True, 3),
        ("高度均值 mm", "h_mean", None, 1),
        ("roll 峰峰 °", "roll_pp", True, 1),
        ("pitch 峰峰 °", "pitch_pp", True, 1),
        ("τ 帧间差 p95", "dtau_p95", True, 3),
        ("τ 峰值 N·m", "tau_peak", True, 2),
        # 算力预算看的是「一帧算多久」，不是「两帧之间隔多久」—— 后者标称
        # 就是 5 ms，永远不可能 ≤ 4。回路周期的抖动另外打一行。
        ("单帧计算 p95 ms", "work_p95", True, 2),
        ("回路周期 p95 ms", None, True, 2),
    ]
    for label, key, lower, nd in rows:
        va = a["loop"]["p95"] if key is None else a[key]
        vb = b["loop"]["p95"] if key is None else b[key]
        who = ""
        if lower:
            who = a["mode"] if va < vb else (b["mode"] if vb < va else "=")
        print(f"{label:<20}{va:>12.{nd}f}{vb:>12.{nd}f}{who:>8}")
    print("=" * 76)


def main() -> int:
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--mode", default="mpc", choices=("vmc", "mpc"))
    ap.add_argument("--action", default="walk",
                    choices=("stand", "walk", "push", "slope"))
    ap.add_argument("--style", default="mech", choices=("mech", "bio"))
    ap.add_argument("--vx", type=float, default=0.25)
    ap.add_argument("--dur", type=float, default=6.0)
    ap.add_argument("--slope", type=float, default=None,
                    help="横坡角度（度），默认按 action 取：slope=5，其余 0")
    ap.add_argument("--push", type=float, default=25.0, help="侧推，牛")
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--gait", default="", help="覆盖步态参数，如 duty=0.55")
    ap.add_argument("--ab", action="store_true", help="两条力路各跑一遍再对比")
    ap.add_argument("--wrench", default="mpc", choices=("mpc", "fixed"),
                    help="mpc = 走 SRBD-MPC；fixed = 常数承重（隔离 WBC 用）")
    ap.add_argument("--ideal", action="store_true", help="关掉真机化（无延迟噪声）")
    ap.add_argument("--burst", action="store_true",
                    help="不按墙钟节拍连跑，只量每帧计算耗时（macOS 上必须这样量）")
    a = ap.parse_args()

    slope = a.slope if a.slope is not None else (
        5.0 if a.action == "slope" else 0.0)
    over = {}
    for kv in filter(None, a.gait.split(",")):
        k, v = kv.split("=")
        over[k] = float(v)

    modes = ["vmc", "mpc"] if a.ab else [a.mode]
    got = []
    for m in modes:
        print(f"\n[{m}] {a.action}  slope={slope}°  vx={a.vx}  dur={a.dur}"
              f"  kp={a.kp or '默认'}  style={a.style}", flush=True)
        r = run(m, a.action, a.vx, a.dur, slope, a.push, a.kp, a.style, over,
                realism=not a.ideal, wrench=a.wrench, burst=a.burst)
        show(r)
        got.append(r)
    if len(got) == 2:
        compare(*got)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
