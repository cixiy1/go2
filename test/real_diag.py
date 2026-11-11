"""真机抖动诊断：把「抖」拆成能对着数字看的三件事。

仿真里 IMU 干净、没有齿隙，抖不起来。真机上「抖得厉害、硬摩擦声、迈不开」
同时出现时，光靠听和看分不清是编码器噪声、IMU 噪声、力矩指令本身在跳，
还是控制回路时序不够。这个脚本分开量，每一段只测一件事。

用法：
  uv run python test/real_diag.py --stage noise          # 只读，不动作（最安全）
  uv run python test/real_diag.py --stage stand          # 站立保持 5 s
  uv run python test/real_diag.py --stage march          # 原地踏步 5 s
  uv run python test/real_diag.py --stage walk --vx 0.15 # 慢走 5 s

`--stage noise` 不启动任何回路，只订阅 lowstate 数噪声。先跑它：如果静止
时的 dq 噪声就有 0.3 rad/s，kd 乘上去就是每帧上下的力矩，那再怎么调控制
参数都是白费。

每个阶段都打印「力矩指令帧间变化」—— 这是抖动的直接量化指标，单位
N·m/帧。200 Hz 下 1 N·m/帧 = 200 N·m/s，超过 1 就够在减速器上听出来了。
"""
import argparse
import math
import sys
import time

import numpy as np

ROOT = __file__.rsplit("/test/", 1)[0]
sys.path.insert(0, ROOT)

from quadruped.robot.body import Go2Robot          # noqa: E402
from quadruped.control.controller import CTRL_LEGS    # noqa: E402


def _rms(x, axis=None):
    x = np.asarray(x, float)
    if not x.size:
        return 0.0
    v = np.sqrt((x * x).mean(axis=axis))
    return v if axis is not None else float(v)


def _p2p(x, axis=None):
    x = np.asarray(x, float)
    if not x.size:
        return 0.0
    return x.max(axis=axis) - x.min(axis=axis)


class Tap:
    """挂到 _publish 上记录每帧下发的指令。

    真机抖动最容易忽略的一环就是这里：我们只看得见关节在颤，看不见
    「颤的输入是什么」。把每帧的 tau / q_target 记下来，帧间差一算，
    是控制指令本身在跳，还是指令很平顺、关节自己在振，立刻分开。
    """

    def __init__(self, c):
        self.c = c
        self.tau = []
        self.qt = []
        self.kp = []
        self._orig = c._publish
        c._publish = self

    def __call__(self, q_target, kp=None, kd=None, tau=None):
        if q_target is not None:
            self.qt.append(np.array(q_target, float))
        if tau is not None:
            self.tau.append(np.array(tau, float))
        if kp is not None:
            self.kp.append(np.array(kp, float))
        return self._orig(q_target, kp=kp, kd=kd, tau=tau)

    def restore(self):
        self.c._publish = self._orig

    def report(self):
        if self.qt:
            a = np.array(self.qt)
            if len(a) > 2:
                d = np.abs(np.diff(a, axis=0))
                print(f"  位置指令 帧间差  rms {d.mean():.5f} rad  "
                      f"max {d.max():.5f} rad/帧", flush=True)
                # 找出跳得最狠的那一下。碰撞声就是这种阶跃打出来的：20 ms 里
                # 要关节转几十度，减速器只有撞上去这一条路。
                i, j = np.unravel_index(int(np.argmax(d)), d.shape)
                print(f"  最大跳变  第 {i}→{i+1} 帧 关节 {j}: "
                      f"{a[i, j]:+.3f} → {a[i + 1, j]:+.3f} rad "
                      f"(Δ{d[i, j]:.3f} rad = {math.degrees(d[i, j]):.1f}°)",
                      flush=True)
                big = (d > 0.05).sum()
                print(f"  单帧跳 >0.05 rad（2.9°）的关节·帧 {big}/{d.size}"
                      f"  ({100.0 * big / d.size:.2f}%)", flush=True)
        if self.tau:
            a = np.array(self.tau)
            if len(a) > 2:
                d = np.abs(np.diff(a, axis=0))
                print(f"  力矩指令 帧间差  rms {d.mean():.4f} N·m  "
                      f"p95 {np.percentile(d, 95):.4f}  max {d.max():.4f} N·m/帧",
                      flush=True)
                print(f"  力矩指令 幅值    rms {_rms(a):.2f}  "
                      f"max {a.max():.2f} N·m", flush=True)
        else:
            print("  力矩指令 无（这轮没开前馈/机身层）", flush=True)


def loop_report(c):
    s = c.loop_stats()
    if not s.get("n"):
        return
    print(f"  回路周期  均值 {s['mean']:.2f} ms  p95 {s['p95']:.2f}  "
          f"max {s['max']:.2f}（标称 {s['nominal']:.2f}）  "
          f"超时 {s['over']}/{s['n']}", flush=True)
    # 均值超过标称 20% 就是真跑不动了，力矩指令会跟着周期一起抖。
    if s["mean"] > 1.2 * s["nominal"]:
        print("  ** 回路跑不到 200 Hz，先修时序，别调增益 **", flush=True)


def stage_noise(c, dur):
    print(f"\n[noise] 只读 {dur:.0f} s，不动作", flush=True)
    print("  静止时这些噪声就是后面所有抖动的下限", flush=True)
    qs, dqs, gxs, gys, azs = [], [], [], [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        time.sleep(1.0 / c.CTRL_HZ)
        if c.dq is None:
            continue
        dqs.append(np.array(c.dq, float))
        if c.q is not None:
            qs.append(np.array(c.q, float))
        if c.gyro:
            gxs.append(float(c.gyro[0]))
            gys.append(float(c.gyro[1]))
        if c.acc:
            azs.append(float(c.acc[2]))
    if qs:
        a = np.array(qs)
        # 逐关节算时间上的峰峰，再取最大。跨关节算会把「蹲姿各关节角度不
        # 一样」当成抖动 —— 12 个关节本来就该差 1 rad 以上。
        pp = _p2p(a, axis=0)
        j = int(np.argmax(pp))
        print(f"  关节角 q  最大单关节峰峰 {math.degrees(pp[j]):.2f}°（关节 {j}）"
              f"  全部 <{math.degrees(pp.max()):.2f}°", flush=True)
    if dqs:
        a = np.array(dqs)
        print(f"  关节速 dq 单关节 rms 最大 {_rms(a, axis=0).max():.4f} rad/s  "
              f"max {np.abs(a).max():.4f}", flush=True)
        print(f"  → kd {c.kd[0]:.1f} 时这部分折成力矩噪声 "
              f"{_rms(a, axis=0).max() * c.kd[0]:.3f} N·m", flush=True)
    if gxs:
        print(f"  陀螺   x rms {_rms(gxs):.4f}  y rms {_rms(gys):.4f} rad/s",
              flush=True)
        print(f"  → WRENCH_KD {c.WRENCH_KD:.0f} 时折成力矩噪声 "
              f"{_rms(gxs) * c.WRENCH_KD:.3f} N·m", flush=True)
    if azs:
        # 加速度带 9.8 的重力分量，rms 会被它整个吃掉，必须用标准差。
        print(f"  加速度 z 均值 {np.mean(azs):+.3f}  "
              f"std {np.std(azs):.3f} m/s²（这个 std 才是噪声）", flush=True)


def stage_hold(c, dur):
    """站立保持：stand_up 之后本来就是 hold 模式，别起步态。

    这一段最干净 —— 没有相位、没有落地冲击，唯一在动的就是位置环和
    重力前馈。如果这里就在抖，问题在 PD/前馈；如果这里很稳，问题在步态。
    """
    tap = Tap(c)
    zs, ps, rls = [], [], []
    t0 = time.time()
    while time.time() - t0 < dur:
        time.sleep(1.0 / c.CTRL_HZ)
        h = c.body_height()
        if h is not None:
            zs.append(h)
        if c.rpy:
            ps.append(float(c.rpy[1]))
            rls.append(float(c.rpy[0]))
    tap.restore()

    print(f"\n[stand] hold 保持 {dur:.0f} s", flush=True)
    if zs:
        print(f"  高度     z {np.mean(zs):.4f}  纹波 {_p2p(zs) * 1000:5.1f} mm  "
              f"rms {_rms(np.array(zs) - np.mean(zs)) * 1000:.1f} mm", flush=True)
    if ps:
        print(f"  俯仰     峰峰 {math.degrees(_p2p(ps)):5.1f}°  "
              f"均值 {math.degrees(np.mean(ps)):+.1f}°    "
              f"滚转 峰峰 {math.degrees(_p2p(rls)):5.1f}°", flush=True)
    tap.report()
    loop_report(c)


def stage_motion(c, hl, dur, vx=0.0, vy=0.0, yaw=0.0, label="walk"):
    """跑一段步态，边跑边统计姿态、高度、指令平滑度。"""
    tap = Tap(c)
    zs, ps, rls, dqs = [], [], [], []
    gxs, gys, odv, dqd = [], [], [], []
    vf = []
    # 起振和加速度都交给控制回路，这里只负责改目标：直接给 ramp=0 会让
    # 第一帧就按全额幅度出腿，真机上那一步是踩空的。
    c.start_velocity()
    time.sleep(0.35)
    t0 = time.time()
    while time.time() - t0 < dur:
        time.sleep(1.0 / c.CTRL_HZ)
        c.set_velocity(vx, vy, yaw)
        h = c.body_height()
        if h is not None:
            zs.append(h)
        if c.rpy:
            ps.append(float(c.rpy[1]))
            rls.append(float(c.rpy[0]))
        if c.dq is not None:
            dqs.append(np.array(c.dq, float))
            # 关节速度的帧间差 ≈ 加速度噪声。位置指令平滑、这个数却大，
            # 说明是地面/传动在反过来冲击关节，不是指令让它抖的。
            if len(dqs) > 1:
                dqd.append(np.abs(dqs[-1] - dqs[-2]))
        if c.gyro:
            gxs.append(float(c.gyro[0]))
            gys.append(float(c.gyro[1]))
        # 机身速度只能用关节里程计：真机被 lowcmd 接管后 sportmodestate
        # 就停发了，c.vel 一直是最后一帧。
        if c._odom_ok:
            odv.append(float(c._odom_v[0]))
        # 进速度环的那个数。它抖，步幅就每帧跟着变，脚就在地上前后蹭。
        vf.append(float(c._v_filt))
    c.stop_velocity()
    time.sleep(0.4)
    tap.restore()

    print(f"\n[{label}] vx={vx:.2f} {dur:.0f} s", flush=True)
    if zs:
        print(f"  高度     z {np.mean(zs):.4f}  纹波 {_p2p(zs) * 1000:5.1f} mm  "
              f"rms {_rms(np.array(zs) - np.mean(zs)) * 1000:.1f} mm", flush=True)
    if ps:
        print(f"  俯仰     峰峰 {math.degrees(_p2p(ps)):5.1f}°  "
              f"均值 {math.degrees(np.mean(ps)):+.1f}°    "
              f"滚转 峰峰 {math.degrees(_p2p(rls)):5.1f}°", flush=True)
    if dqs:
        a = np.array(dqs)
        print(f"  关节速 dq rms {_rms(a):.3f} rad/s  "
              f"max {np.abs(a).max():.3f}", flush=True)
    if dqd:
        a = np.array(dqd)
        print(f"  关节速 帧间差 rms {a.mean():.4f} rad/s²/帧  "
              f"max {a.max():.4f}（大=关节在被硬撬）", flush=True)
    if gxs:
        print(f"  机身角速 x rms {_rms(gxs):.3f}  y rms {_rms(gys):.3f} rad/s"
              f"（这个数大就是「抖」）", flush=True)
    if odv:
        k = max(1, len(odv) // 3)
        print(f"  里程计速度  前段 {np.mean(odv[:k]):+.3f}  "
              f"末段 {np.mean(odv[-k:]):+.3f} m/s  （指令 {vx:+.2f}）", flush=True)
    if vf:
        print(f"  速度环输入 v_filt  均值 {np.mean(vf):+.3f}  "
              f"峰峰 {_p2p(vf):.3f}  rms(去均值) {_rms(np.array(vf) - np.mean(vf)):.3f} m/s",
              flush=True)
    tap.report()
    loop_report(c)
    print(f"  IK 失败 {c._gait.get('fail', 0) if c._gait else 0} 次", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="en5")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--stage", default="noise",
                    choices=("noise", "stand", "march", "walk"))
    ap.add_argument("--vx", type=float, default=0.15)
    ap.add_argument("--dur", type=float, default=5.0)
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--tau", type=float, default=None,
                    help="前馈强度，0 关 1 全补")
    ap.add_argument("--wrench", type=float, default=None,
                    help="机身层强度，0=位置伺服+重力前馈的老路子 1=机身层")
    ap.add_argument("--height", type=float, default=0.24,
                    help="机身高度 m，Go2 官方站姿 0.28~0.30")
    ap.add_argument("--loop", type=float, default=None,
                    help="速度闭环强度，0=纯按指令算步幅")
    a = ap.parse_args()

    r = Go2Robot(preset="real", iface=a.iface, domain=a.domain,
                 height=a.height, kp=a.kp, kd=a.kd, tau_ff=a.tau)
    if not r.connect():
        print(f"连不上 {a.iface} domain {a.domain}", flush=True)
        return 1
    c = r.controller
    hl = c.high_level
    if a.wrench is not None:
        c.gait.wrench = a.wrench
    if a.loop is not None:
        c.gait.speed_loop = a.loop
    print(f"已连接  iface={c.iface} domain={c.domain}  "
          f"kp={c.kp[0]:.0f} kd={c.kd[0]:.0f} tau_ff={c.tau_ff:.2f}  "
          f"wrench={c.gait.wrench:.1f}", flush=True)
    print(f"步态 {c.gait.name}  T={c.gait.period:.2f}s duty={c.gait.duty:.2f} "
          f"swing={c.gait.swing_height:.3f}m stride_max={c.gait.stride_max:.3f}m",
          flush=True)

    try:
        if a.stage == "noise":
            stage_noise(c, a.dur)
        else:
            hl.stand_up()
            time.sleep(1.5)
            print(f"\n[stand] 站姿 z={c.body_height():.4f}", flush=True)
            if a.stage == "stand":
                stage_hold(c, a.dur)
            elif a.stage == "march":
                stage_motion(c, hl, a.dur, 0, 0, 0, label="march")
            else:
                stage_motion(c, hl, a.dur, a.vx, 0, 0, label="walk")
    except KeyboardInterrupt:
        print("\n中断", flush=True)
    finally:
        c.stop_velocity()
        time.sleep(0.3)
        r.shutdown(park=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
