"""无窗口仿真桥：与 unitree_mujoco.py 同一条物理 + DDS 链路，去掉 viewer。

`unitree_mujoco.py` 里的 `launch_passive` 只负责画图，物理推进和 DDS 收发
都在 `UnitreeSdk2Bridge` 里，跟窗口没有关系。去掉窗口之后可以在没有显示器
的会话里跑控制器、量指标。

    cd quadruped/sim/unitree_mujoco/simulate_python
    python headless.py --seconds 60

另开终端跑控制器，网卡 lo0 / domain 1，和带窗口时完全一样。
"""
import argparse
import math
import os
import time

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge

import config
from percept_pub import PerceptPublisher


# 无窗口版本的主循环。
#
# 和原版唯一的差别是 `while viewer.is_running()` 换成了按秒数退出，另外不再
# 持有 `locker` —— 那把锁是给 viewer 线程和物理线程互斥用的，没有 viewer
# 就没有第二个线程碰 mj_data。
def main() -> int:
    ap = argparse.ArgumentParser(description="无窗口 MuJoCo 仿真桥")
    ap.add_argument("--seconds", type=float, default=60.0, help="跑多久自动退出")
    ap.add_argument("--no-scene-info", action="store_true",
                    help="不打印 link/joint/actuator 清单")
    ap.add_argument("--pin-base", action="store_true",
                    help="把机身用弹簧吊在初始位姿（腿照常动、机位会随蹬踏微晃）。"
                         "演示相机视角用：无窗口仿真控制回路只有 5~6 Hz，"
                         "狗会倒地翻过来，相机朝天就只剩一片天空。")
    ap.add_argument("--pin-hard", action="store_true",
                    help="硬固定机身（镜头完全不晃，只适合截图比对）")
    a = ap.parse_args()

    mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = config.SIMULATE_DT

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)
    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION and not a.no_scene_info:
        unitree.PrintSceneInformation()

    # 和带窗口版本同一条感知链路。无窗口跑的时候 V 层更需要它：没有画面可看，
    # 只能靠这些数字判断狗在朝什么走。
    publisher = None
    if config.ENABLE_PERCEPT:
        publisher = PerceptPublisher(mj_model, mj_data, hz=config.PERCEPT_HZ)
        print(f"感知发布：{config.PERCEPT_HZ:.0f} Hz → 127.0.0.1:{publisher.port}"
              f"（机载画面 {publisher.video_port}，全局画面 {publisher.world_port}）",
              flush=True)

    # 吊住机身。两种方式：
    #
    #   软（--pin-base）  位置/姿态各一组 PD 力，机身被腿蹬得轻微位移和转动，
    #                    镜头会跟着晃 —— 真机待机踏步就是这个样子。硬固定时
    #                    腿在下面蹬、镜头纹丝不动，看着像一张静态图。
    #   硬（--pin-hard）  每步写回位姿、速度清零。只适合比对截图，日常别用。
    #
    # 都是演示/联调开关，物理不真实，默认关闭。
    base_qpos = (mj_data.qpos[:7].copy()
                 if (a.pin_base or a.pin_hard) else None)
    base_qvel = (mj_data.qvel[:6].copy()
                 if (a.pin_base or a.pin_hard) else None)
    pin_err = np.zeros(mj_model.nv) if a.pin_base else None
    # mj_differentiatePos 要完整的 nq 长度 qpos：free 部分取初始位姿、
    # 关节部分取当前值（这样关节差值为 0，只比较机身）。
    pin_target = mj_data.qpos.copy() if a.pin_base else None
    # 位置刚度和姿态刚度分开：狗约 15 kg，撑住重力需要 9000 N/m 量级；
    # 姿态用同样的数会把机身锁成一块铁，蹬一脚反而弹飞。
    # 参数可调：先在 5 秒短跑里试，别一上来挂 2 小时。
    # 太大直接 QACC 爆 NaN（实测 9000/520 就会），太小撑不住重力、机身下沉。
    pin_kp_lin = float(os.environ.get("PIN_KP_LIN", 1600.0))
    pin_kd_lin = float(os.environ.get("PIN_KD_LIN", 180.0))
    pin_kp_rot = float(os.environ.get("PIN_KP_ROT", 60.0))
    pin_kd_rot = float(os.environ.get("PIN_KD_ROT", 6.0))
    if a.pin_hard:
        print("机身已硬固定（截图比对用，镜头不会晃）", flush=True)
    elif a.pin_base:
        print("机身已吊在弹簧上（腿会蹬、镜头会晃）", flush=True)

    dt = config.SIMULATE_DT
    t0 = time.perf_counter()
    n = 0
    print(f"无窗口仿真就绪：dt={dt * 1000:.1f} ms，"
          f"网卡 {config.INTERFACE} / domain {config.DOMAIN_ID}", flush=True)

    while True:
        step_start = time.perf_counter()
        unitree.UpdateTorque()
        mujoco.mj_step(mj_model, mj_data)
        if a.pin_hard:
            mj_data.qpos[:7] = base_qpos
            mj_data.qvel[:6] = base_qvel
        elif pin_err is not None:
            # err = 当前位姿相对初始位姿的差（dt 给 1 就是差值本身）。
            pin_target[:7] = base_qpos
            # 水平方向**不锁**：只保留阻尼。锁住 x/y 的话，腿在走、机身被弹簧
            # 拉回原地 —— 读数有 0.28 m/s，位置一动不动，看起来就是「卡住」。
            pin_target[0] = mj_data.qpos[0]
            pin_target[1] = mj_data.qpos[1]
            mujoco.mj_differentiatePos(
                mj_model, pin_err, 1.0, pin_target, mj_data.qpos)
            # 三个平移方向都**只给阻尼**，不做位置弹簧。
            #
            # 之前 z 用 3500 N/m 的弹簧把机身吊住，副作用是腿的承重被卸掉：
            # 地面法向力不够 → 腿打滑 → 指令 0.4 m/s、位置不动，看着就是「卡住」。
            # 现在只在下落时给阻尼，重量正常压在腿上，狗能走；姿态另有 PD，
            # 所以不会翻。
            f = np.zeros(6)
            damp_xy = float(os.environ.get("PIN_DAMP_XY", 0.0))
            f[0] = -damp_xy * mj_data.qvel[0]
            f[1] = -damp_xy * mj_data.qvel[1]
            f[2] = -300.0 * mj_data.qvel[2]
            # roll / pitch 都锁，只给 yaw 一点阻尼。
            #
            # 试过放开 pitch 让狗前倾走路：pitch 一路掉到 -0.6（-35°），机身
            # 低着头、位移反而是负的。仿真的自研步态不闭环速度、支撑腿也不
            # 主动压地，靠前倾推不动，放开只会栽。
            f[3:] = -pin_kp_rot * pin_err[3:6] - pin_kd_rot * mj_data.qvel[3:6]
            mj_data.qfrc_applied[:6] = f
        if publisher is not None:
            publisher.maybe_publish()
        n += 1
        elapsed = time.perf_counter() - t0
        rest = dt - (time.perf_counter() - step_start)
        if rest > 0:
            time.sleep(rest)
        if elapsed >= a.seconds:
            break

    h = float(mj_data.qpos[2])
    print(f"仿真结束：{n} 步 / {elapsed:.1f}s，末态机身高度 {h:.3f} m", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
