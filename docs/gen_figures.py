#!/usr/bin/env python3
"""渲染 MuJoCo 场景，给博文配图／配视频。

姿态不是手摆的：直接调 quadruped/control/controller.py 的解析 IK，和真机跑的是同一套
关节角。所以图上这个姿势，就是控制器实际会发出去的姿势。

    .venv/bin/python docs/gen_figures.py still     # 3 张静态图
    .venv/bin/python docs/gen_figures.py gait      # 两段步态视频
    .venv/bin/python docs/gen_figures.py jump      # 一段跳跃视频
"""

import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import mujoco  # noqa: E402

from quadruped.control.controller import (  # noqa: E402
    CTRL_LEGS, LEGS, MIN_HEIGHT, NOMINAL_HEIGHT, Go2Controller, make_gait,
)

MODEL_DIR = os.path.join(ROOT, "sim", "unitree_mujoco", "unitree_robots", "go2")
OUT_DIR = os.path.join(ROOT, "docs", "img")

# 模型里 12 个关节的顺序是 FL / FR / RL / RR，控制器按执行器顺序 FR / FL / RR / RL
_MODEL_ORDER = ("FL", "FR", "RL", "RR")
_CTRL_OFFSET = {leg: i * 3 for i, leg in enumerate(CTRL_LEGS)}


def load():
    cwd = os.getcwd()
    os.chdir(MODEL_DIR)
    try:
        model = mujoco.MjModel.from_xml_path("scene.xml")
    finally:
        os.chdir(cwd)
    # 离屏渲染的帧缓冲默认只有 640×480，撑不住 1280 宽的图
    model.mat_reflectance[:] = 0.0        # 地面镜面反射太抢眼，压掉
    # 场景里的台阶是给越障测试用的，拍姿态图时只会挡住主体
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == 0 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            model.geom_rgba[g, 3] = 0.0
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 1100
    return model, mujoco.MjData(model)


def apply_q(model, data, q_ctrl, body_z):
    """把 12 个关节角按模型顺序写进 qpos，机身摆到 body_z。"""
    for i, leg in enumerate(_MODEL_ORDER):
        o = _CTRL_OFFSET[leg]
        data.qpos[7 + 3 * i: 7 + 3 * i + 3] = q_ctrl[o:o + 3]
    data.qpos[0:3] = (0.0, 0.0, body_z)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)


def drop_to_ground(model, data, q_ctrl, body_z, iters=8):
    """机身高度微调，让最低的腿正好踩在地面上（不做物理仿真，纯几何）。"""
    z = body_z
    for _ in range(iters):
        apply_q(model, data, q_ctrl, z)
        low = min(data.geom_xpos[g, 2] for g in range(model.ngeom)
                  if model.geom_bodyid[g] != 0)
        z -= low
        if abs(low) < 1e-4:
            break
    apply_q(model, data, q_ctrl, z)
    return z


def camera(azimuth=88.0, elevation=-9.0, distance=0.95, look=(0, 0, 0.27)):
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
    cam.lookat[:] = look
    return cam


def render_still(w, h, q_ctrl, body_z, path, cam=None, ground=True):
    model, data = load()
    z = drop_to_ground(model, data, q_ctrl, body_z) if ground else body_z
    if not ground:
        apply_q(model, data, q_ctrl, body_z)
    renderer = mujoco.Renderer(model, h, w)
    opt = mujoco.MjvOption()
    mujoco.mjv_defaultOption(opt)
    opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    renderer.update_scene(data, camera=cam or camera(), scene_option=opt)
    img = renderer.render()
    renderer.close()
    _save(img, path)
    return z


def _save(img, path):
    import cv2
    cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    print("写出", path, img.shape)


def _write_video(frames, path, fps=30):
    """frames: list of HxWx3 uint8 → mp4（libx264，浏览器都能直接播）。"""
    import tempfile
    import cv2
    with tempfile.TemporaryDirectory() as tmp:
        for i, f in enumerate(frames):
            cv2.imwrite(os.path.join(tmp, f"f{i:04d}.png"),
                        cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps),
             "-i", os.path.join(tmp, "f%04d.png"),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
             "-movflags", "+faststart", path],
            check=True)
    print("写出", path, f"{len(frames)} 帧")


# ---------------------------------------------------------------- 静态图

def stills():
    c = Go2Controller(domain=99, iface="lo0")

    # 站立
    q_stand = c.stand_joint_target(NOMINAL_HEIGHT)
    render_still(1280, 800, q_stand, NOMINAL_HEIGHT,
                 os.path.join(OUT_DIR, "sim-stand.png"))

    # 蹲下
    q_crouch = c.stand_joint_target(0.21)
    render_still(1280, 800, q_crouch, 0.21,
                 os.path.join(OUT_DIR, "sim-crouch.png"),
                 cam=camera(distance=0.95, look=(0, 0, 0.24)))

    # 腾空：腿接近伸直，机身离地
    q_air = c.stand_joint_target(0.405)      # 膝盖快到限位，腿近乎拉直
    render_still(1280, 800, q_air, 0.56,
                 os.path.join(OUT_DIR, "sim-air.png"), ground=False,
                 cam=camera(elevation=-8.0, distance=1.45, look=(0, 0, 0.40)))


# ---------------------------------------------------------------- 视频

def _gait_frames(style, seconds=2.4, fps=30, vx=0.28):
    c = Go2Controller(domain=99, iface="lo0", gait=make_gait(style))
    c.gait.neutral = c.gait.neutral  # 保持构造里的设定
    model, data = load()
    renderer = mujoco.Renderer(model, 560, 900)
    cam = camera(azimuth=88.0, elevation=-9.0, distance=1.15, look=(0, 0, 0.30))

    frames = []
    for k in range(int(seconds * fps)):
        t = k / fps
        feet = {leg: c.foot_target(leg, t, vx, 0.0, 0.0) for leg in LEGS}
        q = c.joint_from_feet(feet, strict=False)
        apply_q(model, data, q, NOMINAL_HEIGHT + c.gait.stand_raise)
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render())
    renderer.close()
    return frames


def gaits():
    for style in ("mech", "bio"):
        frames = _gait_frames(style)
        _write_video(frames, os.path.join(OUT_DIR, f"sim-gait-{style}.mp4"),
                     fps=30)


def _jump_sequence():
    """起跳 → 蹬伸 → 空中 → 触地 → 回正，腿型按 bio 风格摆。"""
    c = Go2Controller(domain=99, iface="lo0")
    T = dict(crouch=0.16, hold_in=0.02, push1=0.07, push2=0.06,
             air=0.14, fall=0.28, land=0.14, hold_land=0.08,
             recover=0.35, hold_end=0.18)

    def jf(h, rear_h=0.0, front_h=0.0, rear_x=0.0, front_x=0.0, pitch=0.0):
        cp, sp = np.cos(pitch), np.sin(pitch)
        feet = {}
        for leg in LEGS:
            rear = leg in ("RL", "RR")
            x, y, z = c.foot_from_body(
                leg, h + (rear_h if rear else front_h),
                xy_offset=((rear_x if rear else front_x), 0.0))
            if pitch:
                x, z = cp * x - sp * z, sp * x + cp * z
            feet[leg] = (x, y, z)
        return c.joint_from_feet(feet, strict=False)

    push, crouch = 0.335, 0.185
    drive, reach, trail = 0.07, 0.09, 0.045
    seq = [
        (jf(crouch, rear_h=-0.015, front_h=0.010), T["crouch"] + T["hold_in"]),
        (jf(crouch + 0.045, rear_h=drive * 0.65, front_h=0.020), T["push1"]),
        (jf(push, rear_h=drive * 0.45, front_h=-0.020, front_x=0.035), T["push2"]),
        (jf(crouch + 0.02, rear_h=0.020, rear_x=-trail, front_h=0.030,
            front_x=reach, pitch=-0.06), T["air"]),
        (jf(crouch + 0.02, rear_h=0.020, rear_x=-trail, front_h=0.030,
            front_x=reach), T["fall"]),
        (jf(MIN_HEIGHT + 0.04, rear_h=-0.025, rear_x=-trail * 0.6,
            front_h=0.030, front_x=reach * 0.7, pitch=0.10),
         T["land"] + T["hold_land"]),
        (jf(NOMINAL_HEIGHT), T["recover"] + T["hold_end"]),
    ]
    return seq


def _read_ctrl_state(model, data):
    """把模型的关节状态读成控制器的执行器顺序（FR/FL/RR/RL）。"""
    q = np.zeros(12)
    dq = np.zeros(12)
    for i, leg in enumerate(_MODEL_ORDER):
        o = _CTRL_OFFSET[leg]
        q[o:o + 3] = data.qpos[7 + 3 * i: 7 + 3 * i + 3]
        dq[o:o + 3] = data.qvel[6 + 3 * i: 6 + 3 * i + 3]
    return q, dq


def jump():
    """真物理仿真：PD 跟踪姿态序列，重力、接触、冲量全由 MuJoCo 算。"""
    model, data = load()
    c = Go2Controller(domain=99, iface="lo0")

    q0 = c.stand_joint_target(NOMINAL_HEIGHT)
    apply_q(model, data, q0, 0.30)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    kp = np.full(12, float(Go2Controller.KP_SIM))
    kd = np.full(12, float(Go2Controller.KD_SIM))
    tau_max = 23.7
    dt = model.opt.timestep
    fps = 30
    every = max(1, int(round(1.0 / (fps * dt))))

    renderer = mujoco.Renderer(model, 560, 900)
    cam = camera(azimuth=88.0, elevation=-7.0, distance=1.62, look=(0, 0, 0.41))

    frames, prev, peak = [], q0, 0.0
    for q_tgt, dur in _jump_sequence():
        n = max(1, int(dur / dt))
        for k in range(n):
            a = (k + 1) / n
            a = a * a * (3 - 2 * a)              # smoothstep
            q_des = prev * (1 - a) + q_tgt * a
            qc, dqc = _read_ctrl_state(model, data)
            data.ctrl[:] = np.clip(kp * (q_des - qc) - kd * dqc, -tau_max, tau_max)
            mujoco.mj_step(model, data)
            peak = max(peak, data.qpos[2])
            if len(frames) < int((len(frames) + 1) * every / every) and \
                    (k % every == 0):
                renderer.update_scene(data, camera=cam)
                frames.append(renderer.render())
        prev = q_tgt
    renderer.close()
    print(f"峰值机身高度 {peak:.3f} m，结束高度 {data.qpos[2]:.3f} m，"
          f"落地俯仰 {2*np.degrees(np.arcsin(np.clip(-data.qpos[6], -1, 1))):+.1f}°")
    _write_video(frames, os.path.join(OUT_DIR, "sim-jump.mp4"), fps=24)


if __name__ == "__main__":
    os.makedirs(OUT_DIR, exist_ok=True)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "still"
    {"still": stills, "gait": gaits, "jump": jump}[cmd]()
