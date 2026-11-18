"""把推理模型在 MuJoCo 里跑一遍，录成视频。

    .venv/bin/python test/policy_video.py --seconds 10

输出 `docs/policy_infer.mp4`。画面左下角是实时状态：倾角、机身高度、命令、力矩来源。

## 这跑的是哪条链路

    30 帧观测 → adaptation_module → 隐变量 2 维 → actor_body → 12 个动作
              → q_des = 动作 × 0.25（髋 × 0.5）+ DEFAULT_Q
              → 驱动器 PD 20 / 0.5 → 力矩 → MuJoCo

物理 500 Hz、策略 50 Hz、视频 31 fps。没有 DDS、没有控制器 —— 那四层的问题在
`policy_sim.py` 里也看不出来，但它们不是这次要看的东西。这条链路与真机部署只差
`LowCmd` 那一跳。

## 相机

自由视角，跟着机身走。俯角 −18°、方位 130°，能同时看到四条腿的落点和机身俯仰。
`--side` 换成纯侧视（方位 90°），看起伏更清楚。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

# 离屏渲染要先定后端，必须在 import mujoco 之前。
os.environ.setdefault("MUJOCO_GL", "glfw")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test"))

import cv2                                                             # noqa: E402
import mujoco                                                          # noqa: E402

import policy_sim as PS                                                 # noqa: E402
from quadruped.policy import load_actuator, load_walker                 # noqa: E402
from quadruped.policy.walker import (                                   # noqa: E402
    DEFAULT_Q, KD, KP, POLICY_TO_SDK, TAU_LIMIT, default_commands, tilt_rad,
)

WIDTH, HEIGHT = 960, 540
FPS = 31
# 500 Hz / 31 fps ≈ 16 步一帧。用 16 就是 31.25 fps，取整到 31 差 0.8%，肉眼不可见。
RENDER_EVERY = 16


def overlay(img, lines: list[str]) -> None:
    # 半透明底条 + 白字。cv2 不支持中文，标签一律 ASCII。
    h = 22 * len(lines) + 12
    strip = img.copy()
    cv2.rectangle(strip, (0, 0), (WIDTH, h), (24, 24, 24), -1)
    cv2.addWeighted(strip, 0.55, img, 0.45, 0, img)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (14, 26 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (235, 235, 235), 1, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="录一段推理模型在仿真里跑的视频")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=0.6)
    ap.add_argument("--vx", type=float, default=0.0, help="前进命令 m/s")
    ap.add_argument("--torque", default="pd", choices=("pd", "net"))
    ap.add_argument("--side", action="store_true", help="纯侧视")
    ap.add_argument("--out", default="docs/policy_infer.mp4")
    a = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(PS.SCENE))
    model.opt.timestep = PS.DT
    # 离屏帧缓冲默认 640×480，开大了才能渲 960×540。必须在建 Renderer 之前设。
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    data = mujoco.MjData(model)

    # 初态与 policy_sim 一致：机身 0.31 m，四条腿按 DEFAULT_Q 摆好，按**关节名**写
    # qpos（MJCF 里是 FL/FR/RL/RR，正好是策略序，不是 SDK 序）。
    data.qpos[:] = 0.0
    data.qpos[2] = 0.31
    data.qpos[3] = 1.0
    for i, leg in enumerate(("FL", "FR", "RL", "RR")):
        for j, part in enumerate(("hip", "thigh", "calf")):
            data.qpos[int(model.joint(f"{leg}_{part}_joint").qposadr[0])] = \
                DEFAULT_Q[i * 3 + j]
    mujoco.mj_forward(model, data)

    walker = load_walker(dt=PS.DT * PS.DECIMATION)
    net = load_actuator() if a.torque == "net" else None
    cmd = default_commands()
    cmd[0] = a.vx
    hold = DEFAULT_Q[POLICY_TO_SDK].copy()

    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 1.15
    cam.elevation = -12 if a.side else -18
    cam.azimuth = 90 if a.side else 130

    warm = int(a.warmup / PS.DT)
    total = warm + int(a.seconds / PS.DT)
    frames = []
    held = hold
    worst = 0.0
    stage = Path(tempfile.mkdtemp(prefix="policy_video_"))
    src = "actuator_net" if net is not None else f"PD kp{KP:g} kd{KD:g}"

    for step in range(total):
        q, dq, quat = PS.sensor_view(model, data)
        rpy = PS.rpy_from_quat(quat)

        if step % PS.DECIMATION == 0:
            q_out, _ = walker.step(q, dq, rpy, cmd)
            if step >= warm:
                held = q_out
            if net is not None:
                held_tau = np.clip(net.torque(q, q_out, dq), -TAU_LIMIT, TAU_LIMIT)

        if net is not None:
            data.ctrl[:] = held_tau
        else:
            target = held if step >= warm else hold
            data.ctrl[:] = np.clip(KP * (target - q) - KD * dq, -TAU_LIMIT, TAU_LIMIT)

        if step % RENDER_EVERY == 0:
            cam.lookat[:] = data.qpos[:3]
            renderer.update_scene(data, camera=cam)
            img = renderer.render().copy()
            tilt = tilt_rad(rpy)
            if step >= warm:
                worst = max(worst, tilt)
            phase = "TAKEOVER" if step >= warm else "HOLD"
            overlay(img, [
                f"policy {phase}   t={(step - warm) * PS.DT:5.2f}s   src={src}",
                f"tilt {np.degrees(tilt):5.1f} deg   body z {data.qpos[2]:.3f} m"
                f"   cmd vx={a.vx:.2f}",
                f"worst tilt {np.degrees(worst):5.1f} deg   x {data.qpos[0]:+.3f} m",
            ])
            frames.append(img)

        mujoco.mj_step(model, data)
        if step >= warm and tilt_rad(rpy) > 1.2:
            print(f"倒了：t={(step - warm) * PS.DT:.2f}s")
            break

    renderer.close()
    print(f"渲染 {len(frames)} 帧，{FPS} fps ≈ {len(frames) / FPS:.1f} s")

    # 先写 PNG 再用 ffmpeg 编。cv2 的 mp4v 编码在浏览器里要装插件，h264 不用。
    for i, img in enumerate(frames):
        cv2.imwrite(str(stage / f"{i:05d}.png"), img)
    out = ROOT / a.out
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS),
         "-i", str(stage / "%05d.png"),
         "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(out)],
        check=True)
    print(f"写好了：{out}（{out.stat().st_size / 1e6:.1f} MB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
