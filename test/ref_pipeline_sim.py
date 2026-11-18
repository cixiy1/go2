"""跑参考仓库的策略：观测、时钟、动作换算逐行照抄，只把仿真器换成 MuJoCo。

    .venv/bin/python test/ref_pipeline_sim.py --vx 0.4 --seconds 12 --torque net

## 这不是它原来的仿真，是把它原来的仿真搬到本机能跑的东西上

原仓库的仿真在 Isaac Gym 里（`scripts/play.py`，`sim_device='cuda:0'`），Isaac Gym
只有 Linux x86_64 + NVIDIA CUDA 的构建，macOS / Apple Silicon 上装不了、也没有 CPU
后端。所以这里换掉的是**物理引擎**，其余按源码复现：

    `--torque net`  训练链：力矩由 `actuator_net`（他们 `resources/actuator_nets/` 那份）
                    算，与 `legged_robot._compute_torques()` 里 control_type=actuator_net
                    的分支一致，50 Hz 算一次、4 个物理步保持
    `--torque pd`   部署链：力矩由驱动器 PD（kp 20 / kd 0.5）算，与 `go2_gym_deploy`
                    那条 LCM → LowCmd 的路径一致

观测组装、时钟、动作换算两边相同，见下一节。**不等价的部分**：PhysX 与 MuJoCo 的接触
求解不同，没有域随机化（质量/摩擦/电机强度都取名义值），地形是平地。

## 抄的是哪几行

    lcm_agent.get_obs()           观测顺序：重力 / 命令 / 关节角 / 关节速度 / 动作 / 上一动作 / 时钟
    lcm_agent.step()              时钟推进、foot_indices 的四个相位
    lcm_agent.publish_action()    q_des = action×0.25（髋再×0.5）+ default_dof_pos
    history_wrapper.step()        30 帧历史左移一位
    deploy_policy.load_policy()   先 adaptation 出 2 维隐变量，再拼进 body 的输入

参数不写在源码里，是从 `runs/.../parameters.pkl` 里读出来的：kp 20 / kd 0.5、
`action_scale` 0.25、`hip_scale_reduction` 0.5、`clip_actions` 10、观测缩放
dof_pos 1.0 / dof_vel 0.05。默认命令取 `cheetah_state_estimator.get_command()` 的
默认值：freq 3.0、footswing 0.08、站宽 0.33、站长 0.40、相位 0。

## 和本项目 `walker.py` 的两处已知差异

1. 动作历史的相位：那边每步先发动作、再用**这一步的动作**组观测；`walker.py` 先组
   观测再前向，所以观测里那一帧动作慢一拍。
2. 默认命令：那边 `phase=0`、`footswing=0.08`、`height=0`；`walker.py` 是
   `phase=0.5`、`0.1`、`−0.05`。

这两条都留着可调，见命令行参数 —— 对照实验要的是「只改一处」。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "glfw")

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "test"))

import cv2                                                             # noqa: E402
import mujoco                                                          # noqa: E402

import policy_sim as PS                                                 # noqa: E402
from quadruped.policy import load_actuator                              # noqa: E402
from quadruped.policy.mlp import Mlp                                    # noqa: E402
from quadruped.policy.walker import (                                   # noqa: E402
    POLICY_TO_SDK, TAU_LIMIT, WEIGHTS_DIR, tilt_rad,
)

WIDTH, HEIGHT, FPS, RENDER_EVERY = 960, 540, 31, 16

# -- 训练侧参数，来自 pretrain-go2 的 parameters.pkl ---------------------------
LEGS = ("FL", "FR", "RL", "RR")
PARTS = ("hip", "thigh", "calf")
JOINTS = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in PARTS)

DEFAULT_Q = np.array([0.1, 0.8, -1.5,
                      -0.1, 0.8, -1.5,
                      0.1, 1.0, -1.5,
                      -0.1, 1.0, -1.5], dtype=np.float64)

KP, KD = 20.0, 0.5
ACTION_SCALE = 0.25
HIP_SCALE = 0.5
CLIP_ACTIONS = 10.0
OBS_DIM, HISTORY = 70, 30
DOF_POS_SCALE, DOF_VEL_SCALE = 1.0, 0.05

# lcm_agent.commands_scale 原样：前四条是速度与身高，中间五个是步态参数（不缩放），
# 之后是抬脚 / 俯仰 / 横滚 / 站宽 / 站长 / 辅助项。
COMMAND_SCALE = np.array([2.0, 2.0, 0.25, 2.0,
                          1.0, 1.0, 1.0, 1.0, 1.0,
                          0.15, 0.3, 0.3, 1.0, 1.0, 1.0])
NUM_COMMANDS = 15


# 部署侧默认命令。`cheetah_state_estimator.get_command()` 在摇杆归零时返回这一组，
# 另外它前面还有 heading 之类位，这里只取 15 条有效项。
def deploy_commands(vx=0.0, vy=0.0, vyaw=0.0, height=0.0, freq=3.0,
                    phase=0.0, offset=0.0, bound=0.0, duration=0.5,
                    footswing=0.08, pitch=0.0, roll=0.0, width=0.33,
                    length=0.40) -> np.ndarray:
    return np.array([vx, vy, vyaw, height, freq, phase, offset, bound,
                     duration, footswing, pitch, roll, width, length, 0.0],
                    dtype=np.float64)[:NUM_COMMANDS]


# 关节名 → MuJoCo 的 qpos 地址与执行器号。两处都不按默认顺序猜：模型里执行器的排列
# 与 qpos 的排列不一定一致，历史上这里错过一次。
def build_index(model):
    qadr = np.array([int(model.joint(n).qposadr[0]) for n in JOINTS])
    dadr = np.array([int(model.joint(n).dofadr[0]) for n in JOINTS])
    act = np.full(len(JOINTS), -1, dtype=int)
    for i in range(model.nu):
        jid = int(model.actuator_trnid[i, 0])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name in JOINTS:
            act[JOINTS.index(name)] = i
    if np.any(act < 0):
        missing = [JOINTS[i] for i in np.where(act < 0)[0]]
        raise RuntimeError(f"这些关节没有对应的执行器：{missing}")
    return qadr, dadr, act


# 参考实现的状态量：重力在机体系里的投影、12 个关节角与角速度（都是策略序）。
def read_state(data, qadr, dadr):
    quat = np.array(data.qpos[3:7], dtype=float)
    w, x, y, z = quat / np.linalg.norm(quat)
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    gravity = R.T @ np.array([0.0, 0.0, -1.0])
    return gravity, np.array(data.qpos[qadr], dtype=float), np.array(data.qvel[dadr], dtype=float)


class RefPolicy:
    """`deploy_policy.load_policy()` 的等价物，加上 `lcm_agent` 的历史与时钟。"""

    def __init__(self, weights_dir: Path = WEIGHTS_DIR, dt: float = 0.02):
        self.body = Mlp(weights_dir / "body.jit")
        self.adaptation = Mlp(weights_dir / "adaptation.jit")
        self.dt = float(dt)
        self.reset()

    def reset(self):
        self.history = np.zeros(OBS_DIM * HISTORY, dtype=np.float32)
        self.actions = np.zeros(12)
        self.last_actions = np.zeros(12)
        self.gait_index = 0.0
        self.clock = np.zeros(4)

    # 一帧观测，顺序照 `lcm_agent.get_obs()`。
    def observe(self, gravity, dof_pos, dof_vel, commands):
        return np.concatenate((
            gravity,
            commands * COMMAND_SCALE,
            (dof_pos - DEFAULT_Q) * DOF_POS_SCALE,
            dof_vel * DOF_VEL_SCALE,
            np.clip(self.actions, -CLIP_ACTIONS, CLIP_ACTIONS),
            np.clip(self.last_actions, -CLIP_ACTIONS, CLIP_ACTIONS),
            self.clock,
        )).astype(np.float32)

    def push(self, obs):
        self.history = np.concatenate((self.history[OBS_DIM:], obs))

    # 前向：历史 → 隐变量 → body。返回原始 12 维动作。
    def act(self) -> np.ndarray:
        latent = self.adaptation(self.history)
        return self.body(np.concatenate((self.history, latent)))

    # `lcm_agent.publish_action()`：动作换成 q_des（策略序），再交给驱动器 PD。
    def publish(self, raw_action):
        self.last_actions = self.actions
        self.actions = np.clip(raw_action, -CLIP_ACTIONS, CLIP_ACTIONS)
        q_des = self.actions * ACTION_SCALE
        q_des[[0, 3, 6, 9]] *= HIP_SCALE
        return q_des + DEFAULT_Q

    # 时钟：foot_indices 的顺序是 (FL, FR, RL, RR)，与 `lcm_agent.step()` 一致。
    def advance_clock(self, commands):
        freq, phase, offset, bound = commands[4], commands[5], commands[6], commands[7]
        self.gait_index = (self.gait_index + self.dt * freq) % 1.0
        g = self.gait_index
        feet = (g + phase + offset + bound, g + offset, g + bound, g + phase)
        self.clock = np.sin(2 * np.pi * np.array(feet))


def overlay(img, lines):
    h = 22 * len(lines) + 12
    strip = img.copy()
    cv2.rectangle(strip, (0, 0), (WIDTH, h), (24, 24, 24), -1)
    cv2.addWeighted(strip, 0.55, img, 0.45, 0, img)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (14, 26 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (235, 235, 235), 1, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="用参考仓库的部署链跑它的策略")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--hold", type=float, default=0.6, help="接管前按 DEFAULT_Q 站多久")
    ap.add_argument("--vx", type=float, default=0.4)
    ap.add_argument("--vyaw", type=float, default=0.0)
    ap.add_argument("--height", type=float, default=0.0)
    ap.add_argument("--freq", type=float, default=3.0)
    ap.add_argument("--phase", type=float, default=0.0,
                    help="四路时钟的相位差。0 = 四条腿同相（跳/pronk），"
                         "0.5 = 对角两组（小跑）")
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--bound", type=float, default=0.0,
                    help="前后两组分开的相位差，0.5 = 前腿一组、后腿一组（bound）")
    ap.add_argument("--footswing", type=float, default=0.08)
    ap.add_argument("--width", type=float, default=0.33)
    ap.add_argument("--length", type=float, default=0.40)
    ap.add_argument("--out", default="docs/ref_gait.mp4")
    ap.add_argument("--joint-damp", type=float, default=None,
                    help="覆盖模型的关节阻尼（go2.xml 是 0.3）。对比它训练用的 URDF "
                         "时会用到 —— 那边没有这一项")
    ap.add_argument("--joint-armature", type=float, default=None,
                    help="覆盖电枢惯量（go2.xml 是 0.015）。训练用的 URDF 里没有这一项")
    ap.add_argument("--joint-friction", type=float, default=None,
                    help="覆盖模型的关节干摩擦（go2.xml 是 0.3）")
    ap.add_argument("--kp", type=float, default=KP,
                    help="关节刚度。默认 20 = 部署链（真机 LowCmd）的值")
    ap.add_argument("--kd", type=float, default=KD)
    ap.add_argument("--torque", default="pd", choices=("pd", "net"),
                    help="pd = 理想 PD kp20/kd0.5（部署链），"
                         "net = 训练环境的电机模型 actuator_net（训练链）")
    ap.add_argument("--flat", action="store_true",
                    help="换成没有台阶的平地。默认 scene.xml 在 x=1.2 m 有两级 8 cm "
                         "台阶、后面还有楼梯，走不到 1 m 就被挡住")
    ap.add_argument("--azimuth", type=float, default=130.0)
    ap.add_argument("--elevation", type=float, default=-18.0)
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()

    scene = PS.flat_scene() if a.flat else PS.SCENE
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = PS.DT
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    if a.joint_armature is not None:
        model.dof_armature[:] = a.joint_armature
    if a.joint_damp is not None:
        model.dof_damping[:] = a.joint_damp
    if a.joint_friction is not None:
        model.dof_frictionloss[:] = a.joint_friction
    data = mujoco.MjData(model)
    qadr, dadr, act = build_index(model)

    # 起始：站在 DEFAULT_Q、机身 0.30 m。参考实现的 init z 是 0.42（训练里是落体），
    # 部署时机器人是被架起来再放下的，所以这里从站高起步、不模拟下落。
    data.qpos[:] = 0
    data.qpos[2] = 0.30
    data.qpos[3] = 1.0
    data.qpos[qadr] = DEFAULT_Q
    mujoco.mj_forward(model, data)

    renderer = None
    video = None
    if not a.no_video:
        renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
        video = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))

    act_net = None
    if a.torque == "net":
        act_net = load_actuator()
        act_net.reset()
    policy = RefPolicy(dt=PS.DECIMATION * PS.DT)
    cmds = deploy_commands(vx=0.0, height=a.height, freq=a.freq, phase=a.phase,
                           offset=a.offset, bound=a.bound,
                           footswing=a.footswing, width=a.width, length=a.length)
    gravity, dof_pos, dof_vel = read_state(data, qadr, dadr)
    policy.push(policy.observe(gravity, dof_pos, dof_vel, cmds))

    hold_steps = int(a.hold / PS.DT)
    total_steps = hold_steps + int(a.seconds / PS.DT)
    q_des = DEFAULT_Q.copy()
    tau = np.zeros(12)
    x0 = None
    worst = 0.0
    fell_at = None
    walking = False
    print(f"{'t':>5} {'z':>6} {'tilt':>6} {'vx':>5} {'|a|max':>7} {'x':>7}")
    dec = PS.DECIMATION

    for step in range(total_steps):
        if step == hold_steps:
            cmds[0] = a.vx
            walking = True
            x0 = float(data.qpos[0])
            print(f"-- 接管：vx={a.vx} phase={a.phase} offset={a.offset} "
                  f"bound={a.bound} footswing={a.footswing}")
        # 状态每个物理步都读一份：力矩要用当拍的 q / dq 算，用 4 步前的就是 40 ms
        # 陈旧反馈。
        gravity, dof_pos, dof_vel = read_state(data, qadr, dadr)
        if step % dec == 0:
            # 顺序照 `lcm_agent.step()`：先读状态组观测，再由观测算动作、当拍下发。
            # 反过来写（先 publish(act()) 再 push(observe())）会让 act 拿到上一拍的
            # 观测，整条链多一拍延迟 —— 那不是部署侧的顺序。
            obs = policy.observe(gravity, dof_pos, dof_vel, cmds)
            policy.push(obs)
            if walking:
                q_des = policy.publish(policy.act())
                policy.advance_clock(cmds)

        # 力矩每个物理步都按当前 q / dq 重算，不从上一拍保持。
        #
        # 训练侧 motor 是隐式执行器，PhysX 每个物理子步（200 Hz）都按当时的关节状态
        # 算一次 PD；真机驱动器在里面以 kHz 级算。40 ms 保持一个力矩是两边都没有的事，
        # 等效刚度会低一大截 —— 实测同一策略同一 kp，保持 50 Hz 走 0.04 m，每步重算
        # 走 1.07 m。之前这里写成了「每 dec 步算一次」，把执行器做软了。
        if step % dec == 0 and act_net is not None:
            tau_policy = act_net.torque(dof_pos[POLICY_TO_SDK], q_des[POLICY_TO_SDK],
                                        dof_vel[POLICY_TO_SDK])
            tau = np.zeros(12)
            tau[POLICY_TO_SDK] = tau_policy
            tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        elif act_net is None:
            tau = a.kp * (q_des - dof_pos) + a.kd * (0.0 - dof_vel)
            tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        data.ctrl[act] = tau
        mujoco.mj_step(model, data)

        if step % dec == 0:
            rpy = PS.rpy_from_quat(np.array(data.qpos[3:7], dtype=float))
            t = tilt_rad(rpy)
            if walking:
                worst = max(worst, t)
                if t > 1.2 and fell_at is None:
                    fell_at = step * PS.DT - a.hold

        if renderer is not None and step % RENDER_EVERY == 0:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.2]
            cam.distance = 1.1
            cam.azimuth = a.azimuth
            cam.elevation = a.elevation
            renderer.update_scene(data, cam)
            img = renderer.render()
            dx = 0.0 if x0 is None else float(data.qpos[0]) - x0
            overlay(img, [
                f"ref {'train env' if act_net is not None else 'deploy'}  "
                f"t={step * PS.DT - a.hold:+5.2f}s  "
                f"cmd vx={cmds[0]:.2f}",
                f"tilt {math.degrees(t):5.1f} deg   height {data.qpos[2]:.3f} m   "
                f"x {dx:+.3f} m",
                f"action max {np.abs(policy.actions).max():5.2f}   "
                f"clock {np.round(policy.clock, 2)}",
            ])
            video.write(img)

        if step % 100 == 0:
            rpy = PS.rpy_from_quat(np.array(data.qpos[3:7], dtype=float))
            dx = 0.0 if x0 is None else float(data.qpos[0]) - x0
            print(f"{step * PS.DT - a.hold:+5.2f} {data.qpos[2]:6.3f} "
                  f"{math.degrees(tilt_rad(rpy)):6.1f} {cmds[0]:5.2f} "
                  f"{np.abs(policy.actions).max():7.2f} {dx:+7.3f}")

    if video is not None:
        video.release()
    if renderer is not None:
        renderer.close()

    dx = 0.0 if x0 is None else float(data.qpos[0]) - x0
    print(f"\n结果：位移 {dx:+.3f} m（命令 {a.vx} m/s × {a.seconds:.0f}s）  "
          f"最大倾角 {math.degrees(worst):.1f}°  "
          f"{'倒地于 %.2fs' % fell_at if fell_at is not None else '没倒'}")
    if not a.no_video:
        print(f"视频：{a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
