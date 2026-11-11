# TODO

## 一、MPC + WBC 自研

现状：力矩层只有 VMC（机身 6 维力/力矩 → 逐腿 Jᵀf），缺上层优化、缺摆动腿惯性补偿，姿态靠 kp 静差兜着（kp 从 100 降不下去就是因为这个）。

四步，每步都能单独验收，不通过不进下一步。

- [x] **0 模型**（不碰机器）— 2026-11-11 完成，`quadruped/control/model.py`
  - 从 `quadruped/sim/unitree_mujoco/unitree_robots/go2/go2.xml` 抽真实惯量：trunk 6.921 / hip 0.678 / thigh 1.152 / calf 0.241 kg，整机 15.206408（原 `BODY_MASS = 15.0` 是估值，已改）
  - `model.py`：连杆长度、质心、转动惯量、3 自由度腿的 RNEA（M / C / g），解析雅可比
  - 验收（`test/model_check.py` 全过）：静态 RNEA 与 `gravity_tau` 差 1.7e-6（要求 < 1%）；与 MuJoCo `mj_rne` / `mj_fullM` 差 1e-13；`mj_kinematics` 差 8e-17
  - 耗时：单腿 RNEA（含 q̇）61 µs、M 224 µs，四腿 ~220 µs / ~900 µs，200 Hz 下占 4% / 18%
  - 顺带发现：`gravity_tau` 一直没算腿自重（站姿膝力矩 −0.27 N·m，是负载项的 4%），WBC 里要补上
  - 惯量张量必须 R·diag·Rᵀ 转回连杆系 —— XML 里机身那三个数绕 y 转了 90°，照抄会把偏航惯量当横滚

- [ ] **1 SRBD-MPC**（`quadruped/control/mpc.py`）
  - 状态 12 维（p, v, rpy, ω），决策变量 = 每个 knot 的四足三维地面反力
  - 接触序列直接取现有步态相位（period 0.5 / duty 0.5），不另做接触规划
  - 约束：fz ≥ 0、|fx,fy| ≤ μ·fz、fz ≤ FZ_MAX；代价：速度/高度/姿态跟踪 + 力正则
  - 依赖：`uv add osqp`
  - 验收（离线跑，无机器人）：Σfz = mg ± 0.1%、fz ≥ 0、单次求解 p95 < 3 ms、与 MuJoCo 单刚体一步积分对拍

- [ ] **2 WBC QP**（`quadruped/control/wbc.py`）
  - 决策变量：12 关节 τ
  - 任务：机身 6 维 wrench 跟踪（最高）、摆动腿足端轨迹（高）、姿态 + 零空间正则 `N·τ_posture`（低）
  - 硬约束：τ 限幅、摩擦锥、单边力
  - 验收：同一帧输入下比 `body_wrench_tau` 的力矩帧间差 p95 更低；kp 能降到 40 而站立高度不变

- [ ] **3 接线 + A/B**（开关 `gait.mode = "vmc" | "mpc"`，可回滚）
  - `_gait_feet_publish` 的相位/落脚点生成不动，只把 `_wrench_cmd` + `_tau_ff` + `body_wrench_tau` 三条力路换成 MPC→WBC 一条
  - 验收：仿真站 / 走 / 5° 横坡 / 侧向推，指标沿用现有那套（pitch-roll 峰峰、横移、位移、打滑率、力矩帧间差 p95、`loop_stats().p95` ≤ 4 ms）

- [ ] **4 真机**
  - 第一步只做站立 10 s，比对 `tau_est` 与 WBC 估算值，偏差 > 15% 先回来查模型，不要直接走
  - 第二步 0.15 m/s 走 5 s

### 会拖慢进度的三个点（按顺序）

1. **时序**：Python 200 Hz 只有 5 ms。必须两线程（MPC 50 Hz 独立，WBC 在 200 Hz 回路里），QP 矩阵结构固定、每帧只换 `b` 和上下界、开 warm start。门限用现成的 `loop_stats()`，p95 > 4 ms 不许上真机。
2. **状态估计**：MPC 要机身线速度，现在 `_v_filt` 是 IMU 加计积分、接触判断走计划相位。VMC 对速度误差不敏感，MPC 会把它滚成提前/滞后迈步。第一版 MPC 先只管高度和姿态、速度环留在外层，能省两天。
3. **接触力可行域**：双支撑时 6 个力分量勉强凑 6 维 wrench，条件数差（就是之前 6×3n 方程秩 5、解出 100 N 假力的根因）。MPC 靠 horizon 把力摊到多个 knot 上解决，前提是接触序列准。

### 原有遗留

- [ ] 横坡横滑：5° 坡 8 s 横移 94 cm，横向没有位置闭环
- [ ] 0.30 m/s 打滑失速：`speed_range` 已收到 0.22 规避，根因未解

## 二、机械步法回退到最初版本（2026-11-11 已完成）

`make_gait("mech")` 回到原样参数：period 0.5 / duty 0.5 / swing 0.06 / stride_max 0.075 / 零反馈 / 零速度闭环 / 站宽 0 / 不压地。真机档不再分叉，与仿真同一套。

撤回的改动（2026-11-10 17:25 之后陆续加的）：

| 参数 | 回退前 | 回退后 |
|---|---|---|
| duty | 0.65 | 0.5 |
| stance_spread | 0.035 | 0 |
| stance_sink | 0.006（真机 0.005） | 0 |
| speed_loop | 1.5 | 0 |
| stride_max | 0.12（真机 0.10） | 0.075 |
| swing_height | 0.09（真机 0.055） | 0.06 |
| pitch_gain / pitch_rate | −0.05 / −0.03 | 0 / 0 |
| speed_range | (0, 0.22) | None |

注意：`wrench = 1.0` 还在（那是 11-11 新加的机身层力分配，不属于步法）。要完全复现 13:18 的行为，把 mech 那行的 `wrench` 设成 0。

回退后的已知边界（不是新 bug）：双支撑时间为 0、横向稳定裕度为 0、单按 w 最快约 0.35 m/s、支撑腿不主动压地。

## 三、工程基建

- [ ] 仓库没有版本控制。做 MPC 之前先 `git init` + 首次提交，否则再回退只能靠猜（本次回退就是从 `.workbuddy/memory/2026-11-10.md` 的记录反推的，没有精确到分钟的快照）
