# unitree_mujoco 的本地改动

`quadruped/sim/unitree_mujoco/` 是 upstream 克隆，被 `.gitignore` 排除，不进本仓库。
但我们在里面改过 3 个文件，**直接重新 clone 会把它们全部冲掉**，所以改动以 patch
形式存在这里，可追踪、可重现。

`controller.py` 的 `MODEL_PATH`、`test/sim_check.py` 的 `scene.xml` 都指向这个克隆，
模型取不到时 RL 环境和 IK 校验都起不来。

## 恢复

```sh
cd quadruped/sim
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco
git apply ../patches/0001-sim-realism-bridge.patch
git apply ../patches/0002-go2-model-realism.patch
# macOS 上跑这条，否则 git status 永远脏（见下）
git update-index --skip-worktree unitree_robots/go2w/assets/terrain.stl
```

克隆的基线是 `1eb6642`（upstream master）。patch 按这个基线的上下文写，
upstream 改动同名文件后 `git apply` 会报冲突，此时用 `git apply -3` 三方合并。

## 两个 patch

**`0001-sim-realism-bridge.patch`** — `config.py` + `unitree_sdk2py_bridge.py`

- `config.py`：`INTERFACE` 改 `lo0`（macOS 回环叫 lo0，Linux 才叫 lo）、
  `USE_JOYSTICK = 0`（没接手柄时 `SetupJoystick` 会抛 `pygame.error` 中断）、
  新增 `REALISM` 一整套开关。
- `unitree_sdk2py_bridge.py`：真机化。`lowcmd` 到关节出力加 5 ms 延迟队列、
  力矩指令过 150 Hz 一阶滞后、编码器读数叠噪声（`q` 1.5e-3 rad、`dq` 0.15 rad/s）。
  噪声只加在**控制器读到的**读数上，桥自己算 PD 用干净值。

  这一层不开，仿真里调好的增益一上机就抖，而且查不出为什么——真机有三样仿真
  默认没有的东西：延迟、噪声、有限带宽。

  环境变量可单独关掉做对照：`SIM_REALISM=0` / `SIM_LATENCY=0` / `SIM_NOISE=0` /
  `SIM_BW=0`。

**`0002-go2-model-realism.patch`** — `unitree_robots/go2/go2.xml`

关节按真机 Go2 填，不是理想电机：

| 参数 | 原厂 | 现在 | 依据 |
| --- | --- | --- | --- |
| `damping` | 0.1 | 0.3 | 反电动势 + 减速器油封，实测 0.2~0.4 N·m·s/rad |
| `armature` | 0.01 | 0.015 | 转子惯量按 6.33 减速比折算到关节侧 |
| `frictionloss` | 0.2 | 0.3 | 减速器静摩擦死区，决定「小力矩推不动腿」 |

足底 `friction` 0.4 → 0.9（橡胶足压硬地面典型 0.8~1.0），原厂的 0.4 对应很滑的
地面，狗迈步时稍抬头就会整体下滑，怎么蹬都推不上去。`solref` 收到 10 ms、
`solimp` 推到接近刚体，原厂默认接触太软，脚会陷进地面、落地冲击被吃掉。

改完这几项，Step 3 的全部 A/B 数据才成立——站立高度、打滑率、力矩帧间差都直接
受它们影响。

## macOS 上的 `terrain.stl`

上游仓库里同时有 `terrain.STL` 和 `terrain.stl`，两个不同的 blob。macOS 文件
系统大小写不敏感，磁盘上只能存在一个，于是无论 checkout 哪一个，另一个恒显示
modified。跟我们的改动无关，也无法真正还原。

`go2` 不用 `go2w` 的资产，用 `skip-worktree` 把这条从状态里摘掉即可。重新 clone
后要重设一次，否则 `git status` 会一直有一条噪声。

## 校验

```sh
cd ../../..   # 回项目根
uv run python -c "
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('quadruped/sim/unitree_mujoco/unitree_robots/go2/scene.xml')
print('mass %.3f  nu %d' % (sum(m.body_mass), m.nu))
print('damping      ', np.round(m.dof_damping[m.jnt_dofadr[1:4]], 3))
print('frictionloss ', np.round(m.dof_frictionloss[m.jnt_dofadr[1:4]], 3))
feet = [i for i in range(m.ngeom) if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE][1:]
print('足底摩擦     ', [round(float(m.geom_friction[i][0]), 2) for i in feet])
"
```

期望：`mass 15.206  nu 12`、`damping [0.3 0.3 0.3]`、`frictionloss [0.3 0.3 0.3]`、
`足底摩擦 [0.9, 0.9, 0.9, 0.9]`。
