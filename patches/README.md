# 第三方仓库的本地改动

这两个目录是克隆来的，不纳入本仓库版本控制；本项目对它们做的修改以补丁形式存在这里。

| 目录 | 上游 | 基线提交 | 补丁 |
|---|---|---|---|
| `unitree_sdk2_python/` | unitreerobotics/unitree_sdk2_python | `814556d` | `unitree_sdk2_python.local.patch` |
| `quadruped/sim/unitree_mujoco/` | unitreerobotics/unitree_mujoco | `1eb6642` | `unitree_mujoco.local.patch` |

## 重新搭建

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git -C unitree_sdk2_python checkout 814556d
git -C unitree_sdk2_python apply ../patches/unitree_sdk2_python.local.patch

git clone https://github.com/unitreerobotics/unitree_mujoco.git quadruped/sim/unitree_mujoco
git -C quadruped/sim/unitree_mujoco checkout 1eb6642
git -C quadruped/sim/unitree_mujoco apply ../../../patches/unitree_mujoco.local.patch

uv sync --group sim
```

## 校验

补丁打完必须先核对模型指纹。`go2.xml` 是训练、`_check_gait`、真机力矩前馈共用的唯一模型来源，
它变了的话之前的 `GaitParams` 全部作废：

```bash
shasum -a 256 quadruped/sim/unitree_mujoco/unitree_robots/go2/go2.xml
# 期望 86a5745d7ac32762ae2aeb1c553b1bd593785781d48f58adf2f6b897acea2fa3
```

## 补丁里有什么

**unitree_mujoco**（四个文件）

- `simulate_python/config.py`：回环口改 `lo0`、关手柄。
- `simulate_python/headless.py`：新增无窗口仿真入口。`launch_passive` 只负责画图，物理与 DDS
  都在 `UnitreeSdk2Bridge` 里，去掉 viewer 就能在无显示器的会话里跑控制器。
- `simulate_python/unitree_sdk2py_bridge.py`：加真机特性（指令延迟 5 ms、编码器噪声、力控带宽）。
- `unitree_robots/go2/go2.xml`：关节阻尼 0.1→0.3、转子惯量 0.01→0.015、静摩擦 0.3；
  足底摩擦 0.4→0.9，缩短接触时间常数。

**unitree_sdk2_python**：`example/go2/high_level/go2_sport_client.py` 与 `utils/thread.py` 的小修。

## 维护

改完这两个目录里的文件，立刻重新生成补丁。否则换机器重建出来的环境与调参时用的不是同一份模型，
而症状是「同样的参数、不同的表现」，很难往回追。

```bash
cd quadruped/sim/unitree_mujoco
git add -N .                      # 让新增文件（如 headless.py）进 diff
git diff > ../../../patches/unitree_mujoco.local.patch
ruff check simulate_python        # 顺手看一眼，别把调试残留带进补丁
```

`gait_best.json` 的 `requires` 目前只记 `kp / kd / motor_bw / speed_range`，没有模型 hash。
换模型之后旧的搜索结果不会被自动判废，靠上面那条 `shasum` 人工核。
