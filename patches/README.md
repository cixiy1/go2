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

## 补丁里有什么

- **unitree_mujoco**：`config.py` 改回环口为 `lo0`、关手柄；`unitree_sdk2py_bridge.py` 加真机特性（指令延迟 5 ms、编码器噪声、力控带宽 150 Hz）；`go2.xml` 按真机填关节阻尼/转子惯量/静摩擦 0.3，足底摩擦 0.4→0.9 并收紧接触刚度。
- **unitree_sdk2_python**：`example/go2/high_level/go2_sport_client.py` 与 `utils/thread.py` 的小修。

补丁是用 `git diff --binary` 生成的，含二进制文件（`go2w/assets/terrain.STL`）。
