# Go2 四足机器人

宇树 Go2 的控制栈。手写步态打底，上面叠 MPC 与 WBC 两层优化；仿真用 MuJoCo，同一套控制器代码跑仿真的 DDS 和真机。

不含强化学习策略训练。`quadruped/rl/` 那层目前只搜步态参数，搜出来的东西还没过真链路验收（见下文「已知问题」）。

## 跑起来

macOS，Python 3.10（`pyproject.toml` 里锁的是 `>=3.10,<3.11`）。

```sh
uv sync --group sim
```

这一条会把 `unitree_sdk2_python/` 以 editable 方式装进来（`[tool.uv.sources]` 里指了本地路径），不用另外 install。

### 一个终端全起来

```sh
uv run python main.py
```

回车三次，默认就是「仿真（一起起）→ MPC + WBC → 键盘遥控」。仿真器和控制器都在前台，Ctrl+C 一起收。

按键：`w/a/s/d` 平移，`q/e` 转向，空格跳跃，Ctrl 蹲起，`1~9` 调速度档。

### 两个终端

仿真器要单独看着的时候：

```sh
./quadruped/sim/run_sim.sh          # 终端 A，带窗口
uv run python main.py keyboard      # 终端 B
```

`run_sim.sh` 用 `mjpython` 起，因为 macOS 要求 GUI 在主线程，普通 `python` 跑 `launch_passive()` 会崩。脚本会先 `cd` 到 `simulate_python/`，`config.py` 里的模型路径是相对那个目录的。

`main.py keyboard` 会列网口让你选，默认指向 `192.168.123` 网段的真机网卡。仿真要显式给 `--preset sim`：

```sh
uv run python main.py --preset sim keyboard
```

### 真机

网线直连时先配 USB 网卡。`setup_en5.sh` 里写死 `IFACE=en5`，实际枚举名可能不同，先 `ifconfig` 看一眼再改。

```sh
./setup_en5.sh static       # 本机 192.168.123.222/24，直连机器人
./setup_en5.sh              # DHCP，网线接交换机或路由器时用
```

然后 `uv run python main.py keyboard`，网口默认指向 `192.168.123` 那一个。

上电后先验链路。这一步只订阅 `rt/lowstate`，不发控制指令：

```sh
uv run python -m quadruped.cli.main telemetry
```

第一次跑步态建议把机器吊起来，或者手留在电源上。

### 直接跑子命令

```sh
uv run python -m quadruped.cli.main stand --duration 10
uv run python -m quadruped.cli.main forward 1.0 --vx 0.2
uv run python -m quadruped.cli.main turn 90
uv run python -m quadruped.cli.main trot --vx 0.2 --duration 8
```

`forward` 的单位是米，`turn` 是度（正为逆时针）。全部子命令：`stand` / `trot` / `act` / `forward` / `turn` / `demo` / `telemetry` / `selftest` / `hierarchy` / `state`。

不用连机器人也能跑的两条：

```sh
uv run python -m quadruped.cli.main selftest    # 运动学与步态自检
uv run python -m quadruped.cli.main hierarchy   # 打印 Robot→Controller→HighLevel 的嵌套关系
```

## 三层运控

`gait.mode = "mpc"` 时走三层，默认就是这个。`"vmc"` 是单层的旧路，留着回滚和 A/B。

| 层 | 频率 | 干什么 | 文件 |
|---|---|---|---|
| 步态 | 200 Hz | 相位、落脚点、速度闭环、足端 IK | `quadruped/control/controller.py` 的 `_gait_feet_publish` |
| MPC | 50 Hz（独立线程） | 单刚体模型，12 段 × 20 ms，预测 240 ms | `quadruped/control/mpc.py` |
| WBC | 200 Hz | 24 维 QP（12 关节力矩 + 12 足端力），带摩擦锥与力矩上限约束 | `quadruped/control/wbc.py` |

两条路的相位和落脚点生成完全共用，只在「发什么力矩」这一步分叉。

**两层管的不是同一件事。** MPC 的 240 ms 预测决定力和力分配，WBC 把这份 wrench 和摆动腿轨迹一起解成 12 个关节力矩。高度和姿态的快环保留在 200 Hz 的 `_mpc_trim` 里 —— 让 MPC 自己在 50 Hz 上闭这两路，`MPC_W_RPY` 从 1e2 扫到 1，roll 峰峰都停在 43° 以上，被 20~50 ms 的链路延迟卡住。

**MPC 的力矩不进 WBC**（`MPC_USE_TORQUE = False`）。它会和快环抢同一个姿态自由度：原地转 0.8 rad/s 时 roll 峰峰 19°，关掉之后 4°。

### 开关

三种给法，优先级从高到低：

```sh
uv run python main.py --motion vmc                      # 命令行
GO2_MOTION=vmc uv run python main.py                    # 环境变量
                                                        # 不写 = mpc
```

键位切档（Tab）也会带着当前力路走，不会切一次掉回 vmc。

站姿和摆姿态不走 MPC：`_mpc_stand_publish` 直接给 `F = [0, 0, mg]`、四脚均分体重。MPC 线程跟着 `Go2Robot.connect()` 起，两份 QP 在构造期就编译好（首次约 170 ms，同一进程后续 3 ms）——放进 200 Hz 回路就是几十帧超时。

## 目录

```
quadruped/
  control/        controller.py 是 200 Hz 主回路；mpc.py / wbc.py / qp.py 是三层里的后两层
                  model.py 存几何与动力学常量，RNEA 和质量矩阵的单一来源
    virtual/      仿真后端（DDS 端点、协议）
    physical/     键盘遥控
    high_level/   高层动作（站、走、转、跳）
  robot/body.py   Go2Robot：装配 + 生命周期，connect 时起 MPC 线程
  cli/            main.py 子命令 · launcher.py 统一入口 · prompt.py 交互
  rl/             离线步态参数搜索（env.py / gait.py / search.py），不参与运行时
  sim/            run_sim.sh + patches/（仿真器本地改动的补丁与恢复步骤）
test/             自检与实验脚本，见下
docs/             设计说明与图
```

## 自检

三个离线校验，不接机器人，几秒钟出结果：

```sh
uv run python test/model_check.py    # 模型常数与几个独立来源对账（RNEA / 质量矩阵 / 左右镜像）
uv run python test/mpc_check.py      # MPC 在已知状态上的预测与约束
uv run python test/wbc_check.py      # WBC 的约束满足与力矩分配
```

无界面闭环仿真 —— 把**真实的** `Go2Controller` 接到 MuJoCo 上，不走 DDS，复用同一个 `scene.xml` 和桥的 PD/真机化处理：

```sh
uv run python test/sim_check.py --ab --action walk --vx 0.20 --dur 6
uv run python test/sim_check.py --mode mpc --action walk --vx 0 --vy 0.20   # 横移
uv run python test/sim_check.py --mode mpc --action walk --vx 0 --yaw 0.8   # 原地转
uv run python test/sim_check.py --action stand --burst --dur 4              # 只量算力
```

`--ab` 同一进程里交替跑 vmc 与 mpc 并列表对比。`--burst` 不按墙钟节拍，只量每帧计算耗时 —— macOS 上 `sleep` 会把进程降到低 QoS，paced 循环里量到的是降频后的数字，不能用来判算力。

其余脚本：`gait_speed.py` 测某个步态实际跑多快，`grid_truth.py` 在真链路上扫参数网格，`real_diag.py` 把真机抖动拆成可对照的数，`dds_probe.py` / `dds_peer.py` 绕过组播做单播探测。

## 已知问题

**回路周期 p95 是 8~9 ms，标称 5 ms。** 单帧计算 1 ms 上下，差值是 50 Hz 的 MPC 线程和 200 Hz 主回路抢 GIL。降频、砍 horizon、调 `sys.setswitchinterval`、睡+自旋都试过，没一个稳定有效。这是上真机前要解决的第一条。

**mpc 行走时机身竖向起伏 7~9 mm**（vmc 4~5 mm），能量集中在 0~5 Hz，跟着步频走。MPC 频率、逐腿力低通、支撑腿位置任务比例、高度环 kz/dz 都试过，没有稳定收益。

**`search` 档的参数没过验收。** `quadruped/rl/out/gait_best.json` 是 duty 0.738，真链路实测 yaw 漂 −137°、速度反向 142%。`rl/env.py` 是简化被控对象（平地、左右对称、无摩擦各向异性），它的最优解落在「高 duty + 低抬脚」，而这两项在真实接触上正是打转与蹭地的来源。补齐环境和真值表的顺序记在 `TODO.md` 第四节。

## 第三方代码

**`unitree_sdk2_python/` 在仓库里**（上游 `unitreerobotics/unitree_sdk2_python` @ `814556d`）。原因是 `utils/thread.py` 上有 macOS 补丁：`timerfd` 是 Linux 专有接口，原实现在 import 阶段就失败，`unitree_mujoco` 的桥起不来。不带这份补丁，clone 下来在 macOS 上跑不了。

**`quadruped/sim/unitree_mujoco/` 不在仓库里。** 它有本地改动（桥的 5 ms 延迟队列、150 Hz 一阶滞后、编码器噪声；`config.py` 的 `lo0` 与关手柄；`go2.xml` 的关节阻尼与足底摩擦），改动导出成 patch 存在 `quadruped/sim/patches/`，重新 clone 后 `git apply` 两条命令还原。步骤见那个目录的 README。

`Yuki/` 与本项目无关，忽略。

## 相关文档

- `docs/go2-rl-guide.md` —— 为什么手写步态有天花板、训练环境为什么不接现有 DDS 链路
- `docs/go2-controller-post.md` —— 控制器设计的长文
- `TODO.md` —— 施工顺序、验收判据、每条没做完的理由
