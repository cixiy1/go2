# 控制权归属

这个仓库里有四类进程、三种「能让狗动」的入口、两条「只是看」的链路。它们之间
的界限不是靠约定，是靠进程边界和 `test/arch_check.py` 钉住的。

一句话总则：**能写 `rt/lowcmd` 的只有三个，且同一时刻只允许一个。**

## 一、谁能写 lowcmd

| 进程 | 入口 | 怎么写的 | 谁负责它的生死 |
|---|---|---|---|
| `comm.server` | `python -m quadruped.comm.server` | `Backend` → `Go2Controller` / 官方 `SportClient` | 网关（自己起的）或用户（`--robot attach`） |
| VLA | `python -m quadruped.control.embodied.vla` | `PolicyBackend` 直连 `rt/lowcmd` | 网关的 `vla_host`；旧路径是 `web/app.py` |
| 前台控制器 | `python main.py` | 同上，`Go2Controller` | 用户自己（Ctrl+C） |

三者互斥。同时开两个，表现不是报错，是腿一抽一抽 —— 两条指令流交替写目标，
电机会跟着走。所以网关切模式时先 `park` 一方再起另一方，中间狗是蹲着的。

## 二、谁只是看

| 进程 | 入口 | 读什么 | 写什么 |
|---|---|---|---|
| 相机馈送 | `python -m quadruped.percept.feed` | videohub（真机）/ UDP 视频（仿真） | 一个 JPEG 文件 |
| 网关 | `python -m quadruped.comm.gateway` | `comm.server` 的状态、VLA 的 UDP 事件、上面那个 JPEG | WebSocket 帧、子进程信号 |
| Flutter 客户端 | `flutter run` | 网关的 WebSocket | 只发请求，不接线 |

前两个不持有 `rt/lowcmd`，也不需要 —— 相机与遥测在接管之后仍然可读（位姿话题
除外，见 `runner.py` 里 2026-11-17 那组实测）。

## 二点五、换控制端：同一类写者内部换人

`official` / `ours` / `policy` 都是「能写」的那一类，换它们不改变「谁能写」，
只改变「谁在解释 vx/vy/vyaw」。但三种后端在 `connect()` 时装配好，没有热插拔，
所以换控制端 = 重启 `comm.server`：

```
请求 → 停步 → 收回操作权 → park 旧服务端 → 用 --client 起新服务端 → BalanceStand
```

四道守卫，缺一条都会留下「名义上还在控制、实际上链路已经换了」的窗口：

| 守卫 | 拒绝理由 |
|---|---|
| 必须在手动模式 | VLA 自己就是一种控制端，切它没意义 |
| 服务端必须是网关起的 | 别人起的用他自己的 `--client`，我们既改不了也不该杀 |
| kind 必须在该端点的 `clients` 里 | 仿真桥没有运动服务，选官方只会起一个立刻退出的进程 |
| 已经是要切的那个 | 幂等，不重启 |

`clients` 由 `comm/server.py` 从 `clients_for(preset)` 上报，网关和 App 都不抄一份表 ——
两边各抄一份的结果是「App 上能点、网关上被拒」。

## 三、一条 `move` 经过谁

```
摇杆/手柄 → ControlIntent → DriveSession(20 Hz)
   → ws setpoint → Gateway._dispatch
       ├─ 检查操作权（lease）
       ├─ 检查模式（必须 manual）
       ├─ 夹取到 limits
       └─ 存进 self.setpoint（带时间戳）
   → Gateway._drive_loop(20 Hz) → RobotLink.call("move") → comm.server → Backend
```

四道闸门，每一道都能单独让这条指令停下：**没有操作权**（403）、**模式不对**（409）、
**超过上限**（夹取，不拒绝）、**250 ms 没新值**（驱动循环发零速）。

## 四、模式交接

```
off ──切手动──→ 起 comm.server（或 attach 现成的）──→ manual ──→ cam_host.start()
manual ──切 VLA──→ cam_host.stop() → park（停步→蹲下→阻尼→退出）──→ vla_host.start()
vla ──切手动──→ vla_host.stop()（SIGINT，等 25 s）──→ robot_link.open() ──→ cam_host.start()
```

顺序不能改的三处，原因都在文件里写着：

- **先停相机再起 VLA**：真机 videohub 只认一个订阅者，不停的话 VLA 拿不到帧，
  而且不报错（`GetImageSample` 返回 `code != 0`）。
- **先 park 再起 VLA**：服务端不断开就一直握着 `rt/lowcmd`。
- **先停 VLA 再连服务端**：VLA 收尾要蹲下，这时候服务端 `BalanceStand` 会跟它抢腿。

## 五、静态边界（`test/arch_check.py`）

| 规则 | 防的是什么 | 文件范围 |
|---|---|---|
| 1 内部不依赖通讯层 | 同一进程里既开 DDS 又连套接字 | `quadruped/` 除 `comm/` |
| 2 通讯层模块级只 import 标准库 | `import quadruped.comm.client` 拖进 SDK | `comm/*.py` |
| 3 常量模块保持轻 | `limits.py` 拉起 numpy 或控制器 | `limits.py` |
| 4 推理层不认识控制栈 | 策略侧顺手调 `move()` | `motion/` |
| 5 DDS 初始化只在进程入口 | 进程单例被第二个模块调，配置静默失效 | 全仓 |
| 6 子进程只在三个所有者里起 | 「谁杀了谁」出现第四个答案 | 全仓 |
| 7 感知层模块级不碰控制栈 | 没有 SDK 的机器上 import 不了 | `percept/` |
| 8 编排只有网关一处 | host 互相拉起、反向依赖网关 | `comm/` |

规则 5~8 的白名单写在脚本顶部（`DDS_ENTRY` / `PROCESS_ENTRY` / `HOST_MODULES`）。
**加一条新链路，先改白名单，再写代码** —— 白名单没动就说明这条链路还没被认领。

## 六、加一条控制链路要改哪儿

以「语音直控」为例：

1. `comm/gateway.py` 的 `_switch_*` 三个函数：什么时候起、什么时候停、和谁互斥。
2. 新 host（如果它要常驻）：`comm/xxx_host.py`，用 `comm/child.py` 起进程。
3. `comm/__init__.py` 和本文件的表：它属于「能写」还是「只看」。
4. `test/arch_check.py`：如果它引入新的进程或 DDS 初始化，进白名单。
5. `test/gateway_check.py`：模式切换顺序要不要加一条断言。

不需要改的：`protocol.py`（命令表是机器人的语义，不是链路的）、`server.py`、
Flutter 客户端的传输层。

## 七、已知例外

- `web/app.py` 仍然自己 `Popen` 一个 VLA（网页控制台的老路径）。它和网关是
  **两套互不了解的入口**，同时开就是两个 VLA —— 谁先起谁先写 lowcmd。用网关时
  把网页控制台关掉。
- `cli/launcher.py` 起仿真器和控制器，是开发入口，不参与「App 控制」这条链。
- `quadruped/sim/unitree_mujoco/` 是第三方代码（及其示例），规则 5 扫到它时跳过。
- 网关的 `percept.feed` 在**仿真**里读的是 `percept_pub.py` 的 UDP 视频流；
  那个端口只能被一个进程绑，所以手动模式有画面、VLA 模式也有画面，两者不同时。
