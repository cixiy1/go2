# Flutter 客户端与网关

手机 / 桌面 / 网页三端共用一个 Flutter 客户端，客户端连的是新加的
`quadruped.comm.gateway`（WebSocket），不直接碰 DDS。

```
Flutter App ──ws://──> gateway ──unix socket──> comm.server ──DDS──> 机器狗
                         │
                         └── 子进程 ──> vla（PolicyBackend 直连 rt/lowcmd）
```

## 为什么中间要加一层

`comm/server.py` 是单客户端：第二个连接直接回 409。手机、平板、桌面同时看的时候
这条规则把人挡在门外，而遥控本来就需要「一个能动的 + 若干个只能看的」。

网关解决三件事：

1. **多路复用**：一条到机器人的连接，扇出给 N 个 App。只有持有操作权的那个能发
   速度指令，其余只读。
2. **模式闸门**：手动遥控与 VLA 自主互斥。`rt/lowcmd` 同一时刻只能有一个发送者，
   切模式时必须先把一方 park 掉、再把控制权交给另一方。
3. **VLA 托管**：起停 VLA 子进程、转发人说的一句话、把事件和画面转给 App。

## 启动

```sh
# 仿真（机器人那台机器上）
uv run python -m quadruped.comm.gateway --preset sim

# 真机，手机要连进来
uv run python -m quadruped.comm.gateway --preset real --host 0.0.0.0 --token 123456

# 已经手动起了 comm.server，只想接上去
uv run python -m quadruped.comm.gateway --robot attach
```

客户端：

```sh
cd flutter-client
flutter run -d macos      # 桌面
flutter run -d chrome     # 网页
flutter run -d <安卓设备>  # 手机
```

也可以带参数直接进某一页（网页刷新、发链接给别人都用得上）：

```
http://<网关主机>:8099/?host=192.168.123.1&port=8780&t=123456&tab=vla
```

`tab` 取值：`drive` / `vla` / `gamepad` / `settings`，不给就是连接页。
遥控页背景可以一起指定：`bg=camera|radar|none`、`bgsize=main|small`。

## 界面

宽屏（1440×900）：左列是它看到的世界，中列是决策与对话，右列是命令与健康度。

![VLA 宽屏](img/flutter-vla-desktop.png)

手机（414×896）：画面在上，判断与雷达并排，对话占下半屏。

![VLA 手机](img/flutter-vla-phone.png)
![遥控手机](img/flutter-drive-phone.png)

遥控页的背景可以是相机画面或八方位雷达，主视图铺满摇杆区、小视图缩到右上角。
左上角那三个字（相机 / 雷达 / 关）加一个图标按钮就是全部开关，选择存本地。

![遥控背景](img/flutter-drive-bg.png)

### 配色与玻璃

色值不是照着 FlClash 的截图调近似值，是取它的派生结果：FlClash 的主题为
`seededColorScheme(seed: Color(0xFF007AFF), dark)`，用它在本机跑
`ColorScheme.fromSeed` 得到同一套 surface 色阶：

| 用途 | 色值 | M3 对应 |
|---|---|---|
| 页面底 | `#0C0E13` | surfaceContainerLowest |
| 侧栏 / 顶栏玻璃 | `#282A2F` @ 62% | surfaceContainerHigh |
| 卡片 | `#1A1B20` | surfaceContainerLow |
| 卡片上的块 | `#282A2F` | surfaceContainerHigh |
| 进度槽 / 浮层 | `#33353A` | surfaceContainerHighest |
| 主色 | `#007AFF` | 种子色本身（不是派生的 `#ADC6FF`） |

色阶整体偏蓝紫，这是种子带过来的；照抄中性灰会「看着差不多但就是不像」。

**主色分两个令牌，别混**（`theme.dart`）：

| 令牌 | 值 | 用在哪 |
|---|---|---|
| `C.accent` | `#007AFF` 种子原值 | 实心色块（按钮底、进度条、选中指示） |
| `C.accentText` | `#6FB4FF` | 深底上的主色**文字与图标** |

`#007AFF` 的蓝光分量低，压在 16% 蓝底上当字用只有 2.5:1 的对比 —— 「连接」
「手动遥控」「自研」这些看着像没写上去（截图放大后就是一块蓝）。M3 在暗色
主题把 primary 提到 tone 80 是同一个道理。连接页的主按钮另改成实心主色 +
白字，这是页面上唯一的主操作，不该靠淡底表达。

玻璃在 `ui/widgets/glass.dart`：半透明填充 + 上缘 0.67 px 高光（`Border.all`，
`foregroundDecoration` 画，不用 Stack —— 顶栏那种「只有 top 没有 bottom」的
`Positioned` 里，`StackFit.expand` 会把高度算成 0，整条栏消失）+ `BackdropFilter`。

三条踩过的坑，改之前先看：

1. **`Scaffold` 不能给 `Colors.transparent`。** Web 上 CanvasKit 的底是白画布，
   `BackdropFilter` 采到那层白，侧栏能亮到 `#4a4d52`。给不透明底色。
2. **不要给背景加「光晕」当折射源。** 试过两团 alpha 0.12 的蓝紫径向渐变，
   整块背景被抬成雾灰蓝，卡片和玻璃一起发白 —— 页面像隔着脏玻璃看。FlClash
   通透是因为背后有 macOS 壁纸，我们没有，就别硬造光源。
3. **纯色背景上模糊看不出差别**（逐像素比对过开关两版），玻璃的观感靠
   「比背景亮一档 + 上缘高光」。真要看到模糊，得让背后有内容：遥控页那个
   压在相机画面上的「相机 / 雷达 / 关」切换条就是。

设置页有「毛玻璃」开关（`settings.glass`），关掉退化成纯填充，布局不变。

### 字体

Flutter Web 的中文是从 `fonts.gstatic.com` 现拉的，首屏那几秒全是方块（实测
8 秒还没到）。`assets/fonts/` 里是按项目实际用字裁的子集（909 字，三个字重
共 654 KB），`theme.dart` 里 `fontFamily: 'NotoSansSC'`。要加新文案发现缺字，
重新跑一遍子集：

```sh
pyftsubset noto.ttf --text-file=glyphs.txt --output-file=assets/fonts/NotoSansSC-400.ttf
```

### 遥控页的相机背景

画面「看起来没有」有两个完全不同的原因，先分清再改代码：

| 现象 | 原因 | 怎么确认 |
|---|---|---|
| 整块纯黑，左下角没有「机载相机」标签 | 没有帧到 App | `test/video_check.py` 数帧；0 帧再看 `cam_host` 状态里的死因 |
| 有标签但画面全蓝、或极暗看不清 | 仿真里的狗翻了/画面被压太暗 | 看遥测 rpy：roll≈±180° 就是仰躺，相机朝天 |

无窗口仿真里控制回路只有 5~6 Hz（物理步长 5 ms、DDS 收发和渲染挤在一个进程），
`BalanceStand` 之后十几秒内必倒，而且倒成仰躺 —— 相机只剩天空。

接线和演示时用 `--pin-base`：**只锁姿态，三个平移方向只给阻尼**。

```sh
cd quadruped/sim/unitree_mujoco/simulate_python
SIM_VIDEO_HZ=20 python headless.py --seconds 7200 --pin-base
```

它解决的是「无窗口仿真控制回路跑不快，狗会翻成仰躺、相机朝天」这一个问题：
姿态用 PD 力锁住不会翻，机身高度和位移交给腿。**不能把机身吊住** —— 试过用
3500 N/m 的位置弹簧锁 z，结果腿的承重被卸掉、地面法向力不够、腿一直打滑：
指令 0.4 m/s，4 秒只挪 2.5 cm，看起来就是「遥控没反应」。

`--pin-hard` 保留硬固定（每步写回位姿），只给逐像素比对截图用。

已知限制：仿真里实际速度比指令慢很多（步态参数和物理参数没对齐），真机按
控制器算出来的速度走。这条属于控制器/仿真标定，不是客户端的范围。

## 报文

一条连接上只跑 JSON 文本帧（另有二进制帧专用于画面）。所有 `op` 都是小写。

### App → 网关

| op | 字段 | 说明 |
|---|---|---|
| `hello` | `token` `name` `proto` | 连接后第一帧。口令不对回 `error 401` 并断开 |
| `lease` | `action: acquire\|release` | 取/还操作权。被占用回 409 和占用者名字 |
| `setpoint` | `vx` `vy` `vyaw` `source` | 速度设定值。只接受操作权持有者，20 Hz 发 |
| `cmd` | `cmd` `args` | `stand / uncrouch / crouch / stop / freeze / jump / gait / move` |
| `mode` | `mode: manual\|vla\|off` | 切模式。立刻回 `switching`，进度靠状态帧和事件 |
| `controller` | `kind: official\|ours\|policy` | 换高层控制端。只在手动模式下、且服务端是网关起的才行；会重启服务端 |
| `vla` | `action: start\|stop` | 停止 VLA（等价于切 `off`） |
| `say` | `text` | 人对 VLA 说一句话（写进它的 stdin） |
| `camera` | `on` | 是否接收画面 |
| `log` | `n` | 取 VLA 最近 n 行日志 |
| `ping` | `ts` | 心跳。App 每 2 秒发一次 |

### 高层控制端

三个控制端是「谁来解释 vx/vy/vyaw」，不是三档速度：

| kind | 名字 | 能力边界 |
|---|---|---|
| `official` | 官方 SportClient | 真机原生运动服务，39 个动作；仿真没有 |
| `ours` | 自研 HighLevelController | 200 Hz 回路 + MPC/WBC，动作最全 |
| `policy` | 推理模型 | 步态策略直连 `rt/lowcmd`，没有高层动作 |

界面上在遥控页左栏「机器人」卡片里，一行三个芯片（官方 / 自研 / 推理），
当前那个高亮、这个端点没有的置灰。点一下弹确认 —— 因为要重启服务端。

![高层控制端](img/flutter-drive-controller.png)

`state.link.controller` 是**服务端报的实际值**，`state.link.available` 是这个端点
能选的列表（仿真只有 ours / policy）。换一个要重启 `comm.server`：狗先蹲下再站起来，
操作权会被收回。网关自己起的服务端才能换；`--robot attach` 时只能由那个服务端的
`--client` 决定。

### 网关 → App

| op | 内容 |
|---|---|
| `welcome` | 上限、当前模式、VLA 状态、最近一帧 VLA 事件、对话历史 |
| `state` | 10 Hz：`mode`、`lease`、`link`、`robot`、`input`、`vla`、`limits` |
| `vla` | 5 Hz：VLA 事件帧（见下表） |
| `ack` | 请求应答，`id` 原样带回 |
| `event` | `kind: mode\|lease\|error\|note`，一句话 |
| 二进制帧 | 1 字节来源（`1`=VLA 渲染窗口，`2`=机载相机）+ JPEG，5–8 Hz |

### VLA 事件帧（`runner.py` 的 `sink.publish`）

| 字段 | 含义 |
|---|---|
| `cmd.{vx,vy,vyaw}` `gait` | 这一拍下发的命令 |
| `sectors[]` | 八方位最近障碍（`name` / `gap` / `top`），`front_gap` 是正前 |
| `scene_text` `scene_age` | 视觉模型对画面的描述，以及它有多旧 |
| `thought` `reply` `reply_seq` | 这一轮的判断、对用户说的话、回话序号 |
| `guard.{trips,last}` | 安全闸触发次数与最后一次原因 |
| `policy_hz` `reflex_hz` `gen_s` `view_fps` | 策略 / 反射回路 / 单次推理 / 渲染频率 |
| `pos` `yaw_deg` `tilt_deg` | 位移（真机接管后为 null）、朝向、倾角 |
| `vision.{frames,dropped,views}` | 收图、丢帧、视觉描述次数 |
| `alert` | 安全闸正在介入 |

字段改动要两边一起改：`quadruped/control/embodied/runner.py` 与
`flutter-client/lib/core/models.dart`。

## 画面从哪来

两个源，一个出口。网关按当前模式选：

| 模式 | 画面 | 谁在取 |
|---|---|---|
| 手动 | 纯相机图（`FRAME_CAM`） | `percept.feed` 子进程，网关在切手动时拉起 |
| VLA | VLA 渲染窗口，带想法字幕和视野条（`FRAME_VLA`） | VLA 自己（`--stream`） |

第一字节标明来源，App 在画面角上标出来。**这张图不能当实景判断**：VLA 那张是
渲过的合成图，字幕压着画面底部。

手动模式之所以要单起一个进程取相机：`ChannelFactoryInitialize` 一进程一次，
相机和控制链路挤在一起，谁先初始化另一条就被绑在那套 domain/网卡上，而且不报错。
详见 `docs/control-ownership.md`。

## 模式状态机

```
off ──切手动──> 起 comm.server（或接已有的）──> manual
manual ──切 VLA──> park（停步→蹲下→阻尼→服务端退出）──> vla
vla ──切手动──> SIGINT 给 VLA（等 25 s 收尾）──> manual
manual|vla ──收工──> off
```

切换期间 `mode = switching`，`note` 写正在做什么。App 这两个按钮不是选项卡，
是交接控制权。

`park` 之后机器人是蹲下 + 阻尼的状态。VLA 起来时 `PolicyBackend` 会自己站回来，
手动模式则靠 `comm.server` 的 `ensure_standing()`。

## 三条安全阀

1. **操作权**：同一时刻只有一个 App 能发速度。别人要控制得先请求，被拒时看到的是
   「iPhone 正在控制」。
2. **设定值过期**：250 ms 内没收到新的设定值就发零速 + `stop`。手机锁屏、WiFi 断、
   进程被杀，最坏结果是停下。
3. **断线即停**：持有操作权的客户端断开，立刻停车并释放操作权。

App 侧还有一层：**使能开关默认关**。摇杆随便推，没使能一个字节都不发；
模式或操作权一变，自动解除使能并补一帧零速。

## 手柄

物理手柄接在跑 App 的那台设备上，经 `gamepads` 包归一化成统一轴/键，再走同一套
`ControlIntent` 抽象层，最后由网关转给机器人。手柄只负责产生「意图」，不直接产生
DDS 报文。

| 输入 | 动作 |
|---|---|
| 左摇杆 | 平移（前后 / 左右） |
| 右摇杆横向 | 转向 |
| A / B / X / Y | 跳跃 / 急停 / 蹲下 / 换步态 |
| Start | 使能开关 |
| 十字键 ↑ / ← → | 起立 / 速度档减增 |
| 键盘 WASD·QE·空格·Esc | 平移转向 / 跳跃 / 急停 |

安卓需要在 `MainActivity` 里把 `KeyEvent` / `MotionEvent` 转给插件（已改好）。
iOS 走 `GCController`，只认标准布局的手柄。Web 要用户先按一下手柄，浏览器才会
把设备交给页面。

**别和仿真器自带的手柄同时用。** `unitree_mujoco` 里也有一套手柄（发布
`rt/wireless_controller`），两条链路同时下指令会互相覆盖。

## 控制权

谁能让狗动、谁能只看、每条边界由哪条规则检查，都在
[`control-ownership.md`](control-ownership.md)。

## 校验

```sh
uv run python test/gateway_check.py     # 33 项：帧编解码、租约、模式闸门、过期停车
uv run python test/arch_check.py        # 通讯层只准模块级 import 标准库
cd flutter-client
flutter analyze
flutter test                            # 报文解析、手感曲线、摇杆符号
```

`test/ui_seed.py` 是联调用的假后端：假机器狗 + 假 VLA + 合成画面，让界面在不开
MuJoCo、不加载模型的情况下有数据可看。

```sh
uv run python test/ui_seed.py --port 8780 --token demo
# 浏览器打开 http://127.0.0.1:<静态服务器>/?host=127.0.0.1&port=8780&t=demo&tab=vla
```

## 这一层不做什么

- 不转发 `forward` / `turn`：那是阻塞调用，几秒内会卡住状态轮询。
- 不转发 `park`：那是网关自己的生命周期，不该由手机决定。
- 不做规划、不排队、不改速度指令的语义，只做上下限夹取（复用 `quadruped.limits`）。
- 不支持 TLS。局域网内用；要跨网就套一层 SSH 隧道或反代，别把 `8780` 暴露到公网。
