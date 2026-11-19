# Go2 遥控台（Flutter 客户端）

宇树 Go2 的手机 / 桌面 / 网页遥控与 VLA 观测客户端。连的是
`quadruped.comm.gateway`（WebSocket），不直接碰 DDS。

协议、模式闸门、安全阀的说明在 [`../docs/flutter-client.md`](../docs/flutter-client.md)。

## 跑起来

先在机器人那台机器上起网关：

```sh
uv run python -m quadruped.comm.gateway --preset sim
```

再起客户端：

```sh
flutter pub get
flutter run -d macos      # 桌面
flutter run -d chrome     # 网页
flutter run -d <安卓设备>  # 手机
```

真机、手机要连进来时，网关要监听所有网卡并设口令：

```sh
uv run python -m quadruped.comm.gateway --preset real --host 0.0.0.0 --token 123456
```

App 里填网关地址 + 口令即可；也可以直接带参数打开：

```
http://<网关主机>:<端口>/?host=192.168.123.1&port=8780&t=123456&tab=vla
```

## 五个页面

| 页面 | 干什么 |
|---|---|
| 连接 | 地址 / 端口 / 口令，以及网关的启动命令（带复制按钮） |
| 遥控 | 双摇杆、速度档、起立 / 蹲下 / 跳跃 / 换步态 / 急停，中间是大号速度读数 |
| VLA | 机载画面 + 八方位视野 + 判断 + 对话 + 频率与健康度 |
| 手柄 | 手柄检测、实时轴值、固定映射表、死区与曲线 |
| 设置 | 速度上限、画面开关、收工（蹲下）、断开 |

顶栏常驻「模式 / 链路 / 操作权 / 急停」，任何页面都点得到急停。

## 目录

```
lib/
  main.dart              入口（先读设置再建树）
  app_state.dart         全局装配：设置、连接、三路输入、驱动会话、深链接
  theme.dart             配色与字号（深色，数字等宽）
  core/
    models.dart          网关报文 → Dart 模型（字段与 runner.py 一一对应）
    gateway_client.dart  WebSocket、自动重连、请求/应答、细粒度 notifier
    settings.dart        持久化 + 死区/曲线
  input/
    intent.dart          ControlIntent：所有输入源共用的抽象
    drive_session.dart   20 Hz 下发、使能、急停、前提检查
    gamepad_source.dart  gamepads 包 → ControlIntent + 动作
    keyboard_source.dart WASD / QE / 空格 / Esc
  ui/
    app_shell.dart       顶栏 + 侧栏/底栏、急停、错误提示
    connect_page.dart    连接与命令参考
    drive_page.dart      遥控（窄屏单列 / 宽屏三列）
    vla_page.dart        VLA（宽屏三列 / 中屏两列 / 窄屏单列）
    gamepad_page.dart    手柄与手感
    settings_page.dart   设置
    widgets/             panel / joystick / vla_visuals / chat_feed / mode_switch
```

## 视觉

配色与结构参照 [FlClash](https://github.com/chen08209/FlClash)：近黑底 + 填充式
灰卡片（不画描边），胶囊 chip，桌面窄图标栏、手机悬浮胶囊底栏。

色值取它的派生结果：FlClash 用 `seededColorScheme(seed: #007AFF, dark)`，这里用
同一个种子跑 `ColorScheme.fromSeed`，得到页面底 `#0C0E13`、卡片 `#1A1B20`、
玻璃 `#282A2F`、主色 `#007AFF`。色阶偏蓝紫，是种子带过来的。

侧栏、顶栏、手机底栏是毛玻璃（`ui/widgets/glass.dart`）：半透明填充 + 上缘
0.67 px 高光 + `BackdropFilter`。设置里有开关，关掉退化为纯填充。

中文字体内嵌（`assets/fonts/`，按项目用字裁到 909 字 / 654 KB）：Flutter Web
默认从 gstatic 现拉中文，首屏几秒是方块。

遥控页的相机背景会被压暗 0.16，数字另用阴影描边。仿真联调时用
`headless.py --pin-base` 钉住机身 —— 无窗口仿真控制回路只有 5~6 Hz，狗会倒成
仰躺，相机朝天就只剩一片蓝。

两处规矩写成了令牌，改皮肤只改一处：

- `theme.dart` 的 `R`：圆角只有三档 —— 容器 16、控件 12、小内块 10、胶囊 999。
  页面里不许再出现 `circular(13)` 这种随手值。
- 图标一律放进固定尺寸的盒子（chip 12、按钮 16、卡片标题 14、侧栏 48）。
  不同图标字形宽度不同，不定宽就一排对不齐；并排按钮的标签也统一字数。

## 几个刻意的选择

**使能默认关。** 摇杆随便推，没使能不发速度。手机在口袋里被蹭到不该让狗走。

**模式不是选项卡。** 「手动」和「VLA」之间是交接控制权，切换时先 park 一方，
中间那几秒狗是蹲着的。界面上会显示正在做什么，而不是一个转圈。

**速度不自己写死上限。** 上限从网关下发的 `limits` 读（vx 1.0 / vy 0.30 /
vyaw 4.0），App 只调「用其中的百分之多少」。

**不引状态管理库。** 10 Hz 的遥测用 `ValueNotifier` 精确到字段，用户操作用一层
`ChangeNotifier`。两层够用，再多一层只会让人搞不清谁在重建。

## 校验

```sh
flutter analyze
flutter test
```

联调不想开 MuJoCo 时，用 `../test/ui_seed.py` 起一个假后端（假机器狗 + 假 VLA +
合成画面）：

```sh
uv run python test/ui_seed.py --port 8780 --token demo
```
