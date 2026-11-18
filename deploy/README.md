# 真机部署（扩展坞）

目标：开机起来就有网页，从网页点一下就接管机器人；不点，狗留在官方运控手里。

## 为什么不是「开机就自动接管」

断电重启后这三件事都是未知的：狗是什么姿态（趴着 / 站着 / 侧翻）、旁边有没有人、
雷达和网线插了没。自动接管等于赌这三件事都正常 —— 赌输一次是机器狗自己往墙上走。
所以自启的只有**网页**，接管是人按的。

## 两步

### 1. 先在这台机器上确认环境

```sh
ip -br addr                       # 哪个口在 192.168.123.x，记下名字（en5 只是 macOS 的）
python3 -V                        # 3.8 会在 import 时炸（X | None 注解），install.sh 会先查
python3 -c "import unitree_sdk2py, cyclonedds"   # DDS 依赖
nvidia-smi                        # 有 GPU 但视觉暂时用不上，见下
```

**视觉这一路现在不吃 GPU。** `llm/vlm.py` 的默认 VLM 是远程的 `deepseek-flash`
（`is_remote()` 判定，只要 `DEEPSEEK_API_KEY` 和外网）；本机那个
`mlx-community/Qwen2-VL-2B-Instruct-4bit` 走 `mlx_vlm`，只有 Apple Silicon 能装，
Linux 上 import 就失败。要在扩展坞的 GPU 上跑本地视觉，得给 `LocalVlm` 加一个
transformers/CUDA 分支（torch ~2.5 GB + 权重 ~4 GB），这是独立一件事，没做。
没有外网时先 `--source proprio`：只有本体遥测，没有画面和测距。

仓库放到 `/opt/go2`（`install.sh` 按脚本所在目录认路径，放别处也行）：

```sh
sudo mkdir -p /opt && sudo git clone <repo> /opt/go2 && cd /opt/go2
sudo uv sync --group sim          # 或者 python -m venv .venv && pip install -e .
```

### 2. 装服务

```sh
sudo ./deploy/install.sh
sudo vi /etc/go2.env              # GO2_IFACE 改成第 1 步看到的那个网口名
sudo systemctl restart quadruped-web
```

装完 `journalctl -u quadruped-web -n 20` 能看到一行
`网页控制面 http://192.168.123.x:8080/?t=xxxxx`。浏览器打开那个地址。

## 接管流程（网页上的「接管」按钮背后）

```
预检：网卡在不在 / 能不能收到 rt/lowstate
   ↓
起 vla 子进程（--preset real --iface <网卡> --source real+vision --no-watch
             --interactive --stream logs/frame.jpg）
   ↓
vla 内部：claim() 释放 mcf → _rise() 摆到策略站姿 1.5 s → 50 Hz 发 lowcmd
   ↓
每 2 s 一行状态回到网页：回路频率 / vx / 前方距离 / 推理耗时 / 位置 / 朝向
```

**按之前狗必须是站着的。** `PolicyBackend.start()` 在倾角超过 45° 时直接拒绝接管，
而从趴着起站会甩飞（`motion/rise.py` 的记录）—— 起站是运控的活，不是策略的。
现在狗站着的方式有两种：官方遥控器 / App 站起来，或者上一次接管没退。

## 网页上有什么

和桌面上那个小窗口是同一张画布：机载画面 + 相机标注 + 「V 画面 / L 想法 / 最近」
右栏 + 8 方位视野条 + 底栏状态，一样不缺。真机上没有图形会话（`--no-watch`），
所以 `Viewer` 不 `imshow`，但仍然渲染，按主循环的 20 Hz 导成 JPEG 写到
`logs/frame.jpg`。网页 `/api/tick` 每 50 ms 问一次版本号，变了才去 `/api/frame`
下整张图。

- 导出是「写 `.tmp` → `os.replace`」，网页不会读到半张图
- 20 Hz 不是估的：一帧渲染 6.1 ms + 编码 0.7 ms（q75、约 32 KB），20 Hz 占单核 14%。
  导出上限 `STREAM_HZ = 30` 只是防编码风暴的保险，必须比主循环快 —— 两边同为 20 Hz
  时 50 ms 各自抖动会交替丢帧，实测掉到 12 fps
- 页面用「上一张下完再问下一张」串起来（`frameBusy`），否则连续改写 `src` 会让浏览器
  取消前一个请求，看着像卡顿
- 画面这条路上没有任何能被写的东西，所以不存在「第二个写者」

**对它说话。** 页面中间的输入框回车就发出去：

```
浏览器 → POST /api/say {"text": "..."} → 子进程 stdin → runner 的读线程
       → Thinker.say() → 排队 → 提示词「用户：…」→ 模型回 {"reply": "..."}
       → 日志 `[L] 回话：…` → 网页抓这一行显示在「它说」下面
```

它记着最近六句对话（`llm/inbox.py`），所以「往前走」之后接一句「再慢点」是有效的
—— 上一句还在提示词的「对话」段里。回话只在人开口的那一轮收下，它自己走路时不会
自言自语。

命令行直接试：

```sh
curl -X POST "http://192.168.123.x:8080/api/say?t=<token>" \
     -H 'Content-Type: application/json' -d '{"text":"你看到什么"}'
```

## 停止 = 蹲下 + 阻尼 + 交还 mcf

网页「停止」发 SIGINT，`runner.run()` 的 finally 依次：停各层线程 → 蹲下 → 阻尼 →
`SelectMode(原模式)`。日志里会看到「已交还 ai」。

不给它 8 秒走完才会 SIGKILL。systemd 那条路同理：`KillMode=mixed` 只停 main 进程，
`TimeoutStopSec=25`，`ExecStopPost` 再跑一次 `deploy/release_mcf.py` 兜底
（处理 kill -9 / OOM 这种谁都没机会收尾的情况）。

## 排错

| 现象 | 先看什么 |
|---|---|
| 页面打不开 | `systemctl status quadruped-web`；端口被占 `ss -lntp \| grep 8080` |
| 点了接管，3 秒后退出 | 日志里「没收到 rt/lowstate」→ `GO2_IFACE` 写错，或狗没上电 |
| 日志里 mcf 没让出去 | `python deploy/release_mcf.py --dry` 看当前模式；官方 App 里重开一次 |
| 狗接管后原地不动 | 状态行里的「策略 XX Hz」掉到 20~30；或 mcf 还在官方手上抢 lowcmd |
| 页面显示「子进程已退出」 | 看 `/opt/go2/logs/web-vla.log` 最后 20 行 |
| 视觉模型一直加载不完 | 先确认走的是远程还是本机：本机那条是 MLX，Linux 上装不了，会一直卡在 import |

## 三个模型都塞进扩展坞，够不够

分三层看：「三个」= A 策略 + L 文本 + V 视觉。

| 层 | 模型 | 权重 | 实测量/估算 |
|---|---|---|---|
| A | `body.jit` + `adaptation.jit` | 1.81 M 参数，7.3 MB fp32 | **0.06 ms/帧**（本机实读），50 Hz 占单核 0.3%，走 CPU，不用 GPU |
| L | Qwen2.5-1.5B-4bit | ~1.1 GB | 估 20~35 token/s（受 102.4 GB/s 带宽限制）。现在远程 deepseek-flash 中位 1.0 s/次；本地一次决策大概 3~6 s |
| V | Qwen2-VL-2B-4bit | ~1.5 GB | 单图 ~250 视觉 token，估 encode 1~2 s + 生成 1~2 s |

内存不是问题：权重合计不到 3 GB，Orin NX 16GB 是统一内存，还要留给相机、雷达、
DDS 和内核。**瓶颈是带宽和互相抢 GPU**，不是 TOPS 标称值（100 TOPS 是 INT8 稀疏）。

两个已知的相互干扰：

- V 和 L 同时本地跑会互相拖。项目里量过一个同构现象（Mac）：VLM 周期 3 s 时，
  L 的单次推理从 2 s 涨到 4 s。所以 V 本地化之后周期要往下调（0.5 Hz 起）。
- 软件栈要换：Linux 上没有 MLX，得用 llama.cpp（GGUF Q4 + CUDA）或 TensorRT-LLM。
  `llm/model.py`、`llm/vlm.py` 现在只有远程和 MLX 两条路。

先确认这块板子到底是哪个型号（100 TOPS 对应 Orin NX 16GB，8GB 版是 70 TOPS）：

```sh
cat /etc/nv_tegra_release      # 或 cat /etc/nv_boot_control.conf
free -g                        # 16 GB 统一内存
sudo tegrastats                # 功耗档、GR3D 占用、EMC 带宽
```

## 手柄（蓝牙，北通）

**为什么需要它**：`claim()` 会把 mcf 踢掉，官方遥控器从那一刻起就不管用了。所以
接管期间人想介入，只有两种办法 —— 网页上的「停止」，或者一个我们自己读的手柄。
后者能在不完全停下来的情况下把人接过来，这件事的意义是安全，不是好玩。

**第一步永远是量映射**，不要猜：

```sh
SDL_VIDEODRIVER=dummy .venv/bin/python test/pad_probe.py     # 动左摇杆、逐个按按钮
```

北通的手柄有 XInput（Xbox 兼容：轴 0/1 左摇杆、4/5 扳机）和 DInput（轴序不同）
两种呈现，同一型号不同批次都可能不一样。探针会打出「动哪个摇杆是哪个轴、哪个按钮是几号」。

**权限与断连**：

```sh
ls -l /dev/input/js*                    # 属 input 组
sudo usermod -aG input <跑服务的用户>    # 加完要重新登录
bluetoothctl devices                    # 配对名带 Beitong
```

蓝牙会掉线重连，重连后设备号可能从 `js0` 变成 `js1` —— 控制层不能把设备号写死，
每次断开重新扫描；断线期间必须当「速度归零」处理，不能保持最后一帧。

**接进控制层的规矩**（还没写，等有手柄再动手，因为映射表要实测）：

- 与 VLA **同一个进程**，不新开一个能碰 lowcmd 的客户端 —— lowcmd 只能有一个写者。
- 优先级：急停键 > 摇杆 > LLM 命令。摇杆离开死区就冻结 LLM 的速度命令；回中多久之后
  才交回去，这个时间要实测（抖一下就把决策权抢走会很难用）。
- 摇杆回中 = 0，不是「保持上一次」。断开 = 0。
- 按键只做边沿动作（切步态、原地停），不做「按住持续发」。


## 还没验证的（第一次上机要专门看）

- 扩展坞上的网卡名、Python 版本、GPU 可用性 —— 本机连不上，一次都没测过
- `--iface` 之外的 DDS 参数：真机固定 domain 0，但如果扩展坞上另有进程初始化过
  `ChannelFactoryInitialize`，会报 already initialized（`DirectLink.connect()` 已吞掉这种失败）
- 相机与雷达的话题名在扩展坞上是否与 `percept/camera.py`、`percept/lidar.py` 里的一致
- `logs/web-vla.log` 目前不做轮转，长期跑要加 logrotate

## 卸载

```sh
sudo ./deploy/install.sh --uninstall
```
