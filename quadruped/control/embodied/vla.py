"""具身智能 · VLA 自主：看画面、想清楚、自己走出去。

菜单里的一项。跑起来是四层 + 一个窗口：

    V 感知     射线 10 Hz + 狗眼画面 5 Hz，画面变化才问视觉模型（≤0.33 Hz）
    R 反射     20 Hz 取感知、查安全闸，贴上了直接停车，不经过模型
    L 决策     2 Hz 节拍，触发时才调文本模型，先写判断再给命令
    G 目标     0.1 Hz 选方向（`--auto` 时），受阻立刻改
    C 执行     推理策略 50 Hz 力矩回路

窗口左边两路画面（机载 / 全局，真机只有机载），右边是 V 的描述、L 的想法、8 方位
视野和实时频率。

## 为什么是推理模型

这一层下发的是 `vx / vy / vyaw / gait` 这一级命令，只有 `PolicyBackend` 认。
自研 / 官方控制端各有自己的运控，接进来要先决定谁管腿 —— 不在这一项的范围里。
所以 `--client` 收下但不使用。

## 场景从哪来

仿真器由启动器拉起，场景也是启动器选的（`GO2_SCENE` 环境变量）。这一层只把它
当标签打印，**不按场景改行为**：自驱时的提示词是 `runner.EXPLORE_HINT`，所有场景
共用同一句（它讲的是怎么用目标点，迷宫、平地、台阶都成立）。

单独跑这一项时仿真已经在跑，场景随便：
    uv run python -m quadruped.control.embodied.vla --preset sim
    uv run python -m quadruped.control.embodied.vla --preset sim --goal "一直往前走"
    uv run python -m quadruped.control.embodied.vla --preset sim --record docs/vla.mp4
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from quadruped.control.virtual.robot import (           # noqa: E402
    add_endpoint_args,
    choose_endpoint,
)

META = dict(
    title="VLA 自主",
    group="embodied",
    desc="狗眼画面 + 文本模型决策，自己在仿真里走（含迷宫演示）",
    owns_cmd=True,
    owns_keys=True,
    # 走策略网络直连 rt/lowcmd，不建 `Go2Controller`，所以运控那套 MPC + WBC
    # 不参与。启动器按这个字段决定摘要怎么写，不靠环境猜。
    client="policy",
)


# systemd 停服务时发的是 SIGTERM，而 Python 默认对它的处理是「直接终止」——
# 不抛异常，`runner.run()` 的 finally 不会执行：mcf 不交还、腿不阻尼、不 park，
# 狗从 27 cm 直接断电掉下来，官方遥控器也要等下次重启才回得来。
#
# 转成 KeyboardInterrupt 是最小改法：那条路本来是给 Ctrl+C 走的，收尾逻辑现成
# （`runner.py` 的 except/finally）。只改信号语义，不动收尾代码。
def _on_sigterm(signum, frame):
    raise KeyboardInterrupt


def main(argv: list[str] | None = None) -> int:
    # stdout 是管道时（网页、systemd）Python 默认块缓冲 4~8 KB。收尾那几行
    # 「蹲下 → 阻尼 → 已交还」在进程被 SIGKILL 之前根本到不了日志文件 ——
    # 排查时会以为收尾没跑，而实际是输出丢了。改成行缓冲。
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass

    ap = argparse.ArgumentParser(
        prog="vla",
        description="VLA 自主：V 层看画面、L 层想、命令走策略网络")
    add_endpoint_args(ap)

    g = ap.add_argument_group("感知")
    g.add_argument("--source", choices=("sim+vision", "sim", "proprio",
                                       "real+vision"),
                   default="sim+vision",
                   help="sim+vision=仿真射线+视觉（默认）；sim=只有仿真射线；"
                        "real+vision=真机：前置摄像头+本体遥测（测距看 --geom，"
                        "开了就挂 Go2 雷达）；proprio=只有本体遥测")
    g.add_argument("--vlm-period", type=float, default=3.0,
                   help="视觉模型最快多久问一次（秒）。和 L 层抢同一块 GPU，"
                        "实测 3 s 时 L 的单次推理从 2 s 涨到 4 s")
    # 测距开关。同一个选项，两种实现：仿真用 MuJoCo 射线，真机用 Go2 的 L1 雷达。
    # 关掉之后安全闸不触发、记忆不进格子、提示词里没有「视野」那一行。
    g.add_argument("--geom", choices=("on", "off"),
                   default=os.environ.get("GO2_GEOM", "on"),
                   help="几何测距：on（默认）仿真走射线、真机走雷达；off 只用画面")
    g.add_argument("--model", default=None,
                   help="模型：默认 deepseek-flash（远程）；要本地就给带斜杠的 HF 仓库名")

    g = ap.add_argument_group("行为")
    g.add_argument("--goal", default=None,
                   help="固定目标。不给就自驱（G 层按 8 个方位选方向）")
    g.add_argument("--say", action="append", default=[],
                   help="按节奏投一句指令，可给多次")
    g.add_argument("--interactive", action="store_true",
                   help="从 stdin 读用户的话（指令或闲聊）。网页控制面靠这一项把"
                        "页面上的输入递进来；stdin 关了不会停机器人")
    g.add_argument("--scene", default=None,
                   help="场景名，只用来挂对应提示（默认读 GO2_SCENE）")

    g = ap.add_argument_group("显示")
    g.add_argument("--record", default=None, help="把窗口内容录成 mp4")
    g.add_argument("--no-watch", action="store_true",
                   help="不开窗口。画面照旧渲染 —— 配合 --stream 给网页看")
    g.add_argument("--stream", default=None, metavar="JPEG",
                   help="把画面（相机 + 想法 + 视野条，带标注）按 5 Hz 导成 JPEG "
                        "写到这个路径，网页控制面从这儿取")
    g.add_argument("--events", default=None, metavar="HOST:PORT",
                   help="把运行状态按行 JSON 发到这个 UDP 地址（网关用它喂 Flutter "
                        "客户端）。给 `on` 走默认 127.0.0.1:8790；不给就不发")
    g.add_argument("--log", default=None,
                   help="日志文件路径，默认 logs/vla-<时间>.log；给空串关掉")

    args = ap.parse_args(argv)

    signal.signal(signal.SIGTERM, _on_sigterm)
    # SIGINT 也要显式装。后台起（shell 的 `&`、`( cmd & )`、systemd 的
    # `ExecStart` 带 `&`）时，SIGINT 是 SIG_IGN，而 CPython 启动时看到 SIG_IGN
    # 就**不**装默认的 KeyboardInterrupt 处理器 —— 于是这个进程对 Ctrl+C 和
    # 网页「停止」发的 SIGINT 完全免疫，只能等 SIGKILL：腿不阻尼、mcf 不交还。
    #
    # 2026-11-16 的记录里 11 次停止全是 code -9，根子就是这一行缺失。
    # 量过：后台起的 `signal.getsignal(SIGINT)` 是 `SIG_IGN`，前台是
    # `default_int_handler`。
    signal.signal(signal.SIGINT, _on_sigterm)

    if args.client and args.client != "policy":
        print(f"这一项用推理模型控制端，--client {args.client} 会被忽略。")
    if args.model is None:
        from quadruped.llm.model import DEFAULT_MODEL

        args.model = DEFAULT_MODEL

    ep = choose_endpoint(preset=args.preset, iface=args.iface, domain=args.domain)
    scene = args.scene or os.environ.get("GO2_SCENE", "flat")

    # 日志落盘。终端窗口通常比屏幕宽，右边那半（位置 / 朝向 / 还差多少度）
    # 正好被切掉，排查时只能截图猜数字。默认写一份完整文件。
    if args.log != "":
        import time

        from quadruped.cli.tee import install

        path = args.log or str(_ROOT / "logs" /
                               f"vla-{time.strftime('%Y%m%d-%H%M%S')}.log")
        install(path)

    from quadruped.control.embodied.runner import run

    watch = not args.no_watch
    says = args.say or ([] if (args.interactive or not args.goal) else [])
    return run(ep, scene=scene, says=says, model=args.model,
               source=args.source, auto=not args.goal, goal=args.goal,
               watch=watch, record=args.record, vlm_period=args.vlm_period,
               geom=(args.geom != "off"), interactive=args.interactive,
               stream=args.stream, events=args.events)


if __name__ == "__main__":
    sys.exit(main())
