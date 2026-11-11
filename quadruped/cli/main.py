#!/usr/bin/env python3
"""Go2 控制入口：构造 Go2Robot，跑一个动作，退出。

仿真（先另开一个终端起仿真器：./quadruped/sim/run_sim.sh）。不写网卡就是仿真：
    uv run python -m quadruped.cli.main stand
    uv run python -m quadruped.cli.main act squat --depth 0.08
    uv run python -m quadruped.cli.main act tilt --pitch 0.15
    uv run python -m quadruped.cli.main forward 2.0        # 闭环，真走 2 米
    uv run python -m quadruped.cli.main turn 90            # 逆时针 90°
    uv run python -m quadruped.cli.main trot --vx 0.3      # 开环按时间跑
    uv run python -m quadruped.cli.main demo               # 一串基础动作
    uv run python -m quadruped.cli.main telemetry
    uv run python -m quadruped.cli.main selftest           # 不用仿真器
    uv run python -m quadruped.cli.main hierarchy          # 打印三层对象树

真机：把网卡一写就切过去（lo0 = 仿真，其它 = 真机，域号跟着变）：
    uv run python -m quadruped.cli.main stand --iface en5

注意：每次重启仿真器之后再跑。退出时脚本会给腿下发阻尼，不重启的话
下一次是从「瘫倒」姿态重新插值，看起来像没动静。
"""

import argparse
import copy
import json
import math
import os
import socket
import sys

if __package__ in (None, ""):       # python quadruped/cli/main.py：把项目根加进来
    sys.path.insert(0, os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))

from quadruped.control.controller import (GaitParams, NOMINAL_HEIGHT,
                                          _swing_poly_coeffs, make_gait)
from quadruped.robot.body import Go2Robot

# act 子命令能调的基础动作，值是要传给 HighLevelController 的参数名
BASIC_ACTIONS = {
    "stand_up": ("stand_up", []),
    "squat": ("squat", ["depth", "hold"]),
    "tilt": ("tilt", ["pitch", "roll", "hold"]),
    "shift": ("shift", ["dx", "dy", "hold"]),
    "nod": ("nod", ["times", "pitch"]),
    "lie_down": ("lie_down", ["hold"]),
    "step": ("step_in_place", ["seconds"]),
}


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Go2 控制入口",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    sub = ap.add_subparsers(dest="action", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--iface", default=None,
                        help="网卡名：lo0 = 仿真（域号读仿真器 config.py），其它 = 真机 domain 0")
    common.add_argument("--preset", choices=("sim", "real"), default=None,
                        help="可选快捷方式：sim=lo0/domain1，real=en5/domain0")
    common.add_argument("--domain", type=int, default=None, help="直接指定 DDS 域号，最高优先级")
    common.add_argument("--height", type=float, default=NOMINAL_HEIGHT,
                        help="标称机身高度 m")
    common.add_argument("--kp", type=float, default=None,
                        help="刚度，不给就用控制器默认（真机 110）")
    common.add_argument("--kp-calf", type=float, default=None, help="小腿单独刚度")
    common.add_argument("--kd", type=float, default=None,
                        help="阻尼，不给就用控制器默认（真机 1.5）")

    p = sub.add_parser("stand", parents=[common], help="站起来并保持")
    p.add_argument("--duration", type=float, default=30.0, help="保持秒数")
    p.add_argument("--ramp", type=float, default=0.8, help="起立插值秒")

    p = sub.add_parser("trot", parents=[common], help="开环小跑（按时间）")
    p.add_argument("--vx", type=float, default=0.3, help="前进 m/s")
    p.add_argument("--vy", type=float, default=0.0, help="左移 m/s")
    p.add_argument("--yaw", type=float, default=0.0, help="偏航角速度 rad/s")
    p.add_argument("--duration", type=float, default=8.0, help="行走秒数（不含起立）")
    p.add_argument("--period", type=float, default=None,
                   help="步态周期 s，不给就用该风格自己的值")
    p.add_argument("--duty", type=float, default=None,
                   help="支撑相占比，不给就用该风格自己的值")
    p.add_argument("--swing", type=float, default=None,
                   help="抬脚高度 m，不给就用该风格自己的值")

    p = sub.add_parser("act", parents=[common], help="跑一个基础动作")
    p.add_argument("name", choices=sorted(BASIC_ACTIONS))
    p.add_argument("--depth", type=float, default=0.08, help="squat 下蹲深度 m")
    p.add_argument("--pitch", type=float, default=0.15, help="tilt/nod 低头角 rad")
    p.add_argument("--roll", type=float, default=0.0, help="tilt 侧滚角 rad")
    p.add_argument("--dx", type=float, default=0.05, help="shift 前后移 m")
    p.add_argument("--dy", type=float, default=0.0, help="shift 左右移 m")
    p.add_argument("--times", type=int, default=2, help="nod 次数")
    p.add_argument("--seconds", type=float, default=3.0, help="step 时长")
    p.add_argument("--hold", type=float, default=0.6, help="姿态保持秒数")

    p = sub.add_parser("forward", parents=[common], help="闭环走指定距离")
    p.add_argument("distance", type=float, help="米，负值后退")
    p.add_argument("--vx", type=float, default=0.3, help="指令速度 m/s")
    p.add_argument("--tol", type=float, default=0.08, help="收步提前量 m")

    p = sub.add_parser("turn", parents=[common], help="闭环转指定角度")
    p.add_argument("degrees", type=float, help="度，正为逆时针")
    p.add_argument("--yaw-rate", type=float, default=0.5, help="指令角速度 rad/s")
    p.add_argument("--tol", type=float, default=10.0, help="收步提前量，度")

    p = sub.add_parser("demo", parents=[common], help="串一遍基础动作")
    p.add_argument("--distance", type=float, default=1.0, help="走多远")
    p.add_argument("--degrees", type=float, default=90.0, help="转多少度")

    p = sub.add_parser("telemetry", parents=[common], help="只读，看 DDS 通不通")
    p.add_argument("--duration", type=float, default=5.0, help="采样秒数")

    sub.add_parser("selftest", help="运动学与步态自检（本地，不需要仿真器）")
    sub.add_parser("hierarchy", parents=[common],
                   help="打印 Robot→Controller→HighLevel 的实际嵌套关系")
    sub.add_parser("state", parents=[common], help="仅构造对象，打印配置与运动学摘要")
    return ap


def make_robot(args) -> Go2Robot:
    gait = None
    if getattr(args, "period", None) is not None:
        # 只换这三样，其余全部沿用该风格的整套参数。原来直接 GaitParams(...)
        # 造一个新的，stride_max / speed_loop / 落脚点反馈全被默认值冲掉了 ——
        # 命令行随手给个 --period，步态就退回到「开环、无速度闭环」的老版本，
        # 跑起来又变成打滑不走。
        gait = copy.copy(make_gait(getattr(args, "style", "mech"),
                                   args.height, real=True))
        gait.period = args.period
        if args.duty:
            gait.duty = args.duty
        if args.swing:
            gait.swing_height = args.swing
        gait.a3, gait.a4 = _swing_poly_coeffs(gait.duty)
    return Go2Robot(args.preset, domain=args.domain, iface=args.iface,
                    height=args.height, kp=args.kp, kd=args.kd,
                    kp_calf=args.kp_calf, gait=gait)


# 连之前先看网卡在不在，免得白等 3 秒后报一句含糊的「没收到 lowstate」。
def check_iface(robot: Go2Robot) -> int:
    c = robot.controller
    if Go2Robot.interface_exists(c.iface):
        return 0
    print(f"失败：本机没有网卡 {c.iface}。")
    print(f"  当前可用的：{', '.join(n for _, n in socket.if_nameindex())}")
    print("  真机的 USB 网卡没枚举上时，拔插一次再跑 setup_en5.sh。")
    return 1


def run_common(robot: Go2Robot, args) -> int:
    if args.action == "telemetry":
        robot.telemetry(args.duration)
        return 0
    if args.action == "trot":
        robot.controller.trot(vx=args.vx, vy=args.vy, yaw=args.yaw,
                              duration=args.duration)
        return 0
    if args.action == "stand":
        robot.controller.stand(duration=args.duration, ramp=args.ramp)
        return 0
    if args.action == "act":
        method, keys = BASIC_ACTIONS[args.name]
        kw = {k: getattr(args, k) for k in keys}
        print(f"\n--- {args.name} {kw if kw else ''} ---", flush=True)
        getattr(robot.controller.high_level, method)(**kw)
        return 0
    if args.action == "forward":
        robot.controller.high_level.forward(args.distance, vx=args.vx, tol=args.tol)
        return 0
    if args.action == "turn":
        robot.controller.high_level.turn(args.degrees, yaw_rate=args.yaw_rate,
                              tol=math.radians(args.tol))
        return 0
    if args.action == "demo":
        steps = ["stand_up",
                 ("squat", {"depth": 0.07}),
                 ("tilt", {"pitch": 0.15}),
                 ("shift", {"dx": 0.05}),
                 ("step_in_place", {"seconds": 2.0}),
                 ("forward", {"distance": args.distance}),
                 ("turn", {"degrees": args.degrees}),
                 "stand_up",
                 "lie_down"]
        robot.controller.high_level.sequence(steps)
        return 0
    raise SystemExit(f"未处理的动作：{args.action}")


def main() -> int:
    args = build_parser().parse_args()

    if args.action == "selftest":
        return 0 if Go2Robot("sim").selftest() else 1

    if args.action in ("state", "hierarchy"):
        robot = Go2Robot(args.preset, domain=args.domain, iface=args.iface,
                         height=args.height, kp=args.kp, kd=args.kd)
        if args.action == "hierarchy":
            robot.print_tree()
            print(json.dumps(robot.hierarchy(), indent=2, ensure_ascii=False))
        else:
            print(json.dumps(robot.state(), indent=2, ensure_ascii=False))
        return 0

    robot = make_robot(args)
    label = "仿真" if robot.preset == "sim" else "真机"
    print(f"目标 {label}：网卡 {robot.controller.iface} / DDS 域 {robot.controller.domain}")

    if check_iface(robot):
        return 1

    if not robot.connect():
        print("失败：3 秒内没收到 rt/lowstate。", flush=True)
        print(f"  {label}走的是 {robot.controller.iface} / domain "
              f"{robot.controller.domain}，按这条查：")
        print("  1) 仿真：另一个终端起了 ./quadruped/sim/run_sim.sh 吗")
        print("  2) 真机：setup_en5.sh 拿到 192.168.123.222 了吗，能 ping 通 .161 吗")
        print("  3) 网卡是不是插错口了（--iface 可以指定别的名字）")
        return 1

    print(f"已连上，首帧关节 q={[round(float(v), 3) for v in robot.q]}\n", flush=True)

    try:
        return run_common(robot, args)
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    finally:
        robot.shutdown()


if __name__ == "__main__":
    sys.exit(main())
