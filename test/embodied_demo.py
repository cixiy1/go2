"""具身智能演示：V 层看环境 → L 层想 → 命令 → 策略走路。

四层串起来跑：

    V   `percept/sim.py` 收仿真射线 + `percept/camera.py` 收狗眼画面
    R   `percept/reflex.py` 20 Hz 取感知、查安全闸，闸触发就直接停车
    L   `llm/think.py` 读这份观测，先写一句判断，再给速度命令
    G   `llm/drive.py` 没有人的时候自己定方向（`--auto` 才启）

三种用法：

    # 只看 V 层给了模型什么（不连机器人，几秒）
    .venv/bin/python test/embodied_demo.py --dry
    .venv/bin/python test/embodied_demo.py --dry --source sim+vision

    # 连仿真，画面和想法同屏（先起 run_sim.sh）
    .venv/bin/python test/embodied_demo.py --source sim+vision --watch
    .venv/bin/python test/embodied_demo.py --watch --record docs/vl_watch.mp4

    # 自驱：没有人说话，G 层自己选方向
    .venv/bin/python test/embodied_demo.py --source sim+vision --auto

    # 手动一句一句说
    .venv/bin/python test/embodied_demo.py --interactive
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.llm.model import DEFAULT_MODEL, make                 # noqa: E402


# 不连机器人：直接把感知帧打出来，看 L 层会看到什么。
#
# 这是调 V 层的主要手段。提示词写得再好，输入是错的也白搭；而输入对不对
# 只能靠看 —— 一排「正前 3.00 m 空」和真实的「1.2 m 有 8 厘米台阶」差很多。
def run_dry(at: float, model_spec: str, ask: str | None,
            source: str = "sim") -> int:
    from quadruped.percept import open_source

    src = open_source(source)
    vision = getattr(src, "vision", None)
    if vision is not None:
        vision.start()
    # `Fused` 没有 port：它只是把两个源贴在一起，真正收包的是里面的 geom。
    port = getattr(src, "port", None) or getattr(getattr(src, "geom", None), "port", "?")
    print(f"等感知帧（端口 {port}）…")
    p = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        p = src.read()
        if p is not None:
            break
        time.sleep(0.1)
    if p is None:
        src.close()
        print("5 秒内没收到感知帧。仿真起了吗？"
              "（cd quadruped/sim/unitree_mujoco && SIM_SCENE=../unitree_robots/"
              "go2/scene_flat.xml ./run_sim.sh）")
        return 1

    if vision is not None:
        # 视觉是异步的，第一帧要等模型跑完。等的时候射线照读，否则几秒不取包，
        # 打印出来的是队列里积压的旧帧，还会误报「感知已停」。
        for _ in range(60):
            latest = src.read()
            if latest is not None:
                p = latest
            if vision.latest:
                break
            time.sleep(0.2)

    geom = getattr(src, "geom", src)
    print(f"[诊断] 射线帧 {geom.frames}，坏帧 {geom.bad_frames}，"
          f"最近一帧 {geom.age:.2f}s 前")
    print(f"\n--- V 层给 L 层的观测（origin={p.origin}）---")
    print(p.text())
    print(f"\n前方最近 {p.front_gap():.2f} m，危险 {p.hazards() or '无'}")

    if ask:
        from quadruped.llm.prompt import build_think
        from quadruped.llm.think import parse_think

        model = make(model_spec)
        raw = model.generate(build_think(p.text(), user=ask, reason="手动测试"), 160)
        thought, reply, fields, notes = parse_think(raw, ask)
        print(f"\n--- L 层（{model.load_seconds:.1f}s 加载）---")
        print(f"想法：{thought}")
        print(f"回话：{reply or '（无）'}")
        print(f"命令：{fields or '维持'}")
        if notes:
            print(f"备注：{'；'.join(notes)}")
        model.close()
    return 0


# 连仿真跑。真正的实现搬到了 `quadruped/control/embodied/runner.py` —— 菜单里的
# 「VLA 自主」跑的也是那一份，两处各写一遍的话，摄像头接法、收工顺序早晚不一致。
#
# 这个脚本留着是为了 `--dry`（只看 V 层给了模型什么），以及不带菜单直接跑一次。
def run_sim(says: list[str], model_spec: str,
            source: str, interactive: bool, *, watch: bool = False,
            auto: bool = False, goal: str | None = None,
            record: str | None = None, vlm_period: float = 3.0) -> int:
    import os

    from quadruped.control.embodied.runner import run
    from quadruped.control.virtual.robot import resolve_endpoint

    return run(resolve_endpoint("sim"), scene=os.environ.get("GO2_SCENE", "flat"),
               says=list(says), model=model_spec, source=source,
               auto=auto, goal=goal, watch=watch, record=record,
               vlm_period=vlm_period, interactive=interactive)


def main() -> int:
    ap = argparse.ArgumentParser(description="具身智能演示：V 层感知 + L 层决策")
    ap.add_argument("--say", action="append", default=[],
                    help="要执行的一句话，可给多次")
    ap.add_argument("--dry", action="store_true",
                    help="只打印 V 层的观测，不连机器人")
    ap.add_argument("--at", type=float, default=0.0,
                    help="--dry 时先记下机器人在哪个 x（仿真里先走过去再看）")
    ap.add_argument("--ask", default=None,
                    help="--dry 时顺带跑一次 L 层，给一句指令")
    ap.add_argument("--source", choices=("sim", "proprio", "sim+vision"),
                    default="sim",
                    help="感知来源：sim 只有射线；sim+vision 加一条视觉通道"
                         "（狗眼相机 → 视觉模型 → 一句描述）；proprio 只用遥测")
    ap.add_argument("--watch", action="store_true",
                    help="开一个窗口，左边两路画面、右边 V 的描述和 L 的想法")
    ap.add_argument("--record", default=None,
                    help="把窗口内容录成 mp4（不开窗口也能录）")
    ap.add_argument("--auto", action="store_true",
                    help="自驱：G 层自己选方向，不需要人说话")
    ap.add_argument("--goal", default=None,
                    help="给一个常驻目标，例如「一直往前走，遇墙就停下」")
    ap.add_argument("--vlm-period", type=float, default=3.0,
                    help="视觉模型最快多久问一次（秒）")
    ap.add_argument("--interactive", action="store_true",
                    help="从 stdin 一句一句读")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"默认 {DEFAULT_MODEL}（远程）；本机给带斜杠的仓库名")
    args = ap.parse_args()

    if args.dry:
        return run_dry(args.at, args.model, args.ask, args.source)
    says = args.say or ([] if (args.interactive or args.auto) else ["往前走"])
    return run_sim(says, args.model, args.source, args.interactive,
                   watch=args.watch, auto=args.auto, goal=args.goal,
                   record=args.record, vlm_period=args.vlm_period)


if __name__ == "__main__":
    raise SystemExit(main())
