"""文本控制演示：一句话 → 命令 → 机器人。

两种用法：

    # 只测「文本 → 命令」，不连机器人（几秒钟，先确认提示词对不对）
    .venv/bin/python test/text_demo.py --dry --text "以每秒0.3米小跑"

    # 连仿真走一遍（先起 run_sim.sh）
    .venv/bin/python test/text_demo.py --text "以每秒0.3米小跑" --text "跳着走" --seconds 6
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.llm.model import DEFAULT_MODEL, make          # noqa: E402
from quadruped.llm.prompt import build                             # noqa: E402
from quadruped.llm.schema import parse, to_command                 # noqa: E402
from quadruped.llm.service import TextService                      # noqa: E402


# 只跑模型和解析，不碰机器人。用来调提示词：改一句、跑一次、看输出。
def run_dry(texts: list[str], model_spec: str) -> int:
    model = make(model_spec)
    print(f"模型 {model_spec} 加载 {model.load_seconds:.2f}s\n")
    for text in texts:
        t0 = time.perf_counter()
        raw = model.generate(build(text))
        dt = time.perf_counter() - t0
        fields, notes = parse(raw, text)
        cmd = to_command(fields)
        print(f"「{text}」  {dt:.2f}s")
        print(f"  原文 {raw.strip()[:100]!r}")
        print(f"  字段 {fields}")
        if notes:
            print(f"  备注 {'；'.join(notes)}")
        if cmd:
            shown = " ".join(f"{k}={v:g}" for k, v in cmd.items()
                             if k in ("vx", "vy", "vyaw", "phase", "offset",
                                      "bound", "freq", "footswing"))
            print(f"  命令 {shown}")
        print()
    model.close()
    return 0


# 连仿真跑一遍。每句话给 `seconds` 秒，期间每秒打印实测速度，最后看位移。
def run_sim(texts: list[str], seconds: float, period: float,
            model_spec: str) -> int:
    from quadruped.control.virtual.policy import PolicyBackend
    from quadruped.control.virtual.robot import resolve_endpoint

    backend = PolicyBackend(resolve_endpoint("sim"), verbose=False, torque="pd")
    print("接管策略 …")
    backend.start()

    svc = TextService(backend.svc, model=model_spec, period=period)
    svc.start()
    p0 = backend.pos.copy()
    x0 = float(backend.pos[0])
    try:
        for text in texts:
            svc.submit(text)
            t0 = time.monotonic()
            while time.monotonic() - t0 < seconds:
                s = svc.status()
                c = s["last_command"]
                dx = float(backend.pos[0]) - p0[0]
                print(f"  {time.monotonic()-t0:4.1f}s  "
                      f"vx={c.get('vx', 0):+.2f}  "
                      f"脚本 {s['rate_hz']:.2f} Hz  "
                      f"推理 {s['gen_seconds']:.2f}s  "
                      f"x {dx:+.2f} m")
                if s["last_error"]:
                    print(f"    错误 {s['last_error']}")
                time.sleep(1.0)
            p0 = backend.pos.copy()
    finally:
        svc.close()
        dx = float(backend.pos[0]) - x0
        print(f"\n总位移 x {dx:+.2f} m，计划 {svc.plans} 次，"
              f"上次推理 {svc.gen_seconds:.2f}s")
        backend.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="文本控制演示")
    ap.add_argument("--text", action="append", default=[],
                    help="要执行的一句话，可给多次，按顺序执行")
    ap.add_argument("--dry", action="store_true",
                    help="只解析，不连机器人")
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="每句话持续多少秒，默认 6")
    ap.add_argument("--period", type=float, default=1.0,
                    help="文本服务周期，秒，默认 1.0")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"默认 {DEFAULT_MODEL}（远程）；本机给带斜杠的仓库名")
    args = ap.parse_args()

    texts = args.text or ["以每秒 0.3 米小跑"]
    if args.dry:
        return run_dry(texts, args.model)
    return run_sim(texts, args.seconds, args.period, args.model)


if __name__ == "__main__":
    raise SystemExit(main())
