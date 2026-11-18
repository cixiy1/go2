"""只读探针：查 mcf 当前模式、官方运控还活不活。不接管，不发 lowcmd。

为什么需要这一项
----------------
真机上「像在被两边抢」有两种完全不同的可能，修法相反：

1. `ReleaseMode()` 没生效 —— 官方 sport mode 仍以 500 Hz 发 `rt/lowcmd`，
   我们 50 Hz 插进去，关节一拍是我们的一拍是它的。特征：抖、一冲一冲。
2. 放掉了，但狗的固件在某些状态（摔倒、被抬起、遥控器抢回）下自动回到
   sport mode。特征：跑了十几秒突然自己动/自己停。

两种都能从两个数看出来，不用猜：

- `CheckMode()` 返回的 `name`：非空 = 官方运控**此刻**在管。
- `rt/sportmodestate` 的频率：被 lowcmd 接管后它应当**停发**（`direct.py`
  的注释里记着这件事）。它还在刷，就是官方还在跑。

用法：
    .venv/bin/python test/mcf_watch.py --iface en5 --dur 3
    .venv/bin/python test/mcf_watch.py --iface en5 --dur 3 --release   # 释放后复测

`--release` 会真的把 mcf 让出去。**释放后官方不再管腿**，狗会塌下去 ——
只在手能扶住、或紧接着就要起控制器的时候用。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402


def _mcf():
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
        import MotionSwitcherClient
    sc = MotionSwitcherClient()
    sc.SetTimeout(5.0)
    sc.Init()
    return sc


def _mode(sc) -> str:
    code, data = sc.CheckMode()
    if code != 0:
        return f"<CheckMode 失败 code={code}>"
    return (data or {}).get("name", "") or "<空 = 没有运控在管>"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="en5")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--dur", type=float, default=3.0)
    ap.add_argument("--release", action="store_true",
                    help="真的释放 mcf（狗会塌，手扶住）")
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize, ChannelSubscriber,
    )
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
        LowState_, SportModeState_,
    )

    ChannelFactoryInitialize(a.domain, a.iface)

    n_sport = [0]
    pos = [None]
    t_sport = [0.0]

    def on_sport(msg):
        n_sport[0] += 1
        t_sport[0] = time.time()
        pos[0] = np.array([float(msg.position[i]) for i in range(3)])

    n_low = [0]

    def on_low(msg):
        n_low[0] += 1

    ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(on_sport, 10)
    ChannelSubscriber("rt/lowstate", LowState_).Init(on_low, 10)

    print(f"连上 {a.iface} domain {a.domain}", flush=True)
    time.sleep(0.5)

    sc = _mcf()
    print(f"mcf 模式（释放前）：{_mode(sc)}", flush=True)

    # 按 1 s 分段打。整段平均看不出「接管那一刻」的变化 —— 而那正是判据：
    # 官方停发的时刻就是写入权交割的时刻。
    def measure(tag: str) -> None:
        t0 = time.time()
        n0, l0 = n_sport[0], n_low[0]
        while time.time() - t0 < a.dur:
            time.sleep(1.0)
            dt = time.time() - t0
            print(f"{tag} {dt:4.1f}s  sport {n_sport[0] - n0:5d} "
                  f"({(n_sport[0] - n0) / 1.0:6.1f} Hz)   "
                  f"lowstate {n_low[0] - l0:5d} "
                  f"({(n_low[0] - l0) / 1.0:6.1f} Hz)", flush=True)
            n0, l0 = n_sport[0], n_low[0]

    measure("[释放前]")

    if a.release:
        print("发 ReleaseMode …", flush=True)
        sc.ReleaseMode()
        time.sleep(0.5)
        print(f"mcf 模式（释放后）：{_mode(sc)}", flush=True)
        measure("[释放后]")

    if pos[0] is not None:
        print(f"最后一帧里程计 ({pos[0][0]:+.2f}, {pos[0][1]:+.2f})", flush=True)
    print("\n判读：sportmodestate 有频率 = 官方运控在跑；停发（0 Hz）= 已被接管。",
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
