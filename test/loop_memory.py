"""记忆的长度：走回去的时候，系统知不知道那是自己走过的路。

用户 2026-11-17 报「走着走着回头去走自己走过的路，说明没有记忆」。查下来三层
原因，这个脚本盯住前两层：

    记忆窗口       `been()` / `summary()` 默认只看 `HOLD`（25 秒）。`visit` 的
                   时间戳本来就没删，读的时候只取 25 秒，等于白存。
    目标点在脚印上  旧目标点留在走过的路上，`AIM_TIMEOUT` 只按时间判，而狗在动
                   就不算「追不到」—— 于是模型每轮看到「向我身后转 160 度」。

第三层（`direction` 是相对方位名、转身后失效）在 `drive._by_distance` 里，
需要真感知才能构造，这里不测。

    .venv/bin/python test/loop_memory.py
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quadruped.llm.drive import REACHED, Driver                       # noqa: E402
from quadruped.percept.memory import BEEN_SECONDS, HOLD, Trace         # noqa: E402
from quadruped.percept.types import Ray                               # noqa: E402

ok = 0
bad: list[str] = []


def check(name: str, cond: bool, extra: str = "") -> None:
    global ok
    if cond:
        ok += 1
        print(f"  ✓ {name}" + (f"  （{extra}）" if extra else ""))
    else:
        bad.append(name)
        print(f"  ✗ {name}" + (f"  （{extra}）" if extra else ""))


# `Trace._update` 只碰这几个字段，造一个够用的假感知。
def feed(trace: Trace, x: float, y: float, yaw: float, stamp: float) -> None:
    ego = type("E", (), {"x": x, "y": y, "yaw": yaw})()
    # `Trace.update()` 在没有射线时整帧跳过（没有测距就不知道哪能走，连
    # 「走过」也不记）。所以这里必须给一条假射线，否则测的是「关掉几何测距
    # 之后没有地图」这件事，不是记忆窗口。
    trace.update(type("P", (), {"stamp": stamp, "ego": ego,
                                "rays": (Ray(0.0, 3.0, 0.0),),
                                "range_max": 3.0})())


def main() -> int:
    trace = Trace()
    print(f"常量：BEEN_SECONDS={BEEN_SECONDS:.0f} s  HOLD={HOLD:.0f} s")

    print("\n记忆窗口")
    # 从原点往东走 3 m，每格 0.3 m 一帧 —— 和真机 10 Hz、0.5 m/s 的密度接近。
    stamp = 0.0
    for i in range(11):
        feed(trace, i * 0.3, 0.0, 0.0, stamp)
        stamp += 0.1
    check("走完 3 m 后脚下这一格记下了", trace.been_at((3.0, 0.0)) >= 1)

    # 把这些脚印整体推到 60 秒前：模拟「走了一圈回来了」。
    cut = time.monotonic() - 60.0
    with trace._lock:
        trace.visit = {k: cut for k in trace.visit}
        trace._x, trace._y, trace._yaw = 3.0, 0.0, 0.0

    check(f"60 秒前的脚印仍然查得到（窗口 {BEEN_SECONDS:.0f} s）",
          trace.been_at((0.0, 0.0)) >= 1, f"{trace.been_at((0.0, 0.0))} 格")
    check("同一个点、按旧窗口 25 s 查不到 —— 这就是原来「忘了」的原因",
          trace.been_at((0.0, 0.0), seconds=25.0) == 0)
    check("没走过的地方仍然是 0", trace.been_at((0.0, 3.0)) == 0)

    print("\n目标点落在脚印上就该作废")
    d = Driver.__new__(Driver)                       # 只测纯函数，不建世界
    d.thinker = type("T", (), {
        "percept": type("R", (), {"trace": trace})()})()
    p = type("P", (), {"ego": type("E", (), {"x": 3.0, "y": 0.0, "yaw": 0.0})()})()

    d.target = (0.0, 0.0)                            # 3 m 外，60 秒前站过
    d._aim_at = time.monotonic()
    check("走过的路口 → 作废", d._aim_stale(p) is True)

    d.target = (2.0, 1.5)                            # 1.8 m，从没去过
    check("没去过的方向、距离也合适 → 保留", d._aim_stale(p) is False)

    # 走过头了。距离 2.7 m 既不「到了」也不「太远」（< `AIM_MAX_DIST`），
    # `been_at` 也查不到 —— 只有方位能说明这个点已经落在身后。
    # 2026-11-17 开阔地里那句「前面这一片都走过了，往回走」就是这么来的。
    d.target = (0.5, -1.0)                           # 身后 -158°，没去过
    check("跑到身后（> 150°）→ 作废", d._aim_stale(p) is True)
    d.target = (0.0, 3.0)                            # 4.2 m，超出量程
    check("太远（> AIM_MAX_DIST）→ 作废", d._aim_stale(p) is True)

    d.target = (2.95, 0.0)                           # 就在脚下
    check(f"{REACHED} m 以内 → 作废（走到的）", d._aim_stale(p) is True)

    print(f"\n{'通过' if not bad else '失败：' + '、'.join(bad)}（{ok} 项）")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
