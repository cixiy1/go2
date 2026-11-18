"""地图规划器的离线回归：二维射线 + 二维位姿，不启动 MuJoCo，不用模型。

## 为什么需要这个

迷宫跑一次要一分钟，而「狗没走出来」有三个完全不同的原因：

    地图给的的目标点本身在乱跳     → 规划器的问题
    目标点是对的，但 L 层没有照做   → 提示词 / 工具的问题
    命令是对的，但策略转不动        → 运控的问题

在完整链路里这三样混在一起，只能靠读日志猜，改一次参数要等一分钟才知道结果。
所以这里把第一段切出来单独测：几何用 `scenes.maze_boxes()`，射线跟真仿真一样
24 条、量程 3 m，规划器直接读 `Trace.guide()`，执行换成一个写死的几何控制器
（朝目标点转、正前够宽就前进）。

它回答一个问题：**只靠局部 3 m 射线和这张栅格图，能不能规划出一条走出迷宫的路。**

判据不是「看起来合理」，是三条可测的：

    走出来了      出口在 (3.8, 4.5)，位姿越过它就停
    多久          仿真时间，不是墙钟
    换了几次向    目标方向变了多少次 —— 一次决策一秒，换向太勤等于原地摆

    python test/maze_plan.py                 # 跑默认参数
    python test/maze_plan.py --verbose       # 逐秒打印位姿和目标点
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from quadruped.percept.memory import Trace
from quadruped.percept.types import Ground, Percept, Ray, SelfState
from quadruped.sim.scenes import CELL, COLS, ROWS, maze_boxes

# 和 `percept_pub.PerceptPublisher` 一致：24 条，15° 一条，从 -180° 开始。
BEAMS = 24
RANGE_MAX = 3.0

# 执行器的物理上限，都是之前实测的：trot 带前进转向 0.192 rad/s（取 0.18 保守），
# 前进 0.3 m/s。
TURN_RATE = 0.18
SPEED = 0.30

# 机身半宽。0.7 x 0.3 的机身，侧面离墙 0.15 m 就是擦上了。
#
# 这里用半宽而不是半对角线（0.38）：狗沿着 3 m 长、1.8 m 宽的空走廊走，旁边
# 0.35 m 有墙是完全正常的，按 0.38 判会把「平行贴着墙走」当成碰撞而停车，
# 而它前面明明还有 3 m。半对角线只在原地转向时才有意义，那一段由真机上的
# 碰撞体积负责，二维仿真不追究。
BODY_R = 0.18

# 正前低于这个距离就只转不走。真机上这一条由 R 层（`percept/reflex.py`）执行，
# 这里的目的是别让 2D 版穿墙，否则测出来的路径真机走不了。
STOP_GAP = BODY_R + 0.25

# 仿真步长。控制环 50 Hz，感知 10 Hz，和实际一致。
DT = 0.02
PERCEPT_EVERY = 5            # 每 5 拍出一帧感知


def _ray_box(x: float, y: float, dx: float, dy: float,
             box: tuple[float, float, float, float]) -> float | None:
    """射线与一个轴对齐盒子的交点距离（slab 法）。没交点返回 None。"""
    cx, cy, hx, hy = box
    near, far = -math.inf, math.inf
    for o, d, c, h in ((x, dx, cx, hx), (y, dy, cy, hy)):
        lo, hi = c - h, c + h
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None                       # 平行且在板外
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        near, far = max(near, t1), min(far, t2)
        if near > far:
            return None
    if far < 0:
        return None                                # 盒子在身后
    return max(near, 0.0)


def scan(x: float, y: float, yaw: float, boxes) -> list[Ray]:
    rays = []
    for i in range(BEAMS):
        bearing = math.radians(i * 15.0 - 180.0)
        ang = yaw + bearing
        dx, dy = math.cos(ang), math.sin(ang)
        best = RANGE_MAX
        for box in boxes:
            t = _ray_box(x, y, dx, dy, box)
            if t is not None and t < best:
                best = t
        rays.append(Ray(bearing, best, 0.0))
    return rays


# 造一个真的 `Percept` 而不是自己再算一遍正前：
# `sectors()` 取的是「方位中心最近的那一条射线」，正前就是 0° 那一条，
# 不是 ±22.5° 里取最小。这里如果自己按「前后几条取最小」重写一遍，测出来的
# 停止距离会比真机紧，狗明明还能走却被判成贴墙 —— 第一次跑就是这个错，
# 在 (4.21, 0.92) 明明前面还有 1.7 m，却被自己的 ±15° 最小值按成 0.59。
def percept_of(stamp: float, x: float, y: float, yaw: float,
               boxes) -> Percept:
    return Percept(stamp=stamp, origin="maze2d",
                   ego=SelfState(x=x, y=y, yaw=yaw),
                   rays=tuple(scan(x, y, yaw, boxes)),
                   ground=Ground(under=0.0, ahead=()),
                   range_max=RANGE_MAX)


# 位姿到最近墙面的距离。机身不是质点，0.7 x 0.3 的机身对角线 0.38 m，
# 贴到 0.2 m 就是蹭着墙走 —— 真机上这会直接触发 R 层停车。
def clearance(x: float, y: float, boxes) -> float:
    best = float("inf")
    for cx, cy, hx, hy in boxes:
        dx = max(abs(x - cx) - hx, 0.0)
        dy = max(abs(y - cy) - hy, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


# 出口：北墙上那个缺口的中心。只在这里算，不进 `quadruped/`。
#
# 它是**评测标准**，相当于阅卷答案：这个离线规划器自己跑一遍看能不能到。
# 运行时不该知道出口坐标 —— 真机上没有这条信息，G 层也不该靠它决定什么时候停。
def maze_exit() -> tuple[float, float]:
    half = CELL / 2.0
    x1 = half + CELL * (COLS - 1)
    return (x1 - CELL * 0.75 / 2, half + CELL * (ROWS - 1))


def run(verbose: bool = False, limit: float = 240.0,
        start: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> dict:
    boxes = maze_boxes()
    ex, ey = maze_exit()
    trace = Trace()
    x, y, yaw = start

    stamp = 0.0
    t = 0.0
    switches = 0
    last_dir: float | None = None
    ticks = 0
    escaped_at: float | None = None
    scrapes = 0
    min_clear = float("inf")
    waypoints: list[tuple[float, float]] = [(x, y)]
    p = percept_of(0.0, x, y, yaw, boxes)

    while t < limit:
        # ---- 感知 ----
        if ticks % PERCEPT_EVERY == 0:
            p = percept_of(stamp, x, y, yaw, boxes)
            trace.update(p)
            stamp += 1.0

        point = trace.guide()
        gap = p.front_gap()
        clear = clearance(x, y, boxes)
        min_clear = min(min_clear, clear)
        if clear < BODY_R:
            scrapes += 1

        # ---- 决策：朝目标点转，正前够宽就走 ----
        if point is None:
            vx, vyaw = 0.0, 0.0
        else:
            err = math.atan2(point[1] - y, point[0] - x) - yaw
            err = (err + math.pi) % (2 * math.pi) - math.pi
            if last_dir is None or abs(err - last_dir) > math.radians(45.0):
                switches += 1
                last_dir = err
            vyaw = max(-TURN_RATE, min(TURN_RATE, err * 2.0))
            # 转向优先：偏差大于 40° 时原地转，边转边走会蹭墙。
            #
            # `clear < BODY_R` 是硬约束，不是调参。机身半对角线 0.38 m，位姿离
            # 墙面 0.04 m 在物理上不可能 —— 那是穿墙了。放开这一条，测出来的
            # 路径会从墙角「擦过去」，真机上到那一步就已经被 R 层停车了。
            vx = 0.0 if (abs(err) > math.radians(40.0)
                         or gap < STOP_GAP or clear < BODY_R) else SPEED

        # ---- 运动学 ----
        yaw += vyaw * DT
        x += vx * math.cos(yaw) * DT
        y += vx * math.sin(yaw) * DT

        if math.hypot(x - waypoints[-1][0], y - waypoints[-1][1]) > 0.5:
            waypoints.append((x, y))
        t += DT
        ticks += 1

        if verbose and ticks % 50 == 0:
            if point is None:
                aim = "—"
            else:
                aim = f"({point[0]:+.1f},{point[1]:+.1f}) 偏 {math.degrees(err):+4.0f}°"
            print(f"  {t:5.1f}s ({x:+5.2f},{y:+5.2f}) yaw{math.degrees(yaw):+6.1f}° "
                  f"正前{gap:.2f} → {aim}")

        # 走出北墙缺口：y 越过出口、且在缺口宽度内。
        if y > ey + 0.15 and abs(x - ex) < 1.2:
            escaped_at = t
            break

    ok = escaped_at is not None
    return {
        "ok": ok,
        "seconds": round(escaped_at if ok else t, 1),
        "pose": (round(x, 2), round(y, 2), round(math.degrees(yaw), 1)),
        "switches": switches,
        "scrapes": scrapes,
        "min_clear": round(min_clear, 2),
        "waypoints": waypoints,
        "travelled": round(trace.travelled, 2) if trace.travelled else 0.0,
        "cells": len(trace.visit),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--limit", type=float, default=240.0)
    ap.add_argument("--start", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    metavar=("X", "Y", "YAW_DEG"),
                    help="起点位姿。真机开机时机头朝哪不确定，用来测不同朝向")
    ap.add_argument("--sweep", action="store_true",
                    help="转一圈八个朝向各跑一次，报最慢的那个")
    args = ap.parse_args()

    if args.sweep:
        worst = 0.0
        fails = 0
        for deg in range(0, 360, 45):
            r = run(limit=args.limit, start=(0.0, 0.0, math.radians(deg)))
            flag = "✓" if r["ok"] else "✗"
            print(f"  {flag} 起始 {deg:3d}°：{r['seconds']:5.1f} s，"
                  f"换向 {r['switches']:2d} 次，最近离墙 {r['min_clear']} m")
            if r["ok"]:
                worst = max(worst, r["seconds"])
            else:
                fails += 1
        print(f"\n八个朝向：{8 - fails}/8 走出来了，最慢 {worst or 0:.1f} s")
        return 0 if fails == 0 else 1

    start = (args.start[0], args.start[1], math.radians(args.start[2]))
    r = run(verbose=args.verbose, limit=args.limit, start=start)
    print(f"\n{'走出来了' if r['ok'] else '没走出来'}：仿真 {r['seconds']} s，"
          f"终点 {r['pose']}，换向 {r['switches']} 次，栅格 {r['cells']} 个")
    print(f"贴墙 {r['scrapes']} 拍，最近离墙 {r['min_clear']} m")
    if args.verbose and r["waypoints"]:
        print("路径：" + " → ".join(f"({a:+.1f},{b:+.1f})" for a, b in r["waypoints"]))
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
