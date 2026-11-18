"""VLA 演示的一次运行：把 V / R / L / G 四层和一个页面串起来。

`vla.py`（菜单里的控制器）和 `test/embodied_demo.py`（调参用的脚本）都走这一个
函数 —— 两处各写一遍的话，摄像头接法、看门狗阈值、收工顺序早晚会不一致。

## 谁在跑

    Reflex   20 Hz   收感知 + 查安全闸（感知源的唯一读者）
    Vision   20 Hz   收图；变化够大才问视觉模型（≤0.33 Hz）
    Thinker  2 Hz    判要不要想，想的时候调文本模型
    Driver   0.1 Hz  选方向，受阻立刻改
    Policy   50 Hz   力矩回路（`PolicyBackend` 自己的线程）

主循环只做三件事：刷窗口、每 2 秒打一行、按节奏投放句子。它慢下来不影响上面任何
一层 —— 除了 GIL：模型生成时 Python 线程抢不到 CPU，实测反射回路会从 18.5 Hz 掉到
12.8 Hz。这一条写在 `rates.py` 的注释里，不在代码里偷偷补偿。

## 连接不在这里

`endpoint` 由调用方给（`choose_endpoint` / `resolve_endpoint`）。本模块不枚举网卡、
不问控制端，那一套只有 `virtual/robot.py` 一处。
"""

from __future__ import annotations

import math
import os
import threading
import time

from quadruped import rates
from quadruped.control.embodied.events import EventSink, NullSink
from quadruped.llm.model import DEFAULT_MODEL

# 显示窗口的刷新频率。比仿真发图（5 Hz）高，是为了让「想法」跟得上按键和安全闸
# 事件；画面本身重复显示上一帧，代价只有一次 imshow。
VIEW_HZ = 20.0

# 事件通道的频率。5 Hz 而不是 20：这一路要过 UDP、过 WiFi、再进手机，
# 20 Hz 的 JSON 在 2.4G 上排队比画面还久，而「想法」和「前方距离」这两个
# 人要看的量，5 Hz 已经比人眼快。窗口仍是 20 Hz，本地看不受影响。
EVENT_HZ = 5.0

# `--say` 一句和下一句之间隔多久。只有在没给总时长时才用：给了就按句数均分。
SPEAK_GAP = 6.0

# 自驱探索时挂在目标后面的那句话。**不分场景**：它说的是「怎么用目标点」，
# 而只要 G 在探索就会给目标点（平地、台阶、迷宫都一样）。
#
# 原来这里叫 `MAZE_HINT` 且只在 `scene == "maze"` 时挂上去，是这条链路上唯一
# 一处按场景名分支的代码。那是不必要的耦合：句子本身没有一个字是迷宫特有的，
# 它约束的是「转向怎么说」和「别自己发明方向」，这两件事在哪个场景都成立。
EXPLORE_HINT = ("挡路或要走的方向不在机头正前时，按「目标点」给的角度转过去，"
                "转完继续走。目标点是上层在地图上挑的没走过的位置，照着走就行")


def _geom_text(source: str, geom: bool, percept) -> str:
    """这一趟的测距到底是谁给的。日志里必须能一眼看出来，别让人猜。"""
    if source == "proprio":
        return "无（只有本体遥测）"
    if not geom:
        return "关闭（只用画面，没有测距）"
    if source in ("sim", "sim+vision"):
        return "仿真射线（MuJoCo raycast，percept_pub 走 UDP）"
    lidar = getattr(percept, "geom", None)
    if lidar is None:
        return "未接"
    if getattr(lidar, "ok", False):
        return f"Go2 雷达（rt/utlidar/cloud，已收 {getattr(lidar, 'clouds', 0)} 帧）"
    return "Go2 雷达（没收到点云，实际没有测距）"


def run(endpoint, *, scene: str = "flat", says: list[str] | None = None,
        seconds: float | None = None, model: str = DEFAULT_MODEL,
        source: str = "sim+vision", auto: bool = True, goal: str | None = None,
        watch: bool = False, record: str | None = None, vlm_period: float = 3.0,
        geom: bool = True, interactive: bool = False,
        stream: str | None = None, events: str | None = None,
        verbose: bool = True) -> int:
    from quadruped.control.virtual.policy import PolicyBackend
    from quadruped.llm.drive import Driver
    from quadruped.llm.think import Thinker
    from quadruped.percept import (
        Guard,
        Reflex,
        VideoStream,
        Viewer,
        open_source,
        open_world,
    )
    from quadruped.percept.viewer import STREAM_HZ

    says = list(says or [])
    print("各层频率：")
    print(rates.describe())
    print(f"场景 {scene} / 感知 {source} / "
          f"{'自驱' if auto else ('固定目标' if goal else '按句执行')}")

    # 事件通道。UDP，没有对端也不报错 —— UI 在看与否不该影响狗走路。
    sink = EventSink(events) if events else NullSink()
    if events:
        print(f"事件通道 → {sink.target}（{EVENT_HZ:.0f} Hz）")

    backend = PolicyBackend(endpoint, verbose=verbose, torque="pd")
    print("接管策略 …")
    backend.start()

    # 感知源只由一个读者持有（`Reflex`）。L 层和显示窗口都读它缓存的那一份。
    #
    # 测距（`geom`）在这个环境里对应哪一个实现，只有这里决定：
    #   仿真 → SimPercept 的 MuJoCo 射线（percept_pub.py 发的 UDP 帧）
    #   真机 → LidarPercept 的 Go2 L1 雷达（rt/utlidar/cloud）
    # 关掉之后两边都没有测距，只剩画面 + 本体遥测。
    if source == "proprio":
        percept = open_source("proprio", backend)
    elif source == "real+vision":
        percept = open_source("real+vision", backend, geom=geom, period=vlm_period)
        percept.vision.start()
        lidar = getattr(percept, "geom", None)
        if geom and lidar is not None and not getattr(lidar, "ok", True):
            print("警告：雷达 3 秒内没有点云（rt/utlidar/cloud），这一趟没有测距。"
                  "安全闸不会触发、记忆不进格子 —— 先跑 test/lidar_probe.py 看话题。")
    elif source == "sim+vision":
        percept = open_source("sim+vision", geom=geom, period=vlm_period)
        percept.vision.start()          # 视觉模型加载要一秒，放在接管之后
    else:
        percept = open_source("sim", geom=geom)

    print(f"测距：{_geom_text(source, geom, percept)}")

    guard = Guard(backend, verbose=verbose)
    reflex = Reflex(percept, guard)
    reflex.start()

    # 自驱（G 层在跑）时让 L 层想完接着想：没有人的时候它就是唯一的决策者，
    # 只在「前方变近」这类事件上被叫醒的话，走廊里一路空着的十几秒它只是在重发
    # 上一条命令。按句执行 / 固定目标时不开，那时候该由人来说话。
    thinker = Thinker(backend.svc, reflex, guard=guard, model=model,
                      self_drive=auto, verbose=verbose)
    thinker.start()

    driver = None
    if auto:
        driver = Driver(thinker, mode="explore", hint=EXPLORE_HINT)
        driver.start()
    elif goal:
        thinker.set_goal(goal)

    vision = reflex.vision
    # 视觉模型每给出一句新描述，就把 L 层正在跑的那一轮推理掐掉重来。
    # 只在自驱时挂：按句执行时人还在说话，打断没有意义。
    if auto and vision is not None:
        vision.on_text = thinker.interrupt
    viewer = None
    world = None
    eye_only = None          # `--source sim` 时只为了看得见，不喂模型
    if watch or record or stream:
        world = open_world(source)       # 真机没有这一路，拿不到就单画面
        if vision is None and source != "proprio":
            eye_only = VideoStream()
        # `stream` 是给网页的：把这张画布按 `STREAM_HZ` 导成 JPEG，网页读文件显示。
        # `--no-watch` 时 `display=False`，照样渲染 —— 真机上没有图形会话，网页
        # 就是唯一的窗口。
        viewer = Viewer(dual=world is not None, display=watch, record=record,
                        stream=stream, fps=VIEW_HZ)
        if record:
            print(f"录到 {record}（{VIEW_HZ:.0f} fps 上限，实际按主循环）")
        if stream:
            print(f"画面导出到 {stream}（{STREAM_HZ:.0f} fps，网页从这儿取）")

    stop = threading.Event()
    if interactive:
        threading.Thread(target=_stdin_loop, args=(thinker, stop), daemon=True,
                         name="stdin").start()

    t0 = time.monotonic()
    said = 0
    next_print = 0.0
    next_event = 0.0
    reply_seen = ""
    reply_seq = 0
    period = 1.0 / VIEW_HZ
    tick = time.perf_counter()
    p0 = None

    # 从起点算起的位移。真机上 `pos` 会一直是 None —— `lowcmd` 接管后
    # 位姿话题全部停发。
    #
    # 2026-11-17 真机实测（`test/pose_probe.py` 与一次真实接管并行）：
    #
    #     rt/sportmodestate      298 Hz → 0
    #     rt/utlidar/robot_pose   19 Hz → 0
    #     rt/utlidar/robot_odom  150 Hz → 0
    #     rt/lf/sportmodestate    20 Hz → 0
    #
    # 不是某一个话题的问题，是整个 SLAM / 里程计服务认为「运动控制交出去了」。
    # `DirectLink.pos` 取自 `rt/sportmodestate`，于是它一直是 None；仿真里桥
    # 一直在发，所以这条只有真机上才暴露。
    #
    # 不编一个 zeros 出来：那会让「位移 0.0 m」看起来像「它没走」，而实际上
    # 是我们不知道。不知道就说不知道。
    def _disp():
        p = backend.pos
        if p is None or p0 is None:
            return None
        return float(p[0]) - p0[0], float(p[1]) - p0[1]

    try:
        # 起点位姿要在 `try` 里面取。这一行原先在 `try` 外面，2026-11-17 真机
        # 连崩两次都是它（先 `pos` 是 None，再 `np` 未导入），而崩在这里
        # `finally` 不会执行：mcf 不交还、lowcmd 不关、腿不阻尼 —— 狗直接
        # 失力，官方遥控器也不认它，得手动 `SelectMode("mcf")` 才救回来。
        #
        # 用 tuple 不用 `np.array`：这个文件没导入 numpy，多一个依赖就多一次
        # 崩在收尾保护之外的机会。
        if backend.pos is not None:
            p0 = (float(backend.pos[0]), float(backend.pos[1]))

        # 不设上限，跑到人按 Ctrl+C 或关窗口。`seconds` 只有测试脚本会传。
        while not stop.is_set() and (seconds is None
                                     or time.monotonic() - t0 < seconds):
            t = time.monotonic() - t0

            if not interactive and said < len(says):
                # 有总时长就按句数均分；没有就一句一句隔 `SPEAK_GAP` 秒投，
                # 否则第一次投放之后的间隔是 `inf / 句数`，第二句永远发不出去。
                gap = (seconds / max(1, len(says))) if seconds else SPEAK_GAP
                if t >= said * gap:
                    thinker.say(says[said])
                    said += 1

            p = reflex.read()
            s = thinker.status()
            mem = reflex.trace.status()
            disp = _disp()
            foot = [
                (f"({disp[0]:+.1f}, {disp[1]:+.1f}) m" if disp is not None
                 else "位置 无（真机接管后没有里程计）"),
                f"推理 {s['thoughts']} 次 / {s['gen_seconds']:.2f}s",
                f"转 {s['turns']} 次",
                f"记忆 {mem['cells']} 格 / {mem['travelled']:.1f} m",
                f"闸 {guard.status['trips']} 次",
            ]
            if vision is not None:
                foot.append(f"视频 {vision.frames} 帧 / 丢 {vision.dropped}")
            foot.append(f"C 实测 {backend.svc.status()['rate_hz']:.0f} Hz")
            foot.append(rates.headline())
            # 窗口底栏也放一个。人盯着画面判断「卡不卡」靠感觉，这里给个数字：
            # 5 fps 和 1 fps 的主观差别很小，但修法完全不同。
            if viewer is not None:
                foot.append(f"画面 {viewer.fps:.0f} fps")

            # 事件帧。字段名与 Flutter 那边一一对应（`docs/flutter-client.md`
            # 有一张对照表），改这里就要改那边，否则界面会安静地停在旧值上。
            #
            # 只在到点时才拼这个 dict：三十来个字段加一次 json.dumps，5 Hz 下
            # 可以忽略，但 20 Hz 刷窗时每帧都算一遍就是白烧 CPU —— 而这一层正
            # 缺的就是 CPU。
            if t >= next_event:
                next_event = t + 1.0 / EVENT_HZ
                if s["last_reply"] and s["last_reply"] != reply_seen:
                    reply_seen = s["last_reply"]
                    reply_seq += 1
                cmd = s["last_command"] or {}
                sink.publish({
                    "kind": "vla",
                    "t": round(t, 2),
                    "scene": scene,
                    "source": source,
                    "auto": auto,
                    "cmd": {k: round(float(v), 3) for k, v in cmd.items()
                            if isinstance(v, (int, float))
                            and not isinstance(v, bool)},
                    "gait": cmd.get("gait"),
                    "front_gap": s["front_gap"],
                    "sectors": ([{"name": x.name, "gap": round(x.gap, 2),
                                  "top": round(x.top, 2)} for x in p.sectors()]
                                if p is not None else []),
                    "scene_text": "" if vision is None else vision.latest,
                    # 视觉描述有多旧。界面要显示这一项：模型可能一直在说同一句，
                    # 而那句话说的是 5 秒前的画面。
                    "scene_age": (None if vision is None or vision.age == float("inf")
                                  else round(vision.age, 1)),
                    "thought": s["last_thought"],
                    "reply": s["last_reply"],
                    "reply_seq": reply_seq,
                    "emit_thought": s["emit_thought"],
                    "goal": s["goal"],
                    "turns": s["turns"],
                    "turn_left_deg": s["turn_left_deg"],
                    "thoughts": s["thoughts"],
                    "gen_s": s["gen_seconds"],
                    "think_rate": s["think_rate"],
                    "last_error": s["last_error"],
                    "guard": {"trips": guard.status["trips"],
                              "last": guard.status["last"]},
                    "memory": mem,
                    "pos": (None if disp is None
                            else [round(disp[0], 2), round(disp[1], 2)]),
                    "yaw_deg": None if p is None else round(math.degrees(p.ego.yaw), 1),
                    "tilt_deg": None if p is None else round(p.ego.tilt_deg, 1),
                    "policy_hz": round(backend.svc.status()["rate_hz"], 1),
                    "reflex_hz": round(reflex.hz_measured, 1),
                    "view_fps": None if viewer is None else round(viewer.fps, 1),
                    "vision": (None if vision is None else {
                        "frames": vision.frames,
                        "dropped": vision.dropped,
                        "views": vision.views,
                        "gen_s": round(vision.gen_seconds, 2),
                    }),
                    "driver": None if driver is None else driver.status(),
                    "alert": bool(guard.status["trips"]),
                })

            view_ok = True
            if viewer is not None:
                eye = vision.frame if vision is not None else (
                    eye_only.read() if eye_only is not None else None)
                view_ok = viewer.pump(
                    rgb=eye,
                    world=world.read() if world is not None else None,
                    scene=None if vision is None else vision.latest,
                    scene_age=(None if vision is None or vision.age == float("inf")
                               else vision.age),
                    thought=s["last_thought"], why=thinker.last_why,
                    thought_off=not s["emit_thought"],
                    command=s["last_command"], goal=s["goal"],
                    sectors=[(x.name, x.gap) for x in p.sectors()] if p is not None else None,
                    history=thinker.history, foot=foot,
                    alert=bool(guard.status["trips"]),
                )

            if t >= next_print:
                next_print = t + 2.0
                gap = s["front_gap"]
                vis_s = (f"  视频 {vision.frames} 帧 / {vision.gen_seconds:.2f}s"
                         if vision is not None else "")
                turning = (f"  转向还差 {s['turn_left_deg']:+.0f}°"
                           if s["turn_left_deg"] is not None else "")
                # 朝向要和位置一起打。转完「转过头」这类问题，只看位置是看不出来的：
                # 狗停下来不动，位置也没变，但机头比目标多转了 20 度，下一拍就得
                # 往回转 —— 这一行是唯一能直接看到那个 20 度的地方。
                yaw_s = f"  朝向 {math.degrees(p.ego.yaw):+.0f}°" if p is not None else ""
                # 策略回路的实测频率单独打。`rates.headline()` 报的是设定值，
                # 而这一层是 50 Hz 训练的，掉到 20~30 Hz 时动作接不上，表现就是
                # 「腿在动、走不快」—— 光看位置和 vx 分辨不出来。
                cs = backend.svc.status()
                # 画面频率和策略频率并排打。两条都是「看起来在跑」，但一个是
                # 人的窗口、一个是狗的动作 —— 只报策略 Hz 的时候，画面卡成
                # 一秒一帧也不会在日志里留下任何痕迹。
                view_s = f"  {viewer.timing_line()}" if viewer is not None else ""
                print(f"  {t:4.1f}s  vx={s['last_command'].get('vx', 0):+.2f}  "
                      f"策略 {cs['rate_hz']:.0f} Hz{view_s}  "
                      f"前方 {gap if gap is not None else '无视野'}  "
                      f"推理 {s['thoughts']} 次 / {s['gen_seconds']:.2f}s  "
                      f"位置 {'(%+.1f, %+.1f) m' % disp if disp else '无'}"
                      f"{yaw_s}{vis_s}{turning}",
                      flush=True)
                if driver is not None:
                    d = driver.status()
                    print(f"        目标 {d['direction']}（换 {d['changes']} 次，"
                          f"受阻 {d['blocked']} 次）", flush=True)

            if not view_ok:
                print("窗口关闭，结束")
                break

            tick += period
            rest = period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()     # 合成一帧超时，不追
    except KeyboardInterrupt:
        print("\n中断", flush=True)
    finally:
        # 收尾顺序有讲究：**先让机器狗安全下来**（蹲下 → 阻尼 → 交还 mcf），
        # 再做日志和资源清理。
        #
        # 原来 `backend.close()` 排在最后，前面任何一步卡住它就永远不执行：
        # `thinker.stop()` 要等推理线程（最多 3 s），`viewer.close()` 要写完 mp4，
        # `world.close()` 在真机上要关 DDS。2026-11-16 网页上七次「停止」全是
        # SIGKILL —— 8 秒超时，狗就那么站着，mcf 也没交还（官方遥控器从此失灵）。
        #
        # `dx/dy` 要在 close 之前读：`DirectLink.close()` 之后 `pos` 不再更新。
        disp = _disp()
        backend.close()
        sink.close()

        if viewer is not None:
            viewer.close()
        if world is not None:
            world.close()
        if eye_only is not None:
            eye_only.close()
        if driver is not None:
            driver.close()
        thinker.close()
        reflex.close()
        st = thinker.status()
        if disp is not None:
            print(f"\n总位移 ({disp[0]:+.2f}, {disp[1]:+.2f}) m，直线距离 "
                  f"{math.hypot(*disp):.2f} m")
        else:
            print("\n总位移 未知（真机上 lowcmd 接管后没有全局位姿）")
        print(f"推理 {st['thoughts']} 次，平均 {st['gen_seconds']:.2f}s/次，"
              f"实测 {st['think_rate']:.2f} 次/s")
        print(f"反射回路实测 {reflex.hz_measured:.1f} Hz（{reflex.ticks} 拍）")
        mem = reflex.trace.status()
        print(f"记忆 {mem['cells']} 格，走过 {mem['travelled']:.1f} m，"
              f"累计转过 {mem['turns_deg']:.0f}°；转到位 {st['turns']} 次")
        print(f"安全闸触发 {guard.status['trips']} 次"
              + (f"（{guard.status['last']}）" if guard.status["last"] else ""))
        if vision is not None:
            print(f"视觉 {vision.views} 次描述，收图 {vision.frames} 帧，"
                  f"丢 {vision.dropped}")
        if driver is not None:
            d = driver.status()
            print(f"目标层换向 {d['changes']} 次，受阻 {d['blocked']} 次，"
                  f"最后方向 {d['direction']}")
        print(f"最后一句判断：{st['last_thought'] or '无'}", flush=True)
    return 0


# 读 stdin 拿人说的话。放到线程里是必须的：主循环 20 Hz 刷窗口，阻塞在
# `read` 上的话窗口会卡住，看不到自己说的话引出了什么判断。
#
# **不能用 `input()`。** 它拿的是 `sys.stdin` 那个 `BufferedReader` 的锁；这个线程
# 是 daemon，进程退出时解释器要清空并关闭 stdin，拿不到锁就 `Fatal Python error:
# _enter_buffered_busy ...`，整个进程 SIGABRT。网页那条路（stdin 是管道、一直没人
# 写）必然踩到：实测收尾都跑完了，最后卡在关机那一步崩掉。
# `os.read` 直接读 fd 0，不碰 Python 的缓冲对象，退出时不会互相等锁。
#
# **EOF 不等于该停。** stdin 被接成 /dev/null（systemd 的默认）或者父进程退出把
# 管道关了，`os.read` 返回空串。原来这里 `stop.set()`，症状是「用 systemd 起就
# 一句话都活不过一秒」。现在只退出这条读线程，接管照旧 —— 要停走 Ctrl+C 或网页
# 上的「停止」。
def _stdin_loop(thinker, stop: threading.Event) -> None:
    buf = b""
    while not stop.is_set():
        try:
            chunk = os.read(0, 4096)
        except OSError:
            return
        if not chunk:                       # EOF
            return
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").strip()
            if line in ("q", "quit"):
                stop.set()
                return
            if line:
                thinker.say(line)
