"""安全闸：V 层发现危险时直接停车，不经过 L 层。

为什么必须有这一层：L 层一拍 1 秒、生成要 0.4 秒，从「看见墙」到「模型说完话」
已经过去一秒多。以 0.3 m/s 算，走掉 0.4 m —— 足够撞上去。所以安全判断不能放在
需要推理的地方，它得是纯代码、每拍都跑、延迟只有一次函数调用。

职责切分：

    V 层      判断「现在危不危险」     `Guard.check(percept)`
    L 层      决定「想怎么走」        提议命令
    Guard     否决「这个命令现在不能发」`Guard.filter(cmd)`

L 层仍然可以做任何决定，只是它的命令要过一道闸。这样模型说「冲过去」也不会真的
冲过去，而闸本身不需要理解语言，出错只会表现为「停下」，不会表现为「乱走」。
"""

from __future__ import annotations

import time

# 压速之后的速度下限。
#
# 策略是位置控制的小跑，命令速度低于这个值它基本不迈腿 —— 腿在原地小幅倒腾，
# 机身几乎不动。所以「压到 0.18 m/s」在功能上等于停车，而停车是 `check()` 的
# 职责（`stop_gap` 之内），不是 `filter()` 的。
#
# 2026-11-16 迷宫仿真：正前 0.55 m 时命令 0.4 被压成 0.18，狗站着不动，L 层
# 看到「命令没兑现」就转向，转完目标点又变 —— 118 秒 21 次转向、累计 1579°、
# 净位移 2.0 m。0.3 是实测还能走起来的最小命令速度。
MIN_VX = 0.30

# 保底只在这么远之外生效。再近就只靠 `stop_gap` 停车线上的判断 —— 0.3 m/s
# 从 0.7 m 走到 0.45 m 要 0.8 s，这段时间足够策略停住。
MIN_VX_GAP = 0.70


class Guard:
    # 一次停车事件里最多退这么久（秒）。退 1.2 s 约 0.3 m，够从 0.8 m 退到
    # 1.1 m 让开；一直退就变成「明明前方空着还在倒车」。
    BACK_TIME = 1.2

    # 贴墙时后退的速度。策略对负 vx 的响应比正的弱（训练里后退样本很少），
    # -0.25 大约对应 0.1 m/s 的实际后退 —— 够脱离，又不会撞到后面的东西。
    BACK_VX = -0.25

    # `stop_gap` 是「已经贴上了」的距离。
    #
    # 这个数必须从**机头前缘**算，不能从质心算：射线的原点是 `qpos[:3]`，也就是
    # 机身质心（`percept_pub.py:351`），而机头前缘在质心前 0.335 m
    # （`go2.xml:91-92` 头部那颗碰撞球 `pos="0.285"` + 半径 `0.047`）。
    #
    # 原来写 0.45，看似留了 0.115 m；实测不够 —— 2026-11-17 迷宫跑里安全闸
    # 8 次都是「停车：前方 0.42 m」，也就是越过停车线之后还在往前挤，最终机头
    # 距墙不到 6 cm。相机 FOV 那么宽，6 cm 就是满屏墙，人看到的是「摄像头顶
    # 到墙了」。0.55 给机头留 0.215 m，扣掉 ~3 cm 的检测滞后和 ~8 cm 的制动
    # 滑行，还剩 10 cm 以上。
    #
    # `slow_gap` 是「该减速了」。原来是 1.0，实测不够早：命令 vx 0.9 在仿真里
    # 0.5 m/s，1.0 m 只剩 2 s，而 L 层一次决策 + 一次转向要 3.8 s（见
    # `limits.GAP_TRIGGER` 那段账）。提到 1.6，让它早点慢下来，给转向腾时间。
    def __init__(self, backend=None, *, stop_gap: float = 0.80,
                 slow_gap: float = 2.0, max_tilt: float = 40.0,
                 hold: float = 1.0, verbose: bool = True):
        self.backend = backend
        self.stop_gap = float(stop_gap)
        self._percept = None
        self.slow_gap = float(slow_gap)
        self.max_tilt = float(max_tilt)
        self.hold = float(hold)
        self.verbose = verbose

        self.trips = 0
        self.last_reason = ""
        self.blocked_until = 0.0
        self._back_until = 0.0      # 本次停车事件里退到什么时候为止
        self.last_check = 0.0

    # 每拍调用。有危险就立刻停车，返回原因列表。
    #
    # R 层是 20 Hz，所以「已经停过」这段时间里只返回原因、不再停车：50 ms 打一次
    # `stop_motion` 会把 `MotionService` 的日志刷满，真正的新事件反而被埋掉。
    def check(self, percept) -> list[str]:
        now = time.monotonic()
        self.last_check = now
        if percept is None:
            return []
        self._percept = percept

        reasons = []
        e = percept.ego
        if e.tilt_deg > self.max_tilt:
            reasons.append(f"倾斜 {e.tilt_deg:.0f}°")

        gap = percept.front_gap() if percept.rays else float("inf")
        # 只看平移。原地转向不靠近墙 —— 机身扫过的是半径 0.38 m 的圆，而
        # `stop_gap` 是 0.45 m，转得动。
        #
        # 之前把 `wz` 也算进「在动」，于是狗贴着墙想转身离开时被判成危险、
        # 直接停车，而它唯一的出路就是转。实测迷宫走廊尽头 (2.3, -0.1) 处
        # 转向误差从 -56° 一路涨到 -81°，八秒没转出 25°，日志里全是「停车」。
        moving = e.speed > 0.05
        if gap < self.stop_gap and moving:
            reasons.append(f"前方 {gap:.2f} m")

        if not reasons:
            return []

        if now < self.blocked_until:
            return reasons

        self.trips += 1
        self.last_reason = "；".join(reasons)
        self.blocked_until = now + self.hold
        self._back_until = 0.0          # 新事件，后退计时重新开始
        self._stop()
        return reasons

    # 过滤 L 层提议的命令。返回 (放行的命令, 被改了什么)。
    #
    # 只改速度三项，不碰步态、抬脚高度这些：那些不会让狗撞墙，改了反而让模型
    # 的意图变形，事后也难解释。
    def filter(self, cmd: dict) -> tuple[dict, str]:
        if not cmd:
            return cmd, ""
        now = time.monotonic()
        gap = getattr(self, "_gap", float("inf"))

        if now < self.blocked_until:
            # 贴死了但后面还有地方 —— 撤一步。
            #
            # 少了这条，狗在墙边会一直原地踏步：`vx` 被压成 0、转向又不带前进量
            # （`TURN_VX_CLEAR` 不满足），于是它站在离墙 0.6 m 的地方踏步，模型
            # 每拍都以为自己在动（里程计在漂），日志里 11 次命令全是 vx=0、
            # 安全闸触发 53 次。2026-11-17 真机上就是这样「盯着墙不动」。
            #
            # 判据用「后方三个扇区里最远的那个 > 1.5 倍停车线」，只要有一条能退
            # 就退，不用管正后方是不是堵着。
            p = getattr(self, "_percept", None)
            back = p.back_gap() if (p is not None and p.rays) else 0.0
            # 三道闸一起判，缺一条都会退过头：
            #
            #   1. 后方够宽 —— 有地方退
            #   2. **正前真的堵**（`gap < stop_gap`）—— 不然前方空着也在倒车。
            #      2026-11-17 真机上就是这样：日志「前方 3.0」的拍里 `vx` 还是
            #      -0.25，连退 0.9 m。原来的判据只看后方的 `back`。
            #   3. 这次停车事件最多退 `BACK_TIME` 秒 —— 一直退就变成倒车。
            if back > self.stop_gap * 1.5 and gap < self.stop_gap:
                if self._back_until == 0.0:
                    self._back_until = now + self.BACK_TIME
                if now < self._back_until:
                    return ({**cmd, "vx": self.BACK_VX, "vy": 0.0},
                            f"安全闸：{self.last_reason}，后退让开（后方 {back:.1f} m）")

            # 只清平移，留着转向。
            #
            # 这条比看上去重要：贴着墙的时候，「原地转过去」是唯一能脱身的动作。
            # 把 `vyaw` 也清零，狗就只会停在那儿，而 L 层每拍还在重发同一个转向，
            # 两边都认为自己发了命令。倾斜是例外 —— 站在坡上转身是真会摔。
            drop = {k: 0.0 for k in ("vx", "vy") if k in cmd}
            if "倾斜" in self.last_reason:
                drop["vyaw"] = 0.0
            if drop:
                tail = "，只保留转向" if "vyaw" not in drop else "，速度清零"
                return {**cmd, **drop}, f"安全闸：{self.last_reason}{tail}"

        # 还没到停车线，但已经很近：按距离压前进速度。压而不是清零，是因为
        # 这个距离上突然停反而会让重心前冲，慢慢贴近更稳。
        #
        # 原来是 `vx * 0.5` 一刀切（2026-11-15 之前），问题在 1.55 m 和 0.5 m
        # 压得一样狠：走廊里正前常态 1.0~1.5 m，等于全程半速。日志里 `vx=+0.40`
        # 出现 9 次、`+0.25` 3 次，而模型请求的是 0.8。
        #
        # 两端仍是 `slow_gap` / `stop_gap`，`check()` 的停车线没动。
        if gap < self.slow_gap and cmd.get("vx", 0.0) > 0.05:
            span = max(1e-6, self.slow_gap - self.stop_gap)
            k = max(0.0, min(1.0, (gap - self.stop_gap) / span))
            # 开方而不是线性。线性的问题在两端：`0.4 + 0.6k` 在停车线处还留
            # 0.4 倍 —— 2026-11-17 实测 `前方 0.49 m，只保留转向` 之后，狗仍以
            # 0.32 m/s 蹭了 15 cm，停在 0.34 m（机头距墙 0.5 cm）。反过来压太
            # 狠又会让 0.75 m 处走不动（那是 2026-11-15 的老问题）。
            #
            # 开方两头都对：1.0 m 处 0.65、0.75 m 处 0.44、0.6 m 处 0.22、
            # 到停车线正好 0。远处几乎不压，近处压到底。
            keep = k ** 0.5
            vx = cmd["vx"] * keep
            # 压到「策略走不动」就没有意义了（见 `MIN_VX`）：那不是减速，那是停车，
            # 而停车归 `check()`。近到 `MIN_VX_GAP` 以内才允许压到底。
            if gap >= MIN_VX_GAP and vx < MIN_VX:
                vx = MIN_VX
            return {**cmd, "vx": vx}, f"安全闸：前方 {gap:.2f} m，减速"

        return cmd, ""

    # 记住这一拍的前方距离，`filter()` 要用。分开是因为 `filter` 可能在
    # `check` 之后被调用多次（L 层一次给几个候选命令）。
    def observe_gap(self, percept) -> None:
        if percept is not None and percept.rays:
            self._gap = percept.front_gap()

    def _stop(self) -> None:
        if self.backend is None:
            return
        try:
            self.backend.stop_motion()
        except Exception as err:                            # noqa: BLE001
            if self.verbose:
                print(f"[guard] 停车失败：{type(err).__name__}: {err}", flush=True)
            return
        if self.verbose:
            print(f"[guard] 停车：{self.last_reason}", flush=True)

    @property
    def status(self) -> dict:
        return {
            "trips": self.trips,
            "last": self.last_reason,
            "blocked": max(0.0, self.blocked_until - time.monotonic()),
        }
