"""R 层：定频取感知、查安全闸。不调模型，不认语言。

## 为什么要单独一个线程

闸放在 L 层的循环里也不是不能跑，只是闸的实际频率就是「L 层一拍有多慢」。L 层的
节拍是 0.5 s，而这一拍里可能正在调模型 0.3~0.5 s，最坏情况闸一秒才查一次。

    0.3 m/s 走 1 s = 0.30 m，而 stop_gap 只有 0.55 m

也就是说「看见墙」到「停下」之间，狗可能已经走了三分之二个安全距离。20 Hz 下
这个数是 0.015 m，比一个摆动相（0.2 m）短一个数量级，剩下的余量才真正花在
机械延迟上。

## 它同时是感知源的唯一读者

`SimPercept` / `VideoStream` 的 `read()` 都是「把攒下的包读完，只留最新」。两个
线程同时读会互相抢包：一个读走了，另一个读到 None，然后各自拿手里的旧帧做判断，
两份判断还不一致。这里把顺序固定下来 —— 源只归 R 层读，L 层和显示窗口都读它
缓存的这一份。

代价是 L 层看到的是「最近一次 R 层读到的那帧」，最多晚 50 ms。0.3 m/s 下是 1.5 cm，
可以忽略；换来的是全场只有一个人碰 socket。
"""

from __future__ import annotations

import threading
import time

from quadruped.percept.memory import Trace
from quadruped.rates import REFLEX_HZ


class Reflex:
    name = "reflex"

    def __init__(self, source, guard=None, *, hz: float = REFLEX_HZ,
                 verbose: bool = True):
        self.source = source
        self.guard = guard
        self.period = 1.0 / max(0.5, float(hz))
        self.verbose = verbose

        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self._percept = None
        self._stamp = 0.0

        # 记忆跟着感知一起更新：这一层已经是感知源的唯一读者，再开一个线程去
        # 读同一份数据只会绕回「两个消费者抢包」那个老问题。
        self.trace = Trace()

        self.reasons: list[str] = []
        self.ticks = 0
        self.reads = 0
        self.hz_measured = 0.0
        self.last_error = ""

    # ---- 生命周期 --------------------------------------------------------

    def start(self) -> None:
        if self._run.is_set():
            return
        self._run.set()
        self._thread = threading.Thread(target=self._loop, name="reflex", daemon=True)
        self._thread.start()
        if self.verbose:
            print(f"[R] 反射回路已启动，{1.0 / self.period:.0f} Hz", flush=True)

    def stop(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def close(self) -> None:
        self.stop()
        self.source.close()

    @property
    def running(self) -> bool:
        return self._run.is_set()

    # ---- 给上层读 --------------------------------------------------------

    # 缓存的最近一份观测。不阻塞、不碰 socket —— L 层 2 Hz、显示 20 Hz 都在读它。
    def read(self):
        return self._percept

    @property
    def age(self) -> float:
        return float("inf") if not self._stamp else time.monotonic() - self._stamp

    @property
    def vision(self):
        """`Fused` 里挂的视觉通道，显示窗口要用它的画面。没有就是 None。"""
        return getattr(self.source, "vision", None)

    def status(self) -> dict:
        return {
            "running": self.running,
            "hz": round(self.hz_measured, 1),
            "ticks": self.ticks,
            "age": None if self.age == float("inf") else round(self.age, 2),
            "reasons": list(self.reasons),
            "memory": self.trace.status(),
            "error": self.last_error,
        }

    # ---- 循环 ------------------------------------------------------------

    def _loop(self) -> None:
        tick = time.perf_counter()
        window = tick
        window_ticks = 0

        while self._run.is_set():
            self.ticks += 1
            window_ticks += 1
            try:
                p = self.source.read()
            except Exception as err:                        # noqa: BLE001
                self.last_error = f"{type(err).__name__}: {err}"
                p = None
            if p is not None:
                self._percept = p
                self._stamp = time.monotonic()
                self.reads += 1
                self.trace.update(p)
                if self.guard is not None:
                    # 先记距离再查：`filter()` 减速用的是这一拍的距离。
                    self.guard.observe_gap(p)
                    self.reasons = self.guard.check(p)
                else:
                    self.reasons = p.hazards()

            span = time.perf_counter() - window
            if span >= 3.0:
                self.hz_measured = window_ticks / span
                window = time.perf_counter()
                window_ticks = 0

            tick += self.period
            rest = self.period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()
