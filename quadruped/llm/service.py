"""文本服务的自驱循环：定频跑，有话就解析，没话就续期命令。

和 `MotionService` 同一套结构（`Event` 开关 + 绝对时刻累加 + sleep 补齐），
差别在周期：这里是 1 秒，不是 20 毫秒。

为什么必须定频，而不是「有输入才动」：

    1. 命令有 3 秒超时。不续期的话，说完一句话 3 秒后速度自己归零，
       表现为「狗走两步停了」。所以空闲拍的职责就是重发上一条命令。
    2. 模型生成 0.35 秒，解析失败要重试一次就接近 1 秒。给一个固定节奏，
       超时就顺延到下一拍，不追帧 —— 文本不需要严格等间隔。

循环里唯一会阻塞的是模型生成。它跑在独立线程上，主控制回路的 50 Hz 不受影响。
"""

from __future__ import annotations

import math
import threading
import time

from quadruped.llm.model import DEFAULT_MODEL, make
from quadruped.llm.prompt import build
from quadruped.llm.schema import parse, to_command


# 角度折到 (-π, π]。给转向反馈用：目标 179°、当前 -179° 时差 358° 是错的，实际 2°。
def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class TextService:
    """把一句话变成命令，按固定周期下发给 `MotionService`。"""

    def __init__(self, motion, *, model: str = DEFAULT_MODEL,
                 period: float = 1.0, max_tokens: int = 96,
                 verbose: bool = True):
        self.motion = motion
        self.period = float(period)
        self.max_tokens = int(max_tokens)
        self.verbose = verbose

        self._model_spec = model
        self._model: LocalModel | None = None
        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._inbox: str | None = None
        # 新输入到达时打断 sleep。不这样的话，一句话最多要等满一个周期才开始
        # 推理，加上生成耗时，从说完到狗动要 2 秒。
        self._wake = threading.Event()

        self.last_text = ""
        self.last_raw = ""
        self.last_command: dict = {}
        self.last_error: str | None = None
        self.plans = 0
        self.rate = 0.0
        self.gen_seconds = 0.0

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[text] {msg}", flush=True)

    # 加载模型并起线程。加载是同步的（缓存命中 0.5 秒），失败直接抛，不静默降级 ——
    # 没有模型这个服务什么都不做，装作启动成功只会让人以为在听。
    def start(self) -> None:
        if self._run.is_set():
            return
        if self._model is None:
            self._model = make(self._model_spec)
            self._log(f"文本模型 {self._model_spec} 已加载，"
                      f"{self._model.load_seconds:.2f}s")
        self._run.set()
        self._thread = threading.Thread(target=self._loop, name="text", daemon=True)
        self._thread.start()
        self._log(f"文本服务已接管，{1.0 / self.period:.1f} Hz")

    def stop(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._log("文本服务已退出")

    def close(self) -> None:
        self.stop()
        if self._model is not None:
            self._model.close()
            self._model = None

    # 投递一句话。只保留最新的一条：人在 1 秒内说两句，第二句覆盖第一句，
    # 排队执行会让狗先执行一句已经过时的指令。
    def submit(self, text: str) -> None:
        if not text.strip():
            return
        with self._lock:
            self._inbox = text.strip()
        self._wake.set()

    def _take(self) -> str | None:
        with self._lock:
            text, self._inbox = self._inbox, None
        return text

    @property
    def running(self) -> bool:
        return self._run.is_set()

    def status(self) -> dict:
        return {
            "running": self.running,
            "period": self.period,
            "rate_hz": round(self.rate, 2),
            "plans": self.plans,
            "gen_seconds": round(self.gen_seconds, 2),
            "last_text": self.last_text,
            "last_command": self.last_command,
            "last_error": self.last_error,
        }

    def _loop(self) -> None:
        tick = time.perf_counter()
        since = tick
        frames = 0
        while self._run.is_set():
            text = self._take()
            try:
                if text is not None:
                    self._plan(text)
                elif self.last_command:
                    self.motion.set_command(**self.last_command)
            except Exception as err:                # noqa: BLE001
                self.last_error = f"{type(err).__name__}: {err}"
                self._log(f"这一拍失败：{self.last_error}")

            frames += 1
            # 按时间窗口统计，不按帧数：周期是秒级的，等攒够 10 帧要 10 秒，
            # 状态行会一直显示 0。
            span = time.perf_counter() - since
            if span >= 1.0:
                self.rate = frames / span
                frames = 0
                since = time.perf_counter()

            tick += self.period
            sleep = self.period - (time.perf_counter() - tick)
            if sleep > 0:
                self._wake.wait(sleep)
                self._wake.clear()
            else:
                tick = time.perf_counter()          # 生成超时，不追帧

    def _plan(self, text: str) -> None:
        t0 = time.perf_counter()
        raw = self._model.generate(build(text, self.last_command), self.max_tokens)
        self.gen_seconds = time.perf_counter() - t0
        self.last_raw = raw
        self.last_text = text

        fields, notes = parse(raw, text)
        if not fields:
            self.last_error = "解析失败：" + ("；".join(notes) or "空输出")
            self._log(f"「{text}」没解析出命令 —— {self.last_error}")
            return

        cmd = to_command(fields)
        yaw_deg = float(fields["vyaw"]) if fields.get("vyaw") else 0.0
        if cmd:
            got = self.motion.set_command(**cmd)
            self.last_command = {**self.last_command, **cmd}
        else:
            got = {}
        self.plans += 1
        self.last_error = None

        shown = " ".join(f"{k}={v:g}" for k, v in cmd.items()) or "（只转向）"
        extra = f"  （{'；'.join(notes)}）" if notes else ""
        clamped = got.get("clamped") or {}
        if clamped:
            extra += "  [被压住：" + ", ".join(clamped) + "]"
        self._log(f"「{text}」→ {shown}  {self.gen_seconds:.2f}s{extra}")

        if yaw_deg:
            self._turn(yaw_deg)

    # 转向：模型给多少度就转多少。策略的 vyaw 是角速度，角度只能靠闭环 ——
    # 读 `MotionService` 的 rpy 当反馈，每 0.1 秒重发一次（命令 3 秒过期）。
    #
    # 阻塞执行是有意的：主循环就在这个线程里，转的这十来秒不会有别的命令来
    # 覆盖。这条路径没有安全闸，中途被打断会转到一个没人认得的朝向。
    def _turn(self, deg: float) -> None:
        def yaw_now() -> float:
            st = self.motion.status()
            return float((st.get("rpy") or [0.0, 0.0, 0.0])[2])

        target = _wrap(yaw_now() + math.radians(deg))
        speed = 1.0 if deg > 0 else -1.0
        t0 = time.monotonic()
        self._log(f"转向 {deg:+.0f}°（目标 yaw {math.degrees(target):+.0f}°）")
        try:
            while self._run.is_set() and time.monotonic() - t0 < 20.0:
                err = _wrap(target - yaw_now())
                if abs(err) <= 0.08:
                    self._log(f"转到位，{time.monotonic() - t0:.1f} s")
                    return
                self.motion.set_command(vx=0.0, vy=0.0, vyaw=speed,
                                        phase=0.0, offset=0.0, bound=0.0)
                time.sleep(0.1)
            self._log(f"转向超时，还差 {math.degrees(_wrap(target - yaw_now())):+.0f}°")
        finally:
            # 转完把转向前的速度拾回来。转向期间发的是 vx=0，不恢复的话
            # 「走一段再转」会在这里变成「转完站住」。
            keep = {k: v for k, v in (self.last_command or {}).items()
                    if k != "vyaw"}
            self.motion.set_command(vyaw=0.0, **keep)
