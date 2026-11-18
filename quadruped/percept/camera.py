"""视觉通道：收画面、看变化、问模型，输出一句话。

和 `sim.py`（几何射线）并列，两者互不依赖：

    sim.py     10 Hz   距离和高度          喂安全闸和 L 层
    camera.py  0.5 Hz  「前面是什么东西」   只喂 L 层，永远不进安全闸

## 为什么不合成一个源

因为它们的节奏差二十倍，失败方式也不同。视觉模型加载要一秒、推理要半秒，还可能
OOM；射线是纯算术，不会挂。放在同一个循环里，模型一次卡顿就把安全闸的输入也拖停。
所以视觉自己起线程、自己算自己的，L 层每次读的都是「最新那份」；描述旧了就标上
时间戳，不阻塞。

## 为什么变化检测

机器人站着不动时画面没变，每秒问一次模型是白烧电，而且答案还会飘（同一个场景
两次描述措辞不同，L 层会以为环境变了）。对比缩略图的平均绝对差，超过阈值才问。

## 为什么它不能进安全闸

模型说「前方平坦」而实际有台阶，这件事实测会发生（2B 模型的常见错误）。安全
判断只能用几何量。语义只用来给 L 层补充「那是什么」。
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import replace

from quadruped.llm.vlm import DEFAULT_VLM, make_vlm
from quadruped.rates import CAMERA_HZ, VLM_HZ, VLM_MIN_CHANGE

# 缩略图尺寸。32x24 足够判断「画面变没变」，再大就只是多算一遍。
THUMB = (24, 32)


class VideoStream:
    """收原始 RGB 分片帧，按帧号拼回一张图。

    不收 JPEG：编码要用 cv2，而 cv2 自带的 SDL2 会和仿真进程里的 pygame 撞（见
    `simulate_python/percept_pub.py` 的说明）。压缩放在这一侧做，这里没有 pygame。
    """

    name = "sim-video"

    def __init__(self, port: int | None = None, host: str = "127.0.0.1",
                 max_parts: int = 64):
        from quadruped.percept.sim import default_port

        self.port = default_port() + 1 if port is None else int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        self._sock.bind((host, self.port))
        self._sock.setblocking(False)
        self._budget = int(max_parts)

        self.frames = 0
        self.dropped = 0
        self.last_stamp = 0.0
        self._parts: dict[int, dict[int, bytes]] = {}
        self._total: dict[int, int] = {}
        self._last = None

    def close(self) -> None:
        self._sock.close()

    @property
    def age(self) -> float:
        return float("inf") if not self.last_stamp else time.monotonic() - self.last_stamp

    # 把这一拍收到的包全部处理掉，返回最新的完整帧（没有新帧就是 None）。
    #
    # 只保留最近两帧的碎片：更早的一定拼不全了（丢包），留着只是占内存，
    # 而且会让字典无限增长 —— 收不到末片时每帧都会新建一个 key。
    def read(self) -> bytes | None:
        newest = None
        while True:
            try:
                raw, _ = self._sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            if len(raw) < 8:
                self.dropped += 1
                continue
            fid, seq, total, w, h = struct.unpack("!HBBHH", raw[:8])
            if total == 0:
                self.dropped += 1
                continue
            self._total[fid] = (total, int(w), int(h))
            self._parts.setdefault(fid, {})[seq] = raw[8:]
            if len(self._parts[fid]) == total:
                total, w, h = self._total[fid]
                data = b"".join(self._parts[fid][i] for i in range(total))
                self._parts.pop(fid, None)
                self._total.pop(fid, None)
                if len(data) != w * h * 3:
                    self.dropped += 1
                else:
                    import numpy as np

                    newest = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
                    self.frames += 1
                    self.last_stamp = time.monotonic()

        if len(self._parts) > 2:
            for fid in sorted(self._parts)[:-2]:
                self._parts.pop(fid, None)
                self._total.pop(fid, None)
                self.dropped += 1
        if newest is not None:
            self._last = newest
        return newest


# 真机前置摄像头。Go2 的 videohub 是一个 RPC 服务（api_id 1001），
# `GetImageSample()` 返回一整张 JPEG。
#
# 和 `VideoStream` 的三处差别决定了它不能直接替换：
#
#   1. 同步 RPC。一次调用阻塞到服务端返回或超时，慢的时候几百毫秒。放进
#      `VisionPercept` 那个 20 Hz 的收图循环里，一次慢请求就把整条视觉通道拖住。
#      所以这里自己起一条线程按 `hz` 拉，`read()` 只交出最新一张、不阻塞。
#   2. 单订阅者。手机 App 连着的时候这边拿不到帧（`code != 0`）。那不是故障，
#      但要能看出来，所以保留 `dropped` 和 `last_code`。
#   3. **不调 `ChannelFactoryInitialize`**。那个工厂是单例，第一次的 domain 和
#      网卡会被后面所有调用沿用（`channel.py:203` 直接 return True）。相机先起、
#      控制后起，控制就会拿到相机那套配置，而且不报错 —— 表现是「连不上真机」，
#      原因却在一个跟网络无关的地方。初始化由调用方负责（`runner` 里先建 backend）。
class CameraStream:
    name = "go2-video"

    def __init__(self, *, hz: float = 5.0, timeout: float = 1.5,
                 backoff: float = 2.0, verbose: bool = True):
        self.period = 1.0 / max(0.5, float(hz))
        self.timeout = float(timeout)
        self.backoff = float(backoff)
        self.verbose = verbose

        self.frames = 0
        self.dropped = 0
        self.last_code = 0
        self.last_stamp = 0.0
        self.last_error = ""
        self._frame = None
        self._client = None
        self._run = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._run.is_set():
            return
        self._run.set()
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        if self.verbose:
            print(f"[V] 相机已启动，{1.0 / self.period:.0f} Hz 拉帧", flush=True)

    def close(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # `VideoClient` 没有 close：它底下是一个 RPC 通道，交给 GC。置 None 是为了
        # 让 `_client_or_none()` 下次重新建 —— 相机服务重启过之后旧句柄是死的。
        self._client = None

    @property
    def age(self) -> float:
        return float("inf") if not self.last_stamp else time.monotonic() - self.last_stamp

    # 有新帧才返回，否则 None。`VisionPercept` 拿到非 None 就当成新画面处理，
    # 反复交同一张会让它每秒都重新问一次模型。
    def read(self):
        frame, self._frame = self._frame, None
        return frame

    def _client_or_none(self):
        if self._client is not None:
            return self._client
        try:
            from unitree_sdk2py.go2.video.video_client import VideoClient

            c = VideoClient()
            c.SetTimeout(self.timeout)
            c.Init()
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            return None
        self._client = c
        return c

    def _loop(self) -> None:
        tick = time.perf_counter()
        while self._run.is_set():
            c = self._client_or_none()
            if c is None:
                # 建不起来（SDK 不在、DDS 没初始化）就退避，不要每 200 ms 刷一遍。
                # `last_error` 留着，`VisionPercept` 会把它带进状态。
                time.sleep(self.backoff)
                continue
            try:
                code, data = c.GetImageSample()
            except Exception as err:                        # noqa: BLE001
                self.last_error = f"{type(err).__name__}: {err}"
                self.dropped += 1
                time.sleep(self.backoff)
                continue

            self.last_code = int(code)
            if code != 0 or not data:
                # 最常见的原因：手机 App 正连着（videohub 只认一个订阅者）。
                self.dropped += 1
            else:
                frame = self._decode(data)
                if frame is None:
                    self.dropped += 1
                else:
                    self._frame = frame
                    self.frames += 1
                    self.last_stamp = time.monotonic()

            tick += self.period
            rest = self.period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()

    # JPEG → RGB。Go2 给的是 1280x720 的 JPEG，`cv2.imdecode` 出来是 BGR，
    # 这里翻成 RGB —— `vlm.describe()` 和 `_thumb_of()` 都按 RGB 写的。
    def _decode(self, data):
        import numpy as np

        try:
            import cv2
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"没有 cv2：{err}"
            return None
        try:
            buf = np.frombuffer(bytes(data), dtype=np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            return None
        if img is None:
            self.last_error = "JPEG 解不开（半包或不是 JPEG）"
            return None
        return img[:, :, ::-1]



# 全局视角。只给人看，不喂模型 —— 模型不需要知道「自己长什么样」，而人需要：
# 机载画面看不出机身姿态，狗翻了之后眼睛看到的是「蓝色地面、白色天空」，
# 和正常走路没有区别。
#
# 真机上没有这一路（人站在旁边看得到狗）。`open_world()` 在拿不到视频源时
# 返回 None，显示窗口据此退回单画面，而不是留一块黑框。
class WorldStream(VideoStream):
    name = "sim-world"

    def __init__(self, port: int | None = None, host: str = "127.0.0.1",
                 max_parts: int = 64):
        from quadruped.percept.sim import default_port

        super().__init__(port=default_port() + 2 if port is None else port,
                         host=host, max_parts=max_parts)


def open_world(source: str = "sim") -> "WorldStream | None":
    if source not in ("sim", "sim+vision", "fused"):
        return None
    try:
        return WorldStream()
    except OSError:
        return None


class VisionPercept:
    """自驱线程：收画面、看变化、问模型，结果挂在 `latest` 上等别人取。

    收图和问模型是两个线程。合成一个的时候，问一次模型（0.2~0.5 s）这段时间里
    没有人在 `stream.read()`，仿真 5 Hz 发的画面全堆在 socket 里，回来之后读到的
    是半秒前的那一张，显示窗口表现为一顿一顿的。分开之后收图 20 Hz 不停，
    模型只在 `period` 内问一次。

    最近一帧留在 `frame` 上，给显示窗口用 —— 它不该自己去 `stream.read()`，
    同一个 socket 两个读者会互相抢包。
    """

    name = "vision"

    def __init__(self, vlm=None, stream: VideoStream | None = None, *,
                 model: str = DEFAULT_VLM, period: float = 1.0 / VLM_HZ,
                 min_change: float = VLM_MIN_CHANGE, max_idle: float = 8.0,
                 drain_hz: float = 20.0, verbose: bool = True):
        self.stream = stream or VideoStream()
        self.period = float(period)
        self.min_change = float(min_change)
        self.max_idle = float(max_idle)
        self.drain = 1.0 / max(1.0, float(drain_hz))
        self.verbose = verbose

        self._vlm_spec = model
        self._vlm = vlm
        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        self._asker: threading.Thread | None = None
        self._busy = False

        self._text = ""
        # 每给出一句新描述就调一次。L 层用它打断正在进行的推理（见
        # `think.Thinker.interrupt`）—— 描述变了说明世界变了，等这一轮想完
        # 再反应就晚 1.4 s。
        self.on_text = None
        self._stamp = 0.0
        self._thumb = None
        self._frame = None
        self._frame_stamp = 0.0
        self.views = 0
        self.gen_seconds = 0.0
        self.last_error = ""
        self.started_at = 0.0

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[V] {msg}", flush=True)

    def start(self) -> None:
        if self._run.is_set():
            return
        # 自己有拉取线程的流（真机相机）在这里起。仿真流没有 `start()` ——
        # 它的包就在 socket 里堆着，`read()` 一次读完，不用单独的线程。
        if hasattr(self.stream, "start"):
            self.stream.start()
        if self._vlm is None:
            self._vlm = make_vlm(self._vlm_spec)
            self._log(f"视觉模型 {self._vlm_spec} 已加载，{self._vlm.load_seconds:.1f}s")
        self._run.set()
        self.started_at = time.perf_counter()
        self._thread = threading.Thread(target=self._loop, name="vision", daemon=True)
        self._thread.start()
        self._log(f"视觉通道已启动，收图 {1.0 / self.drain:.0f} Hz，"
                  f"问模型上限 {1.0 / self.period:.2f} Hz")

    def stop(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        if self._asker is not None:
            self._asker.join(timeout=5.0)
            self._asker = None

    def close(self) -> None:
        self.stop()
        self.stream.close()
        if self._vlm is not None:
            self._vlm.close()
            self._vlm = None

    # 最新一句描述。`age` 是它有多旧，L 层据此决定要不要提醒自己别信太久。
    @property
    def latest(self) -> str:
        return self._text

    @property
    def age(self) -> float:
        return float("inf") if not self._stamp else time.monotonic() - self._stamp

    # 最近一帧原始 RGB，给显示窗口用。掉线时返回上一帧，画面会停住而不是黑掉 ——
    # 停住说明「没数据」，黑掉看起来像「相机拍到了黑的」。
    @property
    def frame(self):
        return self._frame

    @property
    def frame_age(self) -> float:
        return (float("inf") if not self._frame_stamp
                else time.monotonic() - self._frame_stamp)

    # 收图统计转出去，省得调用方写 `vision.stream.frames` —— 这两个数在日志里
    # 总是一起出现，接口露在外面容易写错（`frames` 曾经就不存在）。
    @property
    def frames(self) -> int:
        return self.stream.frames

    @property
    def dropped(self) -> int:
        return self.stream.dropped

    def status(self) -> dict:
        return {
            "running": self._run.is_set(),
            "views": self.views,
            "frames": self.stream.frames,
            "dropped": self.stream.dropped,
            "gen_seconds": round(self.gen_seconds, 2),
            "text": self._text,
            "age": None if self.age == float("inf") else round(self.age, 1),
            "frame_age": None if self.frame_age == float("inf") else round(self.frame_age, 2),
            "error": self.last_error,
        }

    def _thumb_of(self, rgb):
        import cv2

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        return cv2.resize(gray, THUMB, interpolation=cv2.INTER_AREA).astype("float32")

    # 收图循环。20 Hz 读 socket，只做两件事：留下最新一帧、判断要不要问模型。
    # 真正问模型的那 0.2~0.5 s 在另一个线程里，不占这个循环。
    def _loop(self) -> None:
        tick = time.perf_counter()
        last_view = 0.0
        while self._run.is_set():
            frame = self.stream.read()
            if frame is not None:
                self._frame = frame
                self._frame_stamp = time.monotonic()
                try:
                    small = self._thumb_of(frame)
                except Exception as err:                    # noqa: BLE001
                    self.last_error = f"{type(err).__name__}: {err}"
                    small = None
                change = 1.0
                if small is not None and self._thumb is not None:
                    change = float(abs(small - self._thumb).mean()) / 255.0
                if small is not None:
                    self._thumb = small

                now = time.perf_counter()
                waited = now - last_view
                # 两件事分开判：够不够久（period），值不值得问（变化或空闲）。
                # 合在一起写的时候，`period` 会把「画面变了」挡掉，实测表现为
                # 台阶走到跟前一秒钟了描述还是上一张的。
                due = waited >= self.period
                worth = (not self._text or change >= self.min_change
                         or waited >= self.max_idle)
                if due and worth and not self._busy:
                    last_view = now
                    self._ask(frame, change)

            tick += self.drain
            rest = self.drain - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()      # 模型卡住时下面已经另起线程，这里不追帧

    # 起一个一次性线程问模型。`_busy` 保证同一时刻只有一个在跑 ——
    # 上一帧还没看完就再起一个，两个都抢 GPU，L 层的推理会一起被拖慢。
    def _ask(self, frame, change: float) -> None:
        self._busy = True
        self._asker = threading.Thread(target=self._view, args=(frame, change),
                                       name="vision-vlm", daemon=True)
        self._asker.start()

    def _view(self, frame, change: float) -> None:
        t0 = time.perf_counter()
        try:
            text = self._vlm.describe(frame) if self._vlm is not None else ""
        finally:
            self._busy = False
        self.gen_seconds = time.perf_counter() - t0
        if not text:
            self.last_error = "模型没给出内容"
            self._log(f"{self.gen_seconds:.2f}s 没有输出")
            return
        self._text = text
        self._stamp = time.monotonic()
        self.views += 1
        self.last_error = ""
        # 三行压成一行打日志：多行会把状态行和后面几条日志冲散。原始换行留在
        # `self._text` 里给 L 层用，模型需要看到「哪一行是哪一块」。
        self._log(f"{self.gen_seconds:.2f}s 变化 {change:.3f} → "
                  f"{' | '.join(text.splitlines())}")
        if self.on_text is not None:
            try:
                self.on_text(text)
            except Exception as err:                        # noqa: BLE001
                # 回调是 L 层的事，它自己出错不该把视觉线程带下去。
                self.last_error = f"on_text: {type(err).__name__}: {err}"


# 把几何感知和视觉描述合成一份 `Percept`。
#
# L 层只认 `Percept`，不该知道「射线」和「画面」是两条链路。合成只做一件事：
# 把视觉那一侧的文本和时间戳贴上去，几何部分原样透传 —— 不改距离、不改危险判断。
class Fused:
    name = "sim+vision"

    # `name` 只用于 `Percept.origin` 和日志。两条链路共用这个类：
    # 仿真侧是「射线 + 画面」，真机侧是「本体遥测 + 画面」（没有射线）。
    def __init__(self, geom, vision: VisionPercept, name: str | None = None):
        self.geom = geom
        self.vision = vision
        if name:
            self.name = name

    def read(self):
        p = self.geom.read()
        if p is None:
            return None
        text = self.vision.latest
        if not text:
            return p
        return replace(p, scene=text, scene_age=self.vision.age)

    def close(self) -> None:
        self.geom.close()
        self.vision.close()
