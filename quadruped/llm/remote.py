"""DeepSeek 远程模型：文本和视觉走同一个接口。

本机 MLX 那两个模型（1.5B 文本 / 2B 视觉）能离线跑，但有两个实测问题：

    1. 两个都在同一块 GPU 上，L 层推理和 V 层描述撞车 —— 单次从 2 s 拉到 4 s
    2. 视觉模型加载 4~6 s，这段时间 GIL 被占住，策略线程（50 Hz）跑不起来，
       狗会在这几秒里倒下去（迷宫那次就是这么翻的）

换成远程之后这两条都消失：没有本地显存占用，也没有几秒的加载。代价是每拍要走
一次网络，并且需要 API key。

## 接口

OpenAI 兼容：`POST https://api.deepseek.com/chat/completions`。`deepseek-flash`
（DeepSeek-V4.1-Flash-1017）是唯一支持图片输入的模型，文本和视觉都用它。

**思考模式默认是开的**，这里必须关掉：

    "thinking": {"type": "disabled"}

关掉不是因为贵，是因为这一层每 0.5~1 秒发一次请求。开着思考模式时单次响应会先
吐几百个 `reasoning_content` token 再给 JSON，延迟 3~8 秒，而 L 层的命令有效期
只有 3 秒（`MotionService` 超时）—— 等它想完，狗已经停了。思考要放在 G 层那种
10 秒一次的层级，不是这里。

## key 从哪来

    1. 环境变量 `DEEPSEEK_API_KEY`
    2. 仓库根的 `.deepseek.key`（已在 .gitignore 里）

不给就抛，不静默退回本地 —— 那样看到的会是「模型变笨了」，而不是「key 没配」。
"""

from __future__ import annotations

import base64
import json
import os
import threading
import time
from pathlib import Path

BASE_URL = "https://api.deepseek.com"
DEFAULT_REMOTE = "deepseek-flash"          # DeepSeek-V4.1-Flash-1017，支持图片输入

# 仓库根的 key 文件。放在这里是为了不用每次 `export`，但它不进版本库。
KEY_FILE = Path(__file__).resolve().parents[2] / ".deepseek.key"


def api_key() -> str:
    key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
    if key:
        return key
    if KEY_FILE.is_file():
        key = KEY_FILE.read_text().strip()
        if key:
            return key
    raise RuntimeError(
        f"没有 DeepSeek key。设 DEEPSEEK_API_KEY，或写到 {KEY_FILE}")


# 被上层打断的请求。不是错误：调用方应当直接丢掉这一轮，用新的观测重来。
class Cancelled(RuntimeError):
    pass


# 每个线程一条复用连接。
#
# **实测这条链路上省不到什么。** 复用前后各测 6 次同一份请求：中位 0.71 s 与
# 0.99 s，差距落在噪声里 —— 往返的大头是服务端排队和生成，不是 TCP + TLS 握手。
# 留着是因为它不花钱，而且 L 层真要提到 2 Hz 时，每拍都握手就会变成固定成本。
#
# 用 thread-local 而不是共享一个 `Session`：L 层（`TextService`）和 V 层
# （`RemoteVlm`）各在自己的线程里发请求，`Session` 本身不保证线程安全。
_local = threading.local()


def _session():
    import requests

    s = getattr(_local, "http", None)
    if s is None:
        s = requests.Session()
        _local.http = s
    return s


# 一次 chat 补全。返回 (正文, 耗时)。失败抛 RuntimeError，让调用方决定是重试还是
# 空过这一拍 —— 本模块不自己重试：L 层下一秒就再来一次，重试只会叠加延迟。
#
# `cancel` 是一个 `threading.Event`：置位之后这里会立刻关掉连接并抛 `Cancelled`，
# 不等模型把话说完。
#
# 现在 L 层**不**用它：视觉更新改成排队，等这一轮生成完再重想（见
# `Thinker.interrupt`）。掐断会让每轮 1.0 s 的生成从零重来，主循环节拍被撕成
# 0.4~0.7 s 的碎块，而且被丢掉的请求已经付过服务端成本。能力留着，别再默默接上。
def chat(messages: list[dict], *, model: str = DEFAULT_REMOTE, max_tokens: int = 200,
         temperature: float = 0.0, thinking: bool = False,
         timeout: float = 20.0, cancel=None) -> tuple[str, float]:
    body = {
        "model": model,
        "messages": messages,
        "max_tokens": int(max_tokens),
        "temperature": float(temperature),
        "stream": True,
        # 思考模式：默认开，这里显式关掉。理由见模块开头。
        "thinking": {"type": "enabled" if thinking else "disabled"},
    }
    t0 = time.perf_counter()
    # `stream=True` 只是让我们能一块一块地收，好在收到一半时放弃；
    # 请求体里的 `"stream": False` 没变，服务端仍然返回一整段 JSON。
    try:
        r = _session().post(f"{BASE_URL}/chat/completions", json=body, timeout=timeout,
                            stream=True,
                            headers={"Authorization": f"Bearer {api_key()}",
                                     "Content-Type": "application/json"})
    except Exception as err:                                # noqa: BLE001
        raise RuntimeError(f"请求失败：{type(err).__name__}: {err}") from err

    def give_up() -> bool:
        return cancel is not None and cancel.is_set()

    # 走 SSE 流式。不是为了「打字机效果」，是为了能真的中途掐断：非流式时
    # `iter_content` 要等整段正文到达才返回，1.4 s 的生成只能等到最后才检查取消，
    # 那和等它说完没区别。流式下每个 delta 都会唤醒一次，掐断延迟就是网络往返。
    import json

    parts: list[str] = []
    try:
        if r.status_code != 200:
            raise RuntimeError(f"HTTP {r.status_code}: {r.text[:200]}")
        if give_up():
            raise Cancelled("还没开始就被打断")
        for line in r.iter_lines(decode_unicode=False):
            if give_up():
                raise Cancelled("说到一半被打断")
            if not line or not line.startswith(b"data:"):
                continue
            payload = line[5:].strip()
            if payload == b"[DONE]":
                break
            try:
                obj = json.loads(payload.decode("utf-8"))
            except ValueError:
                continue
            try:
                delta = obj["choices"][0].get("delta") or {}
                piece = delta.get("content") or ""
            except (KeyError, IndexError):
                piece = ""
            if piece:
                parts.append(piece)
    finally:
        r.close()                    # 中途放弃时把连接关掉，别让它继续占着
    dt = time.perf_counter() - t0
    text = "".join(parts)
    if not text:
        raise RuntimeError("响应是空的（流式没收到 delta）")
    return text, dt


class RemoteModel:
    """文本模型。接口和 `LocalModel` 一样：`generate(messages, max_tokens)`。"""

    def __init__(self, spec: str = DEFAULT_REMOTE, *, max_tokens: int = 200,
                 timeout: float = 20.0):
        self.spec = spec
        self.max_tokens = int(max_tokens)
        self.timeout = float(timeout)
        self.load_seconds = 0.0        # 没有本地加载，保持字段名一致
        self.calls = 0
        self.last_seconds = 0.0
        self.last_error = ""

    def close(self) -> None:
        pass

    def generate(self, messages: list[dict], max_tokens: int | None = None,
                 cancel=None) -> str:
        text, dt = chat(messages, model=self.spec,
                        max_tokens=self.max_tokens if max_tokens is None else max_tokens,
                        timeout=self.timeout, cancel=cancel)
        self.calls += 1
        self.last_seconds = dt
        return text


class RemoteVlm:
    """视觉模型。接口和 `LocalVlm` 一样：`describe(frame)` → 一句中文。

    `frame` 是 numpy 的 RGB 数组（H, W, 3）。走 base64 data URL，不落盘 ——
    本机模型那张临时文件是为了喂 mlx-vlm 的路径参数，远程不需要。
    """

    def __init__(self, spec: str = DEFAULT_REMOTE, *, max_tokens: int = 64,
                 timeout: float = 25.0, quality: int = 85):
        self.spec = spec
        self.max_tokens = int(max_tokens)
        self.timeout = float(timeout)
        self.quality = int(quality)
        self.load_seconds = 0.0
        self.calls = 0
        self.last_seconds = 0.0
        self.last_error = ""

    def close(self) -> None:
        pass

    def describe(self, frame, question: str = "") -> str:
        from quadruped.llm.vlm import QUESTION, mark_thirds

        question = question or QUESTION
        try:
            url = _data_url(mark_thirds(frame), self.quality)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            return ""
        messages = [{
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": url}},
                {"type": "text", "text": question},
            ],
        }]
        try:
            text, dt = chat(messages, model=self.spec, max_tokens=self.max_tokens,
                            timeout=self.timeout)
        except RuntimeError as err:
            self.last_error = str(err)
            return ""
        self.calls += 1
        self.last_seconds = dt
        # 保留换行。原来这里 `" ".join(...split())` 把三行压成一行 —— 三块各自的
        # 边界就没了，L 层读到的是「左：… 中：… 右：…」一长串，它没有理由按块
        # 分开理解。只在行内压空白、丢掉空行。
        lines = [" ".join(ln.split()) for ln in str(text).splitlines()]
        return "\n".join(ln for ln in lines if ln)


def _data_url(frame, quality: int = 85) -> str:
    import cv2

    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                           [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG 编码失败")
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode()


# 哪些 spec 走远程。本地那几个是「带斜杠的 HF 仓库名」或路径，远程是纯名字。
def is_remote(spec: str) -> bool:
    spec = (spec or "").strip()
    if not spec:
        return False
    if spec.startswith(("mlx:", "local:", "/", ".")):
        return False
    return "/" not in spec
