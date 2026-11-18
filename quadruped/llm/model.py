"""本机文本模型：MLX 后端，模型常驻内存，一次加载多次生成。

选 MLX 而不是 Ollama，是因为这个项目要的是一个库、不是一个后台服务：模型在
进程里，`TextService` 的循环直接调它，不用管端口、进程、健康检查。

实测（Apple M5，Qwen2.5-1.5B-Instruct-4bit）：

    首次下载后从本地快照加载  0.5 s
    生成 40 token             0.35 s

第一次的 78 s 是一次性下载。这里 `resolve()` 先查本地缓存，命中就直接用路径，
不走网络 —— 走网络的话光是超时就要 150 s，因为 HF 会先问一次远端有没有新版本。
"""

from __future__ import annotations

import os
import time
from pathlib import Path

# 默认走远程。改这一行的理由和实测数据见 `remote.py` 的开头：本地两个模型
# 抢同一块 GPU（单次 2 s → 4 s），而且视觉模型加载那 4~6 s 会把 50 Hz 的策略
# 线程饿到停摆，狗在那几秒里倒下。
DEFAULT_MODEL = "deepseek-flash"
# 本机 MLX 那一套还留着：断网、不想花 token、或者要复现旧数据时用。
LOCAL_MODEL = "mlx-community/Qwen2.5-1.5B-Instruct-4bit"

# 国内直连 huggingface.co 会卡在 TCP 握手。只有真要下载时才设镜像，避免影响
# 别的进程；命中缓存时根本不联网。
MIRROR = "https://hf-mirror.com"


# 在 HF 缓存目录里找一个能直接加载的快照。
#
# 不用 `snapshot_download(local_files_only=True)`：它要求快照「完整」，缺一个
# .gitattributes 或 README 就判失败，然后联网重下 —— 而权重都在，只差两个文本
# 文件。自己按目录找，只要 config.json 和权重在就算可用。
def _cached(repo: str) -> str | None:
    home = Path(os.environ.get("HF_HOME", Path.home() / ".cache/huggingface")) / "hub"
    snaps = home / ("models--" + repo.replace("/", "--")) / "snapshots"
    if not snaps.is_dir():
        return None
    for snap in sorted(snaps.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True):
        if (snap / "config.json").is_file() and (snap / "model.safetensors").exists():
            return str(snap)
    return None


# 把「仓库名」或「本地目录」统一成可以喂给 mlx_lm.load 的路径。
#
# 顺序很关键：先本地缓存，再考虑下载。反过来的话每次启动都要等一次远端探测，
# 缓存命中也得 150 s。
def resolve(model: str = LOCAL_MODEL) -> str:
    local = Path(model).expanduser()
    if local.is_dir():
        return str(local)

    cached = _cached(model)
    if cached is not None:
        return cached

    os.environ.setdefault("HF_ENDPOINT", MIRROR)
    from huggingface_hub import snapshot_download

    return snapshot_download(model)


# 按 spec 造一个模型。远程 / 本地的判断只有一处（`remote.is_remote`）：
#
#     deepseek-flash                              → 远程（OpenAI 兼容）
#     mlx-community/Qwen2.5-1.5B-Instruct-4bit    → 本机 MLX（带斜杠的是仓库名）
#     /Users/.../snapshots/abc                    → 本机目录
def make(model: str | None = None, **kwargs):
    spec = (model or DEFAULT_MODEL).strip()
    from quadruped.llm.remote import RemoteModel, is_remote

    if is_remote(spec):
        return RemoteModel(spec, **kwargs)
    return LocalModel(spec)


class LocalModel:
    """一次加载，多次生成。不是线程安全的，调用方保证串行。"""

    def __init__(self, model: str = LOCAL_MODEL):
        from mlx_lm import load

        path = resolve(model)
        t0 = time.perf_counter()
        self.model, self.tokenizer = load(path)
        self.load_seconds = time.perf_counter() - t0
        self.path = path

    # 生成一次。温度 0（贪心）是为了让同样的输入得到同样的输出 —— 调提示词的
    # 时候，随机性只会让人分不清是改了提示词还是抽到了别的采样。
    #
    # 新版 mlx-lm 的 `generate()` 只收 `prompt` / `max_tokens`，其余走 kwargs 转给
    # `generate_step`，采样参数要包成 sampler 对象传进去。
    # `cancel` 只为和远程模型接口一致。本地一次 1~2 s，且 mlx 的生成循环不暴露
    # 中断点，所以这里不实现 —— 传进来也照常生成完。
    def generate(self, messages: list[dict], max_tokens: int = 96,
                 cancel=None) -> str:
        from mlx_lm import generate as mlx_generate
        from mlx_lm.sample_utils import make_sampler

        prompt = self.tokenizer.apply_chat_template(
            messages, add_generation_prompt=True, tokenize=False)
        return mlx_generate(
            self.model, self.tokenizer, prompt=prompt, max_tokens=max_tokens,
            verbose=False, sampler=make_sampler(temp=0.0),
        )

    # 释放权重。长跑不需要，改模型或退出时用得上。
    def close(self) -> None:
        self.model = None
        self.tokenizer = None
