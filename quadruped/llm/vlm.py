"""本机视觉模型：把一帧画面说成一句人话。

选型（Apple M5 实测）：

    Qwen2-VL-2B-Instruct-4bit   权重 2.3 GB，加载 1.0 s，单帧 0.2~0.5 s

更小的 SmolVLM 更快，但中文描述明显更碎；再大的模型在 0.3 Hz 的预算里没有收益——
这一层的输出只有一句话，瓶颈在「看不看得见」而不是「说得好不好」。

## 它负责什么

VLA 的 V。L 层要回答「往哪走」，而这个问题只能拆成两个：**那边是什么**、
**多远**。两件都该由视觉给，不能一件交给相机、一件交给雷达 —— 真机上没有雷达。

所以这一层输出的是三个方位的「是什么 + 多远（档位）」，不是一句图片配文：

    前：墙，很近(1米内)
    左前：地面，远(3米外)
    右前：地面，中(1到3米)

## 它不负责什么

不给米数。单目、无标定，绝对距离估计的误差在 ±100% 量级，说「1.2 米」只是让
下游当真。分档（很近 / 中 / 远 / 说不准）是它能稳定区分的粒度，够 L 层判断
「现在该直行还是拐」。

## 中文短句是硬约束

2B 模型的自由发挥很容易跑偏。提示词里那三项（三方位、四档距离、「说不准」必须
存在）是试出来的：不限定方位它只说正前方；不给档位它编米数；不给「说不准」它
在纯色地面和逆光里也要强行给一个。
"""

from __future__ import annotations

import os
import tempfile
import time

from quadruped.llm.model import resolve

# 默认远程（`deepseek-flash` 是 DeepSeek 唯一收图片的模型）。本机那个 2B 留着，
# 断网或要复现旧数据时用。
DEFAULT_VLM = "deepseek-flash"
LOCAL_VLM = "mlx-community/Qwen2-VL-2B-Instruct-4bit"


# 按 spec 造视觉模型。判断口径和 `model.make()` 一样，走同一处 `is_remote`。
def make_vlm(spec: str | None = None):
    spec = (spec or DEFAULT_VLM).strip()
    from quadruped.llm.remote import RemoteVlm, is_remote

    if is_remote(spec):
        return RemoteVlm(spec)
    return LocalVlm(spec)

# 画面上画两条竖线，把它分成 左 / 中 / 右 三块。
#
# 不画线的时候模型要自己把「左前」映射到画面左边，而 320x240 的图它经常映错：
# 同一帧问两次，一次说「左前 墙壁」，一次说「前 地面，很近」。线是确定的参考，
# 画上去之后它只需要回答「这块里有什么」，不用做空间换算。
#
# 线画在 1/3 和 2/3 处，颜色取红 —— 场景是蓝灰地面 + 米色墙，红不会和它们混。
def mark_thirds(frame):
    import cv2

    out = frame.copy()
    h, w = out.shape[:2]
    for x in (w // 3, 2 * w // 3):
        cv2.line(out, (int(x), 0), (int(x), h), (230, 40, 40), 1)
    return out


# 问法固定。要的是**能用来决策的场景信息**，不是一句配文。
#
# 三块的答案按「东西，能不能过，多远」排，把「能不能过」放第二位：
#
#   能 / 不能 是这张图里最稳的一项。同一帧连问两次，左墙 0.84 m 真值下，
#     「不能」两次一致；而距离那栏在「中」和「远」之间来回跳。
#   距离只分档不给米数。单目、无标定，绝对估计误差在 ±100% 量级，给「1.2 米」
#     下游会当真。档位只作参考，L 层的转向依据用「能不能过」。
#   「说不准」必须存在。不写这一项，它在纯色地面和逆光里会强行编一个。
QUESTION = (
    "画面被两条竖线分成 左 / 中 / 右 三块。每块各写一行，格式：左：东西，能不能过，多远\n"
    "东西只用：墙壁 / 柜子 / 台阶 / 空地\n"
    "能不能过只用：能 / 不能 / 说不准（这块里有没有挡住机器狗去路的东西）\n"
    "多远只用：很近(1米内) / 中(1到3米) / 远(3米外) / 说不准\n"
    "第四行单独写人在哪一块：人：左 / 人：中 / 人：右 / 人：没有。\n"
    "  只有看清是个人（站着、走着的）才写前三项之一 —— 人在画面左边那块就写「左」，"
    "正好在中间那块写「中」。影子、电线杆、晾着的衣服、照片里的人像都算「没有」。"
    "看不清就写「没有」—— 这几行里只有这一项宁缺毋滥。\n"
    "只写这四行，不要解释，不要写「看到」「图中」。\n"
    "示例：\n"
    "左：墙壁，不能，很近(1米内)\n"
    "中：空地，能，远(3米外)\n"
    "右：墙壁，不能，中(1到3米)\n"
    "人：没有"
)


class LocalVlm:
    def __init__(self, spec: str = LOCAL_VLM):
        self.spec = spec
        t0 = time.perf_counter()
        from mlx_vlm import load

        self.model, self.processor = load(resolve(spec))
        self.load_seconds = time.perf_counter() - t0
        self._tmp = os.path.join(tempfile.gettempdir(), "go2_vlm_frame.jpg")
        self.calls = 0
        self.last_seconds = 0.0

    def close(self) -> None:
        self.model = None
        self.processor = None

    # 描述一帧 JPEG。返回一句中文；失败返回空串，不抛 ——
    # 调用方是 V 层的自驱循环，抛一次就把整个视觉通道打死了。
    def describe(self, frame, question: str = QUESTION) -> str:
        from mlx_vlm import generate
        from mlx_vlm.prompt_utils import apply_chat_template

        # 带线的图交给模型。`mark_thirds` 复制一份，不改调用方手里那张 ——
        # 它还要给显示窗口用，画了线人会以为摄像头坏了。
        frame = mark_thirds(frame)
        t0 = time.perf_counter()
        try:
            # 落盘再传路径。mlx-vlm 的图像加载接受路径 / PIL，不接受 numpy 或
            # 裸 bytes，而装 PIL 只为这一件事不值得。一张 320x240 的 JPEG 十几 KB，
            # 覆盖写代价可以忽略。
            import cv2

            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not ok:
                return ""
            with open(self._tmp, "wb") as f:
                f.write(buf.tobytes())
            prompt = apply_chat_template(self.processor, self.model.config,
                                         question, num_images=1)
            # 温度 0：同一张画面必须给出同一句描述。带随机性时实测同一个台阶会
            # 在「灰色的墙壁」和「蓝色的瓷砖」之间来回跳，L 层看到的就是「环境
            # 变了」，白白多推理一次。
            out = generate(self.model, self.processor, prompt, self._tmp,
                           max_tokens=72, verbose=False, temperature=0.0)
            text = getattr(out, "text", "") or str(out)
        except Exception:                                   # noqa: BLE001
            return ""
        self.last_seconds = time.perf_counter() - t0
        self.calls += 1
        return " ".join(text.split()).strip()
