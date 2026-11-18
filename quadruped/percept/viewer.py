"""把「狗眼看到的画面」「仿真里的全局画面」和「L 层的想法」放进同一个窗口。

终端里只有字。「vx=0」这一行为什么出现，靠读日志分不出来 —— 是看见台阶了，还是模型
胡说了一句。画面和想法并排放在一起就能对上：画面里有台阶，想法里就该提到台阶；画面
是平地而想法说「前方半米有墙」，那是模型的问题，不是环境的问题。

    viewer = Viewer(dual=True, record="docs/vl_watch.mp4")
    while ...:
        if not viewer.pump(rgb=eye, world=world, scene=..., thought=...):
            break            # 用户按了 Esc，或者关了窗口

## 退出键只认 Esc

不用 `q`：键盘遥控里 `q` 是左转（`physical/keyboard.py` 的 `DIRS["q"] = (0,0,1)`）。
两套控制器共用一个键、一个退出一个转向，转的时候手一抖就把进程关了。窗口也不再
接管任何带动作语义的字母键 —— VLA 是自驱的，人只需要能停。

## 两个镜头

机载那一路是喂给视觉模型的同一张图；全局那一路从斜后方看整台狗，只给人看。

为什么要第二个：机载画面里看不到机身自己。实测狗翻了之后，眼睛看到的是
「蓝色的地面、白色的天空」，和正常走路没有区别 —— 光看这一路会以为是平地。
全局画面一眼就能看出它四脚朝天。

真机上只有机载一路（人站在旁边看得到狗），这时 `dual=False`，窗口退回单一画面。

## 两个约束

`pump()` 必须在主线程调。macOS 的 cv2 窗口只能在主线程更新（`imshow` + `waitKey`），
放到感知线程里会直接崩。所以这里的循环节奏由上层的 while 决定，本类不自己起线程。

一个窗口只显示最新一帧，不排队。画面 5 Hz、显示循环 20 Hz，攒帧只会让画面比想法晚。

## 为什么不 `cv2.putText`

它只画 ASCII，中文会变成一串问号。文字统一交给 PIL + 系统字体，一次画完：
cv2 先画底、画面和分隔线，PIL 在最后一遍把字贴上去。
"""

from __future__ import annotations

import os
import time

import numpy as np

# 配色（RGB）。画面和终端日志是两回事，这里不跟随终端主题 —— 相机窗口配浅底色
# 会让画面发灰，看不清台阶边缘。
BG = (20, 22, 26)
CARD = (30, 33, 39)
LINE = (54, 58, 66)
TEXT = (230, 233, 238)
DIM = (146, 152, 162)
ACCENT = (122, 214, 168)
WARN = (240, 176, 96)

# 两种版式：两路镜头（仿真）和一路镜头（真机）。
#
# `tiles` 是 (x, y, w, h, 标签)。机载 320x240 放大 2 倍是 640x480，最清晰；
# 两路叠起来要 800 多像素高，屏幕放不下，所以双路时各用 400x300（1.25 倍）。
LAYOUT_DUAL = {
    "size": (916, 690),
    "tiles": ((12, 12, 400, 300, "机载"), (12, 324, 400, 300, "全局")),
}
LAYOUT_SINGLE = {
    "size": (1060, 620),
    "tiles": ((12, 12, 640, 480, "机载"),),
}

# 系统中文字体。找不到就用 PIL 默认字体（画不出中文，但不崩），
# 静默画成方块的话，看到的是「字没了」，会让人去查别处。
FONTS = (
    "/System/Library/Fonts/STHeiti Medium.ttc",
    "/System/Library/Fonts/Hiragino Sans GB.ttc",
    "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
)

BODY = 15          # 正文字号
SMALL = 13
HEAD = 16

# 导出给网页的 JPEG 上限。
#
# **这不该是帧率上限，只是个保险。** 取 30（比主循环的 `VIEW_HZ = 20` 快），
# 让每一帧都导出去；它只拦「有人把 `pump` 调到 200 Hz」那种编码风暴。
#
# 两个数都改过：原来写 5 Hz，依据是「916x690 的 JPEG 编码要 15~25 ms」。实测差一个
# 数量级 —— 一帧渲染 6.1 ms + 编码 0.7 ms（q75、32 KB），20 Hz 占单核约 14%。
#
# 另一个坑：导出上限和主循环同为 20 Hz 时，两边的 50 ms 各自抖动，会交替丢帧，
# 实测只有 12 fps。上限必须比循环快，才不会变成节拍器。
STREAM_HZ = 30.0


def _font_path() -> str | None:
    import os

    for path in FONTS:
        if os.path.exists(path):
            return path
    return None


class Viewer:
    def __init__(self, *, dual: bool = True, title: str = "Go2 · V+L",
                 display: bool = True, record: str | None = None,
                 stream: str | None = None, fps: float = 20.0):
        from PIL import ImageFont

        layout = LAYOUT_DUAL if dual else LAYOUT_SINGLE
        self.dual = bool(dual)
        self.width, self.height = layout["size"]
        self.tiles = layout["tiles"]
        self.title = title
        self.display = bool(display)

        # 导出给网页的那一路。`display=False` 时也照样渲染，只是不 `imshow` ——
        # 真机上没有图形会话（`--no-watch`），网页就是唯一的窗口。
        self._stream = str(stream) if stream else None
        self._stream_tmp = f"{self._stream}.tmp" if self._stream else None
        self._stream_at = 0.0
        self.streamed = 0          # 实际导出了多少帧（诊断 / 测试用）

        # 版式上的固定位置。写死而不是跟着文字累加：文字块的行数随内容变，
        # 累加会让下面的视野条上下跳，录成视频更晃。
        first = self.tiles[0]
        self.panel_x = first[0] + first[2] + 12
        self.panel_w = self.width - self.panel_x - 24
        self.foot_y = self.height - 30
        self.sector_y = self.height - 190
        self.bar_x = self.panel_x + 56
        self.bar_w = self.panel_w - 120

        self._font_path = _font_path()
        self._fonts: dict[int, object] = {}
        for size in (BODY, SMALL, HEAD):
            self._fonts[size] = (ImageFont.load_default() if self._font_path is None
                                 else ImageFont.truetype(self._font_path, size))
        self._wcache: dict[tuple[int, str], float] = {}

        self._canvas = np.full((self.height, self.width, 3), BG, dtype=np.uint8)
        self._bgr = None
        self._eye = None           # 上一帧机载画面，掉线时继续显示它
        self._world = None
        self.frames = 0
        self.closed = False

        # 实测节拍。画面「一秒一帧」时，光看主循环代码分不出是渲染慢、`imshow`
        # 慢，还是主循环被别人拖住 —— 这三处的修法完全不同。记三段的耗时
        # （毫秒，指数滑动平均）和实际 `pump` 频率，由 `runner` 打进日志。
        #
        # 选 EMA 而不是最近 N 帧的列表：这里每帧都写，列表会一直分配；
        # 而「慢」是持续状态，不是单个异常值，平滑过的数更好读。
        self.fps = 0.0
        self.render_ms = 0.0
        self.show_ms = 0.0
        self.stream_ms = 0.0
        self._rate_at = 0.0
        self._rate_n = 0
        self._writer = None
        self._record_path = None
        self._fps = float(fps)

        if record:
            import cv2

            self._writer = cv2.VideoWriter(str(record),
                                           cv2.VideoWriter_fourcc(*"mp4v"),
                                           self._fps, (self.width, self.height))
            if not self._writer.isOpened():                # 编码器缺失时不要静默
                self._writer = None
                raise RuntimeError(f"打不开视频写入：{record}")
            self._record_path = str(record)

    # ---- 对外 ------------------------------------------------------------

    @property
    def recording(self) -> str | None:
        return self._record_path

    # 机载画面。传原始 RGB（320x240），这里放大到版式尺寸。
    #
    # 不在这里做 BGR 转换：转换只在 `imshow` 和写视频这两个出口做。
    def feed(self, rgb) -> None:
        if rgb is not None:
            self._eye = rgb
            self.frames += 1

    def feed_world(self, rgb) -> None:
        if rgb is not None:
            self._world = rgb

    # 拼一帧、显示、返回还能不能继续。
    #
    # 参数全是可选：某一项没有就画「无」，不抛。显示这一层挂掉不该带走控制回路 ——
    # 看画面是给人看的，狗还在走。
    def pump(self, *, rgb=None, world=None, scene: "str | None" = "",
             scene_age: float | None = None, thought: str = "", why: str = "",
             thought_off: bool = False,
             command: dict | None = None, goal: str = "", sectors: list | None = None,
             history: list | None = None, foot: list[str] | None = None,
             alert: bool = False) -> bool:
        import cv2
        from PIL import Image, ImageDraw

        if self.closed:
            return False
        t0 = time.perf_counter()
        self.feed(rgb)
        self.feed_world(world)

        canvas = self._canvas
        canvas[:] = BG

        # 第一遍：只画块和线，全部走 cv2。文字留到第二遍，因为 cv2 画不出中文。
        for (x, y, w, h, _), source in zip(self.tiles, self._sources()):
            canvas[y:y + h, x:x + w] = (
                cv2.resize(source, (w, h), interpolation=cv2.INTER_LINEAR)
                if source is not None
                else np.full((h, w, 3), CARD, dtype=np.uint8))
            cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), LINE, 1)
            # 标签底条，压在画面左下角
            cv2.rectangle(canvas, (x, y + h - 26), (x + 54, y + h - 1), BG, -1)

        cv2.rectangle(canvas, (self.panel_x, 12),
                      (self.panel_x + self.panel_w, self.height - 12), CARD, -1)
        cv2.line(canvas, (12, self.foot_y - 14), (self.width - 12, self.foot_y - 14),
                 LINE, 1)

        # 视野条：8 个方位各一条，长度就是该方向的距离。
        # 画成条而不是极坐标图，是因为要读的是「哪边最空」，条的长度直接能比。
        if sectors:
            top = self.sector_y - 26
            cv2.line(canvas, (self.panel_x + 16, top),
                     (self.panel_x + self.panel_w - 16, top), LINE, 1)
            for i, (_, gap) in enumerate(sectors):
                ry = self.sector_y + 4 + i * 17
                full = int(min(1.0, gap / 3.0) * self.bar_w)
                colour = WARN if gap < 0.6 else (ACCENT if gap > 2.99 else (108, 152, 200))
                cv2.rectangle(canvas, (self.bar_x, ry),
                              (self.bar_x + max(2, full), ry + 6), colour, -1)

        # 第二遍：文字。
        img = Image.fromarray(canvas)
        draw = ImageDraw.Draw(img)

        for (x, y, w, h, label), source in zip(self.tiles, self._sources()):
            draw.text((x + 10, y + h - 22), label, font=self._fonts[SMALL], fill=TEXT)
            if source is None:
                draw.text((x + 16, y + h // 2 - 10), "等待画面（仿真侧关掉了吗）",
                          font=self._fonts[BODY], fill=DIM)

        cx = self.panel_x + 16
        cy = 24
        cy = self._header(draw, cx, cy, "V 画 面")
        age = "" if scene_age is None else f"（{scene_age:.1f} s 前）"
        if scene is None:
            shown = "（这一路没开视觉模型）"
        else:
            shown = (scene or "（视觉模型还没给出描述）") + age
        cy = self._body(draw, cx, cy, shown, max_lines=3,
                        color=TEXT if scene else DIM, size=SMALL)
        cy = self._rule(draw, cx, cy + 2)

        cy = self._header(draw, cx, cy + 2, "L 想 法")
        if goal:
            cy = self._body(draw, cx, cy, f"目标：{goal}", max_lines=1,
                            color=DIM, size=SMALL)
        # 三态分开写，别都落成「还没想过」：
        #   thought_off  这一版根本不产出想法（`prompt.EMIT_THOUGHT = False`）
        #   空字符串     能产出，只是这一拍还没拿到
        #   有内容       正常
        if thought_off:
            said, tone = "（这一版不产出想法，只看命令）", DIM
        else:
            said, tone = thought or "（还没想过）", ACCENT if thought else DIM
        cy = self._body(draw, cx, cy, said, max_lines=3, color=tone, size=BODY)
        if why:
            cy = self._body(draw, cx, cy, f"触发：{why}", max_lines=2,
                            color=WARN, size=SMALL)
        if command:
            shown = "  ".join(f"{k}={v:g}" for k, v in command.items())
            cy = self._body(draw, cx, cy, f"命令：{shown}", max_lines=1,
                            color=DIM, size=SMALL)
        cy = self._rule(draw, cx, cy + 2)

        if history:
            cy = self._header(draw, cx, cy + 2, "最 近")
            for stamp, text in history[-3:]:
                cy = self._body(draw, cx, cy, f"{stamp:5.1f}s  {text}", max_lines=1,
                                color=DIM, size=SMALL)

        if sectors:
            draw.text((cx, self.sector_y - 24), "视 野（机头 0°，左为正）",
                      font=self._fonts[HEAD], fill=TEXT)
            for i, (name, gap) in enumerate(sectors):
                ry = self.sector_y + 6 + i * 17
                draw.text((cx, ry), name, font=self._fonts[SMALL], fill=DIM)
                label = "空" if gap > 2.99 else f"{gap:.2f}"
                draw.text((self.bar_x + self.bar_w + 10, ry), label,
                          font=self._fonts[SMALL],
                          fill=TEXT if gap <= 2.99 else DIM)

        pieces = list(foot or [])
        if pieces:
            self._foot(draw, "   ".join(pieces), color=WARN if alert else DIM)

        self._bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
        t_render = time.perf_counter()

        if self._writer is not None:
            self._writer.write(self._bgr)
        if self._stream:
            self._export()
        t_stream = time.perf_counter()
        if not self.display:
            self._timing(t0, t_render, t_stream, t_stream)
            return True

        cv2.imshow(self.title, self._bgr)
        key = cv2.waitKey(1) & 0xFF
        self._timing(t0, t_render, t_stream, time.perf_counter())
        # 只认 Esc。`q` 留给键盘遥控的「左转」，两套控制器不能给同一个键
        # 相反的含义 —— 转向时按到 q 会直接把整个进程关掉。
        if key == 27:
            return False
        # 用户点红叉：cv2 没有回调，只能靠可见性判断。
        try:
            if cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False
        return True

    # 记一次 pump 的三段耗时和实际频率。
    #
    # `a = 0.1` 是经验值：约 10 帧收敛，慢几帧之后不会一直挂着旧值。
    # 频率按 1 秒窗口算，不用 EMA —— 频率要能被整数直观验证（「3 fps」应该
    # 正好对应 3 次），平滑过的 3.7 反而看不出是漏拍还是真慢。
    def _timing(self, t0: float, t_render: float, t_stream: float,
                t_end: float) -> None:
        a = 0.1
        self.render_ms += a * ((t_render - t0) * 1000.0 - self.render_ms)
        self.stream_ms += a * ((t_stream - t_render) * 1000.0 - self.stream_ms)
        self.show_ms += a * ((t_end - t_stream) * 1000.0 - self.show_ms)
        if not self._rate_at:
            self._rate_at, self._rate_n = t_end, 1
            return
        self._rate_n += 1
        span = t_end - self._rate_at
        if span >= 1.0:
            self.fps = self._rate_n / span
            self._rate_at, self._rate_n = t_end, 0

    # 一行给人看的节拍摘要。日志里和「策略 50 Hz」并排放，才知道是谁慢。
    def timing_line(self) -> str:
        shown = "" if self.display else "（无窗口）"
        return (f"画面 {self.fps:4.1f} fps  画 {self.render_ms:.1f} ms  "
                f"显 {self.show_ms:.1f} ms  "
                f"导 {self.stream_ms:.1f} ms{shown}")

    def _sources(self):
        return [self._eye] + ([self._world] if self.dual else [])

    # 底栏。整窗口宽，和右栏正文的折行宽度不一样，所以单独一个方法。
    #
    # 截断不逐字重测。这段底栏是 8~9 个指标拼起来的（一百多字），超宽时
    # `text = text[:-1]` 那种写法要测一百多次、每次量整串 —— 一帧几十毫秒，
    # 比整个画面渲染还贵，而且只在「恰好超宽」时出现，很难发现。
    def _foot(self, draw, text: str, *, color=DIM) -> None:
        font = self._fonts[SMALL]
        # 网页那一路（`display=False`）没有窗口，也就没有 Esc 可按 —— 写上去只会
        # 让人对着网页找这个键。
        hint = "Esc 退出" if self.display else ""
        hint_w = self._token_w(draw, hint, SMALL) if hint else 0
        limit = self.width - 40 - hint_w - 24
        total = sum(self._token_w(draw, tok, SMALL) for tok in _tokens(text))
        if total > limit:
            text = _ellipsis(draw, text, limit,
                             lambda t: self._token_w(draw, t, SMALL))
        draw.text((20, self.foot_y), text, font=font, fill=color)
        if hint:
            draw.text((self.width - 20 - hint_w, self.foot_y), hint,
                      font=font, fill=DIM)

    # 存一张 PNG。无显示环境（CI、远程）里验证版式用这个。
    def shot(self, path: str) -> str:
        import cv2

        if self._bgr is None:
            raise RuntimeError("还没有可存的画面，先 pump()")
        cv2.imwrite(str(path), self._bgr)
        return str(path)

    # 把刚渲染的这一张写成 JPEG，给网页那条路取。
    #
    # 先写 `.tmp` 再 `os.replace`：网页随时可能来读，直接写目标文件会读到半张图
    # （浏览器显示成一片灰）。同目录 rename 是原子的。
    #
    # 失败不抛 —— 和 `pump` 里同一个原则：显示这一层挂掉不该带走控制回路，
    # 狗还在走，只是网页上看不到画面。
    def _export(self) -> None:
        import cv2

        now = time.monotonic()
        if now - self._stream_at < 1.0 / STREAM_HZ:
            return
        self._stream_at = now
        try:
            ok, buf = cv2.imencode(".jpg", self._bgr,
                                   [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                return
            with open(self._stream_tmp, "wb") as fh:
                fh.write(buf.tobytes())
            os.replace(self._stream_tmp, self._stream)
            self.streamed += 1
        except OSError:
            pass

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self.display:
            import cv2

            try:
                cv2.destroyWindow(self.title)
            except cv2.error:
                pass

    # ---- 排版 ------------------------------------------------------------

    def _header(self, draw, x: int, y: int, text: str) -> int:
        draw.text((x, y), text, font=self._fonts[HEAD], fill=TEXT)
        return y + 22

    def _rule(self, draw, x: int, y: int) -> int:
        draw.line((x, y, x + self.panel_w - 32, y), fill=LINE, width=1)
        return y + 8

    # 画一段会自动折行的文字，返回下一行的 y。
    #
    # 折行按字而不是按词：中文没有空格，按词折会整段不换行。ASCII 的连续字母数字
    # 算一个整体，否则「0.28」会被拆成「0.2」和「8」。
    #
    # **宽度必须逐 token 累加，不能每加一个 token 就重测整行。** 原来那样写是
    # O(n²)：一段 30 字的想法要测 30 次、每次量 15 字，实测一帧 157 次
    # `Font.getlength`、9.5 ms，比所有绘制加起来还多。
    #
    # 累加有 kerning 的误差（相邻字对收窄零点几像素），中文混排里量不出来；
    # 真要紧的是版面宽度，而这里本来就留了 8 px 余量。
    def _body(self, draw, x: int, y: int, text: str, *, max_lines: int = 3,
              color=TEXT, size: int = BODY, gap: int = 4) -> int:
        font = self._fonts[size]
        limit = self.panel_w - 32
        line_h = size + 6
        lines: list[str] = []
        cur = ""
        cur_w = 0.0
        # 先把换行和连续空白压成单空格。
        #
        # `_tokens` 会把 `\n` 当成一个 token 原样返回，而 `draw.textlength()`
        # 对含换行的字符串直接抛 `ValueError: can't measure length of multiline
        # text`。视觉模型的判读是三行的（`RemoteVlm.describe` 特意保留换行），
        # 所以这条路上必炸 —— 不是偶发。
        #
        # 压成一段再折行是对的：版式宽度是固定的，这里本来就要自己折。
        text = " ".join((text or "").split())
        for tok in _tokens(text):
            w = self._token_w(draw, tok, size)
            if cur_w + w <= limit or not cur:
                cur += tok
                cur_w += w
                continue
            lines.append(cur)
            cur = tok.lstrip()
            cur_w = self._token_w(draw, cur, size)
            if len(lines) >= max_lines:
                break
        if cur and len(lines) < max_lines:
            lines.append(cur)
        elif cur:
            lines[-1] = _ellipsis(draw, lines[-1], limit,
                                  lambda t: self._token_w(draw, t, size))

        for i, line in enumerate(lines):
            draw.text((x, y + i * line_h), line, font=font, fill=color)
        return y + len(lines) * line_h + gap + 8

    # 一个 token 的像素宽度，按 (字号, 文本) 缓存。
    #
    # 字号只有三档，token 集合很小（汉字 + 数字 + 常见 ASCII 块），跑几帧之后
    # 基本全是命中。缓存放在实例上而不是模块级：字号真变了（改版式）会跟着重建。
    def _token_w(self, draw, tok: str, size: int) -> float:
        key = (size, tok)
        got = self._wcache.get(key)
        if got is None:
            got = draw.textlength(tok, font=self._fonts[size])
            self._wcache[key] = got
        return got


# 把一段中英混排切成「不能再拆」的单位：连续 ASCII 字母数字算一个，其它每个字一个。
def _tokens(text: str) -> list[str]:
    out: list[str] = []
    buf = ""
    for ch in text:
        if ch.isascii() and (ch.isalnum() or ch in "-_./:%+="):
            buf += ch
            continue
        if buf:
            out.append(buf)
            buf = ""
        out.append(ch)
    if buf:
        out.append(buf)
    return out


def _ellipsis(draw, line: str, limit: float, token_w) -> str:
    # 同样不逐字重测：先按 token 宽度累加，超了才截。`token_w` 由调用方给
    # （`Viewer._token_w`），走的是同一份缓存。
    dot = token_w("…")
    out, used, cut = "", 0.0, False
    for tok in _tokens(line):
        w = token_w(tok)
        if used + w + dot > limit:
            cut = True
            break
        out += tok
        used += w
    return out + "…" if cut else line
