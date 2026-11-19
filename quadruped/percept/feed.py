"""相机馈送：把画面按固定频率写成 JPEG 文件，给网关取。

    python -m quadruped.percept.feed --preset sim --out /tmp/go2-cam.jpg
    python -m quadruped.percept.feed --preset real --hz 8 --quality 75

## 为什么单独一个进程

手动遥控时相机是没人用的：`comm.server` 只要运动控制，VLA 没跑就没有画面。
而遥控时恰恰最需要看到前面的路。

不塞进 `comm.server` 的原因和 `comm/__init__.py` 里那条一样：`ChannelFactoryInitialize`
一进程只调一次，谁先调谁的 domain 和网卡会被后面所有调用沿用。服务端先初始化过
控制链路，再往里加相机，等于把相机挂在控制那套配置上；反过来更糟。分进程就只有
一份 DDS 配置，各管各的。

## 两个源，一个出口

    仿真    VideoStream    MuJoCo 的机载相机，UDP 分片帧（percept_pub.py 发）
    真机    CameraStream   videohub RPC，`GetImageSample()` 直接给 JPEG

两者的取法完全不同（UDP 拼帧 / 阻塞 RPC），但对下游一样：一张 RGB 图。所以这里
只做「取图 → 缩放 → 编码 JPEG → 原子替换文件」。

**videohub 只认一个订阅者。** 官方 App 连着的时候这边拿不到帧（`code != 0`）。
那不是故障，但要能看出来 —— 所以连续取不到帧时会打印原因，而不是静默空转。

## 原子替换

网关按 mtime 变化判断「有新帧」，然后整读文件。直接覆盖写会让它读到半张图
（JPEG 少了尾部就是花的）。所以先写 `.tmp` 再 `os.replace`：同一文件系统上是
原子的，读侧要么看到旧帧、要么看到新帧。
"""

from __future__ import annotations

import argparse
import io
import os
import signal
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


# 缩到 max_width 再编 JPEG。
#
# 优先用 cv2：同一张 1280x720 缩到 640，PIL 在 Orin NX 上要 90~110 ms，
# cv2 只要 8~12 ms。这个差别直接决定画面是 9 fps 还是 25 fps —— 背景图
# 一眼能看出来卡不卡。
#
# PIL 留作兜底：相机解码那侧也依赖 cv2，真机上没装就能走到这里。
def _encode(rgb, quality: int, max_width: int) -> bytes:
    try:
        import cv2
    except Exception:                                       # noqa: BLE001
        cv2 = None

    if cv2 is not None:
        img = rgb
        h, w = img.shape[:2]
        if max_width and w > max_width:
            nh = max(1, round(h * max_width / w))
            img = cv2.resize(img, (max_width, nh), interpolation=cv2.INTER_LINEAR)
        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR),
            [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if ok:
            return buf.tobytes()

    from PIL import Image

    img = Image.fromarray(rgb)
    if max_width and img.width > max_width:
        height = max(1, round(img.height * max_width / img.width))
        # BILINEAR 够了：这张图最终是当背景用的，不是给人放大看细节的。
        img = img.resize((max_width, height), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=quality, optimize=False)
    return buf.getvalue()


# 写一帧。先写 .tmp 再 replace，读侧不会看到半张图。
def _write(out: Path, data: bytes) -> None:
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="feed", description="相机馈送：按频率把画面写成 JPEG")
    ap.add_argument("--preset", choices=("sim", "real"), default="sim")
    ap.add_argument("--iface", default=None, help="网卡名，不给按 preset 推")
    ap.add_argument("--domain", type=int, default=None, help="DDS 域号")
    ap.add_argument("--out", default="/tmp/go2-cam.jpg", help="输出 JPEG 路径")
    ap.add_argument("--hz", type=float, default=8.0, help="出图频率")
    ap.add_argument("--quality", type=int, default=78, help="JPEG 质量 1~95")
    ap.add_argument("--max-width", type=int, default=640,
                    help="超过这个宽度就等比缩小，0=不缩")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    def log(msg: str) -> None:
        if not args.quiet:
            print(f"[cam] {msg}", flush=True)

    stop = False

    def quit_(signum, frame):                               # noqa: ARG001
        nonlocal stop
        stop = True

    # SIGINT 在后台起时可能是 SIG_IGN；显式装一遍，否则「停止」要等 SIGKILL。
    signal.signal(signal.SIGINT, quit_)
    signal.signal(signal.SIGTERM, quit_)

    if args.preset == "real":
        # 真机走 videohub。DDS 必须在本进程里初始化，且只此一次。
        from quadruped.control.virtual.robot import resolve_endpoint

        ep = resolve_endpoint(args.preset, iface=args.iface, domain=args.domain)
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize

            ChannelFactoryInitialize(ep.domain, ep.iface)
        except Exception as err:                            # noqa: BLE001
            if "already" not in str(err).lower():
                print(f"初始化 DDS 失败：{err}", flush=True)
                return 2
        log(f"真机相机：{ep.label}")
        from quadruped.percept.camera import CameraStream

        source = CameraStream(hz=args.hz, verbose=not args.quiet)
        source.start()
        read = source.read
        describe = lambda: (f"收图 {source.frames} 帧 / 丢 {source.dropped} / "
                            f"code {source.last_code}")
    else:
        from quadruped.percept.camera import VideoStream

        try:
            source = VideoStream()
        except OSError as err:
            # 同一个 UDP 端口只能有一个读者。撞上了说明还有 VLA 或另一个 feed
            # 在跑 —— 说清楚，别甩一段 traceback 让人猜。
            print(f"视频端口被占用（{err}）：多半是另一个 VLA 或相机馈送在跑。"
                  "先把那个停掉再试。", flush=True)
            return 3
        log(f"仿真相机：UDP {source.port}")
        read = source.read
        describe = lambda: (f"收图 {source.frames} 帧 / 丢 {source.dropped}")

    period = 1.0 / max(0.5, args.hz)
    tick = time.perf_counter()
    frames = 0
    last_ok = 0.0
    last_note = 0.0
    t0 = time.monotonic()

    try:
        while not stop:
            rgb = read()
            if rgb is not None:
                try:
                    _write(out, _encode(rgb, args.quality, args.max_width))
                    frames += 1
                    last_ok = time.monotonic()
                except Exception as err:                    # noqa: BLE001
                    log(f"编码失败：{type(err).__name__}: {err}")

            now = time.monotonic()
            # 每 5 秒报一次状态。取不到帧的时候这条是唯一能看出原因的线索
            # （videohub 被别的客户端占着、相机在仿真里被关掉、网卡不对）。
            if now - last_note > 5.0:
                age = None if not last_ok else round(now - last_ok, 1)
                last_note = now
                # 雷达在另一个进程里跑（`percept.radar_feed`），这里只报图。
                log(f"{describe()} · 已出图 {frames} · 最近一帧 "
                    f"{'无' if age is None else str(age) + 's 前'}")

            tick += period
            rest = period - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            source.close()
        except Exception:                                   # noqa: BLE001
            pass
        log(f"退出，共出图 {frames} 帧，运行 {time.monotonic() - t0:.0f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
