"""雷达馈送：订阅点云，按频率把八方位距离写成 JSON 给网关取。

    python -m quadruped.percept.radar_feed --preset real --iface eth10 \
        --out /tmp/go2-radar.json --hz 10

## 为什么从 `feed` 里拆出来

原来雷达和相机在同一个进程里跑。真机实测（Orin NX，eth10）：

    只开相机           185 帧 / 20 s   ≈ 9 fps
    相机 + 雷达同进程   17 帧 / 20 s   ≈ 0.9 fps，且 `GetImageSample`
                       开始返回 3104（超时）

点云回调每秒 20 次、每次要把几万个点转成 numpy 再算 24 条射线，全程握着 GIL；
相机那条 RPC 线程抢不到 GIL，单次调用从 9 ms 涨到 1 s。两件事都不重，但**不能
待在同一个解释器里**。

拆开之后各占一个进程、各占一个核，DDS 各初始化一次（这正是 `comm/__init__.py`
那条规则允许的做法：一进程一次）。

## 它不拥有相机

videohub 只认一个订阅者，相机归 `feed`。这里只读 `rt/utlidar/cloud` 和
`sportmodestate`，不碰 `lowcmd`、不碰 camera RPC。
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


# 雷达的取值只需要位姿、倾角、高度和关节角，不需要控制权。
#
# 为什么不复用 `control.virtual.backends.OfficialBackend`：它在构造里又调一次
# `ChannelFactoryInitialize`，而这个进程已经初始化过了 —— 第二次调用抛异常，
# 整条馈送就挂。这里只要读，不需要 SportClient，自己订阅两个话题更省。
class _FeedBackend:
    """够 `LidarPercept` 用的一份只读后端。"""

    def __init__(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_

        self._sport = None
        self._low = None
        ChannelSubscriber("rt/sportmodestate", SportModeState_).Init(self._on_sport, 10)
        ChannelSubscriber("rt/lowstate", LowState_).Init(self._on_low, 10)

    def _on_sport(self, msg) -> None:
        self._sport = msg

    def _on_low(self, msg) -> None:
        self._low = msg

    def _wait(self, timeout: float = 3.0) -> bool:
        import time as _t

        end = _t.monotonic() + timeout
        while _t.monotonic() < end:
            if self._sport is not None and self._low is not None:
                return True
            _t.sleep(0.05)
        return self._sport is not None

    @property
    def pos(self):
        sp = self._sport
        return None if sp is None else tuple(float(v) for v in sp.position[:3])

    @property
    def rpy(self):
        sp = self._sport
        if sp is not None:
            imu = sp.imu_state
            return (float(imu.rpy[0]), float(imu.rpy[1]), float(imu.rpy[2]))
        low = self._low
        if low is None:
            return None
        q = low.imu_state.quaternion           # w, x, y, z
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        import math

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return (roll, pitch, yaw)

    @property
    def height(self):
        sp = self._sport
        return None if sp is None else float(sp.body_height)

    @property
    def q(self):
        low = self._low
        if low is None:
            return None
        return [float(m.q) for m in low.motor_state[:12]]


# 先写 .tmp 再 replace，读侧不会读到半份 JSON。
def _write(out: Path, data: bytes) -> None:
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="radar_feed", description="雷达馈送：八方位距离写成 JSON")
    ap.add_argument("--preset", choices=("sim", "real"), default="real")
    ap.add_argument("--iface", default=None, help="网卡名，不给按 preset 推")
    ap.add_argument("--domain", type=int, default=None, help="DDS 域号")
    ap.add_argument("--out", default="/tmp/go2-radar.json", help="输出 JSON 路径")
    ap.add_argument("--hz", type=float, default=10.0, help="写入频率")
    # 只订一个点云话题。两个都订的代价实测：`rt/utlidar/cloud`（4196 点，5.6 ms）
    # 加上 `rt/utlidar/cloud_deskewed`（11667 点，14.2 ms），各 15.5 Hz，光解包就
    # 占掉两个核 —— cyclonedds 的 Python 绑定把 `uint8[]` 展开成 Python 列表，
    # deskewed 那一帧是 56 万个整数。四核的 Orin NX 上这会直接把相机饿死。
    #
    # `cloud` 足够做八方位障碍判断；换了固件只有 deskewed 的，用这个参数指过去。
    # 默认订 `cloud_base`（1584 点 / 50 KB），不是 `cloud`（4188 点 / 134 KB）。
    # 两者频率一样（21 Hz），但 cyclonedds 的 Python 绑定解析 `uint8[]` 是逐字节的：
    # 实测 `cloud` 要 3.6 个核，`cloud_base` 要 2.0 个 —— 这台机器只有 4 核，
    # 差出来的那 1.6 个核正好是相机能不能跑到 20 fps。
    ap.add_argument("--topic", default="rt/utlidar/cloud_base",
                    help="点云话题名（默认 rt/utlidar/cloud_base）")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    def log(msg: str) -> None:
        if not args.quiet:
            print(f"[radar] {msg}", flush=True)

    if args.preset != "real":
        log("仿真没有点云源，退出")
        return 2

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    from quadruped.control.virtual.robot import resolve_endpoint

    ep = resolve_endpoint(args.preset, iface=args.iface, domain=args.domain)
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(ep.domain, ep.iface)
    except Exception as err:                                # noqa: BLE001
        if "already" not in str(err).lower():
            log(f"初始化 DDS 失败：{err}")
            return 2
    log(f"雷达：{ep.label}")

    from quadruped.percept.lidar import LidarPercept

    backend = _FeedBackend()
    if not backend._wait(3.0):
        log("3 秒内没收到 sportmodestate，先用低层状态顶着")
    # 高度图也关掉：遥控只看八方位有没有东西，台阶/坑那套判断在自动控制里才用。
    # 每多订一路就是一串 uint8 在跑解包，四核机器上省下来给相机。
    radar = LidarPercept(backend, verbose=False, height_map=False,
                         cloud_topics=(args.topic,))
    if not radar.ok:
        log(f"{args.topic} 还没点云（{radar.last_error or '未订阅上'}），等数据")

    stop = False

    def quit_(signum, frame):                               # noqa: ARG001
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, quit_)
    signal.signal(signal.SIGTERM, quit_)

    period = 1.0 / max(0.5, args.hz)
    tick = 0.0
    writes = 0
    last_note = 0.0
    last_clouds = 0
    bad_at = 0.0

    while not stop:
        now = time.monotonic()
        if now >= tick:
            tick = now + period
            try:
                p = radar.read()
                if p is not None and p.rays:
                    payload = {
                        "t": round(time.time(), 3),
                        "ok": True,
                        "front_gap": round(float(p.front_gap()), 2),
                        "sectors": [{"name": x.name, "gap": round(x.gap, 2),
                                     "top": round(x.top, 2)}
                                    for x in p.sectors()],
                    }
                    _write(out, json.dumps(payload).encode())
                    writes += 1
            except Exception as err:                        # noqa: BLE001
                log(f"读取失败：{type(err).__name__}: {err}")

        # 没有点云时也要有话说：`radar.ok` 为 False 时网关会把原因带给前端，
        # 比让雷达图静默空白强。一秒写一次就够，别把文件刷爆。
        if not radar.ok and now - bad_at > 1.0:
            bad_at = now
            try:
                payload = {"t": round(time.time(), 3), "ok": False,
                           "why": radar.last_error or "没有点云",
                           "front_gap": None, "sectors": []}
                _write(out, json.dumps(payload).encode())
            except Exception:                               # noqa: BLE001
                pass

        now = time.monotonic()
        if now - last_note > 10.0:
            hz = 0.0
            if last_note:
                hz = (radar.clouds - last_clouds) / (now - last_note)
            last_clouds = radar.clouds
            last_note = now
            log(f"写入 {writes} 次 · 点云 {radar.clouds} 帧 / {hz:.1f} Hz")

        time.sleep(min(0.02, period))

    out.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
