"""仿真侧的感知发布器：往本机回环发「激光扫描」，替代真实相机。

这是临时件，不是架构的一部分。它存在的唯一理由是：V 层需要一个输入端，而真机
相机还没接。所以它刻意只输出**传感器级别的原始量** —— 每条射线的方位、距离、
命中点高度，加上机身位姿。归纳、命名、判危险这些全部留给 `quadruped/percept/`，
那边换传感器不用改。

接真相机时，这个文件整个删掉，换成一段把深度图转成同样 `rays` 的代码。V 层以上
一行不动。

射线怎么打：

    水平一圈 24 条（15° 一条），测墙和柜子这类高过头的东西
    前方 9 条垂直向下，测台阶和坑 —— 水平射线打不到 8 cm 的台阶，因为它在
    机身高度（0.31 m）以下，而那种台阶恰好是这份模型最容易踩空的

机器狗自身的 geom 会被挪到 group 3 并排除在射线之外。不改 XML：`go2.xml` 是
上游文件，改了下游同步、换模型都会踩。
"""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import threading
import time

import numpy as np

DEFAULT_PORT = 42300

# 射线不打机器人自己的 geom。这里不用 geom group 排除 —— 挪到 group 3 之后
# MuJoCo 就不画它了（`MjvOption` 默认 geomgroup 是 [1,1,1,0,0,0]），仿真窗口里
# 机器狗会消失，只剩一个球。改成一束射线打中自己就跳过重打，渲染一行不受影响。

# 地面采样的方位（度）与距离（米）。前方 1.2 m 是「再走一秒会到的地方」。
_GROUND_BEARINGS = (-40.0, 0.0, 40.0)
_GROUND_RANGES = (0.4, 0.8, 1.2)


class PerceptPublisher:
    def __init__(self, model, data, *, port: int | None = None,
                 host: str = "127.0.0.1", hz: float = 10.0,
                 beams: int = 24, max_range: float = 3.0,
                 video: bool = True, video_hz: float | None = None,
                 video_size: tuple[int, int] | None = None,
                 world: bool = True, world_distance: float = 2.4,
                 world_elevation: float = -32.0, world_azimuth: float = 135.0,
                 eye_offset: tuple[float, float, float] = (0.30, 0.0, 0.06),
                 eye_pitch: float = -12.0, eye_distance: float = 1.5,
                 eye_camera: str = "eye"):
        import mujoco

        self.mj = mujoco
        self.model = model
        self.data = data
        self.max_range = float(max_range)
        self.period = 1.0 / max(1e-3, float(hz))
        self._next = 0.0

        try:
            self.port = int(os.environ.get("PERCEPT_PORT", DEFAULT_PORT))
        except ValueError:
            self.port = DEFAULT_PORT
        if port is not None:
            self.port = int(port)

        self.beams = int(beams)
        self.bearings = np.radians(np.arange(self.beams) * (360.0 / self.beams) - 180.0)

        self._exclude_self()
        # 两个通道各一个 socket。共用一个的话，发到「还没人监听」的端口会留下
        # ICMP port unreachable，下一次发到另一个通道也会抛同一个错误 —— 表现是
        # 两个通道一起哑掉，而日志里看不出是谁的问题。
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._vsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._wsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frames = 0
        self.errors = 0
        self._gid = np.zeros(1, dtype=np.int32)
        self._yaw = 0.0

        # 视频通道走另一个端口。和激光分开是因为下游是两种完全不同频率的消费者：
        # 射线 10 Hz 喂安全闸，画面 5 Hz 喂视觉模型（实际只取 0.5 Hz）。
        self.video_port = self.port + 1
        # 全局视角再占一个端口。它和机载是同一台渲染器、不同的相机 ——
        # 分开发是因为下游用途不同：机载喂视觉模型，全局只给人看。
        self.world_port = self.port + 2
        # 出图频率。默认 30 Hz：5 Hz 那一版是为了省 CPU，但遥操作时画面一顿一顿的
        # 根本没法开（每 200 ms 换一帧，狗走 0.8 m/s 时一帧之间挪 16 cm）。
        # 240x320 的渲染 + 11 KB 分片发送在 30 Hz 下是几个百分点的 CPU，值。
        # 要更高就用 SIM_VIDEO_HZ=60；传参优先于环境变量。
        if video_hz is None:
            try:
                video_hz = float(os.environ.get("SIM_VIDEO_HZ", 30.0))
            except ValueError:
                video_hz = 30.0
        self.video_period = 1.0 / max(0.5, float(video_hz))

        # 渲染走独立线程。
        #
        # 渲染一帧 240x320 在这台机器上约 30 ms，而且**和分辨率几乎无关**
        # （降到 120x160 也还是 148 步 / 6 s）—— 开销在 `update_scene` 遍历场景
        # 本身，不在像素。挂在物理循环里就等于每帧让物理停 30 ms：30 Hz 时物理
        # 从 157 Hz 掉到 22 Hz，策略 50 Hz 的时序全乱，狗一步都不走。
        # 实测（--seconds 6，步数）：关画面 941、30 Hz 130、20 Hz 222、10 Hz 630。
        #
        # 主线程只负责「拷一份状态」，渲染和发送都在下面的线程里做。两块数据
        # 轮换，渲染线程正在用的那块不会被主线程写的下一帧覆盖。
        self._shadow = None                   # [MjData, MjData]
        self._snap = 0                        # 主线程下一帧拷到哪块
        self._ready = -1                      # 渲染线程该用哪块（-1 = 没有）
        self._ready_lock = threading.Lock()
        self._rthread: threading.Thread | None = None
        self._rstop = False
        self._video_next = 0.0
        self.video_frames = 0
        self.video_errors = 0
        self.world_frames = 0
        self.world_errors = 0
        self._renderer = None
        self._camera = None
        self._world_cam = None
        # 渲染尺寸。默认 240x320；用 SIM_VIDEO_W/H 覆盖便于量开销。
        if video_size is None:
            video_size = (int(os.environ.get("SIM_VIDEO_H", 240)),
                          int(os.environ.get("SIM_VIDEO_W", 320)))
        self._video_size = video_size
        self._want_video = bool(video)
        self._want_world = bool(world)
        self._frame_id = 0
        self.eye_offset = tuple(float(v) for v in eye_offset)
        self.eye_pitch = float(eye_pitch)
        self.eye_distance = float(eye_distance)
        self.eye_camera = str(eye_camera)
        self.world_distance = float(world_distance)
        self.world_elevation = float(world_elevation)
        self.world_azimuth = float(world_azimuth)
        self._fixed = False

    # 离屏渲染要 GL 上下文。带窗口的仿真有，纯 headless + EGL 也可能没有 ——
    # 拿不到就退成「只有射线」，不要因为画面发不出去把整个仿真拖垮。
    def _ensure_renderer(self) -> bool:
        if self._renderer is not None:
            return True
        try:
            h, w = self._video_size
            self._renderer = self.mj.Renderer(self.model, h, w)
            cam = self.mj.MjvCamera()
            # 优先用模型里挂好的 `eye` 相机。它是焊在 base_link 上的固定相机，
            # 位姿由 MuJoCo 按关节状态算，比自己摆 free 相机准，也不会随着俯仰
            # 和 yaw 的符号搞反。
            try:
                cam.type = self.mj.mjtCamera.mjCAMERA_FIXED
                cam.fixedcamid = self.model.camera(self.eye_camera).id
                self._fixed = True
            except (KeyError, ValueError):
                cam.type = self.mj.mjtCamera.mjCAMERA_FREE
                cam.distance = self.eye_distance
                self._fixed = False
            self._camera = cam

            # 全局视角：一台跟着狗走的自由相机，斜后方高一点，能同时看到机身
            # 姿态和它周围的地形。机载画面看不出「狗是站着还是在打滚」——
            # 实测狗翻了之后，眼睛看到的是「蓝色的地面和白色的天空」，和
            # 正常站着没有区别。
            world = self.mj.MjvCamera()
            world.type = self.mj.mjtCamera.mjCAMERA_FREE
            world.distance = self.world_distance
            world.elevation = self.world_elevation
            world.azimuth = self.world_azimuth
            self._world_cam = world
            return True
        except Exception:                                   # noqa: BLE001
            self._want_video = False
            self._want_world = False
            return False

    # 在物理循环里每拍调，内部自己限频。返回值表示这一拍有没有真的发出去。
    def maybe_publish(self) -> bool:
        now = time.perf_counter()
        sent = False
        if now >= self._next:
            self._next = now + self.period
            try:
                frame = self._frame()
                payload = json.dumps(frame, separators=(",", ":")).encode()
            except Exception:                               # noqa: BLE001
                # 感知挂了不能让仿真停。计个数，上层发现收不到自然会降级。
                self.errors += 1
            else:
                sent = self._send(self._sock, payload, self.port)

        # 视频通道。这里只把当前状态拷一份交给渲染线程，不等它画完 ——
        # 见 `__init__` 里那段说明（直接在这里渲染会把物理拖垮）。
        if (self._want_video or self._want_world) and now >= self._video_next:
            self._video_next = now + self.video_period
            self._snapshot()
        return sent

    # 把 mjData 拷进轮换缓冲区，交给渲染线程。这一下是几十个 double 的拷贝，
    # 微秒级；渲染那 30 ms 在另一个线程里。
    def _snapshot(self) -> None:
        try:
            if self._shadow is None:
                self._shadow = [self.mj.MjData(self.model), self.mj.MjData(self.model)]
                self._start_renderer()
            i = self._snap
            self.mj.mj_copyData(self._shadow[i], self.model, self.data)
            with self._ready_lock:
                self._ready = i
            self._snap = 1 - i
        except Exception:                                   # noqa: BLE001
            self.video_errors += 1

    def _start_renderer(self) -> None:
        if self._rthread is not None:
            return
        self._rthread = threading.Thread(target=self._render_loop, name="percept-render",
                                         daemon=True)
        self._rthread.start()

    def _render_loop(self) -> None:
        while not self._rstop:
            with self._ready_lock:
                i = self._ready
                self._ready = -1
            if i < 0:
                # 没人要新帧就睡一下。1 ms 的粒度足够，渲染本身就不可能更快。
                time.sleep(0.001)
                continue
            if not self._ensure_renderer():
                continue
            d = self._shadow[i]
            if self._want_video:
                self._publish_video(d)
            if self._want_world:
                self._publish_world(d)

    # 渲染一帧，原样发原始 RGB，不做压缩。
    #
    # 这里刻意不 import cv2。仿真进程已经加载了 pygame，而 opencv 的 dylib 里也
    # 带了 SDL2，两个 SDL2 撞在一起 —— macOS 自己会警告 "may cause mysterious
    # crashes"，实测仿真确实会随机退出。压缩放到接收端做，那边没有 pygame。
    #
    # 代价是体积：320x240x3 = 230 KB 一帧，拆成 29 个 8000 字节的包。回环上
    # 5 Hz 也就 1 MB/s，无所谓；丢一片丢一帧，下一帧 200 ms 后到。
    def _publish_video(self, d=None) -> None:
        if not self._ensure_renderer():
            return
        d = self.data if d is None else d
        try:
            if not self._fixed:
                self._place_eye(d)
            self._renderer.update_scene(d, camera=self._camera)
            rgb = self._renderer.render()
        except Exception:                                   # noqa: BLE001
            self.video_errors += 1
            return
        if self._send_frame(rgb.tobytes(), self._vsock, self.video_port):
            self.video_frames += 1

    # 全局视角：机身位置为中心，斜上方往后看。
    #
    # 方位角跟着 yaw 转，否则狗一转圈就会背对着相机走远。固定方位角也不行 ——
    # 它会让画面里狗的方向随世界坐标变，看久了分不清「是狗转向了」还是
    # 「相机换了方向」。
    def _publish_world(self, d=None) -> None:
        if not self._ensure_renderer():
            return
        d = self.data if d is None else d
        quat = d.qpos[3:7]
        # yaw 的分母是 1−2(y²+z²)。写成 1−2(x²+y²) 不会报错，只是在小角度时
        # 接近、转 90° 只算出 45° —— 转向闭环会一直以为没转够，一路转下去。
        yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))
        cam = self._world_cam
        cam.lookat[:] = (d.qpos[0], d.qpos[1], d.qpos[2])
        cam.distance = self.world_distance
        cam.elevation = self.world_elevation
        cam.azimuth = math.degrees(yaw) + self.world_azimuth
        try:
            self._renderer.update_scene(d, camera=cam)
            rgb = self._renderer.render()
        except Exception:                                   # noqa: BLE001
            self.world_errors += 1
            return
        if self._send_frame(rgb.tobytes(), self._wsock, self.world_port):
            self.world_frames += 1

    # 把相机放到狗头上，眼睛朝前，略向下。
    #
    # 用 free 相机模拟固定相机。`mjv_updateCamera` 算的是
    # `pos = lookat - distance * (cos(el)sin(az), -cos(el)cos(az), sin(el))`，
    # 括号里是「相机指向 lookat」的方向。要让 pos 落在眼睛上，就把 lookat 放到
    # 视线正前方 distance 处，并让括号里那项等于视线方向 —— 解出来 az = yaw + 90°。
    # 写成 yaw - 90° 会让相机跑到狗前方三米处回头看，画面全反。
    #
    # 机身坐标 → 世界：Go2 机身长 0.7 m，眼睛在前端上方，取 +0.30 / +0.06。
    # 俯角 12°：平视只能看到墙，看不到脚下和台阶，而台阶恰好是这台机器最容易
    # 踩空的地方。
    def _place_eye(self, d=None) -> None:
        d = self.data if d is None else d
        quat = d.qpos[3:7]
        yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))
        c, s = math.cos(yaw), math.sin(yaw)
        ex = d.qpos[0] + c * self.eye_offset[0] - s * self.eye_offset[1]
        ey = d.qpos[1] + s * self.eye_offset[0] + c * self.eye_offset[1]
        ez = d.qpos[2] + self.eye_offset[2]

        el = math.radians(self.eye_pitch)
        dist = self.eye_distance
        ce, se = math.cos(el), math.sin(el)
        self._camera.lookat[:] = (ex + dist * ce * c, ey + dist * ce * s, ez + dist * se)
        self._camera.distance = dist
        self._camera.azimuth = math.degrees(yaw) + 90.0
        self._camera.elevation = self.eye_pitch

    # 一整帧拆成多个包发。
    #
    # 不能指望单包发完：macOS 的 `net.inet.udp.maxdgram` 默认 9216 字节，一张
    # 320x240 质量 85 的 JPEG 大约 11 KB，直接 sendto 报 "Message too long"。
    # 每包带 4 字节头（帧号 2 + 序号 1 + 总数 1），接收端按帧号拼。
    #
    # 丢一片就是丢一帧。这个通道的消费者只取 0.5 Hz，下一帧 200 ms 后就到，
    # 不值得为它做重传。
    def _send_frame(self, data: bytes, sock=None, port: int | None = None) -> bool:
        sock = self._vsock if sock is None else sock
        port = self.video_port if port is None else port
        chunk = 8000
        total = (len(data) + chunk - 1) // chunk
        if total > 255:
            return False
        fid = self._frame_id = (self._frame_id + 1) % 65536
        # 头里带上宽高：接收端不用和这边对着一份常量，改分辨率不用两边一起改。
        h, w = self._video_size
        for i in range(total):
            head = struct.pack("!HBBHH", fid, i, total, w, h)
            piece = head + data[i * chunk:(i + 1) * chunk]
            if not self._send(sock, piece, port):
                return False
        return True

    # 发一个包。没人监听不算错误。
    #
    # macOS 上 UDP 发到一个没绑定的端口，会收到 ICMP port unreachable，下一次
    # sendto 直接抛 ConnectionRefusedError。控制器比仿真晚起是常态，把这种情况
    # 记成「感知挂了」会让日志一直刷错误，真正的问题反而看不见。
    def _send(self, sock, payload: bytes, port: int) -> bool:
        try:
            sock.sendto(payload, ("127.0.0.1", port))
            return True
        except OSError:
            return False

    # 收集属于机器人的 geom id。
    #
    # `mj_ray` 的 bodyexclude 只能排一个 body，而腿分布在十几个 body 上，所以
    # 这里拿到 id 集合，射线打中集合里的 geom 就跳过、往前挪一点重打。
    def _exclude_self(self) -> None:
        m = self.model
        self._self_geoms: set[int] = set()
        try:
            root = m.body("base_link").id
        except KeyError:
            return
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            while b != 0:
                if b == root:
                    self._self_geoms.add(g)
                    break
                b = int(m.body_parentid[b])

    # 从 `pnt` 沿 `vec` 打一条射线，跳过机器人自己的 geom。返回 (距离, geom id)。
    #
    # 打中自己时把起点沿射线前移一点再打。次数给到 8：正前方的射线要依次穿过
    # 机身 box、头部那个 0.047 的球、前腿，4 次不够，实测正前方会变成「空」。
    def _ray(self, pnt: np.ndarray, vec: np.ndarray,
             max_range: float) -> tuple[float, int]:
        origin = np.array(pnt, dtype=float)
        travelled = 0.0
        for _ in range(8):
            dist = float(self.mj.mj_ray(self.model, self.data, origin, vec,
                                        None, 1, -1, self._gid))
            if dist < 0:
                return -1.0, -1
            gid = int(self._gid[0])
            if gid not in self._self_geoms:
                return travelled + dist, gid
            step = max(dist, 1e-3) + 0.02
            travelled += step
            if travelled > max_range:
                return -1.0, -1
            origin = origin + vec * step
        return -1.0, -1

    def close(self) -> None:
        self._rstop = True
        self._sock.close()
        self._vsock.close()

    # ---- 下面是把 MuJoCo 状态翻译成帧 -------------------------------------

    def _frame(self) -> dict:
        d, m = self.data, self.model
        quat = d.qpos[3:7]
        yaw = math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                         1.0 - 2.0 * (quat[2] ** 2 + quat[3] ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (quat[0] * quat[2]
                                                    - quat[3] * quat[1]))))
        # roll 的分母才是 1−2(x²+y²)，和上面 yaw 的不是同一个，不要一起改。
        roll = math.atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]),
                          1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
        self._yaw = yaw

        pnt = np.array(d.qpos[:3], dtype=float)
        # 世界系线速度转机体系。qvel 前三项是全局坐标，直接当 vx 会把横移算成前进。
        vx_w, vy_w = float(d.qvel[0]), float(d.qvel[1])
        c, s = math.cos(yaw), math.sin(yaw)
        vx = vx_w * c + vy_w * s
        vy = -vx_w * s + vy_w * c

        rays = []
        origin = pnt.copy()
        origin[2] += 0.05               # 抬一点，避开贴在机身底部的脚
        for b in self.bearings:
            wb = float(b) + yaw
            vec = np.array([math.cos(wb), math.sin(wb), 0.0])
            dist, gid = self._ray(origin, vec, self.max_range)
            if dist < 0:
                rays.append([round(math.degrees(float(b)), 1), self.max_range, 0.0])
                continue
            rays.append([round(math.degrees(float(b)), 1),
                         round(min(dist, self.max_range), 3),
                         round(self._top(gid), 3)])

        ground = self._ground(origin, yaw)
        return {
            "t": round(float(d.time), 3),
            "origin": "sim-lidar",
            "ego": [round(float(v), 4) for v in
                    (pnt[0], pnt[1], pnt[2], yaw, pitch, roll, vx, vy, float(d.qvel[5]))],
            "rays": rays,
            "ground": ground,
            "range_max": self.max_range,
        }

    # 命中几何的顶面高度（离地）。对 box / plane 是准的，对 mesh 是包围盒顶面 ——
    # 场景里的台阶、墙都是 box，够用。
    def _top(self, gid: int) -> float:
        if gid < 0:
            return 0.0
        pos = self.model.geom_pos[gid]
        size = self.model.geom_size[gid]
        return float(pos[2] + size[2])

    # 地面高度：从机身高度竖直向下打，`under` 是脚下地面的绝对高度（地板为 0），
    # `ahead` 是前方各距离处相对脚下地面差多少。负数是坑，正数是台阶。
    def _ground(self, origin: np.ndarray, yaw: float) -> dict:
        down = np.array([0.0, 0.0, -1.0])
        start = origin.copy()
        start[2] += 0.35                # 从障碍顶面之上开始，否则会打到台阶侧面

        under = self._drop(start, down)
        if under is None:               # 脚下是空的（掉出场景），不编数据
            return {"under": 0.0, "ahead": []}

        ahead = []
        for r in _GROUND_RANGES:
            seen = []
            for deg in _GROUND_BEARINGS:
                wb = math.radians(deg) + yaw
                p = start.copy()
                p[0] += r * math.cos(wb)
                p[1] += r * math.sin(wb)
                h = self._drop(p, down)
                if h is not None:
                    seen.append(h)
            # 取三方位里最低的一个：中间平、边上是坑，也该报坑。
            ahead.append(round(min(seen) - under, 3) if seen else 0.0)
        return {"under": round(under, 3), "ahead": ahead}

    # 从 `p` 垂直向下打，返回命中点的绝对高度。
    def _drop(self, p: np.ndarray, down: np.ndarray) -> float | None:
        dist, _ = self._ray(p, down, 3.0)
        if dist < 0:
            return None
        return float(p[2] - dist)
