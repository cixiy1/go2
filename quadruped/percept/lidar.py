"""真机的测距来源：Go2 自带的 L1 雷达。

仿真那边用 `percept/sim.py`，数据是 MuJoCo 的射线求交；真机没有这个发布者，
只有雷达。两个类产出同一种东西 —— `Percept` 里的 `rays` 和 `ground` ——
所以上面的 `Guard`（减速停车）、`Trace`（栅格记忆）、G 层选向、提示词里的
「视野」一行，全都不用知道数据是从哪儿来的。

    rt/utlidar/cloud         sensor_msgs/PointCloud2   点云，机体系
    rt/utlidar/height_map    unitree_go/HeightMap      以机器人中心为原点的离地高度格子

点云做两件事：

    24 个扇区（每 15°）取最小水平距离 → `Ray(bearing, distance, top)`
    每个点的 z 就是 `top`，用来区分「墙/高物」和「矮障碍」（阈值 0.35 m 同上）

高度图直接给 `Ground`：脚下高度 + 前方 0.4 / 0.8 / 1.2 m 三处的地面高度。
仿真里那 9 条向下射线就是干这个的，雷达这边是一张图，比分得更细。

## 坐标与外参

点云是**雷达系**，不是机体系。安装位置和角度（尤其俯仰）会影响方位角，所以
留了 `yaw_offset` / `mount_z` 两个参数，用 `test/lidar_probe.py` 打出来的点云
标定。默认按「雷达朝正前、无俯仰」处理 —— L1 装在机头，这个假设误差在几度内，
而安全闸用的是 8 个方位、每档 45°，容得下。

## 没有雷达的时候

`start()` 等第一帧点云，超时就返回 False，`ok` 保持 False。调用方会打一行
「雷达没数据」，然后照常用画面跑 —— 不会静默降级成「四周空旷」。
"""

from __future__ import annotations

import math
import threading
import time

from quadruped.percept.types import Ground, Percept, Ray, SelfState

# 24 个扇区的中心方位。和仿真射线同一套：0 正前，正为左。
RINGS = 24
BIN = 2 * math.pi / RINGS

# 只把「机身高度附近往上」的点当障碍：低于这个高度的点全是地面，会把每个扇区
# 的距离都压到脚底下那 0.3 m。矮于 Z_LOW 的东西交给高度图。
Z_LOW = -0.15
Z_HIGH = 1.20

# 自身点模型：机体系体素投票。
#
# 雷达装在头顶往下扫，全帧 4162 点里 2683 点（64%）是狗自己的外壳、背、肩
# （2026-11-17 实测）。用距离门限挡它们必然连带挡住墙 —— 试过 0.10/0.30/0.40/
# 0.60 四档，前两档全盲原地转，0.40 把矮台阶的顶沿删掉（正前报 3.00「空旷」
# 然后撞上去），0.60 把墙也关进盲区，狗贴着墙走。
#
# 换判据：**转向时学**。狗原地转的时候，自身点在机体系里一动不动，环境点在
# 扫过一圈 —— 跑十几帧就能把两者分开，不需要任何先验。
SELF_Q = 0.07          # 体素边长（m）
SELF_LEARN_R = 1.2     # 只对这么近的点建模（远处不会是自己的壳）
SELF_FRAMES = 20       # 至少攒这么多帧才下结论
SELF_RATE = 0.55       # 出现率超过这个比例 → 自身点
SELF_TURN = 0.50       # 累计转过这么多弧度（约 29°）才开始学

# 贴脸盲区。
#
# 0.30 的依据是几何不是猜测：机身半长 0.335 m（`go2.xml` 头部碰撞球 pos 0.285
# + r 0.047），0.3 m 以内任何点都不可能是「还能走进去的空隙」。
#
# 2026-11-17 真机试过 0.10 和 0.40 两档，都不对：
#   0.10 —— 雷达外壳（水平 0.002 m、离地 0.392 m）落在每个扇区里，24 档全读 0.10
#   0.40 —— 挡得住外壳，但矮台阶在 0.4 m 处只有顶沿可见（z 0.34~0.40），
#           正好被机身包围带删掉，正前读成 3.00「空旷」，狗一路顶上去撞
# 0.30 挡住了外壳，但真机上 0.3~0.6 还有一圈机身背/肩的点（`test/lidar_stats.py`
# 的直方图：0.3~0.5 m 有 528 点、0.5~0.8 m 有 521 点），于是 24 档又全读 0.30。
#
# 自身点模型建好之后，盲区回到 0.15：那时候 0.15 m 以外只剩环境点，墙、
# 台阶、桌腿都能看见。模型没建好（刚接管、还没转过向）时用 `R_MIN_WARMUP`
# 兜着 —— 大盲区会让狗贴着墙走，但比拿自己的身体当墙、原地转圈强。
R_MIN = 0.15
R_MIN_WARMUP = 0.60

# 机身自身点云的包围带：水平半径 + 离地高度区间。
#
# 雷达装在头顶，往下一扫就是自己的背和肩。2026-11-17 真机实测
# （`test/lidar_stats.py`，狗站在空旷的塑胶地面上，四周十几米没东西）：
#
#     水平 0.0~0.2 m   1332 点   z 0.38~0.45   ← 雷达外壳，最近的点 (0.002, 0, 0.392)
#     水平 0.2~0.3 m    823 点   z 0.20~0.46   ← 机身顶
#     水平 0.3~0.5 m    528 点   z 0.28~0.49   ← 背和肩
#     水平 0.8~1.5 m    396 点   z 0.10~0.70   ← 真实环境从这里开始
#
# 全帧 4162 点里 2683 点（64%）是自己。24 个扇区因此全读成 0.10 m（旧的 R_MIN
# 边界），VLA 的提示词写成「四周贴障碍 0.1 米」，狗 51 秒里转了 2800°、一米没走。
#
# 为什么不用「距离小于 X 就全扔掉」：真实障碍和自身点在同一距离段上重叠
# （0.3~0.5 m 两样都有），一刀切会把贴脸的墙一起切掉，安全闸就再也看不到它。
# 按包围带剔只删「机身高度上、机身范围内」那一块 —— 同样位置上的墙，低处
# （z < 0.20）和高处（z > 0.55）的点仍在，扇区照样能测到 0.4 m。
#
# 2026-11-17 第二轮实测推翻了「0.58」这个半径。狗正对一道**矮台阶**时，雷达
# 在 0.39 m 高处只能看到它的顶沿（z ≈ 0.34~0.40），正好落在这个带里被删掉 ——
# 于是 24 个扇区里正前读成 3.00「空旷」，狗一路顶上去撞。日志原样：
#
#     视野：正前 3.00  左前 3.00  左侧 3.00  左后 0.10  正后 3.00 …
#     [L] 安全闸：前方 0.10 m，减速
#
# 所以包围带只删**雷达外壳本身**（半径 0.14 m、雷达高度那一层）。机身背和肩
# 那一圈（0.2~0.5 m）留着：它们会造成几个扇区读到 0.2~0.3 m 的假障碍，
# 但假障碍只会让狗保守，删掉真障碍会让它撞 —— 两种错误里只能选前者。
# 机身自身的包围带。**半径必然要覆盖整个机身**，因为雷达装在机头 ——
# 机身从雷达到机尾有 0.6~0.7 m，那些点全在「前方」的方向上。
#
# 2026-11-17 实测（狗站在空旷地，`_seg_dist` 诊断）：剔除 0.14 m 内的点之后，
# 0.3~0.7 m 还剩 1268 个点，高度集中在 0.24~0.35 m —— 那是机身外壳。于是
# 24 个扇区里有 5~6 个稳定读成 0.57，安全闸每拍都判「前方 0.57 贴墙」，
# 狗一直后退（`vx=-0.25`）出不去。腿反而只剔掉 183 点：雷达在头顶，被机身
# 挡住，根本看不见自己的腿。
#
# 0.75 和停车线 `guard.stop_gap = 0.80` 是配对的：**盲区必须小于停车线**。
# 这样「看不见的那一段」恰好是「反正已经该停了」的那一段。矮台阶在 0.4 m
# 处确实会落进盲区，但狗在 0.80 m 就停了，走不到 0.4。
BODY_R = 0.75
BODY_Z = (0.02, 0.60)

# 腿的剔除半径。大腿小腿本身的粗细约 0.04 m，加上点云抖动和姿态估计误差，
# 0.09 能盖住整条腿又不会误删贴着腿的墙（墙在 0.09 m 外才看得见，而那条腿
# 本来也走不过去）。
#
# 为什么不用学习：第一版靠「转向时体素投票」学自身点，学完就定死。可狗一走
# 起来腿在摆，摆动的点在投票里出现率不够，学不进去 —— 于是每条腿都漏出来当
# 障碍物，真机上表现为「安全闸触发 50 次（前方 0.15 m）」，狗盯着自己的前腿
# 不动。腿的位置由关节角唯一确定，直接按运动学算，不需要学。
LEG_R = 0.09

# 话题名。不同固件/型号不完全一致，都订上，谁有数据谁生效。名字写错的表现是
# 「雷达没数据」，看起来像硬件坏了 —— 所以宁可多订两个。
CLOUD_TOPICS = ("rt/utlidar/cloud", "rt/utlidar/cloud_deskewed")
MAP_TOPICS = ("rt/utlidar/height_map",)


class LidarPercept:
    name = "lidar"

    def __init__(self, backend, *, range_max: float = 3.0,
                 yaw_offset: float = 0.0, mount_z: float = 0.0,
                 height_map: bool = True, wait: float = 3.0,
                 verbose: bool = True,
                 cloud_topics: tuple[str, ...] | None = None):
        # 位姿、倾角、速度都从本体遥测来，和 `ProprioPercept` 一模一样。雷达只补
        # 外部几何，不碰这两件事 —— 所以这里直接复用它，不复制一份差分逻辑。
        from quadruped.percept.proprio import ProprioPercept

        self.ego = ProprioPercept(backend)
        self.range_max = float(range_max)
        self.yaw_offset = float(yaw_offset)
        self.mount_z = float(mount_z)
        self.want_height_map = bool(height_map)
        self.verbose = verbose
        # 订哪几个点云话题。默认两个都订（不同固件发的名字不一样），但两个都
        # 在发的时候代价翻倍：`PointCloud2.data` 是一长串 uint8，cyclonedds 的
        # Python 绑定会把它展开成 Python int 列表 —— 11667 个点 × 48 字节就是
        # 56 万个对象，15 Hz 下光解析就吃掉两个核。只订一个能砍掉大半。
        self._topics = tuple(cloud_topics) if cloud_topics else CLOUD_TOPICS

        self.ok = False                 # 收到过点云没有
        self.clouds = 0
        self.maps = 0
        self.last_error = ""
        self.cloud_hz = 0.0
        self._sub_ok = False            # 订阅本身建起来了没有

        self._lock = threading.Lock()
        self._xyz = None                # (n, 3) float32，雷达系
        self._cloud_stamp = 0.0
        self._map = None                # HeightMap 的副本
        self._last_ego = None           # 最后一帧可用位姿（里程计断时的兜底）
        self._closed = False

        # 自身点模型（见 `SELF_Q` 那段的说明）。`_self_keys` 非空表示已建好。
        self._self_votes: dict[int, int] = {}
        self._self_frames = 0
        self._self_keys: set[int] | None = None
        self._acc_yaw = 0.0             # 接管以来累计转过的弧度
        self._last_yaw: float | None = None

        self._subscribe()
        self.ok = self._wait_first(wait)

    # ---- 订阅 ------------------------------------------------------------

    def _subscribe(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber

        # 两条订阅各自失败就把各自的原因都留着。只留最后一条会骗人：点云订阅
        # 失败、高度图也失败时，日志里只剩「高度图订阅不上」，看着像点云是好的。
        errors: list[str] = []

        try:
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

            def on_cloud(msg):
                try:
                    xyz = _to_xyz(msg)
                except Exception as err:                    # noqa: BLE001
                    self.last_error = f"{type(err).__name__}: {err}"
                    return
                with self._lock:
                    self._xyz = xyz
                    self._cloud_stamp = time.monotonic()
                    self.clouds += 1

            # 两个名字都订上。不同固件/不同型号发的话题名不完全一样，只认一个的话
            # 换台机器就「没收到点云」，而原因看起来像雷达坏了。
            # 谁有数据谁写 `_xyz`，两个都有就用后到的（内容一样）。
            for name in self._topics:
                try:
                    ChannelSubscriber(name, PointCloud2_).Init(on_cloud, 10)
                    self._sub_ok = True
                except Exception as err:                    # noqa: BLE001
                    errors.append(f"{name} 订阅不上：{type(err).__name__}")
        except Exception as err:                            # noqa: BLE001
            errors.append(f"点云 IDL 导入失败：{type(err).__name__}: {err}")

        if self.want_height_map:
            try:
                from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

                def on_map(msg):
                    with self._lock:
                        self._map = msg
                        self.maps += 1

                for name in MAP_TOPICS:
                    try:
                        ChannelSubscriber(name, HeightMap_).Init(on_map, 10)
                    except Exception as err:                # noqa: BLE001
                        errors.append(f"{name} 订阅不上：{type(err).__name__}")
            except Exception as err:                        # noqa: BLE001
                # 高度图没有也能跑，只是台阶和坑看不见。
                errors.append(f"高度图 IDL 导入失败：{type(err).__name__}")

        if errors:
            self.last_error = "；".join(errors)

    # 等第一帧点云。等不到就认了：这台机器没装雷达，或者雷达没起来。
    #
    # 订阅都没建起来（DDS 没初始化、IDL 不全）就不等了 —— 那种情况等满 3 秒
    # 只是让接管流程卡住，而结论一样是没有测距。
    def _wait_first(self, wait: float) -> bool:
        if not self._sub_ok:
            return False
        t0 = time.time()
        while time.time() - t0 < wait:
            with self._lock:
                if self.clouds > 0:
                    return True
            time.sleep(0.05)
        return False

    def close(self) -> None:
        self._closed = True
        self.ego.close()

    # ---- 读一帧 ----------------------------------------------------------

    def read(self) -> Percept | None:
        base = self.ego.read()
        # 里程计断了（`rt/sportmodestate` 停发）不等于雷达瞎了。
        #
        # 真机上这个 topic 是 mcf 发的，而我们 `claim()` 时会把它释放掉 —— 它到底
        # 还发不发没验证过。如果停发，`ProprioPercept.read()` 返回 None，而点云好
        # 好地在流。那种情况下把整份观测丢掉，等于「有雷达却当没有」：安全闸不
        # 触发、记忆不进格子。所以这里退一步：位姿用最后一帧，倾角用 IMU（
        # `rt/lowstate` 一直有），射线照常用。
        if base is None:
            ego = self._fallback_ego()
            if ego is None:
                return None
        else:
            ego = base.ego
            self._last_ego = ego

        with self._lock:
            xyz = self._xyz
            stamp = self._cloud_stamp
            grid = self._map

        # 没有可用的点云：有本体观测就交本体观测，没有就交一份「只有位姿、
        # 没有测距」的。返回 None 会让 `Reflex` 整拍跳过，L 层连机身数据都看不到。
        if xyz is None or time.monotonic() - stamp > 1.0:
            if base is not None:
                return base
            return Percept(stamp=time.monotonic(), origin="lidar", ego=ego,
                           range_max=0.0)

        rays = self._rays(xyz)
        ground = _ground(grid, ego) if grid is not None else Ground()
        return Percept(
            stamp=time.monotonic(),
            origin=self.name,
            ego=ego,
            rays=rays,
            ground=ground,
            range_max=self.range_max,
            scene=base.scene if base is not None else "",
            scene_age=base.scene_age if base is not None else 0.0,
        )

    # 里程计缺失时的位姿：位置和朝向沿用最后一次，倾角从 IMU 现取。
    def _fallback_ego(self):
        b = getattr(self.ego, "backend", None)
        rpy = getattr(b, "rpy", None)
        last = self._last_ego
        if rpy is None and last is None:
            return None
        pitch = float(rpy[1]) if rpy is not None else (last.pitch if last else 0.0)
        roll = float(rpy[0]) if rpy is not None else (last.roll if last else 0.0)
        return SelfState(
            x=last.x if last else 0.0,
            y=last.y if last else 0.0,
            z=last.z if last else 0.0,
            yaw=last.yaw if last else 0.0,
            pitch=pitch, roll=roll,
            vx=last.vx if last else 0.0,
            vy=last.vy if last else 0.0,
        )

    # 24 个扇区各取一条最近的射线。
    #
    # 不用「每扇区一条平均线」：扇区里有墙角和墙两种东西时，平均值会给出一个
    # 谁都不在的距离。
    # 按运动学剔掉四条腿。
    #
    # 点云的 z 是离地高度，腿的运动学解算出来是相对髋的 —— 中间差一个机身
    # 高度，用「支撑腿足端最深」反推：`h = -min(足端 z)`。这样上下台阶、
    # 蹲起都自动跟着变，不用写死一个 0.30。
    def _leg_mask(self, x, y, z):
        import numpy as np
        b = getattr(getattr(self, "ego", None), "backend", None)
        q = getattr(b, "q", None)
        if q is None:
            return np.zeros(len(x), dtype=bool)
        from quadruped.motion.odom import (GEOM, LEGS, foot_position,
                                          knee_position)

        q12 = list(q)
        if len(q12) < 12:
            return np.zeros(len(x), dtype=bool)
        feet_z = [foot_position(leg, q12[3 * i:3 * i + 3])[2]
                  for i, leg in enumerate(LEGS)]
        h = -min(feet_z)

        pts = np.stack([x, y, z], axis=1)
        mask = np.zeros(len(x), dtype=bool)
        for i, leg in enumerate(LEGS):
            seg = q12[3 * i:3 * i + 3]
            knee = knee_position(leg, seg)
            foot = foot_position(leg, seg)
            hx, hy, _ = GEOM[leg]
            hip = np.array([hx, hy, h])
            kn = np.array(knee) + [0.0, 0.0, h]
            ft = np.array(foot) + [0.0, 0.0, h]
            mask |= _seg_dist(pts, hip, kn) < LEG_R
            mask |= _seg_dist(pts, kn, ft) < LEG_R
        return mask

    # 累计转过多少角度。用 IMU 的 yaw 差分 —— 真机上 `lowcmd` 接管后位姿话题
    # 全停，但 `lowstate` 里的 IMU 一直是好的（`rt/lowstate` 500 Hz 不受影响）。
    def _tick_yaw(self) -> None:
        # `test/lidar_vs_ray.py` 里的对象是手工拼的（只设了 `mount_z`），没有
        # `ego`。测试不该为了配合实现细节去补字段，这里兜住即可。
        ego = getattr(self, "ego", None)
        b = getattr(ego, "backend", None) if ego is not None else None
        rpy = getattr(b, "rpy", None)
        if rpy is None:
            return
        yaw = float(rpy[2])
        if self._last_yaw is not None:
            d = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
            self._acc_yaw += abs(d)
        self._last_yaw = yaw

    # 机体系体素键。13 位一段，偏 4096 格 —— 0.07 m 一格够 ±280 m，
    # 这个量级远大于量程，不会溢出。
    @staticmethod
    def _keys(x, y, z):
        import numpy as np
        q = SELF_Q
        ix = np.floor(x / q).astype(np.int64) + 4096
        iy = np.floor(y / q).astype(np.int64) + 4096
        iz = np.floor(z / q).astype(np.int64) + 4096
        return (ix << 26) | (iy << 13) | iz

    # 挑出「属于机身自己」的点。
    #
    # 判据是**转向时的机体系稳定性**：狗原地转，自身点相对机身不动（体素键
    # 每帧都一样），环境点随扫描扫过去。攒 `SELF_FRAMES` 帧，出现率超过
    # `SELF_RATE` 的体素就是自己。
    #
    # 不能拿「命令有速度」当学习条件：走直线时侧墙在机体系里也不动，会被
    # 学成自身点，然后狗就看不见侧墙了。
    def _self_mask(self, x, y, z):
        import numpy as np
        keys = getattr(self, "_self_keys", None)
        if keys is None:
            return np.zeros(len(x), dtype=bool)
        k = self._keys(x, y, z)
        return np.fromiter((int(v) in keys for v in k.tolist()),
                           dtype=bool, count=len(k))

    def _learn_self(self, x, y, z, horiz) -> None:
        if getattr(self, "_self_keys", None) is not None:
            return
        if getattr(self, "_acc_yaw", 0.0) < SELF_TURN:
            return
        import numpy as np
        near = horiz < SELF_LEARN_R
        if not near.any():
            return
        k = self._keys(x[near], y[near], z[near])
        votes = self._self_votes
        for key in k.tolist():
            votes[int(key)] = votes.get(int(key), 0) + 1
        self._self_frames += 1
        if self._self_frames >= SELF_FRAMES:
            n = float(self._self_frames)
            self._self_keys = {key for key, c in votes.items() if c >= SELF_RATE * n}
            if self.verbose:
                print(f"[lidar] 自身点模型建好：{len(self._self_keys)} 个体素"
                      f"（{self._self_frames} 帧、累计转过 "
                      f"{math.degrees(self._acc_yaw):.0f}°），盲区收到 {R_MIN} m",
                      flush=True)

    def _rays(self, xyz) -> tuple[Ray, ...]:
        import numpy as np

        x = xyz[:, 0]
        y = xyz[:, 1]
        z = xyz[:, 2]

        # 两个阈值都按「离地高度」定义。真机点云的 z **本来离地高度** ——
        #
        # 2026-11-17 实测 `rt/utlidar/cloud`：z 的分位是 -0.10 / +0.22 / +0.38 /
        # +0.46 / +0.96，全是正的，没有负值；最近的点是 (0.002, 0.000, +0.392)，
        # 也就是雷达自己挂在 0.39 m 高处。原点在地面（机身正下方），不是雷达。
        #
        # 所以 `mount_z` 不参与计算。它留着只为一种情况：某些固件改成输出雷达
        # 系点云（那时 z 会以 -mount_z 表示地面），由调用方显式传值。
        # `mount_z` 只进 top，不进这里。
        #
        # 2026-11-17 试过写成 `z > Z_LOW + mount_z`，仿真侧 `test/lidar_vs_ray.py`
        # 立刻挂：第 6 档真值 0.84 m，雷达读成 3.00。原因是仿真点云的 z 是相对
        # 雷达的（地面在 -mount_z），偏移方向应该是 `z > Z_LOW`（原样，偏严一点
        # 正好把地面 -0.35 挡在外面）；再加一个 mount_z 等于把离地 0.2 m 以下的
        # 墙面全滤掉。
        horiz = np.hypot(x, y)
        # 剔掉机身自己。高度用离地值（`z` 在真机上已离地，仿真要加 `mount_z`）。
        zg = z + self.mount_z
        # 先攒转向角、再学、再剔。顺序不能反：刚建好的那一帧就应该生效。
        self._tick_yaw()
        self._learn_self(x, y, zg, horiz)
        body = (horiz < BODY_R) & (zg > BODY_Z[0]) & (zg < BODY_Z[1])
        body |= self._self_mask(x, y, zg)
        body |= self._leg_mask(x, y, zg)
        keep = ((z > Z_LOW) & (z < Z_HIGH - self.mount_z)
                & (horiz < self.range_max) & ~body)
        # 一个障碍点都没有（空旷场地 / 只看到地面）：给 24 条量程上限，而不是
        # 返回空元组。空元组会被上层读成「没有测距」—— 安全闸沉默、G 层停摆、
        # 提示词里写「没有视野」，而事实是「量程内什么都没有」。
        if not keep.any():
            return tuple(Ray(_center(k), self.range_max, 0.0) for k in range(RINGS))
        x, y, z = x[keep], y[keep], z[keep]

        horiz = np.hypot(x, y)
        ready = getattr(self, "_self_keys", None) is not None
        keep = horiz > (R_MIN if ready else R_MIN_WARMUP)
        if not keep.any():
            return tuple(Ray(_center(k), self.range_max, 0.0) for k in range(RINGS))
        x, y, z, horiz = x[keep], y[keep], z[keep], horiz[keep]

        bearing = np.arctan2(y, x) + self.yaw_offset
        # 归到 [-π, π)，再切成 RINGS 个等宽扇区。扇区中心落在 0°、15°、30°…，
        # 和仿真射线同一套（那边也是从 0° 起每 15° 一条），否则 `sectors()` 里
        # 「正前」那一档会落在两条射线之间，取到的方位差一半。
        bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
        idx = np.floor((bearing + math.pi + BIN / 2) / BIN).astype("int32") % RINGS

        out: list[Ray] = []
        for k in range(RINGS):
            sel = idx == k
            if not sel.any():
                # 这个扇区没有点：当成量程内没有东西。空走廊就是这样。
                out.append(Ray(_center(k), self.range_max, 0.0))
                continue
            sub = horiz[sel]
            j = int(np.argmin(sub))
            out.append(Ray(_center(k), float(sub[j]), float(z[sel][j]) + self.mount_z))
        return tuple(out)


def _seg_dist(pts, a, b):
    """每个点到线段 ab 的距离（向量化）。"""
    import numpy as np
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.linalg.norm(pts - a, axis=1)
    t = np.clip(((pts - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(pts - proj, axis=1)


def _center(k: int) -> float:
    # 第 k 个扇区的中心角：0°、15°、30°…，和上面的切分公式对齐。
    a = -math.pi + k * BIN
    return a if a < math.pi else a - 2 * math.pi


# PointCloud2 → (n, 3) float32。字段按 `fields` 里的 offset 取，不假设布局。
def _to_xyz(msg):
    import numpy as np

    step = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    if step <= 0 or n <= 0:
        raise ValueError(f"点云是空的（{msg.width}x{msg.height}）")
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if raw.size < step * n:
        raise ValueError(f"点云长度不对：{raw.size} < {step}*{n}")

    off = {str(f.name): int(f.offset) for f in msg.fields}
    cols = []
    for name in ("x", "y", "z"):
        o = off.get(name)
        if o is None or o + 4 > step:
            raise ValueError(f"点云没有 {name} 字段，只有 {sorted(off)}")
        sub = np.ascontiguousarray(raw[: step * n].reshape(n, step)[:, o:o + 4])
        cols.append(sub.view("<f4").reshape(-1))
    return np.stack(cols, axis=1)


# 高度图 → `Ground`。取机身脚下一点，加机头前方 0.4 / 0.8 / 1.2 m 三处。
#
# 高度图的原点在它的 `origin` 字段里（世界系），所以先把机体系的前方点转成世界系
# 再查表。查不到（越界）就给 0，不给 inf —— `Ground.step_up/down` 是拿来做
# 「有没有台阶」判断的，混进 inf 会让它一直报警。
def _ground(msg, ego: SelfState) -> Ground:
    res = float(msg.resolution)
    w, h = int(msg.width), int(msg.height)
    data = msg.data
    ox, oy = float(msg.origin[0]), float(msg.origin[1])
    if res <= 0 or w <= 0 or h <= 0 or len(data) < w * h:
        return Ground()

    def sample(dx: float, dy: float) -> float:
        wx = ego.x + dx * math.cos(ego.yaw) - dy * math.sin(ego.yaw)
        wy = ego.y + dx * math.sin(ego.yaw) + dy * math.cos(ego.yaw)
        i = int((wx - ox) / res)
        j = int((wy - oy) / res)
        if not (0 <= i < w and 0 <= j < h):
            return 0.0
        return float(data[j * w + i])

    return Ground(under=sample(0.0, 0.0),
                  ahead=(sample(0.4, 0.0), sample(0.8, 0.0), sample(1.2, 0.0)))
