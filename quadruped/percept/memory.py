"""M 层：走过的路、探到过的墙，以及在这些格子上做规划。

为什么单独一层：G 层每一拍只看当拍 8 个方位距离，看不见「十分钟前我从那边过来」。
迷宫墙角处「正后」永远是最宽的（原路没有障碍），于是每拍都想回头，狗在两条
走廊之间来回摆。这不是提示词能修的 —— 缺的是状态，不是措辞。

数据全是已有的：`Percept.ego` 给位姿（x, y, yaw），`Percept.rays` 给方位角和
距离。世界坐标 = 位姿 + 旋转，不需要新传感器：

    wx = x + d * cos(yaw + bearing)
    wy = y + d * sin(yaw + bearing)

`bearing` 是机体系（机头 0°，左边为正），`percept_pub` 发出来的是它，不是世界角。

三张栅格，一律 0.3 m（约半个身长，比一步的摆动短）：

    free    射线扫过的格子（可通行）
    visit   狗经过的格子 + 最后一次经过的时刻
    wall    射线打到的格子

对外三件事，分别服务两层：

    guide()      最近的前沿方向 → G 层据此给出目标点（**主要的**）
    been(name)   某个方位上最近走过多少格 → 老的回退路径用
    summary()    一句话 → L 层提示词，它需要知道自己在绕圈

## 为什么要栅格

只把地图压成「8 个方位各走过几格」，等于扔掉地图。G 每 10 秒看到的东西和
第一次进来时一模一样，迷宫对它来说永远是新的 —— 实测三个版本的累计转角
都在 1100°~1700°，净位移 3~5 m 打转，参数怎么调只是改打转半径。

`guide()` 扫一遍「可走但没走过」的格子，按 `距离 + ANGLE_COST×偏离机头角`
打分，返回得分最低那个方向上来 `reach` 米处的点。这样走过的地方不用重走、
方向由几何给出，不再靠模型从 8 个距离里猜。

这一层不做决策、不发命令、不改感知。它把「刚才发生过什么」和「从这儿能去哪」
记下来、算出来。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

from quadruped.percept.types import SECTOR_NAMES

# 格子边长。取半个身长：比一步的摆动（0.2 m）长，比走廊宽度（1.8 m）短很多，
# 既能分开两条相邻走廊，又不会把同一条走廊切成好几段。
CELL = 0.3

# 身体扫过这么宽的一条带子，算「已经探查过」。
#
# 只把脚下那一格记成走过是不够的。狗在 1.8 m 宽的走廊里贴着北墙走，脚下是
# 一条 0.3 m 宽的线，走廊南半边 0.9 m 外的格子从来没被踩过，于是规划器认为
# 「那边还没去过」，在每个墙角都想折回去看一眼 —— 实测在 row1 西端选中了
# 正南，那是它刚刚扫过、什么都没有的同一段走廊。
#
# 0.55 是机身半对角线（0.38）再留一点：这个范围内的地面一定是安全的，
# 也确实被看过了。
CLEAR_R = 0.55

# 想要的离墙距离。走廊半宽 0.9 m，取 0.75 就是「尽量走中间」。
SAFE = 0.75

# 候选点的离墙下限。低于它的格子直接不选，而不是扣分了事。
#
# 扣分救不回来，这是回归测出来的：走廊走一遍之后，中间被 `CLEAR_R` 标成
# 「走过」，剩下没走过的正好是贴两边墙的 0.3 m 窄条。它们离得近，扣掉的那点
# 分抵不上距离优势，规划器于是被它们领着贴墙走 —— 实测一路贴到离墙面 0.04 m，
# 真机上那台狗早就被 R 层按停了，因为贴墙时正前方永远是墙。
#
# 0.45 是栅格口径：格子中心离墙格中心 0.3（贴墙那一列）、0.6（隔壁一列），
# 取 0.45 就是把贴墙那一列排除，同时不误伤正常的走廊中间。
CLEAR_MIN = 0.45

# 离墙距离每差 1 m 扣多少分（米等价）。主要作用是排序时让「更中间」的格子
# 占优，硬约束由 `CLEAR_MIN` 负责。
WALL_COST = 4.0

# 方向平滑窗口（分数，米等价）。当前**没有用到** —— 2026-11-15 改成沿 BFS 路径
# 取点之后，方向天然是连续的（路径是格链，不会从两格之间跳），不再需要拿一组格子
# 做加权平均。留着是因为一旦退回「单点方向」的写法，这个值仍然是当时调出来的。
SMOOTH = 0.4

# 目标点后面还要留这么多米。选点时要求「朝那个方向能走 target+ROOM」，
# 于是贴着墙角、拐过去就没路的点会被排除，狗不会朝着墙角走过去停住。
ROOM = 0.35

# 多久之内算「刚走过」。迷宫一次掉头约 4~6 秒，25 秒覆盖三四次尝试；再长会把
# 「十分钟前走过」也算成回头路，那时候往回走其实是合理的。
HOLD = 25.0

# 「这条路我走过」记多久。
#
# `HOLD` 管的是几何：`free` / `wall` 会一直涨，25 秒前的墙可以忘掉，不心疼。
# 但「走过没走过」不该跟着一起忘 —— 用户 2026-11-17 报「走着走着回头去走自己
# 走过的路」，日志里正是走了 40 秒回到原处、系统当成新地方又走了一遍。
#
# `visit` 的时间戳本来就没删（只在超过 4000 格时按 `hold` 砍），读的时候只取
# 25 秒等于白存。改成两分钟：一次演示的量级，表也撑得住（0.3 m 格、走 30 m
# 大约 300 格）。
BEEN_SECONDS = 120.0

# 判定「这个方向是回头路」时看多远。走廊宽 1.8 m，2.5 m 足够覆盖到下一个路口，
# 又不至于把走廊尽头之外的地方算进来。
NEAR = 2.5

# 扇区半角，和 `types.SECTOR_NAMES` 的 8 等分一致。
HALF = math.pi / 8

# 前沿目标至少这么远（米）。太近的点会让狗原地挪半步，白费一次决策。
MIN_FRONTIER = 1.0

# 方向代价：偏离机头 1 弧度等价于多走 1.5 米。
#
# 这个数是算出来的，不是调的。走廊里「没走过但可走」的格子铺满整条走廊，
# 单纯比距离的话，正前 1.2 m 和左前 1.0 m 只差 0.2 m，狗会斜着走。乘上
# 1.5 之后正前领先 0.2 + 1.5×0.5 = 0.95 m，够压住一格（0.3 m）的抖动。
#
# 也不能太大：取 4.0 时正前被墙挡住（0.8 m）之后仍然不肯转，因为侧向
# 1.0 m 要付 1.5→6.3 m 的代价，比顶着墙好不了多少。
ANGLE_COST = 1.5

# BFS 最多展开这么多格。地图一张 3.6 x 3.6 m 的迷宫约 150 格，留 3000 是给
# 走到外面开阔地时兜底 —— 没有上界的话，`free` 一直涨，单次规划会越跑越慢。
MAX_CELLS = 3000


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Trace:
    """走过的格子。R 层写（10 Hz），G / L 读。

    写和读在两个线程上，`visit` / `free` 是普通 dict —— 一边迭代一边插入会抛
    `dictionary changed size during iteration`。BFS 一次跑几毫秒，撞上的概率
    不低，所以读接口先拿锁复制一份，再在锁外算。
    """

    name = "trace"

    def __init__(self, cell: float = CELL, hold: float = HOLD):
        self.cell = float(cell)
        self.hold = float(hold)
        self.visit: dict[tuple[int, int], float] = {}
        self.wall: dict[tuple[int, int], float] = {}
        self.free: dict[tuple[int, int], float] = {}
        self.travelled = 0.0          # 累计里程，m
        self.frames = 0
        self.turns = 0                # yaw 累计转过多少弧度（诊断「在原地打转」）
        # 最近一帧的射线。`guide()` 用它判断「朝这个点直线过去，中间有没有墙」——
        # 栅格给不出这个结论：一格宽的墙在 0.3 m 的格子上只有一格，采样时很容易
        # 从两个格子之间穿过去，于是规划器认为墙背面那个点「直线可达」。
        self.rays: tuple = ()
        self.range_max = 3.0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last: tuple[float, float] | None = None
        self._stamp = 0.0
        # 可重入：`summary()` 持锁期间还要调 `been()` / `here_count()`，
        # 用普通 Lock 会自己把自己锁死。
        self._lock = threading.RLock()

    # 当前位姿。G 层要算「目标点相对我在哪」，不用自己去摸私有字段。
    @property
    def pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._yaw)

    # ---- 写 --------------------------------------------------------------

    def update(self, percept) -> None:
        if percept is None or not percept.rays:
            return
        with self._lock:
            self._update(percept)

    def _update(self, percept) -> None:
        # 同一帧会被重复喂进来（R 层 20 Hz，感知只有 10 Hz），跳过。
        if percept.stamp and percept.stamp == self._stamp:
            return
        self._stamp = percept.stamp

        e = percept.ego
        now = time.monotonic()
        self._x, self._y, self._yaw = e.x, e.y, e.yaw
        self.rays = percept.rays
        self.range_max = percept.range_max
        self.frames += 1

        # 里程按「走够 0.1 m 记一次」累加，不是逐拍累加：仿真 10 Hz 时相邻两帧
        # 距离 0.03 m，逐拍累加会把噪声也算成里程。
        if self._last is None:
            self._last = (e.x, e.y)
        else:
            step = math.hypot(e.x - self._last[0], e.y - self._last[1])
            if step >= 0.1:
                self.travelled += step
                self._last = (e.x, e.y)
                self.turns += 0  # 占位，yaw 增量在下面算

        self.visit[self._key(e.x, e.y)] = now
        # 脚下永远是可通的：狗站在这里。不标的话，BFS 从这个格子出发都走不动。
        self.free[self._key(e.x, e.y)] = now

        # 身体扫过的这一片也算探查过（见 `CLEAR_R`）。只标 `free` 里的格子：
        # 还没被射线看到的地方不能凭空算成「看过」。
        span = int(math.ceil(CLEAR_R / self.cell))
        kx, ky = self._key(e.x, e.y)
        r2 = CLEAR_R * CLEAR_R
        for i in range(kx - span, kx + span + 1):
            for j in range(ky - span, ky + span + 1):
                k = (i, j)
                if k not in self.free:
                    continue
                dx = (i + 0.5) * self.cell - e.x
                dy = (j + 0.5) * self.cell - e.y
                if dx * dx + dy * dy <= r2:
                    self.visit[k] = now

        if self._prev_yaw is not None:
            self.turns += abs(_wrap(e.yaw - self._prev_yaw))
        self._prev_yaw = e.yaw

        # 射线扫过的格子标 free，命中的那一格标 wall。
        #
        # 采样步长取半格：整格步长会漏掉射线斜穿过的格子，走廊的拐角就会
        # 在图上留一个洞，BFS 从那里过不去，狗走过去却被判成「不可达」。
        step = self.cell * 0.5
        for ray in percept.rays:
            ang = e.yaw + ray.bearing
            cos_a, sin_a = math.cos(ang), math.sin(ang)
            d = ray.distance
            hit = d < percept.range_max - 1e-3
            # 末段留 0.4 格：命中点会单独标成 wall，别先标了 free 又抠掉。
            limit = (d if hit else percept.range_max) - self.cell * 0.4
            s = step
            while s < limit:
                self.free[self._key(e.x + s * cos_a, e.y + s * sin_a)] = now
                s += step
            if hit:
                k = self._key(e.x + d * cos_a, e.y + d * sin_a)
                self.wall[k] = now
                self.free.pop(k, None)

        # 格子越攒越多会拖慢遍历。清掉过期的，顺便限制总量。
        if len(self.visit) > 4000:
            # 砍的时候用记忆窗口，不是几何窗口 —— 按 `hold` 砍会把「走过」
            # 也一起清掉，等于把 BEEN_SECONDS 白加。
            cut = now - max(self.hold, BEEN_SECONDS)
            self.visit = {k: v for k, v in self.visit.items() if v >= cut}
            self.wall = {k: v for k, v in self.wall.items() if v >= cut}
            self.free = {k: v for k, v in self.free.items() if v >= cut}

    _prev_yaw: float | None = None

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))

    # ---- 读 --------------------------------------------------------------

    # ---- 规划 ------------------------------------------------------------

    # 朝某个世界方向最多能直着走多远。按「方位最近的射线」取，和
    # `types.Percept.sectors()` 同一个口径 —— 15° 一条，1 m 处横向误差 13 cm，
    # 够判「这点在墙后面没有」。
    def depth(self, bearing: float) -> float:
        if not self.rays:
            return self.range_max
        rel = _wrap(bearing - self._yaw)
        near = min(self.rays, key=lambda r: abs(_wrap(r.bearing - rel)))
        return near.distance

    # 下一个该去的点：可走、但还没走过的格子里，离得近又偏向正前方的那个。
    #
    # 这里换过两次判据，两次都错在同一个地方 —— 把「选哪个格子」交给了一个
    # 不稳定的排序：
    #
    #   第一次用「可走但没走过」。狗原地转一圈，身后就都成了「扫过没踩过」，
    #   BFS 认为那儿有新前沿，转过去；转过去之后前面又冒出来一个，再转回来。
    #
    #   第二次用「与未知相邻的边缘」。射线扫出来的是一个 3 m 圆盘，圆周处处
    #   是边缘，各点到脚下差不到一格，「最近」由 BFS 展开顺序决定 —— 目标点
    #   仍在正前和正后之间跳。实测两次都是 60 多秒换 7 次目标、挪不到 1 m。
    #
    # 现在直接扫一遍候选，按三项打分：
    #
    #     d                       走多远才到
    #     ANGLE_COST × 偏机头角    能不转就不转
    #     WALL_COST × 离墙缺口     贴着墙的格子不要
    #
    # 第三项是让狗走走廊中间的那一项，不是锦上添花。少了它，规划器会把目标点
    # 放在紧贴墙的一列格子上，狗一路蹭着墙走；真机上 R 层（`reflex.py`）在
    # 正前 0.6 m 以内就停车，而贴墙时正前方永远是墙 —— 表现为「明明路上没人，
    # 它站在原地不动」。2D 回归里这一项的差别是最近离墙 0.09 m 和 0.6 m。
    #
    # 不做 BFS 路径：可走区域是射线直接测出来的，1 m 内朝目标直线一定通；
    # 候选点也只取 `MIN_FRONTIER`..`span` 这一段，够不到墙背面。
    def guide(self, *, reach: float = 1.6, span: float = 2.5) -> tuple[float, float] | None:
        with self._lock:
            keys = list(self.free)
            visit = set(self.visit)
            free_set = set(self.free)
            x, y, yaw = self._x, self._y, self._yaw
            # 只留附近一圈墙。开阔地上 `wall` 会攒到几千格，每个候选都扫一遍
            # 是全表乘全表 —— 迷宫这种小图看不出来，平地走出去就卡成 0.1 Hz 的东西。
            near = span + SAFE + 0.5
            walls = [((k[0] + 0.5) * self.cell, (k[1] + 0.5) * self.cell)
                     for k in self.wall
                     if math.hypot((k[0] + 0.5) * self.cell - x,
                                   (k[1] + 0.5) * self.cell - y) <= near]

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        cell = self.cell

        # ---- 先在这张已知可走的格子上做一次 BFS ----
        #
        # 以前这里是直线距离：候选前沿在墙背后时，「直线 1.2 m、实际要绕 3 m」，
        # 打分却按 1.2 算。于是规划器认为拐角另一侧比眼前这条走廊近，狗对着
        # 墙角给点、走过去停住；下一拍再换一个墙角，日志里就是目标点在 1~1.6 m
        # 内乱跳、12 次大角度转弯、净位移只有 2.75 m。
        #
        # 走一次 BFS 把「沿实际能走的路多远」算出来，候选按这个距离排，目标点
        # 也沿路径取，不再用「朝某个格子的直线方向」。
        start = self._key(x, y)
        if start not in free_set:
            free_set.add(start)          # 脚下永远是通的（`_update` 也会标）
        dist = {start: 0}
        parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        queue = deque([start])
        limit = int(span / cell) + 1
        while queue and len(dist) < MAX_CELLS:
            cur = queue.popleft()
            dcur = dist[cur]
            if dcur >= limit:
                continue
            cx, cy = cur
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cx + dx, cy + dy)
                if nb in dist or nb not in free_set:
                    continue
                dist[nb] = dcur + 1
                parent[nb] = cur
                queue.append(nb)

        cands: list[tuple[float, float, float, float, float, float, tuple, bool]] = []
        for key in keys:
            if key in visit:
                continue
            if key not in dist:
                continue                 # BFS 到不了：墙那边，或者超出 span
            path = dist[key] * cell
            # 太近的点会让狗原地挪半步；太远的点可能在墙背面，直线过不去。
            if path < MIN_FRONTIER or path > span:
                continue
            wx = (key[0] + 0.5) * cell
            wy = (key[1] + 0.5) * cell
            dx, dy = wx - x, wy - y
            # 机体系角度：前向投影、左向投影。0 是正前，±π 是正后。
            ang = abs(math.atan2(-dx * sin_y + dy * cos_y,
                                 dx * cos_y + dy * sin_y))
            straight = math.hypot(dx, dy)
            if path <= straight + 0.45:
                # 路径基本是直的：这时射线校验有效 —— depth 短就是真被墙挡。
                # 栅格这一层给不出这个结论（0.12 m 的墙在 0.3 m 格子上只有一格，
                # 采样会从两格之间穿过去），所以还要问一次射线。
                depth = self.depth(math.atan2(dy, dx))
                if depth < min(reach, straight) - 0.05:
                    continue
                routed = False
            else:
                # 要绕过去才到（拐角另一侧、走廊尽头转个弯）：**不能再拿直线射线
                # 否掉它**。BFS 已经证明沿已知可走的格子能到。
                #
                # 这一条之前是「所有候选一律用直线 depth 筛」，后果很具体：
                # 拐角后的前沿 path 2.4 m、直线只有 0.8 m，被判成"墙挡"全删，
                # 剩下的只有直线可达的正前和右侧 —— 于是狗走到拐角就右转，
                # 而不是拐进它早就看见的那条左边通道。
                depth = straight
                routed = True
            clear = min((math.hypot(wx - a, wy - b) for a, b in walls),
                        default=SAFE)
            cands.append((wx, wy, path, ang, clear, depth, key, routed))

        if not cands:
            return None                       # 能到的都走过了
        # 目标点后面还要有 0.35 m 余地（见 `ROOM`）。这一条是给「拐角」用的：
        # 走廊尽头往斜后方有个墙角，那个方向 1 m 处确实可走，但再往前就是墙，
        # 狗走过去会正对着墙停住。实测在第二行西端选中了西南角，转向 117°
        # 之后正前只剩 0.94 m，卡在那里 20 秒没动，而往北穿过缺口还有 2.5 m。
        # 绕路的点（`c[7]`）不受这一条约束：它的"后面还有没有余地"要用拐过去之后
        # 的射线才知道，直线量不出来，而 BFS 已经证明能到，就不再拦。
        roomy = [c for c in cands
                 if c[7] or c[5] >= min(reach, c[2]) + ROOM]
        # 贴墙的格子不当候选（见 `CLEAR_MIN`）。整条路都窄到没有合格格子时，
        # 才逐级放松 —— 窄通道里宁可贴墙，也不要原地不动。
        pool = ([c for c in roomy if c[4] >= CLEAR_MIN] or roomy
                or [c for c in cands if c[4] >= CLEAR_MIN] or cands)
        scored = [(c[2] + ANGLE_COST * c[3] + WALL_COST * max(0.0, SAFE - c[4]), c)
                  for c in pool]
        _, best = min(scored, key=lambda s: s[0])

        # 沿 BFS 路径从脚下走到 `step_len` 米处，那个点就是目标。
        #
        # 不是「朝最优格子的直线方向走 step 米」：那条直线可能穿墙，而且会随
        # 格子跳动在 ±17° 之间摆（0.3 m 一格 / 1 m 外）。沿路径取点天然是连续的，
        # 进拐角之前它就指着拐角，不会让狗先对着墙走一段再转。
        chain = []
        cur = best[6]
        while cur is not None and cur != start:
            chain.append(cur)
            cur = parent.get(cur)
        chain.reverse()

        step_len = min(reach, best[2])
        px, py = x, y
        travelled = 0.0
        prev_x, prev_y = x, y
        for key in chain:
            gx = (key[0] + 0.5) * cell
            gy = (key[1] + 0.5) * cell
            seg = math.hypot(gx - prev_x, gy - prev_y)
            if seg > 1e-9 and travelled + seg >= step_len:
                t = (step_len - travelled) / seg
                px = prev_x + (gx - prev_x) * t
                py = prev_y + (gy - prev_y) * t
                break
            travelled += seg
            prev_x, prev_y = gx, gy
            px, py = gx, gy

        # 自己已经贴着墙的时候，把目标点里「朝那面墙」的分量扣掉。
        #
        # 上面算出来的点在几何上没问题 —— 射线是机身中心打出去的，那个方向确实
        # 通。但机身有宽度：目标点在右前方 1 m 时，往前走 0.3 m 就贴上右边的墙。
        # 二维里这是擦边，真机上这是 R 层停车，然后 L 层看到「正前 3 米」继续
        # 让它往前，两边都认为自己有理，狗原地不动。
        #
        # 实测卡死点：迷宫出口缺口内 (4.26, 4.48)，机头朝东北，右侧 0.18 m 是
        # 东墙，出口在正北 0.17 m 外。扣掉朝东的分量之后目标变成正北，直接出去。
        if walls and min(math.hypot(x - a, y - b) for a, b in walls) < CLEAR_MIN:
            a, b = min(walls, key=lambda w: math.hypot(x - w[0], y - w[1]))
            dist = math.hypot(x - a, y - b)
            if dist > 1e-6:
                ux, uy = (x - a) / dist, (y - b) / dist   # 从墙指向狗
                dx, dy = px - x, py - y
                toward = dx * ux + dy * uy                # 负值 = 正在往墙上走
                if toward < 0:
                    px -= toward * ux
                    py -= toward * uy
                    # 扣完了几乎没剩（目标是正对着墙）：那就直着背离墙走。
                    if math.hypot(px - x, py - y) < 0.2:
                        px, py = x + ux * step_len, y + uy * step_len
        return (px, py)

    # 还没走过的方向，最近的几个。给 L 层补的。
    #
    # 原来 L 层只有三样记忆（见 `summary`）：走了多远、哪边是回头路、这一带
    # 来过几次。这三样回答不了「左边那个口子我到底进去过没有」—— 而模型每轮
    # 只看得到当下 24 条射线，一个路口从视野里出去，它就再也想不起来了。
    # 实测现象：「起步时明明看见左边能拐，走到墙角却往右转」。
    #
    # 只报「深」的格子：紧贴走过路径的那一圈是 `CLEAR_R` 的误差带，不算没走过。
    # 不筛的话，1.8 m 宽的走廊里走出 1.8 m 就能攒出十几格「没走过」，全是噪声。
    def openings(self, *, span: float = 2.5, limit: int = 2,
                 deep: float = 0.8) -> list[tuple[str, float]]:
        with self._lock:
            free = dict(self.free)
            visit = dict(self.visit)
            x, y, yaw = self._x, self._y, self._yaw
            cell = self.cell
        if not free:
            return []

        rings = int(math.ceil(deep / cell))
        near_visit = set()
        for kx, ky in visit:
            for i in range(-rings, rings + 1):
                for j in range(-rings, rings + 1):
                    near_visit.add((kx + i, ky + j))

        # 按 BFS 路径距离报，不按直线：墙背后那一格「直线 0.9 m、绕过去 3 m」，
        # 报直线会让模型以为那儿有个近在眼前的岔路。BFS 到不了的不报 ——
        # 那正是「隔着墙看见的可走格」，说了只会让它往墙上撞。
        start = self._key(x, y)
        free_set = set(free)
        free_set.add(start)
        dist = {start: 0}
        queue = deque([start])
        lim = int(span / cell) + 1
        while queue and len(dist) < MAX_CELLS:
            cur = queue.popleft()
            dcur = dist[cur]
            if dcur >= lim:
                continue
            cx, cy = cur
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (cx + dx, cy + dy)
                if nb in dist or nb not in free_set:
                    continue
                dist[nb] = dcur + 1
                queue.append(nb)

        best: dict[str, float] = {}
        for key, steps in dist.items():
            if steps == 0 or key in visit or key in near_visit:
                continue
            path_len = steps * cell
            if path_len < MIN_FRONTIER or path_len > span:
                continue
            wx, wy = (key[0] + 0.5) * cell, (key[1] + 0.5) * cell
            dx, dy = wx - x, wy - y
            rel = _wrap(math.atan2(dy, dx) - yaw)
            name = min(SECTOR_NAMES, key=lambda nv: abs(_wrap(nv[1] - rel)))[0]
            if name not in best or path_len < best[name]:
                best[name] = path_len
        return sorted(best.items(), key=lambda kv: kv[1])[:limit]

    # 某个方位上，最近 `seconds` 秒走过几格。
    #
    # G 层用它抵扣「这个方向很宽」的吸引力。数格子而不是数时间：站在原地说
    # 「走过 3 秒」没有意义，格子才是「这条路上真的有我的脚印」。
    def been(self, name: str, *, seconds: float = BEEN_SECONDS,
             near: float = NEAR) -> int:
        center = dict(SECTOR_NAMES).get(name)
        if center is None:
            return 0
        with self._lock:
            visit = dict(self.visit)
            x, y, yaw = self._x, self._y, self._yaw

        cut = time.monotonic() - seconds
        found = 0
        for (cx, cy), stamp in visit.items():
            if stamp < cut:
                continue
            dx = (cx + 0.5) * self.cell - x
            dy = (cy + 0.5) * self.cell - y
            d = math.hypot(dx, dy)
            if d > near or d < self.cell * 0.9:
                continue                          # 太远，或者就是脚下这一格
            if abs(_wrap(math.atan2(dy, dx) - yaw - center)) <= HALF:
                found += 1
        return found

    # 某个坐标附近最近走过没有。给 G 层判「这个目标点是不是在我自己的脚印上」。
    #
    # `been()` 按方位扇区数，回答的是「我右边走过吗」；这个按坐标数，回答的是
    # 「我要去的那个地方，是我刚从那儿来的吗」。
    #
    # 直接原因是 2026-11-17 的日志：48 秒给的点在 (+0.4, +3.8)，狗往东走了
    # 3 米之后，63 秒那条目标点还在（`AIM_TIMEOUT` 只按时间算，而它在动，
    # 不在「追不到」的分支里），于是模型看到「向右转 14 度，2.1 米外」——
    # 掉头走回 15 秒前站过的路口。
    def been_at(self, point: tuple[float, float], *,
                seconds: float = BEEN_SECONDS, near: float = 0.6) -> int:
        with self._lock:
            visit = dict(self.visit)
            cell = self.cell
        cut = time.monotonic() - seconds
        px, py = float(point[0]), float(point[1])
        found = 0
        for (cx, cy), stamp in visit.items():
            if stamp < cut:
                continue
            if math.hypot((cx + 0.5) * cell - px,
                          (cy + 0.5) * cell - py) <= near:
                found += 1
        return found

    # 脚下这一带最近来过几次。连续值：站久了会一直加，所以要按窗口去重 ——
    # 这里只数「不同时间的到访」，用 3 秒作为最小间隔。
    def here_count(self, *, seconds: float = HOLD, gap: float = 3.0) -> int:
        with self._lock:
            visit = dict(self.visit)
            x, y = self._x, self._y
        cut = time.monotonic() - seconds
        mine = self._key(x, y)
        stamps = sorted(st for k, st in visit.items()
                        if k == mine and st >= cut)
        if not stamps:
            return 0
        count = 1
        last = stamps[0]
        for stamp in stamps[1:]:
            if stamp - last >= gap:
                count += 1
                last = stamp
        return count

    # 给 L 层的一句话。只写它有办法用上的：哪里是回头路、走过多远、是不是在绕圈。
    #
    # 不写坐标（模型算不出相对位置），不写格子数（那是给 G 层的分数）。
    def summary(self, *, seconds: float = BEEN_SECONDS) -> str:
        with self._lock:
            travelled = self.travelled
            turns = self.turns
            empty = not self.visit
        if empty:
            return ""
        parts = [f"已走 {travelled:.1f} m"]
        back = [n for n, _ in SECTOR_NAMES
                if n in ("正后", "左后", "右后") and self.been(n, seconds=seconds) >= 1]
        if back:
            parts.append("、".join(back) + " 是刚走过的路")
        visits = self.here_count(seconds=seconds)
        if visits >= 2:
            parts.append(f"这一带最近来过 {visits} 次，可能在原地绕")
        if turns > 2 * math.pi * 3:
            parts.append(f"累计转过 {math.degrees(turns):.0f}°")
        opts = self.openings()
        if opts:
            parts.append("还没走过：" + "、".join(f"{n} {d:.1f} m"
                                                 for n, d in opts))
        return "；".join(parts)

    def reset(self) -> None:
        with self._lock:
            self.visit.clear()
            self.wall.clear()
            self.free.clear()
            self.travelled = 0.0
            self.frames = 0
            self.turns = 0.0
            self.rays = ()
            self._last = None
            self._prev_yaw = None
            self._stamp = 0.0

    def status(self) -> dict:
        with self._lock:
            return {
                "cells": len(self.visit),
                "free": len(self.free),
                "walls": len(self.wall),
                "travelled": round(self.travelled, 2),
                "turns_deg": round(math.degrees(self.turns), 1),
            }
