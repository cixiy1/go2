"""G 层：没有人的时候，自己定一个目标。

L 层（`think.py`）会看环境、会说理由、会下发命令，但它等的是「有人说话」。自驱就是
把这句话的来源从人换成代码：G 层每 10 秒看一次 8 个方位的距离，选一个能走的方向，
写进 L 层的**常驻目标**。L 层每一拍推理都看得到这句话，直到 G 改掉它。

## 为什么 G 不直接发速度

G 只看 8 个距离和一个地面起伏，看不见台阶有多高、机身倾没倾、画面里是什么。让它
直接发速度，等于把 L 层那一步判断跳过去，撞了也说不清是谁定的。G 只给方向，
`vx` / `vy` / `vyaw` 怎么组合仍然是 L 层的事 —— 这一点对真机更重要，因为横移
和转向的响应差十几倍（实测 trot 原地转向 0.033 rad/s，横移 0.3 m/s）。

## 为什么是 0.1 Hz

    10 s 走 3 m，比一次 L 层决策（1 s / 0.3 m）粗一个数量级
    换目标会让 L 层立刻重想一次，换太勤就是不停改主意

第二条是主要的。方向在 1 Hz 上跳变时，实测狗会在两个方位之间来回摆 ——
换向本身要 1~2 个摆动相，还没转过去目标又变了。加滞回（新方向必须宽出 1.0 m
才换）之后才稳定。

## 受阻怎么办

G 不等 L 层撞上再改。前方距离掉到 0.8 m 以下、且当前方向也不是最空的那个时，
立刻换向 —— 这一次不受 10 s 限制，因为它是**事件**而不是**巡检**。

受阻判定只看几何量。视觉描述（「前面是一堵墙」）不参与选向：它会飘，而且说不清
是几米。
"""

from __future__ import annotations

import math
import threading
import time

from quadruped.percept.types import SECTOR_NAMES
from quadruped.limits import GAP_TRIGGER
from quadruped.rates import GOAL_HZ

# 方向偏好。正前最优先，两侧次之，往后退最后 —— 后退是这份策略的短板
# （命令 -0.4 实测只有 -0.05 m/s），G 不该主动往那个方向指。
# 正前给正分：射线量程上限是 3.0 m，空旷时八个方向都是 3.0，不给正分就会一直
# 随机挑到左前或右前，狗走成斜的。
#
# 后面三个方向原始折扣给得比侧向狠：墙角处「正后」永远是 3 米空旷，只按距离
# 选就会一直指回原路，而狗又转不过身（原地 0.35 rad/s）—— 实测在第一个墙角
# 左右摆了一分钟没出来。
#
# 有了 `percept/memory.py` 之后这里的数值可以放回来一些：**「后向不好」应该由
# 「刚走过」这个事实决定，不该用一个常数假装**。所以后向从 -2.5 收到 -0.9，
# 真正的排斥交给 `VISIT_PENALTY` 按实际脚印扣 —— 万一某天真需要退出去（走进
# 死胡同），没走过的后向仍然可选。
BIAS = {
    "正前": 0.25, "左前": -0.15, "右前": -0.15,
    "左侧": -0.35, "右侧": -0.35,
    "左后": -0.60, "右后": -0.60, "正后": -0.90,
}

# 这个方向上最近走过 N 格，扣掉的分数（米等价，和 BIAS 同量纲）。
#
# 2.0 是量出来的下限：迷宫走廊宽 1.8 m，墙角处「正后」空旷 3.0 m、回头一格
# 走过的脚印就该压过它（3.0 - 0.9 - 2.0 = 0.1 < 左侧 2.0 - 0.35）。
VISIT_PENALTY = 2.0
VISIT_FULL = 2.0         # 扣满需要几格

# 距离项封顶：一个方向够走这么远之后，再空旷也不额外加分。
#
# 不封的时候，`base = gap + BIAS` 实际上就是「那个方位有多空旷」——量程 3.0 m
# 里越远分越高。于是在走廊口上，「身后 3.0 m 空旷」比「正前 1.5 m 能走」高
# 1.5 分，比方向偏好（正前 +0.25 对正后 -0.90，共 1.15）还大，脚印又被
# `HOLD` 25 秒冲掉一半 —— 结果就是它一次次把狗指向最空的那一边，看着像
# 「总想去空旷的地方」。
#
# 取 1.2 而不是 2.0：2.0 只是削掉最上面那截，3.0 m 的空旷照样压过 1.2 m 的
# 走廊（实测「正前 0.4 m 堵住、侧向 1.2 m 通」会选「左后 3.0 m 空旷」，转 135°
# 往回走）。1.2 的意思是「够走到下一次决策就够」——G 一次决策要走 1.6 m，
# 走不了那么远时再空旷也没有用。封平之后，选择权回到方向偏好（转得少）和
# 「刚走过」的脚印上，1.2 m 的走廊和 3 m 的旷野同分，正前/侧前胜出。
#
# 注意这只管**没有地图**时的退路（`_by_distance`）。有地图时走向由
# `memory.guide()` 的三条硬筛决定，那里不存在「越空旷越优先」这一项。
GAP_CAP = 1.2

SWITCH_MARGIN = 1.0      # 新方向要比当前方向宽出这么多才值得换
BLOCK_GAP = GAP_TRIGGER  # 前方低于这个距离算受阻，立刻重选
HOPELESS_GAP = 0.7       # 八个方向都低于这个值：站住
WATCH = 0.2              # 受阻监视的轮询间隔，5 Hz

# 旧目标点到这个半径就换新的。
#
# 它同时是一个**方位角有意义的距离**：目标点到脚下 0.4 m 时，横向差 0.1 m 就是
# 14°，方位角是噪声。实测正是这么来回转的 —— 十次转向里有五次模型看到的是
# 「目标在右前 42 度，0.4 米」，它照着转 42°，转过之后 G 换点，又是 42°，
# 四次下来转了 -200°：掉头被拆成四段，看着就是「转过头来回回」。
#
# 所以这个半径不能太小，得让模型手里的点至少还有零点几米的杠杆。同时它必须
# 明显小于目标间距（`guide(reach=1.6)`），否则目标刚下发就「到了」：狗往前
# 0.2 m，旧目标从 0.7 变 0.5，被判到达，G 立刻换点，方位翻几十度。
REACHED = 0.6

# 目标点离得太远就作废。
#
# `_aim_stale` 原来只判「到了」（`<= REACHED`），没判「太远」。狗一路走出去、
# 目标又不再更新时，那个点就留在原地看自己越拉越远 —— 日志里出现过
# 「向左转 125 度，26.7 米外，这条路探过是通的」。26 m 外这套系统一点数据都
# 没有：`trace.depth` 在量程处封顶，判据 `depth < min(dist,1.5)-0.2` 永远成立，
# 于是它对着一个看不见的方向断言「通」，模型照着转 125°。
#
# 上限取和测距量程同一个数（3 m）。超过就没有方位可言，留着只会误导。
AIM_MAX_DIST = 3.0

# 一个目标最多追这么久。超时才换，是为了处理「目标点在拐角另一侧、直线
# 到不了」的情况 —— 没有它，狗会一直顶着一个够不着的点。
#
# 20 s 太长。狗现在 0.5 m/s，20 秒能跑 10 m，而迷宫整张才 5.4 x 5.4 m ——
# 这段时间足够走错两个路口，手里却一直是那个 20 秒前的点。改成 8 s，配合下面
# 的「越追越远就换」一起用。
AIM_TIMEOUT = 8.0

# 追这个点的过程中，离它比历史最近距离还远出这么多，就换点。
#
# 只看 `AIM_TIMEOUT` 不够：狗从点旁边擦过去，距离会先降后升，8 秒内正好
# 卡在「还没超时、但已经在往回走」。实测那种情况模型拿到的是「向左转 129 度」，
# 于是在墙角掉头，转到一半 G 换点，再转回来 —— 用户看到的「先往墙上转一趟」。
AIM_DRIFT = 0.5

# 追一个点追到超时，但狗自己挪了不到这么远 —— 不换点（见 `_aim`）。
#
# 换点是为了处理「这个点去不了」，前提是狗真的在往那儿走。狗原地打转/顶墙时，
# 换出来的新点还是它周围那一圈前沿，方位却变了 60~120°，L 层照着转 —— 换点
# 从「纠错」变成「制造转向」。0.3 m 是「明显还没开始走」的量级（一步 4~6 cm，
# 走起来的话 8 秒早过 1 m 了）。
AIM_STUCK = 0.3

# 新目标点离狗这么近、方位又比旧点翻过 `AIM_FLIP_DEG`，就不换（见 `_aim`）。
#
# 两个都在一步之内时，先到哪个差别很小；而每换一次 L 层就要按新方位转一次。
# 实测那一串 180° 翻转把 90 秒变成了 880° 的旋转和 2.4 m 的净位移。
AIM_FLIP_NEAR = 2.0
AIM_FLIP_DEG = 120.0

# 目标点跑到身后超过这个角度就作废。
#
# `AIM_MAX_DIST` 只拦得住「越拉越远」，拦不住「被超过」：狗往前走 2 m，目标从
# 正前 1 m 变成身后 1 m，距离只有 1.0 —— 既不满足「到了」（> `REACHED`）也不
# 满足「太远」（< `AIM_MAX_DIST`），`been_at` 又要求那个点被精确踩过。于是这个
# 点一直留着，L 层每轮收到「向我身后 168 度」。
#
# 2026-11-17 迷宫实测：开阔地里八方向全 3.00 m（四周什么都没有），日志照样写
# 「前面这一片都走过了，往回走」—— 于是狗在空地中间掉头。用户看到的就是
# 「走着走着往回走」。
#
# 作废之后走的是 `_loop` 的 stale 分支：前方 >= `AIM_EXTEND` 就把点沿机头往前
# 推，狗继续直行；只有前方也到头了才交回 `guide()` 重选。所以「前面真走完了」
# 的掉头不受影响。
#
# 120 和 `AIM_FLIP_DEG` 同值但两回事：那个判「新点相对旧点翻转」，这个判
# 「旧点相对机头在身后」。
#
# 取 150 而不是 120：120~150 是侧后方，狗转个 60° 就正对着它，那还是个有效
# 方向。日志里真正让狗掉头的是 148~172 这一段。`test/loop_memory.py` 那条
# 「没去过的方向、距离也合适 → 保留」用的就是 124° 的点，120 会误伤。
AIM_BACK_DEG = 150.0

# 走到旧点（`REACHED` 以内）之后，至少隔这么久才换新点。
#
# 原来是一到就换。换点必换方位，而 L 层下一拍就按新方位转 —— 狗刚转开，新点
# 又在背后，再转回去。2026-11-16 迷宫仿真：120 秒 19 次转向、累计 1482°、
# 净位移 1.9 m（走了 28 m）。
#
# 1.5 s 的依据是 L 层的节拍：一轮决策 1~2 s（`THINK_TICK_HZ` + 生成耗时）。
# 短于这个数，狗还没走完一拍就被换了新方向，等于每拍转一次。
AIM_HOLD = 1.5

# 走到点之后沿机头往前推这么远，而不是重新选方向（见 `_loop` 里的 stale 分支）。
#
# 和 `memory.guide(reach=1.6)` 同一个数：狗一次决策期望走这么远。前方射线够这个
# 距离就继续直行；不够说明到头了，才交回 `guide()` 重新选。
AIM_EXTEND = 1.6

# `guide()` 找「没去过的前沿」的搜索半径。
#
# 原来是 2.5（0.1 Hz 复查时数太小看不出来，走出迷宫就暴露了）：开阔地里 2.5 m
# 半径那片格子在十几秒内全被踩过，`cands` 为空 → `guide()` 返回 None →
# 退化成 `_by_distance`（只看 8 方位距离），于是不再有「往没走过的地方去」
# 这件事，狗开始在两片区域之间来回。
#
# 2026-11-17 的日志：走 15 m 后「记忆：……」那一行只剩「左后、正后、右后 是
# 刚走过的路」，再没有「还没走过：X」—— 说明它自己都不知道周围还有哪儿没去。
#
# 4.0 m：大约 5 秒路程，比一次决策远、比记忆窗口短。候选格数约 560、墙格
# 几百个，`guide` 每 10 秒算一次，O(候选 × 墙) 在 0.1 s 量级。
SPAN = 4.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi

# 方位名 → 相对机头的角度（度，左正）。射线方向是几何量，这里如实报出来，
# 让 L 层知道「转到那个方位要转多少」，而不是自己看画面估一个。
#
# 之前 G 只说「向左侧走」，L 层要靠 vyaw 猜角度：说 0.8 也行、说 1.0 也行，
# 转多转少全看它心情，实测出现过 -135° 这种谁都没要求的数。这不是「G 决定
# 角度」—— G 报的是「左侧」这个方位在几何上就是机头 +90°，L 层可以不听，
# 但至少有个准确的数可用。
_HEADING = {name: math.degrees(center) for name, center in SECTOR_NAMES}


def _turn_text(name: str) -> str:
    deg = _HEADING.get(name, 0.0)
    if abs(deg) < 1e-6:
        return ""
    if abs(abs(deg) - 180.0) < 1e-6:
        return "，需掉头（180 度）"
    side = "左" if deg > 0 else "右"
    return f"，需从当前机头{side}转 {abs(deg):.0f} 度"


class Driver:
    """自驱目标层。`thinker` 只要有 `last_percept` / `set_goal()` 两个口。"""

    def __init__(self, thinker, *, period: float = 1.0 / GOAL_HZ,
                 mode: str = "explore", goal: str | None = None,
                 hint: str = "", verbose: bool = True):
        self.thinker = thinker
        self.period = float(period)
        self.mode = mode
        # 固定挂在目标后面的那句话。G 只说方向（「向左侧走」），但某些场景还要
        # 一句「怎么走」：迷宫里正前被挡时得先换 pronk 才转得动，这句话每个目标
        # 都成立，所以不放进 G 的方向逻辑里，单独挂。
        self.hint = hint
        self.verbose = verbose

        self.goal = goal or ""
        self.direction = "正前"
        self.target: tuple[float, float] | None = None   # 地图给的目标点
        self._aim_at = 0.0                               # 这个目标是什么时候定的
        self._aim_best = 0.0                             # 追它以来最近到过多少米
        # 定这个点时狗在哪儿。用来区分「点去不了」和「狗没走」（见 `AIM_STUCK`）。
        self._aim_pose = (0.0, 0.0)
        self.changes = 0
        self.blocked = 0
        self.last_gap = float("inf")
        self.last_reason = ""
        self._fresh = True

        self._run = threading.Event()
        self._thread: threading.Thread | None = None

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[G] {msg}", flush=True)

    # ---- 生命周期 --------------------------------------------------------

    def start(self) -> None:
        if self._run.is_set():
            return
        if self.mode != "explore" and not self.goal:
            self.goal = "自由行动，遇到障碍就停下等新方向"
        if self.goal:
            self.thinker.set_goal(self.goal)
        self._run.set()
        self._thread = threading.Thread(target=self._loop, name="drive", daemon=True)
        self._thread.start()
        self._log(f"目标层已启动，{1.0 / self.period:.2f} Hz，模式 {self.mode}")

    def stop(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def close(self) -> None:
        self.stop()

    @property
    def running(self) -> bool:
        return self._run.is_set()

    def status(self) -> dict:
        return {
            "running": self.running,
            "mode": self.mode,
            "goal": self.goal,
            "direction": self.direction,
            "target": None if self.target is None else
                      (round(self.target[0], 2), round(self.target[1], 2)),
            "changes": self.changes,
            "blocked": self.blocked,
        }

    # ---- 一拍 ------------------------------------------------------------

    # 跑一次决策，返回新目标（没变就是 None）。单独拿出来是为了能不启线程直接测。
    #
    # 两条路：
    #   有地图 → `_aim`：BFS 找最近的前沿，给一个世界坐标的点
    #   没地图 → `_by_distance`：按 8 个方位距离选，没有记忆时的老逻辑
    def tick(self) -> str | None:
        p = getattr(self.thinker, "last_percept", None)
        if p is None or not p.rays:
            return None
        if self.mode != "explore":
            return None

        # 记忆挂在感知源上（`Reflex.trace`），G 层只读不写。
        trace = getattr(getattr(self.thinker, "percept", None), "trace", None)

        if trace is not None:
            point = trace.guide(reach=1.6, span=SPAN)
            if point is not None:
                return self._aim(point, p, trace)
        return self._by_distance(p, trace)

    # 按地图给目标点。这是主要的选向方式 —— BFS 已经算过「哪条路通」，这里只把
    # 那个格子翻成一句话。方位名只用于日志和状态栏。
    #
    # 一旦选好就锁住，直到狗走到（`REACHED`）或者追了 `AIM_TIMEOUT` 秒。
    # 不锁的话，BFS 每 0.1 秒都可能给出另一个方向 —— 射线扫出来的是一个圆盘，
    # 圆周处处是前沿，最近的边缘点在前后左右之间跳，实测目标点在 (1.3, 0.1)
    # 和 (-0.4, -0.4) 之间来回，狗转了 7 次、67 秒只挪了 0.6 m。
    def _aim(self, point, p, trace) -> str | None:
        x, y, _ = trace.pose
        age = float("inf")
        if self.target is not None:
            old_d = math.hypot(self.target[0] - x, self.target[1] - y)
            age = time.monotonic() - self._aim_at
            # 除了「还没走到」和「还没超时」，还要看是不是在靠近它。
            # 只看前两条的话，狗从点旁边擦过去之后会继续被派去追那个点，
            # 距离一天天变大也不换 —— 模型照着旧方位转，转完 G 换点，再转回来。
            if old_d > REACHED and age < AIM_TIMEOUT:
                if old_d <= self._aim_best + AIM_DRIFT:
                    self._aim_best = min(self._aim_best, old_d)
                    return None
            # 超时了，但狗自己一步没挪 —— 换点救不了。
            #
            # 2026-11-15 那段日志（12.2 s → 28.4 s）：狗在原地转了 330°，位置
            # (+3.5,+0.3) → (+3.3,+0.2)。8 秒超时一到就换点，而新点还是它周围
            # 1.6 m 内的前沿，于是目标点绕着狗转：+39° → +81° → -174° → +13°。
            # 每换一次 L 层就按新方位转一次，换点本身成了转圈的发动机。
            #
            # 判据用「从设点那一刻起挪了多远」，不是「离目标点多远」：狗没动
            # 的时候，后者只反映点在哪儿。
            if age >= AIM_TIMEOUT:
                moved = math.hypot(x - self._aim_pose[0], y - self._aim_pose[1])
                if moved < AIM_STUCK:
                    self._aim_at = time.monotonic()     # 再给一个周期
                    self._log(f"目标点 {old_d:.1f} m 追了 {age:.0f} s，"
                              f"但狗只挪了 {moved:.2f} m —— 不是点的错，先不动它")
                    return None
        dx, dy = point[0] - x, point[1] - y
        dist = math.hypot(dx, dy)
        rel = _wrap(math.atan2(dy, dx) - p.ego.yaw)
        # 新点就在脚边、方位却和旧点翻了个个儿 → 不换。
        #
        # 2026-11-15 迷宫西端，目标点在 (-0.1,+2.5) 和 (0.1,+1.6) 之间来回跳，
        # 两个都是「一步就到」，方位差 ~180°。每换一次 L 层就转一次：那 90 秒
        # 里累计转了 880°，净位移 2.41 m。狗在两米内挪来挪去，两个点谁先到都
        # 一样，换了只有坏处。
        #
        # 只拦「还没超时」的翻转。超时了必须换 —— 否则两条保护互相卡死：
        # 2026-11-15 第三段日志里，旧点 (+4.4,+0.8) 追不到（要绕缺口），每次
        # 巡检算出的新点都翻 160~179°，被这条按回去，同一个点连续保持了 14 次
        # ≈ 140 秒。狗在 (1.4~2.4, -0.5~+0.5) 那一小片来回转，用户截图抓到的
        # 就是这一刻。
        if (self.target is not None and dist <= AIM_FLIP_NEAR
                and age < AIM_TIMEOUT):
            old = self.target
            old_rel = math.atan2(old[1] - y, old[0] - x)
            flip = abs(math.degrees(_wrap(math.atan2(dy, dx) - old_rel)))
            if flip > AIM_FLIP_DEG:
                self._log(f"新目标点方位翻了 {flip:.0f}° 且在 {dist:.1f} m 内，"
                          f"保持旧的 ({old[0]:+.1f}, {old[1]:+.1f})")
                return None
        self.target = point
        self._aim_at = time.monotonic()
        self._aim_best = dist
        self._aim_pose = (x, y)
        self.direction = min(SECTOR_NAMES, key=lambda nv: abs(_wrap(nv[1] - rel)))[0]
        # 已经下发过方向了。不清这个标志的话，之后第一次落进 `_by_distance`
        # 会被「第一次必须先给一个目标」的分支强制换向 —— 那一次换向还会把刚
        # 拿到不久的地图目标点作废（见 `_by_distance` 上面的注释）。
        self._fresh = False
        self.changes += 1
        self.last_reason = "地图"
        self.goal = self._compose(f"朝那边走，{dist:.1f} 米外，那边还没去过")
        self.thinker.set_goal(self.goal, target=point)
        self._log(f"目标点 ({point[0]:+.1f}, {point[1]:+.1f})，{dist:.1f} m，"
                  f"{self.direction}（{math.degrees(rel):+.0f}°）")
        return self.goal

    # 没有地图时按 8 个方位距离选。只在 `trace` 缺失、或能到的格子都走过
    # （没有前沿）时才走到这里。
    #
    # 注意这里**不要**一进来就清 `self.target`。清掉的话：`guide()` 只要有一拍
    # 给不出合格候选（障碍物旁边很容易，`depth` 硬筛会把那个方向的格子全筛
    # 掉），目标点就丢了；下一拍 `guide()` 又给得出点，`_aim` 见 `target is
    # None` 直接换个新点，方向可以翻 90° 以上，L 层只好再转一次。
    #
    # 实测（起伏地形，正前 1 m 是圆柱）：0 s 目标 (+1.3,-0.8) 右前 −33°，狗
    # 左绕 +60°；22 s 目标变成 (+1.8,+0.7) 正前 +16°，它又转 −97° 追回去，
    # 再转 +60° —— 三次转向 23.7 秒，位置只挪了 0.5 m。这一拍里狗只挪了
    # 0.4 m，旧点还在 1.5 m 外，按 `REACHED` 根本不该换。
    def _by_distance(self, p, trace) -> str | None:
        secs = {s.name: s for s in p.sectors()}
        cur = secs.get(self.direction)
        # `direction` 是**相对机头**的方位名（"右侧" = 当时机头的右侧）。狗转个
        # 身，同一个名字就指向完全不同的物理方向，而下面 `keep` 是拿它跟当前
        # 扇区比距离 —— 比的已经不是同一个东西了。
        #
        # 2026-11-17 的日志：48 秒选的「右侧」在西边，狗随后掉头朝东，「右侧」
        # 于是变成南边；`keep` 用一个跟 target 无关的扇区保住了旧方向，target
        # 就一直在身后留着，模型每轮看到「向我身后转 160 度」。
        #
        # 有 target 就以 target 的实际方位为准，没有才用 `direction` —— 后者此时
        # 是「上一次选的方向」，仍然是相对当前机头的，语义对得上。
        if self.target is not None:
            rel_aim = _wrap(math.atan2(self.target[1] - p.ego.y,
                                       self.target[0] - p.ego.x) - p.ego.yaw)
            aim_name = min(SECTOR_NAMES,
                           key=lambda nv: abs(_wrap(nv[1] - rel_aim)))[0]
            cur = secs.get(aim_name)
        front = p.front_gap()
        self.last_gap = front

        def score(s) -> float:
            # 距离封顶（见 `GAP_CAP`）：够走就不再为空旷加分。
            base = min(s.gap, GAP_CAP) + BIAS.get(s.name, 0.0)
            if trace is not None:
                marks = trace.been(s.name)
                if marks:
                    base -= VISIT_PENALTY * min(1.0, marks / VISIT_FULL)
            return base

        # 先按「走不走得动」筛一遍，再在能走的方向里比分数。
        #
        # 不能拿「分数最高的那个方向」的距离去判断是否全堵：方向偏好是常数，
        # 正前 0.5 m（+0.25）会赢过正后 3.0 m（−0.90），于是明明后面通着，却
        # 报「四周都不到 0.7 米」站住不动。距离项封顶（`GAP_CAP`）之后分数被
        # 压得更紧，这个误判更容易发生。
        live = [s for s in secs.values() if s.gap >= HOPELESS_GAP]

        # 八个方向都堵：站住，让人接手。这时候继续指出一个方向只是让它往前顶。
        if not live:
            if self.direction != "原地":
                # 真的接管了才作废地图目标（见上面的注释）。
                self.target = None
                self.direction = "原地"
                self.changes += 1
                self.goal = self._compose("四周都不到 0.7 米，原地站住，等新的方向")
                self.thinker.set_goal(self.goal)
                widest = max(s.gap for s in secs.values())
                self._log(f"四周受阻（最宽 {widest:.2f} m），站住")
                return self.goal
            return None

        best = max(live, key=score)
        best_score = score(best)

        # 第一次必须先给一个目标。不给的话 L 层第一拍看到的是空目标，只能自己
        # 猜该干什么；而滞回逻辑（`keep`）会一直认为「正前还能走」，永远不写目标。
        # 滞回也要按分数比，不能按裸距离 —— 否则「当前方向走了很多格」这件事
        # 在换向判断里被忽略，刚指过去的方向一转眼又被指回来。
        keep = (not self._fresh and cur is not None
                and score(cur) >= best_score - SWITCH_MARGIN
                and cur.gap >= HOPELESS_GAP)
        self._fresh = False

        if keep:
            return None

        self.target = None               # 换了方向，地图目标作废
        why = "受阻" if front < BLOCK_GAP else "复查"
        if why == "受阻":
            self.blocked += 1
        self.direction = best.name
        self.changes += 1
        self.goal = self._compose(
            f"向{best.name}走{_turn_text(best.name)}，那个方向 {best.gap:.1f} 米内"
            f"没有东西（正前 {front:.1f} 米）")
        self.last_reason = why
        self.thinker.set_goal(self.goal)
        self._log(f"{why}换向 → {best.name}（{best.gap:.2f} m）")
        return self.goal

    def _compose(self, text: str) -> str:
        return f"{text}。{self.hint}" if self.hint else text

    # 目标点已经走到（`REACHED` 以内）就该立刻换一个，不能等 10 s 巡检。
    #
    # 实测迷宫日志：0 s 目标 (+1.1, 0)，狗 4 s 就走到 (+1.1, +0.1)，但 G 下一次
    # tick 要等 10 s。这 6 秒里 L 层拿到的一直是「目标点在背后 1.1 m」，于是
    # 8.1 s 开始转 +173° —— 一次掉头，方向彻底反过来。整段日志 12 次大转向，
    # 有几次就是这么来的：不是模型想掉头，是它在追一个已经走过去的点。
    def _aim_stale(self, p) -> bool:
        if self.target is None:
            return False
        dx = self.target[0] - p.ego.x
        dy = self.target[1] - p.ego.y
        d = math.hypot(dx, dy)
        # 「到了」或者「太远了」都算该换。太远那一半见 `AIM_MAX_DIST`。
        if d <= REACHED or d > AIM_MAX_DIST:
            return True
        # 被超过了也算。判据用方位不用距离：距离在 (REACHED, AIM_MAX_DIST]
        # 之间时，唯一能说明「这个点已经是身后的旧路」的就是它转到了背后。
        # 见 `AIM_BACK_DEG`。
        if d > REACHED:
            rel = abs(math.degrees(_wrap(math.atan2(dy, dx) - p.ego.yaw)))
            if rel > AIM_BACK_DEG:
                return True
        return self._on_tracks()

    # 目标点落在自己刚走过的地方。
    #
    # 这种点**不是**「还没去过」，而是「我 20 秒前站的地方」。留着它，模型每轮
    # 看到的就是「向我身后转 160 度，1.5 米外」—— 照着转就是掉头。
    #
    # 2026-11-17 的日志：48 秒给的点 (+0.4, +3.8)，狗随后往东走了 3 米，
    # 63 秒那条还在（`AIM_TIMEOUT` 只按时间算，而狗一直在动，走不到「追不到」
    # 那条分支），于是「向右转 14 度，2.1 米外」把狗拉回 15 秒前的路口。
    def _on_tracks(self) -> bool:
        trace = getattr(getattr(self.thinker, "percept", None), "trace", None)
        if trace is None or self.target is None:
            return False
        return trace.been_at(self.target) >= 1

    # 巡检 0.1 Hz，受阻 5 Hz 看一眼。
    #
    # 只按 0.1 Hz 睡的话，「前方掉到 0.5 米」这件事最多要等 10 秒才被发现，那时
    # 早就贴上了。所以循环本身快，只有选向受 `period` 限制 —— 受阻是事件，不是巡检。
    def _loop(self) -> None:
        tick = time.perf_counter()
        last = 0.0
        while self._run.is_set():
            now = time.perf_counter()
            p = getattr(self.thinker, "last_percept", None)
            front = p.front_gap() if p is not None and p.rays else float("inf")
            # 首拍还没写目标时按快循环试。`tick()` 拿不到感知就提前返回、
            # `_fresh` 仍是 True；如果这里按 0.1 Hz 的 `period` 睡，启动瞬间那
            # 一次没拿到就要再等 10 秒，这段时间 L 层没有目标，狗不动。
            wait = WATCH if self._fresh else self.period
            stale = p is not None and p.rays and self._aim_stale(p)
            if stale and time.monotonic() - self._aim_at >= AIM_HOLD:
                # 走到点了。**不重新选方向**，沿机头往前推一个点。
                #
                # 原来这里直接把点作废、让 `guide()` 重选。重选是按「BFS 距离 +
                # 偏离机头角」打分的，狗一转，分数就变，于是新点常在侧后方 ——
                # L 层照着转，转完再换点。实测（2026-11-16 迷宫）229 秒 22 次
                # 转向、累计 1858°、净位移 3.2 m（走了 30 多米）。
                #
                # 走廊里本来就该直着走。把点沿机头往前推，狗继续直行，直到前方
                # 真的到头（射线 < 这个距离）才交回 `guide()` 重选 —— 那时确实
                # 该转弯了。
                if front >= AIM_EXTEND:
                    e = p.ego
                    self.target = (e.x + AIM_EXTEND * math.cos(e.yaw),
                                   e.y + AIM_EXTEND * math.sin(e.yaw))
                    self._aim_at = time.monotonic()
                    self._aim_best = AIM_EXTEND
                    self._aim_pose = (e.x, e.y)
                    self.last_reason = "直行"
                else:
                    self.target = None
            if now - last >= wait or front < BLOCK_GAP or stale:
                last = now
                try:
                    self.tick()
                except Exception as err:                    # noqa: BLE001
                    self._log(f"选向失败：{type(err).__name__}: {err}")

            tick += WATCH
            rest = WATCH - (time.perf_counter() - tick)
            if rest > 0:
                time.sleep(rest)
            else:
                tick = time.perf_counter()
