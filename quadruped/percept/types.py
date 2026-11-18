"""V 层的通用数据结构：把任意传感器的一次观测，压成同一套人能读、模型能读的量。

这一层不碰任何传感器 API，也不 import mujoco / cv2。上游（仿真射线、深度相机、
激光雷达）负责把原始数据变成 `Ray` 和 `Ground`，这里只负责三件事：

    1. 定义「一次观测」长什么样（`Percept`）
    2. 把 24 条射线归纳成 8 个方位（`sectors()`）—— 模型看不过来 24 个数
    3. 渲染成一段中文（`text()`），直接塞进提示词

坐标约定：所有方位角都在机体系，0 是机头方向，逆时针为正（左边是正）。这样
「左前方有障碍」不用先知道机器狗朝向世界哪一边，也就不会因为 yaw 漂了而误判。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# 8 个方位，按从正前逆时针排。名字给模型看，中心角给代码用。
SECTOR_NAMES = (
    ("正前", 0.0),
    ("左前", math.pi / 4),
    ("左侧", math.pi / 2),
    ("左后", 3 * math.pi / 4),
    ("正后", math.pi),
    ("右后", -3 * math.pi / 4),
    ("右侧", -math.pi / 2),
    ("右前", -math.pi / 4),
)

# 这几个方位算「前方」，会参与距离告警和地面检查。
FRONT_SECTORS = ("正前", "左前", "右前")


@dataclass(frozen=True)
class SelfState:
    """机身状态。世界系位姿 + 机体系速度。"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def tilt_deg(self) -> float:
        # 和 `walker.tilt_rad()` 同一个算法：acos(cos(pitch)*cos(roll))。
        # 不要写成 acos(-cos(pitch))，roll=0 时它恒等 180°。
        up = math.cos(self.pitch) * math.cos(self.roll)
        return math.degrees(math.acos(max(-1.0, min(1.0, up))))

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class Ray:
    """一条测距射线。深度相机的单个像素、激光雷达的一束光，都能落到这个形状。"""

    bearing: float          # 机体系方位，rad，0 正前，正为左
    distance: float         # m；没打中就是传感器的量程上限
    top: float = 0.0        # 命中处的高度（离地），m；用于区分台阶和墙


@dataclass(frozen=True)
class Ground:
    """地面起伏。`ahead` 是从近到远的地面高度，负数是坑。"""

    under: float = 0.0
    ahead: tuple[float, ...] = ()

    @property
    def step_up(self) -> float:
        return max((h for h in self.ahead if h > 0), default=0.0)

    @property
    def step_down(self) -> float:
        return min((h for h in self.ahead if h < 0), default=0.0)


@dataclass(frozen=True)
class Sector:
    """一个方位上的归纳结果。"""

    name: str
    bearing: float
    gap: float              # 该方位最近障碍距离，m
    top: float              # 该障碍的高度，m


@dataclass(frozen=True)
class Percept:
    """一次观测。`origin` 说明数据从哪来，出问题时报出来。"""

    stamp: float = 0.0
    origin: str = "proprio"
    ego: SelfState = field(default_factory=SelfState)
    rays: tuple[Ray, ...] = ()
    ground: Ground = field(default_factory=Ground)
    range_max: float = 3.0
    notes: tuple[str, ...] = ()
    # 视觉模型对画面的描述。和 `rays` 是两个通道：这里只有语义（「前面是台阶」），
    # 没有距离。距离一律走 `rays`，因为语言模型的数字靠不住，而安全闸要的是数字。
    scene: str = ""
    scene_age: float = 0.0

    # 把射线按 8 个方位归组，每个方位取最近的一条。
    #
    # 不直接给模型 24 条：它会把「左前 1.2 m」和「正前 0.9 m」当成两件不同的事，
    # 然后在回复里各自说一遍。8 个方位足够描述「哪边能走」。
    def sectors(self) -> tuple[Sector, ...]:
        if not self.rays:
            return ()
        out = []
        for name, center in SECTOR_NAMES:
            # 比的是「环形距离的绝对值」，不是有符号差。少了 abs，正前那一组会
            # 选中 bearing=-180° 的射线（-π 比 0 小），8 个方位全被判成「没有覆盖」，
            # 整份视野变成一片空。
            near = min(self.rays, key=lambda r: abs(_ring(r.bearing - center)))
            delta = abs(_ring(near.bearing - center))
            if delta > math.pi / 4:
                # 这个方位没有射线覆盖（比如相机只有 90° 视野）。给量程上限，
                # 而不是给最近的另一条 —— 后者会让人以为「这边很空旷」。
                out.append(Sector(name, center, self.range_max, 0.0))
                continue
            out.append(Sector(name, center, near.distance, near.top))
        return tuple(out)

    def sector(self, name: str) -> Sector | None:
        for s in self.sectors():
            if s.name == name:
                return s
        return None

    # 正前方还能直着走多远。安全判断、受阻判定、状态栏都用它。
    #
    # 只看「正前」这一个扇区（±22.5°），不把左前/右前算进来。
    #
    # 之前取的是三者最小：在 1.68 m 宽的走廊里，左前和右前永远能看到 0.8 m
    # 外的侧墙，min 之后正前恒为 0.8 —— 狗在走廊正中，G 层每 10 秒判一次
    # 「受阻」、L 层每秒触发一次「前方变近」，全都在要求它转向。实测迷宫
    # 就是走 33 秒到走廊尽头，之后 170 秒停在 (3.5, 0) 反复转。
    #
    # 侧向的安全由 `hazards()` 和 `guard` 按各自阈值管，不需要挤进这一个数。
    #
    # 没有视野时返回 inf，不是 0：本体观测看不到东西，安全闸该保持沉默，
    # 而不是把「看不见」当成「贴脸了」，那样一接管就永远停着。
    def front_gap(self) -> float:
        for s in self.sectors():
            if s.name == "正前":
                return s.gap
        return float("inf")

    # 正后方的距离。贴墙要后退让开时用它判断「退得动吗」。
    # 用三个后方扇区里最大的那个：只要有一条能退，就能退。
    def back_gap(self) -> float:
        best = -1.0
        for s in self.sectors():
            if s.name in ("正后", "左后", "右后"):
                best = max(best, s.gap)
        return best if best >= 0.0 else float("inf")

    # 渲染成给文本模型看的一段话。不给 JSON：模型对数字的容错比对话差，
    # 而它需要的只是「哪边近、多远、多高」。
    def text(self) -> str:
        e = self.ego
        lines = [
            f"机身：位置 ({e.x:.2f}, {e.y:.2f})，高度 {e.z:.2f} m，"
            f"倾角 {e.tilt_deg:.0f}°，速度 {e.speed:.2f} m/s",
        ]
        if self.ground.ahead:
            ahead = " ".join(f"{h:+.2f}" for h in self.ground.ahead)
            lines.append(f"地面：脚下 {self.ground.under:+.2f} m，"
                         f"前方由近到远 {ahead} m")
        else:
            lines.append(f"地面：脚下 {self.ground.under:+.2f} m")

        secs = self.sectors()
        if secs:
            lines.append("视野（机头为 0°，左边为正）：")
            for s in secs:
                if s.gap >= self.range_max - 1e-3:
                    lines.append(f"  {s.name} 空")
                else:
                    kind = "墙/高物" if s.top > 0.35 else "矮障碍"
                    lines.append(f"  {s.name} {s.gap:.2f} m  {kind}（高 {s.top:.2f} m）")

        if self.scene:
            # 打上时间戳。这份描述可能比当前这一拍晚好几秒（视觉模型慢），模型需要
            # 知道「这句话是几秒前看到的」，否则会拿旧地形做新决定。
            #
            # 视觉那侧本来就是按 左 / 中 / 右 三行答的，原样分行贴上来：并成一行
            # 之后「哪一块写着不能」这个结构就没了，模型读到的是一长串。
            lines.append(f"画面（{self.scene_age:.1f} s 前）：")
            for row in self.scene.splitlines():
                lines.append(f"  {row}")
        elif not secs:
            # 既没有测距也没有画面。这时候确实什么都看不见，明说，别让它编。
            lines.append("视野：没有测距，也没有画面描述，只能靠机身数据")

        if self.notes:
            lines.append("注意：" + "；".join(self.notes))
        return "\n".join(lines)

    # 需要立刻处理的情况。返回空列表表示没危险。
    #
    # 阈值不是拍的：射线从**质心**打出去（`percept_pub.py:351` 的 `qpos[:3]`），
    # 机头前缘在质心前 0.335 m（`go2.xml:91-92`），而电机从收到停车命令到真停
    # 还要滑 8 cm 左右。0.55 是「机头距墙还有 10 cm 以上」的那条线。
    # 和 `percept/guard.py` 的 `stop_gap` 必须同一个数 —— 两处不一致时，提示词
    # 里说「危险」而安全闸不动作，或者反过来。
    def hazards(self, stop_gap: float = 0.80) -> list[str]:
        out = []
        e = self.ego
        if e.tilt_deg > 40:
            out.append(f"机身倾斜 {e.tilt_deg:.0f}°")
        if self.ground.step_down < -0.12:
            out.append(f"前方落差 {self.ground.step_down:.2f} m")
        if e.speed > 0.05:
            gap = self.front_gap()
            if gap < stop_gap:
                out.append(f"前方 {gap:.2f} m 有障碍，还在以 {e.speed:.2f} m/s 前进")
        return out


# 把角度归一到 (-π, π]。方位差必须过这一步再比，否则 +179° 和 -179° 会算成 358°。
def _ring(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
