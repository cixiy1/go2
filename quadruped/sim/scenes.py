"""仿真场景：可选清单，加上迷宫的生成与缓存。

`scene.xml` 在 x = 1.2 m 放了两级 8 cm 台阶，2.3 m 起还有一段楼梯。测步速时机器狗
走到 1.2 m 就顶住了，读到的位移是撞停距离，不是走了多远。所以测试统一用去掉台阶的
平地版本。

两件事必须一起成立：

    文件落在 `scene.xml` 同目录    `go2.xml` 里的网格是相对路径（base_1.obj 之类），
                                  MuJoCo 按主文件所在目录解析，挪到 /tmp 就找不到
    生成后不删                    它同时被 `SIM_SCENE=.../scene_flat.xml` 引用。哪个
                                  进程退出时删掉，下一次起仿真器就是
                                  ParseXML: Error opening file

之前 `policy_sim.py` 用 `atexit` 在退出时清理，跑完一次回归就把文件删了，仿真器
随后起不来。现在生成逻辑只在这里，谁都不删。它落在被忽略的
`quadruped/sim/unitree_mujoco/` 里，不会进版本库。

## 迷宫

`scene_maze.xml` 是三条横向走廊 + 两个 180° 掉头，出口在北墙。选这个形状不是为了
难，是因为这份策略的短板就在转弯：实测 trot 原地转向 0.033 rad/s，pronk 0.32 rad/s，
九十度要五秒。走廊给 1.8 m 宽是为了让它能在原地转过来而不蹭墙。

转弯能力决定迷宫能长什么样，所以这里不写更绕的拓扑 —— 绕不过去，演示就变成
「顶在墙角不动」。
"""

from __future__ import annotations

from pathlib import Path

# 本文件在 quadruped/sim/，场景在 quadruped/sim/unitree_mujoco/unitree_robots/go2/。
HERE = Path(__file__).resolve().parent
GO2_DIR = HERE / "unitree_mujoco" / "unitree_robots" / "go2"
SCENE = GO2_DIR / "scene.xml"
FLAT = GO2_DIR / "scene_flat.xml"
TERRAIN = GO2_DIR / "scene_terrain.xml"
MAZE = GO2_DIR / "scene_maze.xml"


# 平地场景。已存在就直接用，不存在才从 scene.xml 滤掉台阶生成。
#
# 只认 `type="box"` 这一类：scene.xml 里的台阶和楼梯都是 box，地板是 plane，
# 机器狗的量测几何里没有 box。换成别的写法要同时改这里。
def flat_scene() -> Path:
    if not FLAT.is_file():
        keep = [ln for ln in SCENE.read_text().splitlines(True) if 'type="box"' not in ln]
        FLAT.write_text("".join(keep))
    return FLAT


# 迷宫尺寸。改这几个数就是在改难度：走廊宽度必须大于机身对角线
# （0.7 x 0.3 → 0.76 m），否则原地转向时四条腿会卡在两面墙之间。
CELL = 1.8          # 相邻走廊中心线间距
WALL_T = 0.12       # 墙厚
WALL_H = 0.6        # 墙高。机载相机在 0.4 m，低于这个高度就看不见墙顶，
                    # 射线也打得到，但视觉模型会把它说成「地面上的条纹」
COLS, ROWS = 3, 3


# 迷宫：三行走廊，隔墙留口交替在两端，出口在北墙偏东。
#
# 布局（俯视，x 向东，y 向北，机器狗出生在 (0,0)）：
#
#     y=4.5  ──────────────┐        ← 出口在 x≈3.6
#     row2   ← 走廊 3（向西走完再回头）
#     y=2.7  ┌───────────────
#     row1   走廊 2（向西）
#     y=0.9  ──────────────┐
#     row0   走廊 1（向东）→
#     y=-0.9 ─────────────────
#
# 每一行的隔墙整段封住，只在一端留 1.35 m 的缺口 —— 缺口就是掉头点。
#
# 盒子单独抽出来，是因为 2D 离线仿真（`test/maze_plan.py`）要拿同一份几何
# 做射线求交。两边各写一份的话，改迷宫尺寸只改一处，规划器的回归测试就会
# 悄悄变成在测另一张地图。
def maze_boxes() -> list[tuple[float, float, float, float]]:
    half = CELL / 2.0
    x0, x1 = -half, half + CELL * (COLS - 1)
    y0, y1 = -half, half + CELL * (ROWS - 1)
    rows = [i * CELL for i in range(ROWS)]            # 走廊中心线 y，第一行过原点
    gaps = CELL * 0.75                                  # 缺口宽度，1.35 m

    boxes: list[tuple[float, float, float, float]] = []
    t = WALL_T / 2.0

    # 外墙：南北两面，东西两面
    boxes.append(((x0 + x1) / 2, y0, (x1 - x0) / 2 + t, t))
    boxes.append((x0, (y0 + y1) / 2, t, (y1 - y0) / 2 + t))
    boxes.append((x1, (y0 + y1) / 2, t, (y1 - y0) / 2 + t))

    # 北墙留出口：缺口中心在最后一格（x1 那一列）
    cut = x1 - gaps
    boxes.append(((x0 + cut) / 2, y1, (cut - x0) / 2 + t, t))

    # 行间隔墙。缺口交替：第 0 道在东端，第 1 道在西端。
    for i in range(1, ROWS):
        wy = rows[i - 1] + CELL / 2
        if i % 2 == 1:                                  # 缺口在东端
            cut = x1 - gaps
            boxes.append(((x0 + cut) / 2, wy, (cut - x0) / 2 + t, t))
        else:                                           # 缺口在西端
            cut = x0 + gaps
            boxes.append(((cut + x1) / 2, wy, (x1 - cut) / 2 + t, t))
    return boxes


def maze_scene() -> Path:
    if MAZE.is_file():
        return MAZE
    half = CELL / 2.0
    x0, x1 = -half, half + CELL * (COLS - 1)
    y0, y1 = -half, half + CELL * (ROWS - 1)
    boxes = maze_boxes()

    body = "\n".join(
        f'    <geom type="box" pos="{cx:.3f} {cy:.3f} {WALL_H / 2:.3f}" '
        f'size="{hx:.3f} {hy:.3f} {WALL_H / 2:.3f}" material="wall"/>'
        for cx, cy, hx, hy in boxes)

    MAZE.write_text(f"""<mujoco model="go2 maze">
  <include file="go2.xml"/>

  <statistic center="{(x0 + x1) / 2:.2f} {(y0 + y1) / 2:.2f} 0.1" extent="5"/>

  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <global azimuth="-130" elevation="-25"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.25 0.32 0.4" rgb2="0.15 0.2 0.26"
      markrgb="0.75 0.75 0.75" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="6 6" reflectance="0.15"/>
    <material name="wall" rgba="0.72 0.68 0.58 1"/>
  </asset>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
{body}
  </worldbody>
</mujoco>
""")
    return MAZE


# 菜单里能选的场景。key 出现在 `--scene` 和 `SIM_SCENE` 两处。
#
# `build` 给的是生成函数，返回文件路径 —— 平地和迷宫是算出来的，台阶和地形是
# 仓库里自带的原始场景，直接指过去。
SCENES = (
    dict(key="flat", title="平地", desc="棋盘地面，测速度和步态用（默认）",
         build=flat_scene),
    dict(key="step", title="台阶", desc="x=1.2 m 两级 8 cm 台阶，再往前是楼梯",
         build=lambda: SCENE),
    dict(key="maze", title="迷宫", desc="三行走廊 + 两个掉头，出口在北边，给 VLA 演示",
         build=maze_scene),
    dict(key="terrain", title="起伏地形", desc="高度场，训练时的地形分布",
         build=lambda: TERRAIN),
)


def scene_path(key: str) -> Path:
    for item in SCENES:
        if item["key"] == key:
            return item["build"]()
    raise ValueError(f"没有叫 {key!r} 的场景，可选：{', '.join(s['key'] for s in SCENES)}")


def scene_menu() -> list[tuple[str, str, str]]:
    return [(s["key"], s["title"], s["desc"]) for s in SCENES]
