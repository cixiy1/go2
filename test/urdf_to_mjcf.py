"""把训练用的 go2.urdf 的物理参数转成一份 MJCF。

    .venv/bin/python test/urdf_to_mjcf.py

## 为什么不重建整个模型

正向运动学已经比过：同一组 `DEFAULT_Q`，URDF 和我们的 MJCF 四足位置差不到 2 mm
（前足 +0.1778 vs +0.176，后足 −0.2705 vs −0.272）。所以几何是同一套，不需要重画。

真正会改变动力学的只有四项，这里逐项替换：

| 量 | URDF（训练） | go2.xml（原来） |
|---|---|---|
| calf 质量 | 0.154 kg + foot 0.040 kg | 0.2414 kg（把 foot 并进去了） |
| 关节 armature | 0（`asset.armature = 0.0`） | 0.015 |
| 关节 damping | 0（`angular_damping = 0.0`） | 0.3 |
| 关节 frictionloss | 0（训练无此项） | 0.3 |

visual mesh、collision geom、sensor、actuator 全部沿用 go2.xml —— 仿真桥按
`sensordata` 的索引读关节，重建一遍顺序一变就对不上。

## 惯性张量的表示

URDF 给 6 个数（ixx iyy izz ixy ixz iyz），MJCF 的 `<inertial>` 要 `quat` +
`diaginertia`（主轴 + 升序对角）。做特征分解转换。URDF 的 inertial `rpy` 这里
都是 0，不处理非零旋转的情形。
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
GO2_DIR = ROOT / "quadruped/sim/unitree_mujoco/unitree_robots/go2"
URDF = GO2_DIR / "go2_train.urdf"
SRC_XML = GO2_DIR / "go2.xml"
OUT_XML = GO2_DIR / "go2_urdf.xml"
OUT_SCENE = GO2_DIR / "scene_urdf.xml"

# URDF 的 link 名 → MJCF 的 body 名。只有根节点不同名。
LINK_TO_BODY = {"base": "base_link"}

# MJCF 里 calf 与 foot 是两个 body，URDF 里 foot 有 0.04 kg。合并进 calf：
# MuJoCo 会把 fixed joint 的子 body 融进父 body，分开写和合并写动力学等价，
# 但分开写要改 XML 结构，合并写只动一个 inertial。
FOOT_MASS_FROM_URDF = True


def parse_urdf(path: Path) -> dict[str, dict]:
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()]) if origin is not None else np.zeros(3)
        rpy = np.array([float(v) for v in origin.get("rpy").split()]) if origin is not None else np.zeros(3)
        if np.any(np.abs(rpy) > 1e-9):
            raise ValueError(f"{link.get('name')} 的 inertial rpy 不是 0，这个脚本没处理")
        i = inertial.find("inertia")
        ixx, iyy, izz = (float(i.get(k)) for k in ("ixx", "iyy", "izz"))
        ixy, ixz, iyz = (float(i.get(k)) for k in ("ixy", "ixz", "iyz"))
        inertia = np.array([[ixx, ixy, ixz],
                            [ixy, iyy, iyz],
                            [ixz, iyz, izz]])
        links[link.get("name")] = dict(mass=mass, com=xyz, inertia=inertia)
    return links


# 两个刚体合并：质量相加，质心按质量加权，惯量用平行轴定理搬到新质心。
def merge(a: dict, b: dict, b_offset: np.ndarray) -> dict:
    m = a["mass"] + b["mass"]
    b_com = b["com"] + b_offset
    com = (a["mass"] * a["com"] + b["mass"] * b_com) / m

    def shift(item, com_abs):
        d = com_abs - com
        return (item["inertia"]
                + item["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d)))

    return dict(mass=m, com=com, inertia=shift(a, a["com"]) + shift(b, b_com))


# 惯性张量 → (主轴四元数 wxyz, 升序对角)。det<0 时翻一列，保证是右手系。
def diagonalize(inertia: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    w, V = np.linalg.eigh(inertia)
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    return mat_to_quat(V), w


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([0.25 * s,
                      (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    return q / np.linalg.norm(q)


def inertial_xml(item: dict, indent: str) -> str:
    quat, diag = diagonalize(item["inertia"])
    # MuJoCo 要求 diaginertia 升序，eigh 已经给了。
    return (f'{indent}<inertial pos="{item["com"][0]:.6g} {item["com"][1]:.6g} '
            f'{item["com"][2]:.6g}" quat="{quat[0]:.6g} {quat[1]:.6g} '
            f'{quat[2]:.6g} {quat[3]:.6g}" mass="{item["mass"]:.6g}" '
            f'diaginertia="{diag[0]:.6g} {diag[1]:.6g} {diag[2]:.6g}"/>')


def build() -> None:
    links = parse_urdf(URDF)

    # calf 与 foot 合并
    if FOOT_MASS_FROM_URDF:
        for leg in ("FL", "FR", "RL", "RR"):
            calf, foot = links[f"{leg}_calf"], links[f"{leg}_foot"]
            links[f"{leg}_calf"] = merge(calf, foot, np.array([0.0, 0.0, -0.213]))

    text = SRC_XML.read_text(encoding="utf-8")

    # 1) 逐个 body 换 inertial。inertial 是 body 的第一个子元素，可能跨行。
    pattern = re.compile(r'(<body name="([A-Za-z_0-9]+)"[^>]*>)(\s*)(<inertial\b[^>]*?/>)',
                         re.DOTALL)

    replaced = []

    def sub(m):
        head, body_name = m.group(1), m.group(2)
        link = LINK_TO_BODY.get(body_name, body_name)
        if link not in links:
            return m.group(0)
        indent = m.group(3).split("\n")[-1]
        replaced.append(body_name)
        return f"{head}{indent}{inertial_xml(links[link], indent)}"

    text = pattern.sub(sub, text)

    # 2) 关节参数换成训练值。default 里那一行被 abduction / hip / knee 继承。
    text, n_joint = re.subn(
        r'<joint axis="0 1 0" damping="0\.3" armature="0\.015" frictionloss="0\.3"/>',
        '<joint axis="0 1 0" damping="0" armature="0" frictionloss="0"/>',
        text)

    text = text.replace('<mujoco model="go2">', '<mujoco model="go2_urdf">', 1)

    # 3) 头部注释说明来源与用途，免得以后有人以为是手写模型。
    text = text.replace(
        '<mujoco model="go2_urdf">',
        '<!-- 本文由 test/urdf_to_mjcf.py 从 go2_train.urdf 生成，勿手改。\n'
        '     与 go2.xml 的区别只在物理量：质量/惯量取 URDF 的值，关节\n'
        '     armature / damping / frictionloss 全部归零（对应训练配置\n'
        '     asset.armature=0、angular_damping=0）。几何、mesh、sensor、\n'
        '     actuator 与 go2.xml 相同，仿真桥可以直接用。 -->\n'
        '<mujoco model="go2_urdf">', 1)

    OUT_XML.write_text(text, encoding="utf-8")

    # 场景文件照抄 scene.xml，只换 include。
    scene = (GO2_DIR / "scene.xml").read_text(encoding="utf-8")
    scene = scene.replace('<include file="go2.xml"/>', '<include file="go2_urdf.xml"/>')
    scene = scene.replace('<mujoco model="go2 scene">', '<mujoco model="go2 urdf scene">')
    OUT_SCENE.write_text(scene, encoding="utf-8")

    print(f"替换了 {len(replaced)} 个 inertial，{n_joint} 处关节参数")
    print(f"  没找到的 body：{sorted(set(LINK_TO_BODY.values()) - set(replaced))}")
    print(f"输出 {OUT_XML.name} / {OUT_SCENE.name}")


if __name__ == "__main__":
    build()
