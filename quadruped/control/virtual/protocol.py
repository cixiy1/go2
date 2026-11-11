"""控制器协议：一个模块要能被项目入口拉起来，必须长成什么样。

这层只定义形状和校验，不实现任何控制逻辑，也不连硬件。

## 为什么是模块级协议而不是基类

「控制器」在这里是一个**进程**：自己有 `argparse`、自己有主循环、自己收工。
写成 `class Controller(ABC)` 会逼着每个控制器把主循环塞进一个方法里 ——
`physical/keyboard.py` 一千三百行，按键源、加速度限幅、收工兜底全在模块级，
改成类比收益大得多。所以协议是：模块级 `META` 字典 + `main(argv)` 函数。

`Protocol` 用在**静态检查**上（IDE、mypy 能认出形状），`normalize_meta`
用在**运行时校验**上（启动器列菜单前跑一遍，错在哪直接显示在菜单里）。

## 写一个新控制器

    META = dict(title="语音遥控", desc="说一句走两步", group="physical",
                owns_cmd=True, owns_keys=False)

    def main(argv=None) -> int:
        ...
        return 0

    if __name__ == "__main__":
        sys.exit(main())

放对目录（`group` 必须等于目录名）、能 `python -m quadruped.control.<group>.<文件名>`
独立跑起来，启动器就会自动列出来。
"""

from typing import Protocol, runtime_checkable

#: 认得的 META 字段。多写的键不会报错，但不会被用到 —— 需要新字段时两边一起改。
META_KEYS = ("title", "desc", "group", "owns_cmd", "owns_keys")

#: 必填字段。`title` 缺了菜单里没法显示，直接算错。
REQUIRED_KEYS = ("title",)

#: 分组元数据认得的字段。
GROUP_KEYS = ("title", "order")


@runtime_checkable
class ControllerModule(Protocol):
    """控制器的形状。只是给 IDE 和 mypy 看的，运行时靠 normalize_meta 校验。"""

    META: dict

    def main(self, argv: list[str] | None = None) -> int:
        ...


@runtime_checkable
class ControllerGroup(Protocol):
    """分组包的形状：`quadruped/control/<目录名>/__init__.py` 里声明 `GROUP`。

    声明了才上菜单，没声明就是内部实现（抽象层、工具包）。启动器扫目录来找，
    不在自己那边登记任何分组。
    """

    GROUP: dict


def normalize_group(meta, name: str) -> dict:
    """校验一个分组的 `GROUP`。出错抛 ValueError，消息会显示在菜单里。

    `order` 决定菜单里的先后，同值按目录名排。默认 50，写到中间好调整。
    """
    if not isinstance(meta, dict):
        raise ValueError(f"GROUP 应该是 dict，实际是 {type(meta).__name__}")

    title = str(meta.get("title", "")).strip()
    if not title:
        raise ValueError("GROUP 缺字段：title")

    order = meta.get("order", 50)
    if isinstance(order, bool) or not isinstance(order, (int, float)):
        raise ValueError(f"GROUP 的 order 应该是数字，实际是 {order!r}")

    clean = dict(name=name, title=title, order=float(order))
    unknown = sorted(set(meta) - set(GROUP_KEYS))
    if unknown:
        clean["_unknown_keys"] = unknown
    return clean


def normalize_meta(meta, group: str, stem: str) -> dict:
    """校验并补全 META，返回一份干净的副本。

    出错抛 ValueError，消息里带上字段名和实际值 —— 启动器会把这句话原样
    显示在菜单里，所以它得让人一眼能改。`group` 由所在目录决定，写错了报错
    而不是覆盖：搬文件忘改 group 是最容易发生的错，静默修正会让人以为生效了。
    """
    if not isinstance(meta, dict):
        raise ValueError(f"META 应该是 dict，实际是 {type(meta).__name__}")

    missing = [k for k in REQUIRED_KEYS if not meta.get(k)]
    if missing:
        raise ValueError(f"META 缺字段：{', '.join(missing)}")

    declared = meta.get("group", group)
    if declared != group:
        raise ValueError(
            f"META 里写的是 group={declared!r}，但它放在 {group}/ 目录下")

    title = str(meta["title"]).strip()
    desc = str(meta.get("desc", "")).strip()
    clean = dict(
        title=title or stem,
        desc=desc,
        group=group,
        owns_cmd=bool(meta.get("owns_cmd", False)),
        owns_keys=bool(meta.get("owns_keys", False)),
    )
    unknown = sorted(set(meta) - set(META_KEYS))
    if unknown:
        # 不报错，但把名字记下来，写错键名（比如 own_cmd）时不至于毫无反应。
        clean["_unknown_keys"] = unknown
    return clean


def check_module(mod, group: str, stem: str):
    """校验一个已导入的模块。返回 (meta, error)，两者必有一为 None。

    `main` 缺失不在这里查 —— 有些模块只在 `__main__` 里跑，能 `-m` 起来就够了。
    """
    meta = getattr(mod, "META", None)
    if meta is None:
        return None, None                  # 没有 META 的不是控制器，交给调用方跳过
    try:
        return normalize_meta(meta, group, stem), None
    except ValueError as e:
        return None, str(e)
