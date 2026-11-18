"""网页遥控：在浏览器里起停 VLA、看画面和日志。

菜单里的一项。它自己**不碰 DDS**，只起一个 `quadruped.web.app` 服务 ——
页面上的「开始」再由那个服务起 `quadruped.control.embodied.vla` 子进程。
运动控制全在子进程里，网页和 `lowcmd` 之间没有第二条链路。

为什么要有这一项：真机上 `--no-watch` 时没有窗口，人只能看终端日志；
VLA 的决策过程（画面、视野八方位、模型的想法）在网页上看得清楚得多。
选它等于「VLA 自主 + 一个能看能停的遥控器」，而且不用记命令行参数。

    uv run python main.py web -- --iface en5
    # 或者从菜单里选「网页遥控」，服务起来后会自动打开浏览器

单独跑（不经启动器）：
    python -m quadruped.web.app --iface en5 --open
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

META = dict(
    title="网页遥控",
    group="embodied",
    desc="浏览器里起停 VLA 自主，看画面、视野和日志",
    owns_cmd=True,
    owns_keys=True,
    # 页面起的子进程就是 VLA，走策略网络直连 lowcmd。
    client="policy",
)


# 启动器会给每个控制器塞 `--client`（它按 META 声明算出来的）和 `--motion`。
# 这两个 `web.app` 不认识，摘掉别让 argparse 报错。
#
# **`--preset` 不能摘。** 原来它在摘除名单里，理由是「用哪套运控由页面上的
# 开始决定」—— 可页面那个「开始」写死了 `--preset real`，于是仿真里点开始
# 起的是真机 VLA。现在 `web.app` 认这个参数，按它决定子进程连 en5 还是 lo0。
_DROP_WITH_VALUE = ("--client", "--motion")


def _strip(argv: list[str]) -> list[str]:
    out: list[str] = []
    skip = False
    for tok in argv:
        if skip:
            skip = False
            continue
        if tok in _DROP_WITH_VALUE:
            skip = True
            continue
        if any(tok.startswith(k + "=") for k in _DROP_WITH_VALUE):
            continue
        out.append(tok)
    return out


def main(argv: list[str] | None = None) -> int:
    from quadruped.web.app import main as web_main

    args = _strip(list(sys.argv[1:] if argv is None else argv))
    if "--open" not in args:
        args.append("--open")
    return web_main(args)


if __name__ == "__main__":
    sys.exit(main())
