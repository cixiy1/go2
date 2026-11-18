#!/usr/bin/env python3
"""把被踢走的官方运控（mcf）还回去。给 systemd 的 `ExecStopPost` 用。

    python deploy/release_mcf.py            # 看现在是谁在管，必要时恢复
    python deploy/release_mcf.py --dry      # 只看，不动

为什么需要它：`DirectLink.claim()` 会把 mcf 从当前模式踢掉，正常退出时
`close()` 会 `SelectMode(原来的名字)` 还回去。但 KILL / OOM / 拔电源之后没有
这一段 —— 官方遥控器从此失效，只能重开 mcf。这个脚本在服务停止的**任何**路径
上跑一次（正常退出时 mcf 已经还了，它什么都不做）。

判据只有一条：`CheckMode()` 返回的名字。非空 = 有人管着（我们的进程或官方），
空 = 没人管，那就是要恢复的那一种。

要恢复成什么，读 `/tmp/quadruped-mcf.json`（`DirectLink` 踢之前写的）。没有这个
文件时不猜：Go2 上 `ai` 和 `normal` 都有，猜错等于把人家的配置改了。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quadruped.motion.direct import MCF_STATE                    # noqa: E402


def read_state() -> str:
    try:
        return str(json.loads(MCF_STATE.read_text()).get("mode") or "")
    except (OSError, ValueError):
        return ""


def main() -> int:
    ap = argparse.ArgumentParser(description="交还官方运控")
    ap.add_argument("--dry", action="store_true", help="只看状态，不恢复")
    ap.add_argument("--mode", default="", help="直接指定要恢复的模式，覆盖记录文件")
    a = ap.parse_args()

    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client \
            import MotionSwitcherClient
    except Exception as err:                                    # noqa: BLE001
        print(f"读不到 unitree_sdk2py（{type(err).__name__}: {err}），跳过")
        return 0

    sc = MotionSwitcherClient()
    sc.SetTimeout(5.0)
    sc.Init()
    code, data = sc.CheckMode()
    if code != 0:
        print(f"CheckMode 失败（code {code}）。真机上常见原因：网卡不对、"
              f"没和机器人同一个 DDS 域。")
        return 1
    current = (data or {}).get("name", "")
    if current:
        print(f"当前 mcf 模式：{current} —— 有人管着，不用恢复")
        return 0

    mode = a.mode or read_state()
    if not mode:
        print(f"mcf 没人管，但没有记录该恢复成什么（{MCF_STATE} 不存在）。"
              f"用 --mode 指定，或在官方 App / 遥控器上重开。")
        return 2
    if a.dry:
        print(f"mcf 没人管，会恢复成 {mode}（--dry，未执行）")
        return 0

    sc.SelectMode(mode)
    code, data = sc.CheckMode()
    now = (data or {}).get("name", "") if code == 0 else "?"
    print(f"已恢复成 {mode}，现在：{now or '仍然是空'}")
    return 0 if now else 1


if __name__ == "__main__":
    raise SystemExit(main())
