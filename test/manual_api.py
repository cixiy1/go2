"""手动传 apiId 的三种方式。

用途：固件支持、但 SDK 没封装的接口，只能绕过 Move() 这类方法直接发。
接狗之前也能跑，看返回码就知道每一步走到哪了。

    .venv/bin/python test/manual_api.py            # 自动选网卡
    .venv/bin/python test/manual_api.py en5        # 指定网卡
"""

import json
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_api import SPORT_API_ID_MOVE
from unitree_sdk2py.go2.sport.sport_client import SportClient


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else None
    ChannelFactoryInitialize(0, iface)

    client = SportClient()
    client.SetTimeout(2.0)
    client.Init()

    print("=== A. 走白名单：_Call / _CallNoReply ===\n")

    # 1008 已在 Init() 里注册过，等价于 client.Move(0, 0, 0)
    code = client._CallNoReply(SPORT_API_ID_MOVE, json.dumps({"x": 0.0, "y": 0.0, "z": 0.0}))
    print(f"A1  已注册 id {SPORT_API_ID_MOVE}  -> {code}   (0=已发出 3102=无对端)")

    # 2060 没注册过 → 3103，一个字节都不发
    code = client._CallNoReply(2060, "{}")
    print(f"A2  未注册 id 2060      -> {code}   (3103=本地拒绝)")

    print("\n=== B. 先登记再调用 ===\n")

    client._RegistApi(2060, 0)          # 补进白名单，第二个参数是优先级
    code = client._CallNoReply(2060, "{}")
    print(f"B   登记后 2060         -> {code}   (3102=发出了但没人收)")

    print("\n=== C. 完全不查白名单：*Base ===\n")

    # _CallBase 后两个参数有默认值；_CallNoReplyBase 没有，必须写明 priority 和 leaseId
    code = client._CallNoReplyBase(9999, "{}", 0, 0)
    print(f"C1  _CallNoReplyBase 9999 -> {code}")
    print("    绕过一切校验，id 写错也会照样发到网上")

    print("\n=== D. 需要等回复的接口 ===\n")

    # 1007 Euler 是查询类，必须用 _Call 拿数据
    code, data = client._Call(1007, "{}")
    print(f"D   _Call 1007 (Euler)  -> code={code} data={data}")
    if code == 3104:
        print("    3104 = 狗收到了但没回，通常是对端没处理这个 id")


if __name__ == "__main__":
    main()
