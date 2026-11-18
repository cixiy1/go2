#!/usr/bin/env python3
"""给 rtw88 downstream 源码打上「禁用 RX 聚合」补丁。

背景
----
扩展坞（Jetson Orin NX + tegra-xhci）上，这块 RTL8822BU（0bda:b82c）只要连
加密的 5 GHz 网络，整机就 hard lockup，约 120 秒后被 AON 看门狗复位：

    tegra-pmc: ### PMC reset source: BCCPLEXWDT

内核连日志都来不及写盘 —— `journalctl -b -1` 永远是 `No such boot ID`，
pstore 也是空的。电压全程稳在 5.088 V，排除了掉电。

根因在 RX DMA 聚合。main.c 的 watchdog 里：

    rtw_hci_dynamic_rx_agg(rtwdev,
                           tx_unicast_mbps >= 1 || rx_unicast_mbps >= 1);

8822B 走 rtw_usb_dynamic_rx_agg_v1，会把 USB DMA 聚合寄存器改成
size=0x5 / timeout=0x20。而 WPA 握手要收 EAPOL 帧，吞吐正好越过 1 Mbps ——
聚合寄存器的改写和握手的 USB 事务撞在一起，tegra-xhci 挂死。

补丁
----
加一个模块参数 `disable_rx_agg`，打开后 rtw_usb_dynamic_rx_agg 直接走
enable=false 分支（size=0 / timeout=1），聚合不再随吞吐变化。

实测（信道 149，信号 -47 dBm）：
    打开  连上稳定，30 MB 传输 + 200 包 ping，0.5% 丢包，无复位
    关闭  每次连都锁死，5 次全中

代价是吞吐下降、CPU 中断变多。扩展坞只做遥控用，值。

用法
----
    python3 apply-rtw88-rxagg-patch.py /path/to/rtw88-master

幂等：已经有 disable_rx_agg 就直接跳过。
"""
from __future__ import annotations

import pathlib
import sys

PARAM_BLOCK = """
/* Force RX DMA aggregation off.
 *
 * main.c 的 watchdog 会在吞吐 >= 1 Mbps 时调 rtw_hci_dynamic_rx_agg(..., true)，
 * 对 8822B 走 rtw_usb_dynamic_rx_agg_v1：size=0x5, timeout=0x20。这个操作会改写
 * USB DMA 聚合寄存器，而 WPA 握手（收 EAPOL 帧）恰好也在那个吞吐区间 ——
 * 在 Jetson 的 tegra-xhci 上，两者撞上的结果是整机 hard lockup
 * （dmesg 只留下 PMC reset source: BCCPLEXWDT，内核来不及写任何日志）。
 *
 * 设 Y 之后这一路直接走 enable=false 分支（size=0, timeout=1），
 * 换来的是吞吐下降、CPU 中断变多，稳不稳定先看结果。
 */
static bool rtw_disable_rx_agg;
module_param_named(disable_rx_agg, rtw_disable_rx_agg, bool, 0644);
MODULE_PARM_DESC(disable_rx_agg,
		 "Set to Y to force RX DMA aggregation off (workaround for USB stalls on tegra-xhci)");
"""

ANCHOR_PARAM = """MODULE_PARM_DESC(switch_usb_mode,
\t\t "Set to N to disable switching to USB 3 mode to avoid potential interference in the 2.4 GHz band (default: Y)");
"""

ANCHOR_FUNC = """static void rtw_usb_dynamic_rx_agg(struct rtw_dev *rtwdev, bool enable)
{
\tswitch (rtwdev->chip->id) {
"""

PATCHED_FUNC = """static void rtw_usb_dynamic_rx_agg(struct rtw_dev *rtwdev, bool enable)
{
\t/* 见文件头 disable_rx_agg 的说明：强制走关闭分支。 */
\tif (rtw_disable_rx_agg)
\t\tenable = false;

\tswitch (rtwdev->chip->id) {
"""


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    src = pathlib.Path(sys.argv[1]) / "usb.c"
    if not src.is_file():
        print(f"找不到 {src}", file=sys.stderr)
        return 1

    text = src.read_text(encoding="utf-8", errors="replace")
    if "disable_rx_agg" in text:
        print("[=] 已经打过补丁，跳过")
        return 0

    if ANCHOR_PARAM not in text:
        print("找不到 switch_usb_mode 的 MODULE_PARM_DESC，源码版本可能不一样", file=sys.stderr)
        return 3
    if ANCHOR_FUNC not in text:
        print("找不到 rtw_usb_dynamic_rx_agg 的函数头，源码版本可能不一样", file=sys.stderr)
        return 3

    text = text.replace(ANCHOR_PARAM, ANCHOR_PARAM + PARAM_BLOCK, 1)
    text = text.replace(ANCHOR_FUNC, PATCHED_FUNC, 1)
    src.write_text(text, encoding="utf-8")
    print(f"[+] 已打补丁：{src}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
