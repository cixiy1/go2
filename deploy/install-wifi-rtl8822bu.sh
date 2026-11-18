#!/bin/sh
# 装 RTL8822BU 无线驱动（扩展坞上那块 USB 网卡 0bda:b82c）。
#
# 为什么需要单独一个脚本：这颗芯片内核不认 —— `rtl8xxxu` 的 USB ID 列表里
# 没有 b82c（`modinfo rtl8xxxu | grep -i B82C` 是空的），所以 `modprobe` 任何
# 内置模块都救不了，只能外挂一份 rtw88 downstream 驱动。
#
#   sudo ./deploy/install-wifi-rtl8822bu.sh [源码目录]
#
# 默认源码目录是 /tmp/rtw88-master。跑完 `ip -br link` 应该出现 wlan 接口，
# `nmcli dev status` 里出现一个 wifi 设备，页面的「网络」页签就能扫到周围网络。
set -eu

SRC="${1:-/tmp/rtw88-master}"
MOD=rtw_8822bu
HERE="$(cd "$(dirname "$0")" && pwd)"

[ "$(id -u)" = "0" ] || { echo "要用 root 跑：sudo $0 ${1:-}" >&2; exit 1; }

if [ ! -f "$SRC/rtw_8822bu.ko" ] && [ ! -f "$SRC/usb.c" ]; then
    echo "在 $SRC 里既没有 rtw_8822bu.ko 也没有 usb.c。" >&2
    echo "先解包并编译：" >&2
    echo "  tar xzf /tmp/rtw88.tar.gz -C /tmp && cd /tmp/rtw88-master && make -j4" >&2
    exit 2
fi

# 关键补丁：关掉 RX DMA 聚合。不打的话连加密 5GHz 会整机 lockup（看门狗复位），
# 详见 deploy/apply-rtw88-rxagg-patch.py 的文件头。幂等，打过了会跳过。
if [ -f "$SRC/usb.c" ] && [ -f "$HERE/apply-rtw88-rxagg-patch.py" ]; then
    echo "[*] 打 RX 聚合补丁"
    python3 "$HERE/apply-rtw88-rxagg-patch.py" "$SRC"
fi

# 源码在、产物还没有（或者刚打过补丁），就地编译。
if [ ! -f "$SRC/rtw_8822bu.ko" ]; then
    echo "[*] make -j$(nproc)"
    make -C "$SRC" -j"$(nproc)"
fi

# 编译产物和当前内核不匹配时装上去也加载不了，先说清楚，别让人对着
# `modprobe: invalid module format` 猜。
KV="$(modinfo -F vermagic "$SRC/rtw_8822bu.ko" 2>/dev/null | awk '{print $1}')"
RUN="$(uname -r)"
if [ "$KV" != "$RUN" ]; then
    echo "编译时内核是 $KV，运行的是 $RUN —— 要在这个内核上重新 make。" >&2
    exit 3
fi

cd "$SRC"
echo "[*] make install"
make install

echo "[*] depmod"
depmod -a

# 三个稳定性参数。其中 disable_rx_agg 依赖上面打的补丁 —— 没打补丁就配这个，
# modprobe 会报 unknown parameter，网卡直接起不来，所以先判断参数在不在。
if [ -f "$HERE/rtw88-usb.conf" ]; then
    echo "[*] 安装模块参数 /etc/modprobe.d/rtw88-usb.conf"
    install -m 644 "$HERE/rtw88-usb.conf" /etc/modprobe.d/rtw88-usb.conf
    if ! modinfo -p "$MOD" 2>/dev/null | grep -q disable_rx_agg \
       && ! modinfo -p rtw_usb 2>/dev/null | grep -q disable_rx_agg; then
        echo "    警告：模块里没有 disable_rx_agg 参数（补丁没生效？），" >&2
        echo "    先去掉 conf 里 disable_rx_agg 那一行再重跑。" >&2
    fi
fi

# 先卸掉可能存在的旧实例，再按原名加载。已经加过的不重复 modprobe。
modprobe -r "$MOD" 2>/dev/null || true
echo "[*] modprobe $MOD"
modprobe "$MOD"

# 开机自动加载。不写这一条的话，重启之后网卡又「消失」，人以为是驱动装坏了。
echo "$MOD" > /etc/modules-load.d/rtw88-usb.conf

sleep 2
echo
echo "=== 结果 ==="
if ip -br link | grep -qE '^wlan'; then
    ip -br link | grep -E '^wlan'
else
    echo "没有 wlan 接口。看内核日志：" >&2
    dmesg | tail -20 | grep -iE 'rtw|8822|usb' || true
    exit 4
fi

nmcli dev status | grep -E 'wifi' || echo "nmcli 里还没有 wifi 设备（等 NetworkManager 扫一下）"
echo
echo "下一步：打开网页的「网络」页签扫 Wi-Fi，或者直接"
echo "  nmcli dev wifi connect <SSID> password <密码>"
