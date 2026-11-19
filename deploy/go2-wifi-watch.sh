#!/usr/bin/env bash
#
# 无线网卡看门狗：wlan0 掉线 / 刚插上 / 开机没连上，都靠它把网络拉回来。
#
# 由 go2-wifi-watch.timer 每 15 秒叫一次，插网卡时 udev 也会立刻叫一次。
# 已经连上就安静退出（退出码 0），所以在 journal 里只会看到它「干活」的那几次。
#
# 三种情况：
#   1. wlan0 有 IPv4          → 什么都不做（绝大多数时候）
#   2. wlan0 不存在、USB 在   → 重载 rtw_8822bu，让驱动重新探测
#   3. wlan0 不存在、USB 也不在 → 记一行日志说明要物理重插（软件救不了）
#
# 这块网卡（Realtek 8822BU）的问题不是信号弱，是从 USB 总线上掉：
#   usb 1-2: USB disconnect, device number 2
#   rtw_8822bu 1-2:1.2: failed to do USB write, ret=-19      (ENODEV)
# 掉之后设备节点消失，重载驱动也拉不回来 —— 只能重插。所以这里再往上一层
# 加了个 USB 口不自动挂起的 udev 规则，见 install-wifi-auto.sh。
set -u

SSID="${GO2_WIFI_SSID:-Yukiho}"
IFACE="${GO2_WIFI_IFACE:-wlan0}"
# 这个 dongle 的 VID:PID。换成别的网卡就改这里（`lsusb | grep -i wireless`）。
USB_ID="${GO2_WIFI_USB_ID:-0bda:b82c}"

log() { echo "[wifi] $*"; }

has_ip() {
    ip -4 -o addr show "$IFACE" 2>/dev/null | grep -q "inet "
}

# 1. 已经好了，不打扰任何人
if has_ip; then
    exit 0
fi

# 2. 设备不在，看看是不是 USB 还挂着、只是驱动没绑上
if [ ! -e "/sys/class/net/$IFACE" ]; then
    if lsusb 2>/dev/null | grep -qi "$USB_ID"; then
        # 2a. 先「软重插」：把那个 USB 口的 authorized 关掉再打开，等于让内核
        # 重新枚举。比 modprobe -r 更接近物理插拔，而且不用卸驱动。
        #     路径按 lsusb 的 Bus/Device 推：001/002 → /sys/bus/usb/devices/1-2
        dev=$(lsusb 2>/dev/null | grep -i "$USB_ID" | head -1 \
              | sed -n 's|Bus \([0-9]*\) Device \([0-9]*\).*|\1-\2|p')
        if [ -n "${dev:-}" ] && [ -e "/sys/bus/usb/devices/$dev/authorized" ]; then
            log "软重插 USB $dev"
            echo 0 > "/sys/bus/usb/devices/$dev/authorized" 2>/dev/null || true
            sleep 2
            echo 1 > "/sys/bus/usb/devices/$dev/authorized" 2>/dev/null || true
            sleep 5
        fi

        # 2b. 还没起来就重载驱动
        if [ ! -e "/sys/class/net/$IFACE" ]; then
            log "$IFACE 仍然缺失，重载 rtw_8822bu"
            modprobe -r rtw_8822bu 2>/dev/null
            sleep 2
            modprobe rtw_8822bu 2>/dev/null
            sleep 4
        fi
    else
        log "USB 无线网卡不在总线上（$USB_ID），需要物理重插"
        exit 2
    fi
fi

if [ ! -e "/sys/class/net/$IFACE" ]; then
    log "重载驱动后仍然没有 $IFACE，需要物理重插"
    exit 2
fi

# 3. 设备在，但没连上
log "连 $SSID …"
# `nmcli con up` 在连不上时会卡很久（默认好几分钟），必须自己掐断：
# 卡住的那一次会占着 timer 的下一次触发，反而更慢。
timeout 35 nmcli con up "$SSID" >/dev/null 2>&1
rc=$?

if has_ip; then
    log "已连上 $(ip -4 -o addr show "$IFACE" | awk '{print $4}')"
    exit 0
fi

# 密码错 / 热点没开 / 信号弱都长这样。打一行就够，别每次刷屏。
log "连接失败（nmcli 退出码 $rc），下一轮再试"
exit 3
