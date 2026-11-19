#!/usr/bin/env bash
#
# 在扩展坞上装「插上无线网卡就自动连好」这套：
#   1. /usr/local/bin/go2-wifi-watch.sh      看门狗脚本
#   2. go2-wifi-watch.timer                  每 15 秒检查一次
#   3. 90-go2-wifi.rules                     wlan0 一出现就立刻拉一次 + 关 USB 自动挂起
#   4. NetworkManager 里把该连接设成自动重连、关省电
#
# 用法（在扩展坞上）：
#     sudo ./deploy/install-wifi-auto.sh
#     sudo GO2_WIFI_SSID=别的热点 ./deploy/install-wifi-auto.sh
#
# 幂等，重跑只覆盖文件和设置。
set -eu

SSID="${GO2_WIFI_SSID:-Yukiho}"
BIN=/usr/local/bin/go2-wifi-watch.sh
UNIT=/etc/systemd/system/go2-wifi-watch.service
TIMER=/etc/systemd/system/go2-wifi-watch.timer
RULE=/etc/udev/rules.d/90-go2-wifi.rules
ENVFILE=/etc/go2-wifi.env
SRC="$(cd "$(dirname "$0")" && pwd)/go2-wifi-watch.sh"

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
die() { printf '\033[31m[-] %s\033[0m\n' "$*" >&2; exit 1; }

[ "$(id -u)" = "0" ] || die "要用 root 跑：sudo $0"
[ -f "$SRC" ] || die "找不到 $SRC（脚本要和本文件放在一起）"

say "1. 看门狗脚本"
install -m 755 "$SRC" "$BIN"
echo "  → $BIN"

say "2. 配置"
cat > "$ENVFILE" <<EOF
# 看门狗连哪个热点。改完 systemctl restart go2-wifi-watch.timer
GO2_WIFI_SSID=$SSID
EOF
chmod 644 "$ENVFILE"
echo "  → $ENVFILE（SSID=$SSID）"

say "3. systemd 单元"
cat > "$UNIT" <<'EOF'
[Unit]
Description=无线网卡看门狗（wlan0 掉线自动重连）
Wants=NetworkManager.service
After=NetworkManager.service

[Service]
Type=oneshot
EnvironmentFile=-/etc/go2-wifi.env
ExecStart=/usr/local/bin/go2-wifi-watch.sh
# 它自己会 timeout 35 秒，这里兜一层：确实卡死也要让 timer 的下一轮能跑。
TimeoutStartSec=70
# 2 = 等物理重插，3 = 这一轮连不上。都是「下一轮再来」的正常状态，
# 不标成 failed，否则 journal 每 15 秒刷一条红色。
SuccessExitStatus=2 3
EOF

cat > "$TIMER" <<'EOF'
[Unit]
Description=每 15 秒检查一次 wlan0

[Timer]
OnBootSec=20s
OnUnitActiveSec=15s
AccuracySec=1s
Unit=go2-wifi-watch.service

[Install]
WantedBy=timers.target
EOF

say "4. udev 规则"
# 第一条：wlan0 一出现就让 systemd 立刻跑一次看门狗（不用等 15 秒的定时器）。
# 第二条：这个 dongle 不让内核自动挂起 —— 它掉线前 dmesg 里先是 -110（超时）
#         再是 -19（ENODEV），典型的口子被挂起后叫不醒。
cat > "$RULE" <<'EOF'
ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlan0", TAG+="systemd", ENV{SYSTEMD_WANTS}+="go2-wifi-watch.service"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="b82c", TEST=="power/control", ATTR{power/control}="on"
EOF

if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules 2>/dev/null || true
    udevadm trigger --subsystem-match=net --action=add 2>/dev/null || true
fi

# NetworkManager 那一侧：开机自动连、失败无限重试、关省电（省电会让 8822BU 睡死）。
if nmcli -t -f NAME con show 2>/dev/null | grep -qx "$SSID"; then
    nmcli con mod "$SSID" \
        connection.autoconnect yes \
        connection.autoconnect-retries 0 \
        connection.autoconnect-priority 10 2>/dev/null || true
    nmcli con mod "$SSID" 802-11-wireless.powersave 2 2>/dev/null || true
    echo "  NetworkManager：$SSID 已设成自动重连、关省电"
else
    echo "  [!] NetworkManager 里没有叫 $SSID 的连接，先手动连一次："
    echo "      nmcli dev wifi connect <SSID> password <口令>"
fi

say "5. 启动"
systemctl daemon-reload
systemctl enable --now go2-wifi-watch.timer >/dev/null 2>&1
systemctl start go2-wifi-watch.service 2>/dev/null || true
sleep 1
systemctl is-active go2-wifi-watch.timer | sed 's/^/  timer: /'
ip -4 -o addr show wlan0 2>/dev/null | awk '{print "  wlan0: " $4}' || echo "  wlan0: 还没有地址（看 journalctl -u go2-wifi-watch）"
echo
echo "  以后插上网卡什么都不用做，15 秒内会自动连上。"
echo "  看日志：journalctl -u go2-wifi-watch -f"
