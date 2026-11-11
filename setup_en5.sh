#!/bin/zsh
# Go2 有线连接配置。
#   ./setup_en5.sh              -> 用 DHCP 获取地址（网线接交换机/路由器时用这个）
#   ./setup_en5.sh static       -> 设 192.168.123.222/24（网线直连机器人时用，官方推荐 .222）
#   ./setup_en5.sh static 192.168.123.50
set -u

IFACE="en5"
SERVICE="AX88179B"
ROBOT="192.168.123.161"

MODE="${1:-dhcp}"
IP="${2:-192.168.123.222}"

# 等接口出现（网卡枚举需要时间）
echo "[*] 等待 $IFACE 出现…"
for i in {1..30}; do
    ifconfig "$IFACE" >/dev/null 2>&1 && break
    sleep 1
done

if ! ifconfig "$IFACE" >/dev/null 2>&1; then
    echo "[-] 30 秒内 $IFACE 一直不存在：USB 网卡没被系统枚举。"
    echo "    1. 直插 Mac 机身的 USB-C 口，不要经过 Hub / 扩展坞"
    echo "    2. 换一根支持数据传输的 USB-C 线（部分线只供电）"
    echo "    3. 没有识别就装 ASIX 官方 macOS 驱动（AX88179B 不是所有系统都自带）"
    echo "    4. 重插后 ifconfig -a | grep en 看新接口名，改了名就改脚本里 IFACE"
    exit 1
fi

LINK=$(ifconfig "$IFACE" | awk -F: '/status:/{print $2}' | xargs)
echo "[*] $IFACE 存在，链路: ${LINK:-unknown}"
if [ "$LINK" = "inactive" ]; then
    echo "[-] 网线没通。检查网线两端、对端设备是否上电。"
    exit 1
fi

run_admin() {
    osascript -e "do shell script \"$1\" with administrator privileges"
}

if [ "$MODE" = "static" ]; then
    echo "[*] 静态地址 $SERVICE -> $IP/24"
    run_admin "/usr/sbin/networksetup -setmanual $SERVICE $IP 255.255.255.0" || exit 1
else
    echo "[*] $SERVICE 切回 DHCP"
    run_admin "/usr/sbin/networksetup -setdhcp $SERVICE" || exit 1
fi

run_admin "/sbin/ifconfig $IFACE up"
sleep 2
echo
networksetup -getinfo "$SERVICE"
echo

if [ "$MODE" = "static" ]; then
    echo "[*] ping $ROBOT"
    if ping -c 3 -t 2 "$ROBOT" >/dev/null 2>&1; then
        echo "[+] 通了。跑 test/connect.py 试试。"
        exit 0
    fi
    echo "[-] ping 不通，扫同网段在线设备："
    ping -c 2 -t 1 192.168.123.255 >/dev/null 2>&1
    arp -an | grep "(192.168.123\." | sed 's/^? //'
    echo "    —— 有地址说明网段对、机器人 IP 不是 .161；空则链路层仍不通。"
else
    MYIP=$(ipconfig getifaddr "$IFACE" 2>/dev/null)
    echo "[*] 拿到地址: ${MYIP:-(无，对端可能没有 DHCP 服务)}"
    [ -n "$MYIP" ] && echo "[*] 同网段探测：" && arp -an | grep "($(echo $MYIP | cut -d. -f1-3)\." | sed 's/^? //'
fi
