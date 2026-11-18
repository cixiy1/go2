#!/bin/zsh
# 真机一键起网关：连机器人（DDS over en5）、出网页/App 的 WebSocket、转发相机。
#
#   ./start_real.sh                    # 默认 token=demo，端口 8780
#   TOKEN=xxx PORT=8780 ./start_real.sh
#
# 不带 --client 也可以：真机会自动落到 official（官方 SportClient），
# 那条路走 rt/api/sport/request，不碰 rt/lowcmd，狗的姿态由机器自己的运控兜着。
set -u

cd "$(dirname "$0")" || exit 1

TOKEN="${TOKEN:-demo}"
PORT="${PORT:-8780}"
# 变量名不能叫 HOST —— zsh 里 HOST 是内置参数（主机名），`${HOST:-0.0.0.0}`
# 会直接取到 YukihodeMacBook-Air.local，网关就只绑回环，手机连不上。
BIND="${BIND:-0.0.0.0}"
IFACE_HINT="${IFACE_HINT:-en5}"
PY="${PY:-.venv/bin/python}"

if [ ! -x "$PY" ]; then
    echo "[-] 找不到 $PY。先在仓库根跑：python3 -m venv .venv && .venv/bin/pip install -e ." >&2
    exit 1
fi

# 机器人侧的网段。网卡名别写死：USB 网卡换口重插之后 en5/en6/en7 都会出现，
# 这里按地址反查，和 setup_en5.sh 的判法一致。
iface="$IFACE_HINT"
robot_net_ok() {
    ifconfig "$1" 2>/dev/null | grep -q "inet 192\.168\.123\."
}

if ! robot_net_ok "$iface"; then
    found=""
    for i in $(ifconfig -l); do
        if robot_net_ok "$i"; then found="$i"; break; fi
    done
    if [ -n "$found" ]; then
        echo "[*] $iface 上没有 192.168.123 地址，改用 $found"
        iface="$found"
    else
        echo "[-] 没有网卡拿到 192.168.123.x。先跑 ./setup_en5.sh static" >&2
        echo "    然后确认 ping 192.168.123.161 通。" >&2
        exit 1
    fi
fi

echo "[*] 机器人网卡: $iface ($(ifconfig "$iface" | awk '/inet /{print $2}'))"

if ! ping -c 1 -t 2 192.168.123.161 >/dev/null 2>&1; then
    echo "[!] ping 192.168.123.161 不通。网关仍会起，但切手动时会报「服务端起不来」。"
fi

# 8780 被占就是上一次没退干净。先报出来，别让新进程静默失败。
if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "[-] 端口 $PORT 已被占用：" >&2
    lsof -nP -iTCP:"$PORT" -sTCP:LISTEN | tail -n +2 >&2
    echo "    先 kill 掉，或 PORT=8781 ./start_real.sh" >&2
    exit 1
fi

# 上一条留下的 socket 会让新服务端连不上旧服务端，表现是「连上了但不推状态」。
rm -f /tmp/go2_robot.sock

wifi_ip="$(ipconfig getifaddr en0 2>/dev/null || true)"
lan_ip="$(ifconfig "$iface" | awk '/inet /{print $2; exit}')"

echo "[*] 启动网关（preset=real, client=official, 相机 30 Hz）"
echo "    局域网:  ws://${wifi_ip:-<WiFi没连>}:${PORT}/?t=${TOKEN}"
echo "    机器人网: ws://${lan_ip}:${PORT}/?t=${TOKEN}"
echo "    App 里只填地址和端口，token 填 ${TOKEN}"
echo

exec "$PY" -m quadruped.comm.gateway \
    --preset real \
    --client official \
    --host "$BIND" \
    --port "$PORT" \
    --token "$TOKEN" \
    --cam-hz 30 "$@"
