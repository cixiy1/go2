#!/usr/bin/env bash
#
# 在扩展坞上装网关（WebSocket 8780），让机器狗自带服务端。
#
# 架构前提：手机和 Mac 都只是客户端。网关跑在狗身上，DDS 在本地走 eth10，
# 不依赖任何外部电脑。这样「Mac 插不插网线」跟能不能用无关。
#
# 用法（在扩展坞上，仓库已经在 /home/unitree/go2）：
#     sudo ./deploy/install-gateway-dock.sh
#     sudo GO2_IFACE=eth10 GO2_TOKEN=demo ./deploy/install-gateway-dock.sh
#
# 脚本是幂等的，重跑只会覆盖 unit 和 env 文件。
set -eu

REPO="$(cd "$(dirname "$0")/.." && pwd)"
IFACE="${GO2_IFACE:-eth10}"
TOKEN="${GO2_TOKEN:-demo}"
PORT="${GO2_PORT:-8780}"
UNIT=/etc/systemd/system/quadruped-gateway.service
ENVFILE=/etc/go2.env
PIP_INDEX="${PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
warn() { printf '\033[33m[!] %s\033[0m\n' "$*"; }
die() { printf '\033[31m[-] %s\033[0m\n' "$*" >&2; exit 1; }

[ "$(id -u)" = "0" ] || die "要用 root 跑：sudo $0"

say "0. 现场"
echo "  仓库      $REPO"
echo "  python    $(python3 -V 2>&1)"
echo "  架构      $(uname -m)"
echo "  网卡      $IFACE"
ip -4 -br addr show "$IFACE" 2>/dev/null || warn "网卡 $IFACE 上没有 IPv4 地址"
ping -c 1 -W 2 192.168.123.161 >/dev/null 2>&1 \
    && echo "  机器人    ping 通" \
    || warn "ping 192.168.123.161 不通，装完切手动会失败"

say "1. Python 依赖"
python3 -m pip install -q --upgrade pip -i "$PIP_INDEX" 2>/dev/null || true
# websockets 给网关用，numpy/pillow 给感知馈送用，osqp 给控制器的 QP。
python3 -m pip install -q -i "$PIP_INDEX" \
    websockets numpy pillow osqp 2>&1 | tail -3 || warn "部分依赖装失败，看上面的输出"

say "2. unitree_sdk2py"
# 纯 Python（aarch64 的 crc 是预编译 .so，仓库里已经带了），不需要从 GitHub 拉。
if [ -d "$REPO/unitree_sdk2_python/unitree_sdk2py" ]; then
    python3 -m pip install -q --no-build-isolation --no-deps \
        -e "$REPO/unitree_sdk2_python" -i "$PIP_INDEX" 2>&1 | tail -3 \
        || warn "pip 装 SDK 失败，改用 PYTHONPATH（脚本里已经这么兜了）"
else
    warn "仓库里没有 unitree_sdk2_python，跳过"
fi

say "3. cyclonedds（唯一的硬骨头）"
# 情况：PyPI 上没有任何 aarch64 wheel（实测 276 个 wheel 里 0 个），
# sdist 也不含 C 源码 —— 它要求系统里先有一份 CycloneDDS >= 0.10 带头文件。
# 扩展坞自带的是 Unitree 出厂那份，头文件在 dds/ddsrt/ 下（旧布局），
# 所以默认的 pip install cyclonedds 会停在 `idl/retcode.h: No such file`。
if python3 -c "import cyclonedds" 2>/dev/null; then
    echo "  已可导入，跳过"
else
    LIBVER=$(strings /usr/local/lib/libddsc.so* 2>/dev/null \
             | grep -oE '^[0-9]+\.[0-9]+\.[0-9]+$' | sort -V | tail -1 || true)
    echo "  系统 libddsc 版本：${LIBVER:-未识别}"
    if [ -n "${LIBVER:-}" ]; then
        echo "  按版本试装匹配的 Python 绑定…"
        python3 -m pip install -q "$(echo "$LIBVER" | cut -d. -f1,2)" \
            "cyclonedds==$LIBVER" -i "$PIP_INDEX" 2>&1 | tail -4 || true
    fi
    if ! python3 -c "import cyclonedds" 2>/dev/null; then
        warn "cyclonedds 仍然装不上。这一步不解决，网关起不来（它在 import 时就炸）。"
        cat <<'TIP'
  两条出路，选一条：
  A. 装一份 CycloneDDS 0.10.x 的 C 源码（给头文件），编译后：
         export CYCLONEDDS_HOME=/usr/local
         python3 -m pip install cyclonedds
     源码通常从 GitHub 拿；这台机器如果连不上，找一台能连的机器下载
     eclipse-cyclonedds/cyclonedds 的 tar.gz 拷过来。
  B. 确认系统那份 libddsc 的**真实**版本（`ldd --version` 之外，
     看 /usr/local/include/dds/version.h 或者 strings 里的版本串），
     装同一个版本的 Python 绑定。版本必须严格对上，差一个次版本就编不过。
TIP
        die "先解决 cyclonedds，再往下装"
    fi
fi

say "4. systemd 服务"
install -m 600 /dev/null "$ENVFILE"
cat > "$ENVFILE" <<EOF
GO2_IFACE=$IFACE
GO2_GATEWAY_TOKEN=$TOKEN
EOF

cat > "$UNIT" <<EOF
[Unit]
Description=Go2 网关（WebSocket $PORT，客户端是手机/Mac）
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=unitree
WorkingDirectory=$REPO
EnvironmentFile=$ENVFILE
# 网关自己 import 到 DDS，必须能在仓库根目录直接跑起来。
Environment=PYTHONPATH=$REPO:$REPO/unitree_sdk2_python
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/python3 -m quadruped.comm.gateway \\
    --preset real --client official \\
    --iface \${GO2_IFACE} \\
    --host 0.0.0.0 --port $PORT --token \${GO2_GATEWAY_TOKEN} \\
    --cam-hz 30
Restart=on-failure
RestartSec=3
# 网关不写 lowcmd，收尾只是关 socket；给 10 秒够。
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable quadruped-gateway.service >/dev/null
systemctl restart quadruped-gateway.service
sleep 6

say "5. 自检"
systemctl is-active quadruped-gateway.service || {
    journalctl -u quadruped-gateway -n 30 --no-pager
    die "服务没起来，看上面的日志"
}
ss -tlnp 2>/dev/null | grep ":$PORT" || warn "端口 $PORT 没有监听"
echo
echo "  客户端连接地址："
for ip in $(ip -4 -br addr show | awk '$2 ~ /^[0-9]/ {split($2,a,"/"); print a[1]}' | grep -v '^127\.'); do
    echo "    ws://$ip:$PORT/?t=$TOKEN"
done
echo
echo "  验证：curl -s 'http://127.0.0.1:$PORT/' | head -c 80"
