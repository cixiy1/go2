#!/bin/sh
# 轻量部署：只装网页（总览 / 配网 / 蓝牙），不装 VLA 栈。
#
# 为什么单独一个：`deploy/install.sh` 假定仓库里有 `.venv`（uv sync 装好的
# mujoco / cyclonedds / unitree_sdk2py）。扩展坞上没外网，那一步根本走不通。
# 而这个网页只依赖标准库 —— 系统自带的 python3.9 就够。配网和蓝牙恰恰是
# 「还没接管机器人」时要做的事，那时候本来也用不上 DDS。
#
#   sudo ./deploy/install-web-lite.sh
#   sudo ./deploy/install-web-lite.sh uninstall
#
# 装完「接管」按钮是灰的（页面上写着缺 DDS 依赖），总览/网络/蓝牙正常。
set -eu

APP_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PY="${GO2_PYTHON:-/usr/bin/python3.9}"
SERVICE=quadruped-web
UNIT="/etc/systemd/system/$SERVICE.service"
ENV_FILE=/etc/go2.env

[ "$(id -u)" = "0" ] || { echo "要用 root 跑：sudo $0 ${1:-}" >&2; exit 1; }

if [ "${1:-}" = "uninstall" ]; then
    systemctl disable --now "$SERVICE" 2>/dev/null || true
    rm -f "$UNIT"
    systemctl daemon-reload
    echo "已卸载 $SERVICE。$ENV_FILE 留着，要删自己删。"
    exit 0
fi

[ -x "$PY" ] || { echo "找不到 $PY（用 GO2_PYTHON 指定解释器）" >&2; exit 2; }
echo "解释器：$PY  ($("$PY" -V 2>&1))"

# 先在没有 .venv 的情况下 import 一遍。这一条才是轻量版的全部依赖检查 ——
# 别等到 systemd 起来才发现少了模块，那时候只有 journalctl 里一行 traceback。
if ! "$PY" -c "import sys; sys.path.insert(0, '$APP_DIR'); \
import quadruped.web.app, quadruped.web.sysinfo"; then
    echo "导入失败，先修上面的报错再装。" >&2
    exit 3
fi

if [ ! -f "$ENV_FILE" ]; then
    cat > "$ENV_FILE" <<EOF
# 由 install-web-lite.sh 生成。systemd 用 EnvironmentFile 读它，不进仓库。
# 真机网卡名：扩展坞上是 eth10，用 \`ip -br addr\` 确认。
GO2_IFACE=eth10
GO2_PORT=8080
# 留空 = 每次启动随机生成，打印在 journalctl 里。
GO2_WEB_TOKEN=
EOF
    chmod 600 "$ENV_FILE"
    echo "已写 $ENV_FILE"
fi

# 这两个值在生成 unit 时就展开，不留给 systemd 运行时替换。
# 原因：`EnvironmentFile` 里缺 GO2_PORT 时，`${GO2_PORT}` 会传给 argparse 一个
# 空串，app.py 报 `invalid int value: ''`，然后 Restart=always 每 3 秒重启一次
# —— 扩展坞上就这么空转了 81 次。systemd 245 也不支持 `${VAR:-默认值}` 写法
# （会原样传进去），所以默认值必须在这里给。
WEB_IFACE="${GO2_IFACE:-eth10}"
WEB_PORT="${GO2_WEB_PORT:-8080}"

cat > "$UNIT" <<EOF
[Unit]
Description=Go2 网页控制面（轻量：总览/配网/蓝牙）
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=$APP_DIR
EnvironmentFile=-$ENV_FILE
# 只用系统 python：这个服务不 import DDS，装不装 .venv 都能起。
# token 从 GO2_WEB_TOKEN 读（app.py 的默认值就是它），所以这里不写 --token ——
# 变量为空时命令行会多出一个空参数，argparse 会把它当成下一个选项的值。
ExecStart=$PY -m quadruped.web.app --iface $WEB_IFACE --port $WEB_PORT
Restart=always
RestartSec=3
# 轻量版没有 VLA 子进程要收尾（蹲下/阻尼/交还 mcf 是完整版的事）。
KillMode=control-group
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "$SERVICE" >/dev/null 2>&1
systemctl restart "$SERVICE"
sleep 2
systemctl --no-pager --lines=10 status "$SERVICE" || true

IP="$(ip -br addr 2>/dev/null | awk '/^eth[0-9]+/ && /192\.168\./ {print $3}' | cut -d/ -f1 | head -1)"
PORT="$(sed -n 's/^GO2_PORT=//p' "$ENV_FILE" | tail -1)"
TOKEN="$(sed -n 's/^GO2_WEB_TOKEN=//p' "$ENV_FILE" | tail -1)"
echo
echo "打开：http://${IP:-<本机IP>}:${PORT:-8080}/${TOKEN:+?t=$TOKEN}"
echo "改完 $ENV_FILE 记得 systemctl restart $SERVICE。"
echo "配网/蓝牙需要特权：再跑一次 sudo ./deploy/sudoers.sh"
