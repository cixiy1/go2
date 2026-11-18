#!/bin/sh
# 把网页控制面装成开机自启的 systemd 服务。在扩展坞上以 root 跑。
#
#   sudo ./deploy/install.sh            # 装（幂等，重跑就是升级）
#   sudo ./deploy/install.sh --uninstall
#
# 只做四件事：写 /etc/go2.env（已经有了就不动）、写 unit、daemon-reload、enable。
# 不碰仓库里的代码，不动网卡配置，不改 DDS 域号 —— 那些错了要能一眼看出来是人为的。
set -eu

APP_DIR="$(cd "$(dirname "$0")/.." && pwd)"
UNIT=/etc/systemd/system/quadruped-web.service
ENV_FILE=/etc/go2.env
SERVICE=quadruped-web

if [ "$(id -u)" -ne 0 ]; then
    echo "要 root：sudo $0" >&2
    exit 2
fi

if [ "${1:-}" = "--uninstall" ]; then
    systemctl disable --now "$SERVICE" 2>/dev/null || true
    rm -f "$UNIT"
    systemctl daemon-reload
    echo "已卸载服务。$ENV_FILE 留着，要删自己删。"
    exit 0
fi

# 解释器：优先仓库里的 venv —— 依赖（mujoco / cyclonedds / unitree_sdk2py）都装在那里。
PY="$APP_DIR/.venv/bin/python"
if [ ! -x "$PY" ]; then
    echo "找不到 $PY。先在这个目录跑 uv sync（或 python -m venv .venv && pip install -e .）" >&2
    exit 2
fi
echo "解释器：$PY  ($("$PY" -V 2>&1))"

# Python 3.8 会在这里炸：几个文件用了 `X | None` 这种注解，没有
# `from __future__ import annotations` 的会直接 SyntaxError。先 import 一遍，
# 别等到 systemd 起来才发现。
if ! "$PY" -c "import quadruped.web.app, quadruped.control.embodied.vla"; then
    echo "导入失败，先修上面的报错再装。" >&2
    exit 3
fi

if [ ! -f "$ENV_FILE" ]; then
    install -m 600 "$APP_DIR/deploy/go2.env.example" "$ENV_FILE"
    echo "已写 $ENV_FILE —— 里面 GO2_IFACE 默认 en5，用 ip -br addr 确认一下。"
else
    echo "$ENV_FILE 已存在，不动它。"
fi

sed -e "s#/opt/go2#$APP_DIR#g" "$APP_DIR/deploy/quadruped-web.service" > "$UNIT"
chmod 644 "$UNIT"

systemctl daemon-reload
systemctl enable "$SERVICE"
systemctl restart "$SERVICE"
sleep 1

# 端口从 env 里读不到就在这里写死默认值，只影响提示不影响到服务。
PORT="$(sed -n 's/^GO2_PORT=//p' "$ENV_FILE" | tail -1)"
PORT="${PORT:-8080}"
TOKEN="$(sed -n 's/^GO2_WEB_TOKEN=//p' "$ENV_FILE" | tail -1)"

echo
systemctl --no-pager --lines=12 status "$SERVICE" || true
echo
echo "页面：http://<这块网卡的 IP>:$PORT/?t=${TOKEN:-<看 journalctl 里那 5 位>}"
echo "日志：journalctl -u $SERVICE -f"
echo
echo "改完 $ENV_FILE 记得 systemctl restart $SERVICE。"
