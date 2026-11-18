#!/bin/sh
# 给网页进程放行几条只需要 root 的命令。
#
# 为什么非要有这个：网页服务是 systemd 拉起来的，没有登录会话、没有 polkit
# agent。页面上点「连接 Wi-Fi」要写 NetworkManager 的连接配置，点「蓝牙配对」
# 要经 D-Bus 调 BlueZ —— 这两件在那种上下文里都会被拒。与其让整个服务跑成
# root（那这个网页就成了一个人人可用的 root 面板），不如只放行这两条命令。
#
#   sudo ./deploy/sudoers.sh          # 给当前登录用户放行
#   sudo ./deploy/sudoers.sh unitree  # 指定用户名
set -eu

USER_NAME="${1:-${SUDO_USER:-unitree}}"
DST=/etc/sudoers.d/go2-web

if [ "$(id -u)" != "0" ]; then
    echo "要用 root 跑：sudo $0 ${1:-}" >&2
    exit 1
fi

NMCLI="$(command -v nmcli || echo /usr/bin/nmcli)"
BTCTL="$(command -v bluetoothctl || echo /usr/bin/bluetoothctl)"

cat > "$DST" <<EOF
# 由 deploy/sudoers.sh 生成。只放行网页控制面用到的两条命令，
# 不要改成 NOPASSWD: ALL —— 这个网页只靠一个 token 挡着，别给它整机权限。
$USER_NAME ALL=(root) NOPASSWD: $NMCLI
$USER_NAME ALL=(root) NOPASSWD: $BTCTL
EOF

chmod 440 "$DST"

# 语法错了会让 sudo 整体失效，装之前先验一遍。
if ! visudo -c -f "$DST" >/dev/null 2>&1; then
    echo "sudoers 语法检查失败，已删除 $DST" >&2
    rm -f "$DST"
    exit 2
fi

echo "已写入 $DST："
sed 's/^/  /' "$DST"
echo
echo "验证：sudo -n $NMCLI --version && sudo -n $BTCTL --version"
