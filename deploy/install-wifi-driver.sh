#!/usr/bin/env bash
#
# 在扩展坞上换掉 rtw88，改用厂商驱动 88x2bu。
#
# ## 为什么换
#
# 这块 RTL8822BU（0bda:b82c）走 rtw88 时，只要连上加密 WiFi 就把整机挂死：
#
#     tegra-pmc: ### PMC reset source: BCCPLEXWDT
#
# 日志停在 wpa_supplicant 关联那一刻，之后一行都没有，120 秒后被 AON 看门狗复位。
# 机理见 https://github.com/fschutt/rtw88-usb-wifi-watchdog ：rtw88 在 USB 返回
# -110 时仍持着 wiphy 互斥锁反复重试，用户态全部卡在 D 状态。
#
# 关掉 disable_rx_agg / switch_usb_mode / disable_lps_deep 三个参数都压不住。
# 换厂商驱动后同样的连接稳定，且扫描、关联、取 IP 全正常。
#
# ## 前提
#
# 源码包放在 /tmp/88x2bu-src.tar.gz（从 Mac 传，扩展坞自己的网络不可靠）：
#     curl -L -o 88x2bu.zip \
#       https://ghproxy.net/https://github.com/morrownr/88x2bu-20210702/archive/refs/heads/master.zip
#
# 用法：
#     sudo ./deploy/install-wifi-driver.sh
#
# 回退：删 /etc/modprobe.d/blacklist-rtw88.conf，在源码目录 make uninstall，
#       然后 modprobe rtw_8822bu。
set -eu

SRC_TGZ="${SRC_TGZ:-/tmp/88x2bu-src.tar.gz}"
WORK="${WORK:-/tmp/88x2bu-build}"
KREL="$(uname -r)"
KSRC="/lib/modules/$KREL/build"
BLACKLIST=/etc/modprobe.d/blacklist-rtw88.conf

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }
die() { printf '\033[31m[-] %s\033[0m\n' "$*" >&2; exit 1; }

[ "$(id -u)" = "0" ] || die "要用 root 跑：sudo $0"
[ -d "$KSRC" ] || die "没有内核头 $KSRC，编不了模块"
[ -f "$SRC_TGZ" ] || die "找不到源码包 $SRC_TGZ（见文件头注释里的下载方法）"

say "1. 解包"
rm -rf "$WORK"; mkdir -p "$WORK"
tar xzf "$SRC_TGZ" -C "$WORK" 2>/dev/null || true
DIR=$(find "$WORK" -maxdepth 1 -type d -name "88x2bu-*" | head -1)
[ -n "$DIR" ] || die "解包里没有 88x2bu-* 目录"

say "2. 编译"
# 扩展坞没有联网，make 只用本地的内核头，不下载任何东西。
make -C "$DIR" -j"$(nproc)" KSRC="$KSRC" 2>&1 | tail -4
[ -f "$DIR/88x2bu.ko" ] || die "没编出 88x2bu.ko，看上面的报错"

say "3. 安装"
make -C "$DIR" install KSRC="$KSRC" 2>&1 | tail -3
depmod -a
echo "  → $(find /lib/modules/$KREL -name 88x2bu.ko | head -1)"

say "4. 黑名单 rtw88"
cat > "$BLACKLIST" <<'EOF'
# 换用厂商驱动 88x2bu。rtw88 在这块板子上连加密 WiFi 会整机硬锁死，
# 120 秒后被 Tegra 看门狗复位（PMC reset source: BCCPLEXWDT）。
# 回退：删掉本文件 + 在源码目录 make uninstall，然后 modprobe rtw_8822bu
blacklist rtw_8822bu
blacklist rtw_8822b
blacklist rtw_usb
blacklist rtw_core
EOF
echo "  → $BLACKLIST"

say "5. 切驱动（不重启）"
nmcli radio wifi off 2>/dev/null || true
modprobe -r rtw_8822bu 2>/dev/null || true
modprobe -r rtw_usb 2>/dev/null || true
modprobe -r rtw_core 2>/dev/null || true
modprobe -r rtw_8822b 2>/dev/null || true
sleep 2
modprobe 88x2bu
sleep 4
echo "  已加载: $(lsmod | grep -c '^88x2bu')"
lsusb -t 2>/dev/null | grep -i "rtl88x2bu" | sed 's/^/  /' || true

say "6. 射频开关一次"
# 换驱动后第一次扫描常常返回 0 个 AP（射频没真正起来），关/开一次就好。
nmcli radio wifi on 2>/dev/null || true
sleep 3
nmcli radio wifi off 2>/dev/null || true
sleep 2
nmcli radio wifi on 2>/dev/null || true
sleep 6
echo "  可见 AP: $(nmcli -t -f SSID dev wifi list 2>/dev/null | wc -l) 个"

cat <<'TIP'

注意：这块卡只认 2.4 GHz。iPhone 的「个人热点 → 最大兼容性」必须打开，
否则热点走 5/6 GHz，扩展坞扫不到（表现为 `reason 'ssid-not-found'`）。
TIP
