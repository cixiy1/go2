"""扩展坞自身与网络/蓝牙的运维接口。

这个模块只用标准库和系统命令（nmcli / bluetoothctl / ip / ping），**不碰 DDS、
不 import 控制栈**。理由很实际：配网和蓝牙恰恰要在「还没接管机器人」的时候用，
而扩展坞上的 python3.9 没有装 unitree_sdk2py（也没外网装）。

三块：

    overview()   扩展坞自身：型号/内核/运行时长/负载/内存/磁盘/温度/网口
    net_*()      配网：列接口、扫 Wi-Fi、连接、忘记
    bt_*()       蓝牙：适配器、已配对设备、扫描、连接、断开

需要特权的操作统一走 `_priv()`：先试 `sudo -n`（免密），拿不到就返回一句
「需要在扩展坞上装 sudoers 片段」—— 页面上显示这个，比显示一句没头没尾的
stderr 有用得多。
"""

from __future__ import annotations

import os
import platform
import re
import socket
import subprocess

# 真机主控地址。整个项目里写死这一个值（见 `test/telemetry.py`）。
ROBOT_IP = "192.168.123.161"

# 系统命令的超时。nmcli 连 Wi-Fi 可能要等 DHCP，给宽一点；其它都是本地查询。
T_QUICK = 3.0
T_SCAN = 12.0
T_CONNECT = 30.0


# 跑一条命令，统一返回 (code, out, err)。code 为 None 表示超时或起不来。
#
# 不让异常穿出去：这个服务的调用方是 HTTP 处理器，抛出去就是 500，页面上
# 只看到「连不上」，看不出是 nmcli 不在还是超时。
#
# `LC_ALL` 必须是 UTF-8，不能是 C：设成 C 之后 Python 的 filesystem encoding
# 退化成 ASCII，**中文 SSID 传进 nmcli 会变成 ??????**（实测报
# `No network with SSID '??????XYZ' found`），中文 Wi-Fi 名永远连不上。
# `nmcli -t` 的输出格式本来就不随 locale 变，所以解析照样稳。
def _run(cmd: list[str], timeout: float = T_QUICK) -> tuple[int | None, str, str]:
    try:
        # 参数自己编码成 bytes，输出显式按 UTF-8 解。systemd 拉起的进程
        # 环境里 LANG 可能是空的，那时 Python 的 filesystem encoding 会退到
        # ASCII，中文 SSID 在编码那一步就炸，而不是在 nmcli 那边。
        argv = [c.encode("utf-8") if isinstance(c, str) else c for c in cmd]
        p = subprocess.run(argv, capture_output=True, timeout=timeout,
                           encoding="utf-8", errors="replace",
                           env={**os.environ, "LC_ALL": "C.UTF-8"})
    except FileNotFoundError:
        return None, "", f"找不到命令 {cmd[0]}"
    except subprocess.TimeoutExpired:
        return None, "", f"{cmd[0]} 超时（{timeout:.0f}s）"
    except Exception as err:                                # noqa: BLE001
        return None, "", f"{type(err).__name__}: {err}"
    return p.returncode, p.stdout, p.stderr


# 需要 root 的命令。先免密试，不行就把「该配什么」直接写在返回里。
#
# 扩展坞上 unitree 在 sudo 组，但 sudo 默认要密码；systemd 服务里没有终端，
# 弹不出密码框，只能配 NOPASSWD。`deploy/install.sh` 会写这个片段。
def _priv(cmd: list[str], timeout: float = T_QUICK) -> tuple[int | None, str, str]:
    code, out, err = _run(["sudo", "-n", *cmd], timeout)
    if code is not None:
        return code, out, err
    return None, out, ("需要 sudo 权限但没配免密。在扩展坞执行："
                       "`sudo ./deploy/sudoers.sh`（或给 nmcli/bluetoothctl 配 NOPASSWD）")


# nmcli -t 的字段用 `\:` 转义冒号、`\\` 转义反斜杠。SSID 里带冒号很常见
# （手机热点默认名就可能是这样），不按规则拆会把一行切错位。
def _split_terse(line: str) -> list[str]:
    out, buf, i = [], [], 0
    while i < len(line):
        ch = line[i]
        if ch == "\\" and i + 1 < len(line):
            buf.append(line[i + 1])
            i += 2
            continue
        if ch == ":":
            out.append("".join(buf))
            buf = []
            i += 1
            continue
        buf.append(ch)
        i += 1
    out.append("".join(buf))
    return out


def _read(path: str, default: str = "") -> str:
    try:
        return open(path, encoding="utf-8", errors="replace").read().strip()
    except OSError:
        return default


# -- 扩展坞自身 ---------------------------------------------------------

# 秒 → "3 天 4 小时"。页面上要一眼看出跑了多久，纯秒数没人换算。
def _uptime_text(seconds: float) -> str:
    s = int(seconds)
    d, s = divmod(s, 86400)
    h, s = divmod(s, 3600)
    m, _ = divmod(s, 60)
    if d:
        return f"{d} 天 {h} 小时"
    if h:
        return f"{h} 小时 {m} 分"
    return f"{m} 分"


# Jetson 的型号在设备树里，`nvidia-smi` 在 Orin 上不一定有。
def _board_model() -> str:
    raw = _read("/proc/device-tree/model")
    return raw.replace("\x00", "").strip()


# 温度取最高那个 zone。Jetson 有 cpu/gpu/soc 好几个，逐个列出来太吵，
# 但超温是掉频掉到走不动的原因，所以给个峰值 + 是否过 85℃。
def _thermal() -> dict:
    zones = []
    base = "/sys/class/thermal"
    try:
        names = [n for n in os.listdir(base) if n.startswith("thermal_zone")]
    except OSError:
        names = []
    for n in names:
        t = _read(f"{base}/{n}/temp")
        label = _read(f"{base}/{n}/type")
        if not t:
            continue
        try:
            val = float(t) / 1000.0
        except ValueError:
            continue
        zones.append({"zone": label or n, "c": round(val, 1)})
    zones.sort(key=lambda z: -z["c"])
    return {"zones": zones[:4], "max": zones[0]["c"] if zones else None}


def _mem() -> dict:
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].endswith(":"):
            try:
                info[parts[0][:-1]] = int(parts[1])          # kB
            except ValueError:
                pass
    total = info.get("MemTotal", 0) / 1024.0
    avail = info.get("MemAvailable", 0) / 1024.0
    return {"total_mb": round(total), "used_mb": round(total - avail),
            "used_pct": round((total - avail) / total * 100, 1) if total else None}


def _disk(path: str = "/") -> dict:
    try:
        st = os.statvfs(path)
    except OSError:
        return {}
    total = st.f_blocks * st.f_frsize
    free = st.f_bavail * st.f_frsize
    if not total:
        return {}
    return {"total_gb": round(total / 2**30, 1),
            "free_gb": round(free / 2**30, 1),
            "used_pct": round((total - free) / total * 100, 1)}


def _cpu() -> dict:
    cores = os.cpu_count() or 0
    load = _read("/proc/loadavg").split()
    return {"cores": cores,
            "load1": float(load[0]) if load else None,
            "load5": float(load[1]) if len(load) > 1 else None}


# 网口列表。`ip -br addr` 一行一个，输出稳定，比解析 ifconfig 靠谱。
def ifaces() -> list[dict]:
    code, out, _ = _run(["ip", "-br", "addr"])
    items = []
    if code != 0:
        return items
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        name = parts[0]
        if name == "lo":
            continue
        state = parts[1]
        ips = [p for p in parts[2:] if ":" not in p or p.count(":") > 1]
        ipv4 = [p.split("/")[0] for p in ips if "." in p]
        items.append({"name": name, "state": state,
                      "ipv4": ipv4, "addr": ips[0] if ips else ""})
    return items


# 从本机到机器人主控的往返时间。ICMP 要 raw socket，用 ping 命令最省事。
#
# 这条是「网线还插着吗」的唯一判据 —— 接管失败时先看它，能省掉一轮瞎猜。
def robot_ping(count: int = 2) -> dict:
    code, out, err = _run(["ping", "-c", str(count), "-W", "1", ROBOT_IP],
                          timeout=count * 2 + 2)
    if code == 0:
        m = re.search(r"= [\d.]+/([\d.]+)/", out)
        return {"ok": True, "ip": ROBOT_IP,
                "ms": round(float(m.group(1)), 2) if m else None}
    return {"ok": False, "ip": ROBOT_IP, "ms": None, "err": err.strip()[:120]}


# 总览用的一个包。故意分块，页面按块渲染，缺哪块显示「—」而不是整页失败。
def overview() -> dict:
    up = _read("/proc/uptime").split()
    boot = float(up[0]) if up else 0.0
    osrel = {}
    for line in _read("/etc/os-release").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            osrel[k] = v.strip('"')
    return {
        "board": _board_model() or "未知",
        "host": socket.gethostname(),
        # 没有 /etc/os-release 的环境（macOS 上跑调试）不要写死成 Linux ——
        # 那会在页面上一本正经地报错型号，比显示 "–" 更糟。
        "os": osrel.get("PRETTY_NAME") or platform.platform(terse=True),
        "kernel": os.uname().release,
        "arch": os.uname().machine,
        "python": "%d.%d.%d" % __import__("sys").version_info[:3],
        "uptime": _uptime_text(boot),
        "uptime_s": round(boot),
        "cpu": _cpu(),
        "mem": _mem(),
        "disk": _disk("/"),
        "thermal": _thermal(),
        "ifaces": ifaces(),
        "robot": robot_ping(),
        "wifi": wifi_device(),
        "bt": bt_summary(),
    }


# -- 配网 ---------------------------------------------------------------

# 系统里有没有无线网口。扩展坞那块 RTL8822BU 现在没驱动，`nmcli` 里根本没有
# wlan 设备 —— 页面必须能把这件事说清楚，否则用户会以为是密码错了。
def wifi_device() -> dict:
    code, out, _ = _run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "dev", "status"])
    if code != 0:
        return {"present": False, "device": None, "reason": "nmcli 不可用"}
    for line in out.splitlines():
        f = _split_terse(line)
        if len(f) >= 3 and f[1] == "wifi":
            return {"present": True, "device": f[0], "state": f[2]}
    return {"present": False, "device": None,
            "reason": "系统里没有无线网口（USB 网卡可能缺驱动，见 dmesg | grep rtl）"}


# 接口 + 已保存的连接。
def net_status() -> dict:
    code, out, _ = _run(["nmcli", "-t", "-f", "NAME,DEVICE,TYPE,STATE", "con", "show"])
    saved = []
    if code == 0:
        for line in out.splitlines():
            f = _split_terse(line)
            if len(f) >= 4:
                saved.append({"name": f[0], "device": f[1],
                              "type": f[2], "state": f[3]})
    return {"ifaces": ifaces(), "wifi": wifi_device(), "saved": saved}


# 解析 `nmcli -t -f IN-USE,SSID,SIGNAL,SECURITY dev wifi list` 的输出。
# 同一个 SSID 会有多条（2.4G / 5G 各一个 BSSID），只留信号最强的那条。
def _parse_aps(out: str) -> list[dict]:
    seen, aps = {}, []
    for line in out.splitlines():
        f = _split_terse(line)
        if len(f) < 4:
            continue
        in_use, ssid, signal, sec = f[0], f[1], f[2], f[3]
        if not ssid:
            continue
        try:
            sig = int(signal)
        except ValueError:
            sig = 0
        old = seen.get(ssid)
        if old is not None:
            old["signal"] = max(old["signal"], sig)
            old["active"] = old["active"] or in_use.strip() == "*"
            continue
        item = {"ssid": ssid, "signal": sig, "security": sec or "开放",
                "active": in_use.strip() == "*"}
        seen[ssid] = item
        aps.append(item)
    aps.sort(key=lambda a: -a["signal"])
    return aps


# 扫一遍周围的 Wi-Fi。`--rescan yes` 会真的触发一次扫描，慢（3~8 秒），
# 所以这个接口不适合轮询 —— 页面上是「点一下扫」。
def wifi_scan() -> dict:
    wifi = wifi_device()
    if not wifi["present"]:
        return {"ok": False, "why": wifi.get("reason") or "没有无线网口", "aps": []}
    code, out, err = _run(
        ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "dev", "wifi", "list",
         "--rescan", "yes"], timeout=T_SCAN)
    aps = _parse_aps(out) if code == 0 else []
    if not aps:
        # 接口正忙（上一次连失败、或扫描被别的东西占用）时，`--rescan yes`
        # 会安静地返回一张空表。这时候直接报「周围没有网络」是骗人 ——
        # 再读一次缓存，只有缓存也空才这么说。
        code2, out2, err2 = _run(
            ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "dev", "wifi", "list"],
            timeout=T_QUICK)
        if code2 == 0:
            aps = _parse_aps(out2)
        if not aps:
            return {"ok": False, "aps": [],
                    "why": (err or err2 or "扫描没有返回结果，接口可能正忙，再点一次").strip()[:160]}
    return {"ok": True, "aps": aps, "device": wifi["device"]}


# 连一个 Wi-Fi。成功时 nmcli 会返回 0 并写出连接；失败原因（密码错、超时）
# 原样带回页面，别翻译成「连接失败」—— 那把人往错的方向带。
def wifi_connect(ssid: str, password: str = "", hidden: bool = False) -> dict:
    ssid = (ssid or "").strip()
    if not ssid:
        return {"ok": False, "why": "SSID 不能为空"}
    if len(ssid) > 32:
        return {"ok": False, "why": "SSID 最长 32 字节"}
    wifi = wifi_device()
    if not wifi["present"]:
        return {"ok": False, "why": wifi.get("reason") or "没有无线网口"}
    cmd = ["nmcli", "dev", "wifi", "connect", ssid]
    if password:
        cmd += ["password", password]
    if hidden:
        cmd += ["hidden", "yes"]
    code, out, err = _priv(cmd, timeout=T_CONNECT)
    if code == 0:
        return {"ok": True, "out": out.strip()[:200]}
    return {"ok": False, "why": (err or out).strip()[:200] or "连接失败"}


# 忘记一个已保存的连接（下次不再自动连）。
def wifi_forget(name: str) -> dict:
    name = (name or "").strip()
    if not name:
        return {"ok": False, "why": "连接名不能为空"}
    code, out, err = _priv(["nmcli", "con", "delete", name])
    if code == 0:
        return {"ok": True}
    return {"ok": False, "why": (err or out).strip()[:160]}


# -- 蓝牙 ---------------------------------------------------------------

# bluetoothctl 在非交互模式下的输出五花八门，这里只认几种固定行：
#   Controller 90:DE:80:31:D4:12 ubuntu [default]
#   Powered: yes
#   Device AA:BB:CC:DD:EE:FF 名字
#   Paired: yes / Connected: yes
CONTROLLER_RE = re.compile(r"^Controller\s+([0-9A-F:]{17})\s+(.*)$", re.M)
DEVICE_RE = re.compile(r"^Device\s+([0-9A-F:]{17})\s+(.*)$", re.M)


def bt_summary() -> dict:
    code, out, err = _run(["bluetoothctl", "show"], timeout=T_QUICK)
    if code is None:
        return {"present": False, "reason": err or "bluetoothctl 不可用"}
    m = CONTROLLER_RE.search(out)
    if not m:
        return {"present": False, "reason": "没有蓝牙控制器"}
    powered = re.search(r"^\s*Powered:\s*(\w+)", out, re.M)
    return {"present": True, "mac": m.group(1), "name": m.group(2).strip(),
            "powered": (powered.group(1) == "yes") if powered else None}


# 已配对设备。`devices Paired` 只列配对过的，比全量列表短得多。
def bt_devices() -> list[dict]:
    code, out, _ = _run(["bluetoothctl", "devices"], timeout=T_QUICK)
    if code != 0:
        return []
    items = []
    for line in out.splitlines():
        m = DEVICE_RE.match(line.strip())
        if not m:
            continue
        mac, name = m.group(1), m.group(2).strip()
        info = _run(["bluetoothctl", "info", mac], timeout=T_QUICK)[1]
        items.append({
            "mac": mac, "name": name or "(未命名)",
            "paired": "Paired: yes" in info,
            "connected": "Connected: yes" in info,
            "trusted": "Trusted: yes" in info,
        })
    items.sort(key=lambda d: (not d["connected"], not d["paired"], d["name"]))
    return items


# 扫一遍。`--timeout` 是 5.4+ 的用法，扩展坞上是 5.53，可用。
# 扫描会阻塞到这个秒数结束 —— 页面上给按钮 + 转圈，不要轮询。
def bt_scan(seconds: float = 8.0) -> dict:
    bt = bt_summary()
    if not bt["present"]:
        return {"ok": False, "why": bt.get("reason") or "没有蓝牙控制器", "devices": []}
    secs = max(3.0, min(20.0, float(seconds)))
    code, out, err = _run(["bluetoothctl", "--timeout", str(int(secs)), "scan", "on"],
                          timeout=secs + 6)
    if code is None:
        return {"ok": False, "why": err, "devices": []}
    return {"ok": True, "devices": bt_devices()}


# connect / disconnect / trust / remove。白名单，不接受任意子命令。
BT_ACTIONS = ("connect", "disconnect", "trust", "remove", "pair")


def bt_action(action: str, mac: str) -> dict:
    if action not in BT_ACTIONS:
        return {"ok": False, "why": f"action 只能是 {'/'.join(BT_ACTIONS)}"}
    mac = (mac or "").strip().upper()
    if not re.fullmatch(r"[0-9A-F]{2}(:[0-9A-F]{2}){5}", mac):
        return {"ok": False, "why": "MAC 格式不对（AA:BB:CC:DD:EE:FF）"}
    code, out, err = _run(["bluetoothctl", action, mac], timeout=T_CONNECT)
    if code == 0:
        return {"ok": True, "out": out.strip()[:200]}
    return {"ok": False, "why": (err or out).strip()[:200] or f"{action} 失败"}
