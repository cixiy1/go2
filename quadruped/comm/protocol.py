"""通讯协议：一行一个 JSON，请求与响应各一行。

## 报文

    请求  {"id": 7, "cmd": "move", "args": {"vx": 0.2}}
    响应  {"id": 7, "ok": true, "code": 0, "msg": "", "data": {"vx": 0.2}}

UTF-8，行尾 `\\n`。`id` 由调用方给、原样回传，只用于对日志；单连接串行收发，
不存在乱序，不需要靠它匹配请求。

## 命令表为什么写在这里

参数的取值范围是**安全边界**，不是客户端的礼貌。写在一处、由服务端按它校验，
两个好处：客户端拼错字段会收到 400 和具体字段名，而不是服务端抛 TypeError 后断开；
上下限改了只有一处要动，不会出现「客户端还在发 0.6，服务端早就夹到 0.4」。

上下限的来历：

    vx    ±0.40   search 档声明的 speed_range 上界；再高 stride_max 削顶，落地速度对不上
    vy    ±0.30   横移超过这个值，四条腿的侧向行程不够
    vyaw  ±0.80   和键盘遥控的上限一致
"""

import json

import quadruped.limits as limits

#: 默认套接字路径。放 /tmp 是因为它随重启清空，不会有陈旧文件挡住下一次启动。
DEFAULT_SOCKET = "/tmp/go2_robot.sock"

#: TCP 模式的默认地址。只绑回环，不监听外网 —— 这个接口能让机器狗动起来。
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765

#: 单行上限。超过就断开：正常请求不到 200 字节，64 KiB 只可能来自错帧。
MAX_LINE = 64 * 1024

ENCODING = "utf-8"

# 错误码。前三个是本层自己的，3000 以上是机器人原生的返回码（见 high_level/ours.py）。
OK = 0
BAD_REQUEST = 400
BUSY = 409
UNAVAILABLE = 503
API_UNSUPPORTED = 3104        # 原生语义：这条 api 在这个控制端上不存在


class ProtocolError(ValueError):
    """报文不合法。message 直接回给调用方，所以它得说清改哪个字段。"""


# 命令表：命令 → 参数名 → (类型, 默认值, 下限, 上限)。
#
# 默认值写 None 表示必填；上限下限写 None 表示不做数值校验（字符串型参数）。
# 上下限一律引用 `quadruped.limits`：A 模型裁剪输出用的是同一份数字，两处各写一份
# 就会出现「命令行能发、模型被拒」。
COMMANDS: dict[str, dict[str, tuple]] = {
    # 只确认服务端活着，不碰机器人。
    "ping": {},

    # 位置、姿态、机身高度、回路耗时、当前步态。只读。
    "status": {},

    "stand": {},
    "uncrouch": {},
    "crouch": {"depth": (float, limits.CROUCH, limits.CROUCH_MIN, limits.CROUCH_MAX)},

    # 停车但保持站立，用于「走到一半改主意」。
    "stop": {},

    # 中断当前动作并站定，比 stop 更狠：跳跃、蹲伏都会被它打断。
    "freeze": {},

    # 持续速度指令。发一次就一直走，直到 stop —— 不是「走多少米」。
    "move": {
        "vx": (float, 0.0, -limits.VX_MAX, limits.VX_MAX),
        "vy": (float, 0.0, -limits.VY_MAX, limits.VY_MAX),
        "vyaw": (float, 0.0, -limits.VYAW_MAX, limits.VYAW_MAX),
    },

    # 闭环位移：按 rt/sportmodestate 的位置积分，走够 distance 才返回。
    # 阻塞调用，时长约 |distance| / vx 再加 2 秒。
    "forward": {
        "distance": (float, None, -limits.FORWARD_MAX, limits.FORWARD_MAX),
        "vx": (float, limits.FORWARD_VX, limits.FORWARD_VX_MIN, limits.VX_MAX),
    },

    # 闭环转向：按 IMU 的 yaw 积分。
    "turn": {
        "degrees": (float, None, -limits.TURN_MAX, limits.TURN_MAX),
        "yaw_rate": (float, limits.TURN_RATE, limits.TURN_RATE_MIN, limits.VYAW_MAX),
    },

    "jump": {},

    # 不带 style 就是轮换到下一档；带 style 直接切（只有自研控制端支持指定）。
    # 默认写空串而不是 None：None 的语义是「必填」，这里要的是「可选」。
    "gait": {"style": (str, "", None, None)},

    # 收工：停步 → 蹲下 → 阻尼。服务端随后退出，连接断开。
    "park": {},
}


# 把一行字节解析成 (id, cmd, args)。不合法就抛 ProtocolError，消息可直接回给调用方。
def parse_request(line: bytes) -> tuple[object, str, dict]:
    if len(line) > MAX_LINE:
        raise ProtocolError(f"请求超过 {MAX_LINE} 字节")
    try:
        obj = json.loads(line.decode(ENCODING))
    except UnicodeDecodeError:
        raise ProtocolError("请求不是 UTF-8") from None
    except json.JSONDecodeError as err:
        raise ProtocolError(f"不是合法 JSON：{err.msg}（第 {err.colno} 列）") from None

    if not isinstance(obj, dict):
        raise ProtocolError(f"请求应该是 JSON 对象，实际是 {type(obj).__name__}")

    cmd = obj.get("cmd")
    if not isinstance(cmd, str) or not cmd:
        raise ProtocolError("缺字段 cmd")

    spec = COMMANDS.get(cmd)
    if spec is None:
        hint = ", ".join(sorted(COMMANDS))
        raise ProtocolError(f"没有命令 {cmd!r}，可用：{hint}")

    raw = obj.get("args") or {}
    if not isinstance(raw, dict):
        raise ProtocolError(f"args 应该是对象，实际是 {type(raw).__name__}")

    unknown = sorted(set(raw) - set(spec))
    if unknown:
        allowed = ", ".join(spec) or "（无参数）"
        raise ProtocolError(f"命令 {cmd} 不认参数 {', '.join(unknown)}，可用：{allowed}")

    args: dict = {}
    for name, (kind, default, lo, hi) in spec.items():
        if name not in raw:
            if default is None:
                raise ProtocolError(f"命令 {cmd} 缺必填参数 {name}")
            args[name] = default
            continue
        value = raw[name]
        if kind is str:
            if not isinstance(value, str) or not value:
                raise ProtocolError(f"参数 {name} 应该是非空字符串")
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ProtocolError(f"参数 {name} 应该是数字，实际是 {type(value).__name__}")
            value = float(value)
            if lo is not None and not (lo <= value <= hi):
                raise ProtocolError(f"参数 {name}={value:g} 超出范围 [{lo:g}, {hi:g}]")
        args[name] = value
    return obj.get("id"), cmd, args


# 拼一行响应。data 允许是任何可 JSON 序列化的东西；numpy 标量要先由调用方转 float。
def ok(req_id: object, data: dict | None = None) -> bytes:
    return encode({"id": req_id, "ok": True, "code": OK, "msg": "", "data": data or {}})


# 拼一行错误响应。code 用本层的 4xx/5xx，或机器人原生返回码（如 4201 动作超时）。
def error(req_id: object, code: int, msg: str) -> bytes:
    return encode({"id": req_id, "ok": False, "code": code, "msg": msg, "data": {}})


# 序列化成一行字节。ensure_ascii=False 是为了中文错误消息在 tcpdump 里还能读。
def encode(obj: dict) -> bytes:
    return (json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n").encode(ENCODING)
