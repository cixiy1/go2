import os

ROBOT = "go2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
# 场景路径可以用 SIM_SCENE 覆盖，用来换物理模型做对照（比如用 go2_urdf.xml
# 跑训练时的 URDF 参数）。不给就用默认的 go2/scene.xml。
ROBOT_SCENE = os.environ.get("SIM_SCENE", "../unitree_robots/" + ROBOT + "/scene.xml") # Robot scene
DOMAIN_ID = 1 # Domain id
# macOS: 回环接口叫 lo0，Linux 才是 lo。写 "lo" 会让 CycloneDDS 报
# "does not match an available interface"，和之前 en5 那次是同一类错误。
# 上真机时改成 "en5" 并把 DOMAIN_ID 改回 0。
INTERFACE = "lo0" # Interface

# 没接手柄必须设 0，否则 SetupJoystick 会抛 pygame.error 直接中断。
USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

# 物理步长。200 Hz。试过改成 0.002（500 Hz，和策略训练一致），原地转向反而
# 从 0.13 掉到 0.08 rad/s —— 物理更细但 lowcmd 还是 50 Hz 更新，多出来的步
# 只是空转，还把 CPU 挤掉了。别再改这个数指望转向变快。
SIMULATE_DT = 0.005
VIEWER_DT = 0.02  # 50 fps for viewer

# ---- 真机特性开关 ----------------------------------------------------
# 关掉它就是理想仿真：指令当帧生效、传感器无噪声。真机上跑的代码在理想仿真
# 里调好了，一上机就抖 —— 因为真机有三样仿真默认没有的东西：力矩晚几毫秒
# 才到、编码器读数带噪声、力控带宽有限。打开后仿真才和真机对得上，
# 调出来的参数也才有意义。
#
# 每一项都能用环境变量单独关掉做对照（写调试脚本时很有用）：
#   SIM_REALISM=0    全关，退回理想仿真
#   SIM_LATENCY=0    只关延迟
#   SIM_NOISE=0      只关编码器噪声
#   SIM_BW=0         只关力控带宽
import os


def _envf(key, default):
    try:
        return float(os.environ[key])
    except (KeyError, ValueError):
        return default


REALISM = _envf("SIM_REALISM", 1.0) != 0.0

# 指令到力矩的总延迟（秒）。真机链路：CycloneDDS 收发 + 电机驱动电流环，
# 实测 3~6 ms。这里按物理步长取整，5 ms 刚好一帧。
REALISM_LATENCY = _envf("SIM_LATENCY", 0.005)
# 编码器角度/速度噪声（1σ）。角度是 14 位绝对编码器，很小；速度是差分出来
# 的，噪声大一个量级 —— kd 乘上去就是高频力矩噪声，真机啸叫主要来自这里。
REALISM_Q_NOISE = _envf("SIM_Q_NOISE", 0.0015)      # rad
REALISM_DQ_NOISE = _envf("SIM_DQ_NOISE", 0.15)      # rad/s
# 力矩指令的跟随带宽（Hz）。Go2 电机驱动器是电流环，标称带宽 1 kHz 以上，
# 200 Hz 的指令周期根本压不到它 —— 这里给 150 Hz 只是留一点相位滞后，
# 不是主要瓶颈。别再往低里调：那等于把整条 PD 环降增益，腿会软得不像话。
REALISM_TAU_BW = _envf("SIM_BW", 150.0)

# ---- 感知发布（V 层）--------------------------------------------------
# 往 127.0.0.1 发激光扫描，给 `quadruped/percept/` 用。关掉不影响运动控制，
# 只是 V 层退回纯本体观测。
ENABLE_PERCEPT = os.environ.get("SIM_PERCEPT", "1") != "0"
# 10 Hz：射线本身是瞬时的，这个频率由「障碍以 0.4 m/s 接近时，一帧走 4 cm」
# 定 —— 比这更慢，安全闸拿到的是上一帧的旧距离。
PERCEPT_HZ = _envf("PERCEPT_HZ", 10.0)
