"""L 层：看着 V 层的观测做决定，并且把理由说出来。

和 `service.py` 的区别不只是「多了一段思考」。`TextService` 是「一句话进来，出
一条命令」的翻译器，没有输入时它什么都不发生；`Thinker` 是「每拍看一次环境，
判断要不要重新想」。前者由人驱动，后者由状态驱动。

## 频率

不是固定一个数，是三件事叠在一起：

    循环节拍   0.5 s   读感知、查安全闸、续期命令。这些都不调模型。
    最小间隔   1.0 s   两次推理之间至少隔一秒。低于这个，上一句还没执行完
                       就被下一句覆盖，狗会在原地反复改主意。
    自驱       连想着   想完就排下一轮，间隔只受「最小间隔」和生成耗时约束。
    空闲上限   6.0 s   非自驱时，没有新指令、环境也没变，最多这么久想一次。

真正决定「多久想一次」的是触发条件，不是频率参数：

    用户说了新话          立即（< 0.5 s 进循环）
    前方 0.6 m 内出现障碍  立即
    倾角越过 20°          立即
    前方距离变化超过 0.4 m 立即
    视觉模型给了新描述      立即（V 层只在画面真的变了时才说新话）
    自驱（没人说话）        上一轮想完立刻排下一轮，间隔只受 1 Hz 下限
    其它                  等到空闲上限

被句执行（有人说话）时几乎不调模型，接近障碍时反应是半个循环节拍。

自驱是另一回事：没有人、也不一定有事件，只靠「被叫醒」的话它大部分时间只是在
重发上一条命令，看上去就是不推不动。所以每一轮想完就把下一轮预约上（`_chain`），
不等事件也不等定时器 —— 节奏由生成耗时和 `min_think`（1 Hz）决定，实测约 0.6 Hz。
转向期间这个钩子不生效，转完再接着想。

## 安全和它的关系

`Guard` 独立于这一层跑。Thinker 的每一条命令都要过 `guard.filter()`，模型说
「冲过去」也不会真的冲过去。闸触发时 Thinker 会强制重想一次，否则下一拍会把
旧的 vx 续回去，等于绕过闸门。
"""

from __future__ import annotations

import math
import re
import threading
import time
import traceback
from collections import deque

from quadruped.llm.inbox import Inbox
from quadruped.llm.model import DEFAULT_MODEL, make
from quadruped.llm.remote import Cancelled
from quadruped.llm.prompt import EMIT_THOUGHT, build_think
from quadruped.llm.schema import validate
from quadruped.limits import GAP_TRIGGER, TURN_START_GAP
from quadruped.policy import training as T
from quadruped.rates import THINK_MAX_HZ, THINK_TICK_HZ

# 触发重想的阈值。改这三个数就是在改「L 层有多敏感」，和频率参数是两回事。
# `GAP_TRIGGER` 在 `limits.py`，和 G 层的 `BLOCK_GAP`、提示词里的那句
# 「不到 1.2 米就转」共用同一个数。
GAP_DELTA = 0.4          # 前方距离突变多少算「环境变了」
TILT_TRIGGER = 20.0      # 倾角超过这个角度

# ---- 转向：一次命令 = 一个转到位为止的动作 -------------------------------
#
# 策略原地最大约 0.35 rad/s（`test/turn_sweep.py` 实测：pronk + vyaw 1.0，
# 换步频和抬脚只差 0.04，vyaw 再大就超出训练区间会摔）。转 90° 需要 4.5 秒，
# 而 `MotionService` 的命令 3 秒就过期 —— 光发一条命令，永远差最后 30°。
#
# 更麻烦的是下一拍会重想：模型看到「还是被挡」可能换个方向，前一次转的角度
# 白扔。日志里那一分钟原地摆，就是这个循环。
#
# 所以 vyaw 按「角度」用，不按「速度」用：模型说转多少度，这里闭环转到那个
# 角度，中途不重想，到了才把控制权交回去。角度是模型给的，不是这里定的 ——
# 之前写过 `TURN_ANGLE = 90°` + 「|vyaw| >= 0.5 就转 90°」，等于把一个模型
# 从没要求过的角度塞进它的命令里，模型想转 20° 避一下也办不到。
TURN_TOL = 0.05          # 差 3° 以内算到位
# 实测转 90° 现在 2.1 s（`TURN_SPEED=4.0`，trot，`test/turn_overshoot.py`）。给
# 20 s 是给「顶墙转不动」「打滑」留的余量：真卡住才放弃，不因为估得不准就判失败。
TURN_TIMEOUT = 20.0
TURN_SPEED = 4.0         # 转向时给的角速度幅度（`walker.COMMAND_RANGE` 的上限）

# 转向期间的闭环周期。33 Hz。
#
# 这一条是「总是转过头来回转」的直接原因。闭环原来挂在 L 层的 2 Hz 节拍上：
# 策略转速 0.2 rad/s，半秒就是 5.7°，而到位判定 4.6° —— 几乎每一拍都压过目标，
# 误差反号，下一拍往回转，再压过，来回摆。转向本身就是个低带宽执行器，2 Hz 的
# 控制周期对它来说太粗了。
#
# 从 20 Hz 提到 33 Hz 是因为 `TURN_SPEED` 放到了 4.0：trot 在 vyaw=4.0 时实测
# 0.7~0.9 rad/s，20 Hz 每拍转 2.5°，和到位判定 4.6° 是同一量级，收尾会冲过头。
# 33 Hz 是 1.5°/拍。
#
# 只缩短睡眠间隔，不加快推理：转向期间 `_should_think` 早就在 `_turn is not None`
# 那里返回 None，模型一次都不会被多叫。
TURN_TICK = 0.03

# 剩得少就转慢一点：每弧度误差给 16 的角速度（差 14° 就给满 4.0）。
#
# 这一项直接决定转 90° 要多久，实测（`test/turn_overshoot.py`，trot）：
#
#     GAIN   用时    到位误差   收尾过冲
#       8    4.8s     -2.7°      +0.2°
#      12    3.2s     -2.7°      +0.3°
#      16    2.4s     -2.8°      +0.5°
#      22    2.2s     -1.6°      +0.8°
#
# 收尾那段是瓶颈：误差 35° 以内命令跟着 err×GAIN 一起降，最后 20° 要磨两秒。
# 取 16 —— 再大只快 0.2 s，过冲却翻倍。`TURN_FLOOR` 是下限：命令太小策略不
# 响应（机身不动、误差也不变），会一直耗到超时。
TURN_GAIN = 16.0
TURN_FLOOR = 0.35

# 目标点近于这个距离就不报方位角。0.4 m 到脚下时，横向差 0.1 m 就是 14°，那个
# 「偏 42 度」是噪声不是意图。和 `drive.REACHED`（0.6）配套：那边负责早点换点，
# 这边负责在换点之前不要把噪声当方向。
#
# 0.6 是原来取的值，2026-11-17 提到 1.0。原因是日志里那几条
# 「向右转 174 度，0.7 米外」「向右转 166 度，1.3 米外」——1 米出头给 170° 的
# 方位，模型照着转就是掉头，而它的物理含义是「那个点在我身后偏左一点点」，
# 在 0.7 m 的量级上跟「正后方」没有区别。1 m 内不给方位，让它按走过的路继续走。
AIM_BLIND = 1.0

# 角速度反馈：命令里减掉 `TURN_DAMP × wz`。
#
# 只按角度闭环、不压角速度的话，「到了」的那一刻机体还在转，切断命令后靠地面
# 摩擦慢慢停，还会多转一点。MuJoCo 上量（`test/turn_overshoot.py`，转 90°）：
# 到位误差 -2.9°，收尾又转 +2.1°，来回幅度 2.4°；加上角速度反馈之后是
# -1.8° / +0.2° / 0.8°。绝对值不大，但模型看到的就是这 2° 加上走路的 1~2°，
# 两三个小角度攒起来就会补一次转向。
#
# 注意别把这项当成「走路会偏」的解药：平地直行 20 s 只漂 0.9°
# （`test/heading_hold.py`），trot 的航向是稳的，需要压的只有转向收尾这一段。
TURN_DAMP = 1.2
# 角速度低于这个值才算真的停住了。阈值取得比转向时的角速度小一个量级
# （实测转向中 0.1~0.2 rad/s）：0.06 rad/s 转一秒是 3.4°，再往下压没有意义，
# IMU 噪声就在这个量级。
TURN_WZ_STILL = 0.06
# 提前制动的预测时间：现在角速度是 wz，从发反推到真正停住大约要这么久，
# 这段时间还会再转 wz × TURN_LEAD。0.15 s 在 0.74 rad/s 下就是 6.4°。
# 实测（`test/turn_overshoot.py`，转 90°），三档对比：
#
#     0     用时 2.4 s   到位 -2.8°   收尾 +0.4°   来回 0.9°
#     0.15  用时 2.8 s   到位 -2.7°   收尾 +0.0°   来回 0.6°
#     0.25  用时 3.3 s   到位 -2.7°   收尾 -0.0°   来回 0.6°
#
# 取 0.15：收尾过冲已经归零，只比原来慢 0.4 s。0.25 再多花 0.5 s 换不到东西。
TURN_LEAD = 0.15
# 制动阶段的上限。超过就不再等，按当前角度交付 —— 顶着墙转不动时角速度会一直
# 是 0，不会进这一段；只有真在转的时候才会等，所以正常几秒内就结束。
TURN_SETTLE = 1.5
# 转向时顺带小步前进。trot 在 vyaw=4.0 下实测原地 0.74 rad/s、带 0.15 前进
# 0.90（+22%）。
#
# 原来取 0.15，理由是「0.30 会把原地转变成画圈」。0.15 在日志里的实际效果是
# **几乎不位移**：`Guard.filter` 在正前 < 1.6 m 时把 vx 减半，0.15 变 0.075，
# 转 4 秒只有 0.3 m。2026-11-15 那段 `12.2s → 28.4s` 的日志里，16 秒转了 330°、
# 净位移 0.22 m，vx 一直是 +0.07 —— 0.15 减半就是 0.07。
#
# 取 0.25（减半后 0.12，4 秒 0.5 m，转 90° 的圈半径 0.34 m）。走廊半宽 0.9 m，
# 这个半径转过去不会顶对面墙。
TURN_VX = 0.25
# 正前有这么多空间才给上面那个 vx。
#
# 原来写 1.0（「几乎从不前进」），改 0.75 让转向时能挪一点。**0.75 太低。**
# 2026-11-17 迷宫实测，一次 +170° 掉头在 0.56 m 处开始，过程中正前从 0.56
# 一路掉到 0.35：狗一边转身（机头扫过 180°）一边以 vx=0.12 往前蹭，蹭到
# 相机里只剩一堵墙。判据用的是「当前正前」，但转身会扫过整圈，当前开阔
# 不代表扫过去的那片开阔。
#
# 1.2 之后：窄走廊里（正前 < 1.2 m）纯原地转，只在真开阔时才带前进。
# 代价是掉头慢一点（原地 0.74 rad/s vs 带前进 0.90），换不掉墙。
TURN_VX_CLEAR = 1.2

# ---- 一次转向之后必须先走出距离 ---------------------------------------------
#
# 2026-11-15 那段日志的账（12.2 s → 28.4 s，迷宫东侧）：
#
#     朝向 +5°  → -41° → -101° → +139° → +67° → +70°
#     16 秒转了 330°，转到位 4 次，位置 (+3.5,+0.3) → (+3.3,+0.2)
#
# 每一次都是「转到位」触发重想 → 新朝向正前还是墙（迷宫 1.2 m 处必有墙）→
# 模型再挑一个方位转。中间的 `TURN_COOL = 1.0` 是**时间**冷却，而那时 vx 只有
# 0.07 m/s，1 秒等于 7 cm —— 冷却等于没有。而且它只挡反向，那五次转向里有四次
# 同向，全被放行。
#
# 改成按**位移**：转到位后必须累积走 `TURN_MOVE` 米，才接受下一次转向。
# `TURN_MOVE_TIME` 是兜底 —— 正前 0.7 m 时安全闸把 vx 压到 0.1，走 0.5 m 要
# 十几秒，光看位移会锁死。超过这个时间就放行。
TURN_MOVE = 0.5
# 兜底时间。原来是 4 s：正前贴着墙、vx 被压到 0.18 时 4 秒走不了 0.5 m，于是
# 每次都被兜底放行 —— 等于「转完马上又能转」。2026-11-16 迷宫实测 120 秒 19 次
# 转向，加上 `MIN_VX` 之后 4 秒能走 1.2 m，兜底不再是主路径，拉到 6 s 让位移条件
# 先起作用。
TURN_MOVE_TIME = 6.0
# 正前低于这个距离时无视位移冷却。真的钻进死角了，转是唯一出路。
#
# 原来取 0.6，和「贴上了」一个量级。但 `TURN_MOVE` 要的是「走出 0.5 m」，
# 而安全闸在正前 < 1.6 m 就开始压速度 —— 正前 1.0 m 时只有 0.3~0.4 m/s，
# 走 0.5 m 要 1.5 s，中间还要被模型每一拍重新请求一次转向。
#
# 2026-11-15 第三段日志：175 秒里被这条挡了 54 次，几乎每 3 秒一次，而
# `TURN_MOVE_TIME` 是 4 秒 —— 等于每次都是「挡到超时再放行」。
#
# 取 1.2，等于说：正前 1.2 m 以上（走得动）才要求先走出去；1.2 以下本来就
# 走不快，选方向比硬走有用。
TURN_CLEAR_OUT = 1.2

# 正前有这么多空间就不许「为了走得更宽」而转。
#
# 原来是 `GAP_TRIGGER`（1.8 m），依据是「L 层把 1.8 m 以上当通畅」。**这个门槛
# 在迷宫里从不成立**：三行走廊的正前常态是 0.7~1.5 m，1.8 一次也够不着。
#
# 后果是整条闸形同虚设 —— 2026-11-17 的迷宫日志里 `grep "能走，不转"` 是 **0 次**，
# 而模型 22 次转向、累计 1692°、跑了 30.9 m 只挪出 3.9 m。它每次的依据都是
# 「正前 1.4 米有墙，右侧 2.7 米最宽，右转走宽处」，转 60°，走 0.5 m，再转。
#
# 1.2 是和 G 层对齐的数：`memory.guide(reach=1.6)` 会建议「1.3 米外，直线能走」，
# 而 `MIN_FRONTIER = 1.0` 说明 1.0 m 以上就算有前沿。执行层却要 1.8 m 才认，
# 中间 0.6 m 的空档就是模型每次都能转过去的缝。
#
# 1.2 之下（真贴墙）照旧放行 —— 那时候转身是唯一出路。
#
# 只拦 15°~90° 这一段：小角度本来就归 `TURN_MIN_DEG`，大角度掉头是 G 的要求
# （目标点在身后），不拦。
TURN_FREE_GAP = TURN_START_GAP

# 目标点偏到这个角度以内，「正前能走就别转」才生效。
#
# 偏得更多说明 G 要它横移（贴墙了要回到走廊中间），那不是「为走得更宽而转」。
#
# **但这条只管一种情况：目标在正前方、模型还想转身走开。** 判据不能写成
# 「目标必须在 ±15° 内，否则不管」—— 那样「目标在左前 31°、模型却右转 45°」
# 这种明显背道而驰的动作也会被放行，2026-11-17 的日志里第一次转向就是这样：
#
#     [G] 目标点 (+3.8, +1.3)，0.8 m，左前（+31°）
#     触发：前方出现障碍（1.49 m）
#     想法：正前1.49米有墙，左侧贴墙，右侧开阔，右转绕开
#     [L] 开始转向 -45°
#
# 前方 1.49 m 够走，目标是左前方，模型偏要往右 —— 闸因为 31 > 15 放行了。
# 所以下面 `_turn` 里的条件多一支：**模型转的方向和目标的方位相反时也拦**。
TURN_AIM_NEAR = 15.0

# 一次转向的幅度上限。超过这个角度要掉头，而模型看不见身后：它转 120° 的依据
# 只是某条射线足够长，那可能是转身途中路过的一条窄缝。
#
# 实测（同一段日志，18.2 s）：转 -120° 花了 4.0 s，转到位正前 1.31 m，
# 而转身前 22.3 s 那一拍正前是 3.0 m。转了一圈，能走的空间反而变少了。
# 目标点在正前方时把幅度收到 90°；目标点本来就在身后时放行（那是 G 的要求）。
TURN_MAX_DEG = 90.0

# ---- 原地打转闸 --------------------------------------------------------------
#
# 2026-11-15 迷宫那一次的账：90 秒、36 次推理、累计转过 880°、转到位 15 次，
# 净位移 2.41 m。日志里是一串同向转：-15° → -27° → -30° → -25° → -60°，每一拍
# 的理由都是「右前最开阔，往宽处走」—— 而正前那时有 1.5~2.5 米。1.8 米宽的
# 走廊里两侧各 0.85 米，它每拍都觉得「这边不够宽，往那边」，转到最后一圈回来。
#
# 这条不判断「该不该转」，只判断「转过之后有没有换来位移」：窗口内转了超过
# `SPIN_DEG` 却挪不到 `SPIN_MOVE` 米，说明这一串转向白转了，下一拍只许走不许转。
#
# 只在正前 >= `SPIN_CLEAR` 时生效：前面是墙的时候转是唯一的出路，拦了就是死锁。
#
# 参数调过一次（2026-11-15 第二段日志，12.2 s → 28.4 s）：
#
#     窗口 6 s / 阈值 270° / 正前 >= 1.0 m   → 整段没触发过一次
#
# 那 6 秒里实际转了 45 + 60 + 120 = 225°，没到 270°；而且正前一直在 0.83~1.31
# 之间晃，一半时间不到 1.0。阈值和窗口都是照着「很夸张的打转」取的，真打转比
# 那个温和得多。8 s / 180° / 0.6 m 才对应「一秒转 20° 不出来」这个现象。
SPIN_WINDOW = 8.0
SPIN_DEG = 180.0
SPIN_MOVE = 0.6
SPIN_CLEAR = 0.6
SPIN_VX = 0.5            # 被闸住时至少给这么大的前进速度，别站着

# 「这一拍被拦住了转向、那就直走」时给的速度。
#
# 原来这里也复用 `SPIN_VX = 0.5`，而 `Guard.filter` 在正前 < 1.6 m 时还会再减半
# → 0.25 m/s。2026-11-15 第二段日志里，狗被连着挡了 5 次、13 秒只挪 1.3 m，
# 状态行 `vx=+0.25` 一大片。
#
# 用模型自己的正常步速。安全闸仍然按正前距离压，不会因为它高就撞墙 —— 那个
# 压缩本来就是按距离连续算的，起点低只会让它更慢，不会更安全。
STRAIGHT_VX = 0.8

# ---- 命令兑现 ----------------------------------------------------------------
#
# 模型每拍读得到瞬时速度（`机身：… 速度 0.05 m/s`），但读不出「这是我命令 0.3
# 换来的」。少了这层对比，它会把同一个命令重复十几遍，而地上什么都没发生。
#
# 2026-11-16 关掉射线那次的账：130 秒里 56 条 `vx=0.3`，位移 0.2 m，模型的想法
# 全是「低速前探确认路况」「保持低速前探」。它不知道自己在原地磨。
#
# 判据只看「想走却没挪」：窗口内平均命令 >= `PROGRESS_VX`，位移 < `PROGRESS_MIN`。
# 不拿兑现率做基准 —— 那个比例本身还没测准（仿真里 0.3 出 0.05~0.1，真机没量过），
# 用一个未知数当分母，只会把「本来就走得慢」误判成「卡住」。
PROGRESS_WINDOW = 3.0
PROGRESS_VX = 0.2
PROGRESS_MIN = 0.1

# 转完之后恢复前进时，**不再**按正前距离二次压缩。
#
# 第 3 版在 `_resume()` 里压了一次（正前 1.2 m 开始、0.45 m 压到 0），而
# `Guard.filter` 又压一次 —— 正前 0.9 m 时 0.8 → 0.49 → 0.24，日志里
# `vx=+0.25` 一大片，13 秒只挪 1.3 m。
#
# 压缩归 `Guard.filter` 一家：它按 `slow_gap` / `stop_gap` 线性算，`check()`
# 还在 0.45 m 停车。这里只负责「给一个能走的速度」，不再叠一层。

# 小于这个角度不转，直接走。
#
# 这一条治的是「小角度来回转」，和控制层的冲过头是两回事。控制层收敛好之后仍然
# 会看到 +50 / +12 / -23 / +25 / +35 / -25 / +8 这样一串 —— 每次转完，目标点前移
# 一米，新目标的方位又差十几度，模型就再补一个小角度。转 8° 现在要 0.5 s
# （GAIN=16、trot），而带 8° 偏差走一米，横向只差 14 cm，目标点下一秒就更新了 ——
# 这一转仍然是亏的，还会让狗看起来在左右摆。
#
# 15° 是算出来的：这个角度以下，走 1 m 的横向偏差 0.26 m，小于走廊半宽；再大就
# 真会蹭到边墙，那时候该转。
TURN_MIN_DEG = 15.0

# 转向用 trot，不用 pronk。
#
# 这里曾经写着「trot 原地只有 0.03 rad/s，等于没转，必须切 pronk」——那个数是
# 在 vyaw=1.0 下测的，而 1.0 是 `COMMAND_RANGE` 自己设的上限，不是策略的上限。
# 放开之后 trot 的响应在高段反而好（`test/turn_sweep.py`，15 s）：
#
#     vyaw 1.0 → 原地 0.13 rad/s，半径 0.44 m（画圈，走廊里做不出来）
#     vyaw 3.0 → 原地 0.34，半径 0.16 m
#     vyaw 4.0 → 原地 0.74，半径 0.01 m（带 0.15 前进是 0.90）
#
# 15 s 连续转、正反向都试过，位移 0.13~0.18 m，倾角 7~8°，没有打滑。
#
# 不切 pronk 的理由不只是速度：pronk 是四条腿同时蹬地，机身一起一落 7.3 cm
# （实测，和抬脚高度无关 —— 把 footswing 从 0.19 压到 0.06，起伏还是 7.4 cm），
# 看起来就是「一蹦一蹦地转」。trot 是连续对角迈步，没有这个起伏，而且转完不用
# 再切回来 —— 步态自始至终没变过。
#
# 抬脚高度取训练的标称值 0.08。它是落地冲击的唯一旋钮：`test/pd_audit.py`，
# vx 0.4 平地 12 s，命令 0.19 时足端实测抬 0.17 m、触地 3.6 m/s；命令 0.08 时
# 抬 0.12 m、触地 2.3 m/s，位移从 3.48 m 变 3.29 m（−5%）。真机上那声「砸地板」
# 就是这个速度，压它比加 kd 有用 —— 见 `policy/walker.py` 顶部关于增益的说明。
TURN_FOOTSWING = T.NOMINAL_COMMAND["footswing"]    # trot 的正常抬脚，转向和走路用同一个值
TROT = {"phase": 0.5, "offset": 0.0, "bound": 0.0,
        "footswing": TURN_FOOTSWING}

# 直走的步频。模型不主动给 `freq` 时用它。
#
# 为什么由执行层定而不是让模型自己给：文本模型看不见腿，也判断不了「这一步迈了
# 多远」，它只管往哪走。不给就一直用 `default_commands()` 的 3.0，而 3.0 的每步
# 只有 2.5 cm（`test/policy_sim.py`，vx 0.4，8 s）—— 腿抖得欢、机身挪得慢，就是
# 眼睛看到的「原地踏步」。
#
# 实测（vx 1.0，8 s）：freq 3.0 走 1.894 m（每步 4.0 cm），2.0 走 2.048 m
# （每步 6.4 cm），4.0 只走 0.954 m（每步 1.5 cm）。2.0 既快、步幅又大。
#
# 模型自己发了 `freq` 就用它的 —— 这是默认值，不是覆盖。
STRAIGHT_FREQ = 2.0


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


# 目标里那句「需从当前机头左转 90 度」是 G 按射线方向算的转向提示，转完就作废。
# 不摘掉的话，模型看到「触发：转到位」+ 目标又写着 90 度，会照着再转一次 ——
# 实测就是这样在同一个墙角来回摆，累计转 1172° 净位移 3.6 m。
_TURN_HINT = re.compile(r"，?需从当前机头[左右]转\s*\d+\s*度|，?需掉头（180 度）")


def _consume_turn_hint(goal: str) -> str:
    return _TURN_HINT.sub("", goal or "")


# 转向的输出：角度误差 + 机体系角速度 → 这一拍要发的 vyaw。None 表示转完了。
#
# 抽成纯函数是为了能被物理试验直接调用：`test/turn_overshoot.py` 拿同一份常数
# 在 MuJoCo 上把「转 90° 后停得住吗」跑一遍，不用把控制律抄第二份。
# ---- 原地扫视找人 -------------------------------------------------------
#
# 人喊「过来」「找到我」时狗是看不见人的：视觉模型只判三块能不能过，第四行
# 「人：有 / 没有」只知道画面里有没有人，不知道偏多少度。办法是原地一档一档
# 地转，转到某一档视觉说「有」，那个朝向就是人所在的方向。
#
# 步长 30° 不是拍的：`TURN_SPEED=4.0` 时实测转 90° 要 2.1 s，30° 约 0.7 s；
# 停稳 0.3 s，等视觉出新描述最多 1.5 s（视觉 1 Hz），一档约 2 s。整圈 12 档
# 大约 24 s。步长再小没意义 —— 视觉一秒只判一次，转 15° 和 30° 都只来得及
# 判一次，只会把一圈拖成 48 s。
SCAN_STEP = 30.0        # 每档转多少度
SCAN_LIMIT = 360.0      # 扫满这么多就放弃
SCAN_HOLD = 1.6         # 转到位后先停这么久，等视觉把这一档的画面判完
SCAN_STALE = 4.0        # 画面描述比这更旧就不能拿它判「有没有人」
# 一档最多等这么久（秒）。视觉 3 秒一帧，而「转到位」和「下一帧」的相位是随机的，
# 只等 1.6 秒的话经常拿到的是**转之前**那一帧 —— 拿旧画面判「这一档有没有人」，
# 会把人的方向报错一到两档。所以停稳后一直等到「比转动更新的那一帧」为止。
SCAN_HOLD_WAIT = 4.2

# 相机水平视场（度）。仿真：`go2.xml` 的 eye 相机没写 fovy，MuJoCo 默认 45°，
# 画面 320x240 是 4:3，横折算下来 57.8°。真机前视更宽，改这一条就行 ——
# 它只影响「人在侧块时该转多少度」。
SCAN_FOV = 58.0
# 画面左/右块的**中心**离光轴多远：三分块各占 FOV/3，块中心就在 ±FOV/3。
SCAN_THIRD_DEG = SCAN_FOV / 3.0

# 看着的时候，人在侧块就转过去把他放回中间。两条修正之间至少隔这么久（秒），
# 否则视觉还没刷新就按同一帧连转两次。
TRACK_TURN_GAP = 1.0

# 看到人之后正前至少要有这么宽才真的朝他走。窄于它就是「看到但过不去」，
# 那时候说「我朝你那边走」是空话 —— 走一步安全闸就把它压成 0。
SCAN_GO_GAP = 1.2
# 扫描时每转过这么多度报一次进度。整圈报 3 次（90 / 180 / 270），加上开始和
# 结果一共 5 句 —— 一秒钟一句没人受得了，一句话不说人又不知道它在干嘛。
SCAN_REPORT = 90.0
# 档数用整数，不用弧度累加。`swept += radians(30)` 六次之后是 3.14159265…
# 略小于 `radians(180)`，`>=` 判不过就直接跳过了 180° 那次播报 —— 实测日志
# 里是 90 / 210 / 270，中间的 180 消失。
SCAN_LEGS = int(round(SCAN_LIMIT / SCAN_STEP))          # 12 档 = 一圈
SCAN_REPORT_LEGS = int(round(SCAN_REPORT / SCAN_STEP))  # 每 3 档报一次
# 「看着我」这类持续任务最多看这么久（秒）。超时自己收尾，不无限占着执行层。
SCAN_WATCH = 90.0
# 看着的时候每隔这么久说一句。人不动、画面不变，视觉那边没有新描述，只能靠
# 定时播报让人知道它还盯着。
SCAN_WATCH_REPORT = 20.0

# 人话里认这些词就原地扫一圈找他。**「转过来」不算** —— 那是叫人转身，
# 不是叫人过来；不排除的话每次让它「转过来」都会先原地扫一圈。
_SCAN_WORDS = ("找我", "找到我", "找你", "你在哪", "到我这", "看到我", "看见我",
               "看着我", "过来", "跟着我", "找人")
_SCAN_SKIP = ("转过来", "转回来", "别过来", "不要过来", "转过去")


# 这一轮要不要原地扫一圈找人。
def wants_scan(text: str) -> bool:
    t = (text or "").strip()
    if not t:
        return False
    if any(w in t for w in _SCAN_SKIP):
        return False
    return any(w in t for w in _SCAN_WORDS)


# 要的是「一直看着我」还是「找到我就行」。前者是持续任务：找到之后不结束，
# 站在原地保持朝向，人走出画面就重新扫。
_LOOK_WORDS = ("看着我", "盯着我", "看着我", "看着你", "看我")


def wants_look(text: str) -> bool:
    t = (text or "").strip()
    return any(w in t for w in _LOOK_WORDS)


# 叫人停下。扫描/看着期间收到这句要立刻收手 —— 否则它继续转，人说什么都
# 像没听见。
_STOP_WORDS = ("不要动", "别动", "停下", "站住", "停下来", "别转", "不要转")


def wants_stop(text: str) -> bool:
    t = (text or "").strip()
    return any(w in t for w in _STOP_WORDS)


# 有动作意图的话才算「任务」。纯提问（「你能做什么」「为什么」）不该变成一个
# 挂两分钟的任务 —— 模型会一直「执行」一个问题，reply 也变成重复报告。
_ACT_WORDS = ("走", "转", "停", "退", "动", "过来", "找", "看", "跟", "调头",
              "转身", "快", "慢", "左", "右", "前", "后", "跳", "蹲", "站",
              "冲", "绕")


# 问原因的话一律不算任务，哪怕里面带动作词 ——「为什么停了」含「停」，但它
# 是在问为什么，不是叫它停。不排除的话每句追问都会变成一条挂两分钟的任务。
_WHY_WORDS = ("为什么", "为啥", "怎么回事", "怎么了")


def is_command(text: str) -> bool:
    t = (text or "").strip()
    if not t:
        return False
    if any(w in t for w in _WHY_WORDS):
        return False
    return any(w in t for w in _ACT_WORDS)


# 任务里的距离。`往前走两米` 这种带数字的指令必须量出来，否则模型只能靠感觉
# 编进度 —— 2026-11-18 实测：任务「往前走两米」下给它，走到 15.6 米了，
# thought 还在说「任务还差两米没走完」。它没有里程，只能按上一轮的想法往下续。
_CN_DIGIT = {"零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
             "五": 5, "六": 6, "七": 7, "八": 8, "九": 9, "十": 10}

_DIST_RE = re.compile(r"(\d+(?:\.\d+)?|[零一二两三四五六七八九十]+)\s*(?:米|公尺)")


def _cn_num(raw: str) -> float | None:
    if raw in _CN_DIGIT:
        return float(_CN_DIGIT[raw])
    if "十" not in raw:
        return None
    # 十几、二十、三十一 这几种够用了。更大的距离没人用嘴说。
    hi, _, lo = raw.partition("十")
    h = _CN_DIGIT.get(hi, 1 if not hi else None)
    l = _CN_DIGIT.get(lo, 0 if not lo else None)
    if h is None or l is None:
        return None
    return float(h * 10 + l)


def distance_of(text: str) -> float | None:
    m = _DIST_RE.search(text or "")
    if not m:
        return None
    raw = m.group(1)
    try:
        v = float(raw)
    except ValueError:
        v = _cn_num(raw)
    # 0 米和 300 米都不是「往前走一点」的意思，当作没提取到，别拿它当判据。
    if v is None or not (0.2 <= v <= 50.0):
        return None
    return v


# 任务是两类东西，不能一个模型套：
#
#   act  —— 走完就结束的一件事：「往前走两米」「调头」。
#   stay —— 要一直守着的一个状态：「看着我」「保持一米」「别动」。
#
# 全是 act 的话，「别动」会被当成一件做完就自由的事：站 4 秒（`TASK_HOLD`）之后
# 它自己走开。2026-11-18 用户的原话：「任务可能是长期状态保持，不是执行完毕即自由」。
_STAY_WORDS = ("保持", "一直", "继续", "跟着", "跟住", "盯着", "看着我", "等我",
               "待着", "守住", "蹲着", "站着")

# 只在**没有距离**的时候才算「守着」。「往前走两米，走够就停下」里有「停下」，
# 但它要的是一段路程，按状态处理就永远结束不了。
_HALT_WORDS = ("别动", "不要动", "停下", "停下来", "停住", "站住")


def task_kind(text: str) -> str:
    t = (text or "").strip()
    if any(w in t for w in _STAY_WORDS):
        return "stay"
    if distance_of(t) is not None:
        return "act"
    if any(w in t for w in _HALT_WORDS):
        return "stay"
    return "act"


# 任务多久没被重新下达就自动作废（秒）。人走了、或者已经做完了，都得有个头 ——
# 不清掉的话模型两分钟之后还在「执行」一句早就过时的话。
TASK_TIMEOUT = 120.0

# 带距离的任务放宽一倍。120 秒够它说一句话，不够它在迷宫里绕两圈走到 2 米 ——
# 按 0.3 m/s 加两个掉头，两米能花掉 40 秒以上，超时一清它就白走了。
TASK_TIMEOUT_WALK = 240.0

# 守状态的任务给更长的时限。它是「一直保持」，按两分钟清掉就等于人没说话它就
# 自己解禁了 —— 而人恰恰是说完「别动」就不说话了。
TASK_TIMEOUT_STAY = 900.0

# 任务做完之后原地站这么久（秒）。G 层下一拍就会挑「下一步方向」，模型也会照走，
# 人看到的是「停了半秒又自己走开」，「我到了」这句话就白说了。
TASK_HOLD = 4.0

# 守位（「别动」「站着」）的闭环参数。
#
# 只发 `vx=0` 守不住：小跑步态在 vx=0 时仍在抬脚，仿真实测以约 0.025 m/s 往后左
# 漂（12 秒 0.3 m），守一分钟就是一个身位。所以按任务起点闭环修：误差死区内不动，
# 出了死区用很小的速度往回找。
# 漂了才修，修就修到位。实测这个策略在低速下「命令 -0.15 m/s 几乎推不动，还多出
# 一股侧滑」—— 连续小幅修正反而让位置更散（0.09 → 0.46 米）。所以死区放大，
# 出了死区按比例给到能推动的速度，一次修回来，然后安静等下一次漂。
HOLD_DEAD = 0.30      # 死区（米）
HOLD_GAIN = 1.5       # 误差 → 速度（1/s）
HOLD_MAX = 0.35       # 修正速度上限（m/s）。够推得动，又不至于变成一次小跑

# 朝向也要守。站着不动时机头仍会慢慢漂（仿真实测约 1°/s），漂着漂着「别动」就
# 变成了「原地慢慢转」—— 而转向会把机体系的位置修正带偏，位置也跟着散。
HOLD_YAW_DEAD = 8.0   # 度。同位置一样：小幅角速度推不动，还容易把位置带散
HOLD_YAW_GAIN = 2.0   # 1/s
HOLD_YAW_MAX = 0.80   # rad/s

# 下「别动」的那一刻它可能正以 0.9 m/s 跑着，滑出去半米才停。锚点用那一瞬的
# 位置的话，它得先滑半米再自己慢慢走回来 —— 实测「别动」锚点 0.78 米、守了
# 半分钟才收回 0.2 米。所以锚点延后一会儿再钉，钉在它**停稳的位置**。
HOLD_LOCK = 1.5

# 守位时的抬脚高度（米）。0.03 是训练范围的下限，抬得越低侧滑越小。
HOLD_FOOT = 0.03

# 任务期间两条回话至少隔这么久（秒）。模型每轮都会写 reply，不限流的话
# 1 Hz 刷屏，人根本读不过来。
REPLY_GAP = 4.0


# 画面判读里有没有人。第四行的格式是 `人：有` / `人：没有`，
# 直接找「人：有」不会误命中「人：没有」（「人：没」后面不是「有」）。
# 人在画面的哪一块。返回 "左" / "中" / "右" / None。
#
# 原来是 `_person_in()` 一个布尔 —— 只有「有/没有」，没有方位。后果是它转到
# 某一档、画面里**任何位置**出现人都算找到，停下来的朝向可能偏 30°；也说不出
# 「再往哪转一点」，跟踪就无从谈起（2026-11-18 用户问「真能准确找到我吗」）。
def _person_third(scene: str) -> str | None:
    if not scene:
        return None
    for third in ("左", "中", "右"):
        if f"人：{third}" in scene:
            return third
    # 老格式（`人：有`）当中间处理：以前没有方位信息，只能认为它「在画面里」。
    if "人：有" in scene:
        return "中"
    return None


def _person_in(scene: str) -> bool:
    return _person_third(scene) is not None


def turn_command(err: float, wz: float) -> float | None:
    # 两个都进容差才算完。只看角度的话，判定到位的那一刻机体还在转，切断命令后
    # 靠摩擦慢慢停，还会多转 2° 左右（见 `TURN_DAMP` 里的实测数）。
    if abs(err) <= TURN_TOL and abs(wz) <= TURN_WZ_STILL:
        return None

    # 提前制动：**离目标还有 `wz × TURN_LEAD` 度时就开始反推**，不等进容差。
    #
    # 以前只在 `err <= TURN_TOL`（3°）时制动，而 `err × TURN_GAIN` 要到 14°
    # 以内才低于 4.0 —— 满速 0.74 rad/s 下 14° 只要 0.33 s，策略从收到反推到
    # 真正停下又要 0.2~0.3 s，这段时间足够冲过去。大角度（170°）时角动量更大，
    # 日志里出现过转到位后还差 20 多度、模型再补一次小转向的情况。
    # 0.25 s 是按「反推后大约 0.25 s 停住」取的，最终值由 `turn_overshoot.py` 验。
    if abs(err) <= TURN_TOL + abs(wz) * TURN_LEAD:
        return max(-TURN_SPEED, min(TURN_SPEED, -TURN_DAMP * wz))

    speed = max(-TURN_SPEED, min(TURN_SPEED, err * TURN_GAIN - TURN_DAMP * wz))
    # 命令太小策略不响应（机身不动、误差也不变），会一直耗到超时。
    if abs(speed) < TURN_FLOOR:
        speed = math.copysign(TURN_FLOOR, err)
    return speed


class Thinker:
    def __init__(self, motion, percept, *, guard=None, model: str = DEFAULT_MODEL,
                 period: float = 1.0 / THINK_TICK_HZ,
                 min_think: float = 1.0 / THINK_MAX_HZ,
                 idle_think: float = 6.0, self_drive: bool = False,
                 # 开着 thought 时输出是「想法 40 字 + 命令 JSON」，160 是当初的
                 # 实测值。关掉 thought 只剩 35 字，96 也够；两者都留了余量，
                 # 实际生成量由模型自己决定，不用为省 token 卡这个数。
                 max_tokens: int = 160, verbose: bool = True):
        self.motion = motion
        self.percept = percept
        self.guard = guard
        self.period = float(period)
        self.min_think = float(min_think)
        self.idle_think = float(idle_think)
        # 自驱：这一轮想完就预约下一轮。见 `_should_think`。
        # 只有自驱模式（G 层在跑）才开：按句执行时没有任务，连着想只会让它自己
        # 找方向乱走，而人还没说完话。
        self.self_drive = bool(self_drive)
        self._chain = False
        # 打断：视觉模型给出新描述时置位，正在跑的请求会被掐掉（见 `interrupt`）。
        self._cancel = threading.Event()
        self._interrupted = False
        self.interrupts = 0
        # 上一次看过的画面描述。视觉模型每给一句新的，就当一次唤醒用（见
        # `_should_think`）。空字符串不算：那是「还没看到」，不是「变了」。
        self._last_scene = ""
        self.max_tokens = int(max_tokens)
        self.verbose = verbose

        self._model_spec = model
        self._model = None
        self._run = threading.Event()
        self._thread: threading.Thread | None = None
        # 人说的话：待处理的排队、说过的话留历史。`say()` 投进来，`_take()` 取走，
        # `history()` 进提示词 —— 三件事都在这个对象里，见 `inbox.py`。
        self.inbox = Inbox()
        self._wake = threading.Event()

        self.last_thought = ""
        self.last_reply = ""             # 最近一次回给人的话
        # 正在执行的任务：{"text": 用户原话, "at": 接下它的时刻}。
        #
        # 原来没有这个东西 —— 用户那句话只在那一个 `_think` 里出现过一次
        # （`build_think(user=...)`），下一拍 `user` 是 None，提示词里连「用户：」
        # 那行都没有了。于是模型只知道「这一拍被某句话触发过」，不知道有一件
        # 事在等它做完。2026-11-17 用户的原话：「我发布任务之后，在模型的思考中
        # 根本没有去思考我的任务」。
        self.task: dict | None = None
        self._odom_broken: str = ""      # 里程读取失败的原因，只报一次
        self._hold_until = 0.0           # 任务刚做完，站到这个时刻为止
        self._hold_log_at = 0.0          # 守位修正日志的节流时间戳
        # 守位时的角速度。`_renew()` 每 0.5 秒补发一次命令，原来那里写死
        # `vyaw=0`，把守朝向的修正整个冲掉 —— 现象是日志里一直在转、朝向却照旧漂。
        self._hold_vyaw = 0.0
        self._reply_at = 0.0             # 上一条回话的时刻，给 `REPLY_GAP` 用
        # 回话的去处。默认打日志（`_log`），网页那层用 `set_reply_sink` 换成
        # 一行能抓的格式。
        self.on_reply = None
        self.last_command: dict = {}
        self.last_error: str | None = None
        self.last_gap = float("inf")
        self.cur_gap = float("inf")      # 这一拍看到的距离，喂状态栏
        self.last_percept = None
        self.goal = ""                  # 常驻目标，G 层写、每拍带进提示词
        # 目标点的世界坐标。G 用地图规划出来，L 每拍拿当前朝向重算「转多少度」——
        # 角度不能只在写目标那一刻算一次，狗转了一半就已经过期了。
        self.target: tuple[float, float] | None = None
        self._goal_dirty = False
        self.last_why = ""              # 这一拍为什么想；显示窗口要用
        self.history: list[tuple[float, str]] = []   # (秒, 想法)，只留最近几条
        # 带命令的历史，进提示词用。和 `history` 分开：那份只给窗口画，格式不能
        # 因为提示词改需求而变。
        self.recent: list[tuple[float, str, dict]] = []
        # 正在执行的转向：{"target": 目标 yaw, "started": 起始时刻}
        self._turn: dict | None = None
        self._turn_done: str | None = None   # 刚转完，下一拍要重想一次（存原因）
        # 原地扫视找人的状态机（见 `SCAN_STEP` 那段）。非 None 时模型不参与，
        # 这一拍的全部动作由 `_run_scan` 决定。
        self._scan: dict | None = None
        # 上一次转向结束之后累积走了多远（路径长度，不是直线距离）、结束了多久。
        # 两个一起判：位移到 `TURN_MOVE`，或者时间过 `TURN_MOVE_TIME`。
        # 初始给一个大值 = 开局不禁转。
        self._free_move = 1e9
        self._free_at = 0.0
        self._last_pose: tuple[float, float] | None = None
        # 上一拍转向被按住的原因，带进下一轮触发原因（见 `_think` 开头）。
        self._blocked = ""
        # 最近几拍的 (时刻, x, y, yaw)，给原地打转闸算「转了多少 / 挪了多少」。
        self._pose_hist: deque = deque()
        self._spin_tot = 0.0                 # 窗口内累计转角（度）
        self._spin_move = 0.0                # 窗口内位移（米）
        # 命令兑现的评估窗口（见 `_progress`）。`stuck` 是上一段结算出来的结论，
        # 下一拍塞进提示词。
        self.stuck = ""
        self._prog_at = 0.0
        self._prog_pose: tuple[float, float] | None = None
        self._prog_vx: list[float] = []
        # 连续几个窗口「命令有速度、位置一点没变」。真机上接管后位姿话题全停，
        # `ego` 是冻结的最后一帧，这个数会一直涨；仿真里不会。
        self._blind_windows = 0
        self.turns_done = 0              # 转到位几次，诊断用
        self.thoughts = 0
        self.frames = 0
        self.gen_seconds = 0.0
        self.think_rate = 0.0
        self.started_at = 0.0

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[L] {msg}", flush=True)

    def start(self) -> None:
        if self._run.is_set():
            return
        if self._model is None:
            self._model = make(self._model_spec)
            self._log(f"模型 {self._model_spec} 已加载，{self._model.load_seconds:.2f}s")
        self._run.set()
        self.started_at = time.perf_counter()
        self._thread = threading.Thread(target=self._loop_guard, name="think",
                                        daemon=True)
        self._thread.start()
        self._log(f"决策循环已启动，节拍 {1.0 / self.period:.1f} Hz，"
                  f"推理间隔 >= {self.min_think:.1f} s")

    def stop(self) -> None:
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._log("决策循环已退出")

    def close(self) -> None:
        self.stop()
        if self._model is not None:
            self._model.close()
            self._model = None

    # 投一句话进来：下指令或闲聊都走这里。
    #
    # 原来是「只留最新一条，第二句覆盖第一句」—— 下指令够用，对话不行。人常连说
    # 两句（「往前走」「慢一点」），覆盖式只剩后半句，前半句的意图丢了。现在排队
    # （`inbox.PENDING_MAX` 条），并且全部记进历史，下一轮的提示词里能看到。
    def say(self, text: str) -> None:
        if self.inbox.push(text):
            self._wake.set()

    # 回话的去处。默认只打日志；网页那条路要一行固定前缀的格式（它从 stdout 里抓）。
    def set_reply_sink(self, sink) -> None:
        self.on_reply = sink

    # 任务打包成提示词要的形状。过期就清掉并记一行 —— 不清的话模型两分钟
    # 之后还在「执行」一句早就过时的话。
    #
    # 进度是**量出来的**：里程取 `percept/memory.py` 的 `travelled`（足端里程计
    # 累加，真机仿真都有），不是让模型自己估。任务开始那一刻记下起点里程和坐标，
    # 之后每一拍都算「已走多少」。
    def _odom(self) -> tuple[float, tuple[float, float, float] | None]:
        trace = getattr(self.percept, "trace", None)
        if trace is None:
            return 0.0, None
        try:
            travelled = float(trace.status().get("travelled") or 0.0)
            # `pose` 在 `percept/memory.py` 里是 **property**，不是方法。按方法调
            # 会抛 TypeError，被下面的 except 吞成 0.0 —— 现象是「走了 13.9 米，
            # 任务进度一直 0.0」，而且日志里一个错都看不见。
            pose = trace.pose
            if callable(pose):
                pose = pose()
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        except Exception as err:                            # noqa: BLE001
            if not self._odom_broken:
                self._odom_broken = f"{type(err).__name__}: {err}"
                self._log(f"里程读不出来，任务进度按 0 算：{self._odom_broken}")
            return 0.0, None
        return travelled, (x, y, yaw)

    # 接一条新任务。`want` 是嘴里的距离（「两米」→ 2.0），没有就是 None。
    #
    # `stay` 任务不取距离：「保持一米距离」里的 1 米是**要维持的间距**，不是要走的
    # 路程。当路程算的话，它走满一米就宣布完成，然后开始自由探索 —— 正好反过来。
    def _new_task(self, text: str) -> dict:
        walked0, pos0 = self._odom()
        kind = task_kind(text)
        task = {"text": text, "at": time.monotonic(), "walked0": walked0,
                "pos0": pos0, "kind": kind,
                "want": distance_of(text) if kind == "act" else None,
                "done": False}
        if kind == "stay":
            # 守位锚点等它停稳再钉，理由见 `HOLD_LOCK`。
            task["pos0"] = None
            task["lock_at"] = task["at"] + HOLD_LOCK
        return task

    def _task_arg(self) -> dict | None:
        if self.task is None:
            return None
        age = time.monotonic() - self.task["at"]
        walked, pos = self._odom()
        walked -= float(self.task.get("walked0") or 0.0)
        # 守状态的任务：到点了就把锚点钉在当前位姿上（`HOLD_LOCK`）。
        if (self.task.get("pos0") is None and self.task.get("kind") == "stay"
                and pos is not None
                and time.monotonic() >= float(self.task.get("lock_at") or 0.0)):
            self.task["pos0"] = (pos[0], pos[1])
            self.task["yaw0"] = pos[2]
            self._log(f"守位锚点钉在 ({pos[0]:+.1f},{pos[1]:+.1f})、朝向 "
                      f"{math.degrees(pos[2]):+.0f}°，接下来围着它守")
        pos0 = self.task.get("pos0")
        disp = None
        if pos is not None and pos0 is not None:
            disp = math.hypot(pos[0] - pos0[0], pos[1] - pos0[1])
        want = self.task.get("want")
        kind = self.task.get("kind") or "act"
        if want and walked >= float(want) and not self.task.get("done"):
            self.task["done"] = True
            self._log(f"任务距离已到：走了 {walked:.1f} 米"
                      f"（要到 {float(want):.1f} 米），该停下汇报了")
            # 立刻推一拍：不推的话要等下一个触发（视觉打断 3 秒、或者下一次
            # 安全闸事件），这中间它还在按上一轮的命令往前冲。
            self._wake.set()
        if kind == "stay":
            limit = TASK_TIMEOUT_STAY
        elif want and not self.task.get("done"):
            limit = TASK_TIMEOUT_WALK
        else:
            limit = TASK_TIMEOUT
        if age > limit:
            text = self.task["text"]
            if kind == "stay":
                # 状态保持到时限：说一声再放开，不然它会先是安静地站着、
                # 然后突然自己走开，人以为它听懂了。
                self._log(f"状态保持到时限：{text}（{age:.0f} 秒）")
                self._reply(f"「{text}」我守了 {age / 60:.0f} 分钟，先放开去自己走了。",
                            force=True)
            elif self.task.get("done"):
                self._log(f"任务收尾：{text}（走了 {walked:.1f} 米）")
            else:
                # 作废也得说一声。原来只打日志，人那边一片安静 —— 他以为狗还在干，
                # 其实任务已经悄悄没了。
                self._log(f"任务超时作废：{text}（走了 {walked:.1f} 米）")
                self._reply(f"「{text}」我没做完，先放下了：走了 {walked:.1f} 米。",
                            force=True)
            self.task = None
            return None
        return {"text": self.task["text"], "seconds": age, "kind": kind,
                "walked": walked, "want": want, "disp": disp,
                "done": bool(self.task.get("done"))}

    # 离任务起点多远（米）。守位修正的日志要用它，人一眼就能看出漂了多少。
    def _off_anchor(self, p) -> float:
        pos0 = (self.task or {}).get("pos0")
        if p is None or pos0 is None:
            return 0.0
        return math.hypot(p.ego.x - pos0[0], p.ego.y - pos0[1])

    # 守位修正量。返回 (vx, vy)，机体系，正 vy 向左（见 `controller.py:1407`）。
    # 误差按任务起点算，不是按上一拍 —— 按上一拍累积误差，漂多少都追不回来。
    def _hold_fix(self, p) -> tuple[float, float, float]:
        pos0 = (self.task or {}).get("pos0")
        if p is None or pos0 is None:
            return (0.0, 0.0, 0.0)
        clamp = lambda v: max(-HOLD_MAX, min(HOLD_MAX, v * HOLD_GAIN))  # noqa: E731
        ex = pos0[0] - p.ego.x
        ey = pos0[1] - p.ego.y
        if math.hypot(ex, ey) < HOLD_DEAD:
            vx = vy = 0.0
        else:
            yaw = p.ego.yaw
            vx = clamp(ex * math.cos(yaw) + ey * math.sin(yaw))
            vy = clamp(-ex * math.sin(yaw) + ey * math.cos(yaw))
        # 朝向单独闭环。位置误差转机体系用的是**当前**朝向，朝向一直在漂的话，
        # 同一个修正量在不同朝向下会画出弧线 —— 实测「别动」守到 0.17 米后转头
        # 13°，位置反而散到 0.69 米。
        yaw0 = (self.task or {}).get("yaw0")
        err = 0.0 if yaw0 is None else math.degrees(_wrap(yaw0 - p.ego.yaw))
        vyaw = 0.0
        if abs(err) > HOLD_YAW_DEAD:
            vyaw = max(-HOLD_YAW_MAX, min(HOLD_YAW_MAX,
                                          math.radians(err) * HOLD_YAW_GAIN))
        return (vx, vy, vyaw)

    # 距离任务走够了、人也已经听到汇报，就该撤了。单独拆出来是为了能离线测 ——
    # 它埋在 `_think` 里的时候只能靠真机跑一趟才知道撤没撤。
    def _settle_task(self, replied: bool) -> None:
        if not replied or self.task is None or not self.task.get("done"):
            return
        self._log(f"任务完成：{self.task['text']}")
        self.task = None
        self._hold_until = time.monotonic() + TASK_HOLD

    # 回话。`force=True` 只用在「这一轮有人开口」——他要立刻看见反应。
    #
    # 任务期间模型每轮都会写 reply（提示词要求的），不限流的话 1 Hz 刷屏，
    # 网页上全是「正往右转，还差 20 度」这种句子，人反而读不出重点。
    #
    # 返回这一句到底发没发出去 —— 调用方靠它判断「任务完成该不该撤」。
    def _reply(self, text: str, *, force: bool = False) -> bool:
        if not text:
            return False
        now = time.monotonic()
        if not force and self.task is not None and now - self._reply_at < REPLY_GAP:
            return False
        self._reply_at = now
        self.last_reply = text
        self.inbox.note_reply(text)
        if self.on_reply is not None:
            self.on_reply(text)
        else:
            self._log(f"回话：{text}")
        return True

    # 设常驻目标。和 `say()` 的区别：`say()` 是一次性指令，执行完就没了；
    # 目标会跟着每一拍推理进提示词，直到被改掉。自驱时 G 层走这个口。
    def set_goal(self, text: str, *, target: tuple[float, float] | None = None) -> None:
        text = (text or "").strip()
        if text == self.goal and target == self.target:
            return
        self.goal = text
        self.target = target
        self._goal_dirty = True
        self._wake.set()

    def wake(self) -> None:
        self._wake.set()

    def _take(self) -> str | None:
        return self.inbox.take()

    @property
    def running(self) -> bool:
        return self._run.is_set()

    def status(self) -> dict:
        return {
            "running": self.running,
            "thoughts": self.thoughts,
            "think_rate": round(self.think_rate, 2),
            "gen_seconds": round(self.gen_seconds, 2),
            "last_thought": self.last_thought,
            "last_reply": self.last_reply,
            "inbox": self.inbox.status(),
            # 这一版要不要产出 thought。窗口据此把空着的那栏写成「已关闭」而不是
            # 「还没想过」—— 后者看起来像模型没反应过来。
            "emit_thought": EMIT_THOUGHT,
            "last_command": dict(self.last_command),
            "goal": self.goal,
            "front_gap": round(self.cur_gap, 2) if self.cur_gap != float("inf") else None,
            # 正在转的话还差多少度（正数=还要往左）。没在转就是 None。
            "turn_left_deg": (None if self._turn is None else
                              round(math.degrees(self._turn_error(self.last_percept)))),
            "turns": self.turns_done,
            "last_error": self.last_error,
        }

    # ---- 循环 ------------------------------------------------------------

    # `_loop` 的护壳：异常不许把这条线程打死。
    #
    # 2026-11-16 的真实事故：`_turn_elapsed` 这个方法根本不存在（名字写错了），
    # 只有「转向途中收到人话」才会走到那行 —— 结果说完第一句、狗开始转，再说
    # 第二句时 `_loop` 抛 AttributeError 直接死掉。之后所有指令都进了队列没人
    # 取，网页上却看不出任何异常：策略还在跑、画面还在更新，只是不再思考。
    #
    # 决策循环是 L 层唯一的心跳，它静默消失等于自动驾驶挂掉。这里兜住：记一条
    # 错误、打一份 traceback，隔一秒重进，而不是让线程无声无息地结束。
    def _loop_guard(self) -> None:
        while self._run.is_set():
            try:
                self._loop()
                return                       # 正常退出（`stop()` 设了 `_run`）
            except Exception as err:         # noqa: BLE001 —— 兜的就是所有异常
                self.last_error = f"决策循环崩了：{type(err).__name__}: {err}"
                self._log(f"决策循环异常，1 秒后重进：{type(err).__name__}: {err}")
                traceback.print_exc()
                if not self._run.is_set():
                    return
                time.sleep(1.0)

    def _loop(self) -> None:
        tick = time.perf_counter()
        last_think: float | None = None
        last_user: str | None = None
        window = tick
        window_frames = 0

        while self._run.is_set():
            self.frames += 1
            # 转向期间节拍被提到 `TURN_TICK`（30 Hz），那些拍只是闭环微调，
            # 不是决策节拍。算进来会让日志出现「14.3 拍/s」这种数，看着像节拍失控。
            if self._turn is None and self._scan is None:
                window_frames += 1
            now = time.perf_counter()

            p = self.percept.read() if self.percept is not None else None
            self.last_percept = p
            gap = p.front_gap() if p is not None else float("inf")
            # 两个距离分开存：`cur_gap` 是这一拍看到的（给人看状态），
            # `last_gap` 是**上次推理时**看到的（给「前方变近」做比较）。
            # 混用一个的话，要么状态栏在推理之前一直显示「无视野」，要么
            # 比较窗口退化成 0.5 秒、`GAP_DELTA` 永远触发不了。
            self.cur_gap = gap
            # 安全闸的正常位置在 R 层（`percept/reflex.py`，20 Hz）。传进来的
            # `percept` 如果是 `Reflex`，危险原因已经在它那边算好了，这里直接读，
            # 不要重复 check —— 重复 check 会让 `trips` 计数翻倍，停车也发两次。
            if p is not None and self.guard is not None:
                self.guard.observe_gap(p)
            reasons = getattr(self.percept, "reasons", None)
            if reasons is None and p is not None:
                if self.guard is not None:
                    reasons = self.guard.check(p)
                else:
                    reasons = p.hazards()
            reasons = list(reasons or [])

            user = self._take()
            if user is not None:
                last_user = user
                # 页面靠这一行确认「我说的那句真的到了」。原来只有模型的回话
                # （`回话：…`）能看见，而它对**指令**常常只改命令不说话 ——
                # 页面上就一片安静，人以为话没送到。
                self._log(f"收到：{user}")
                # 有动作意图的话就是一条新任务，顶掉旧的；纯提问不动任务。
                # 任务是**常驻**的：之后每一拍都进提示词，直到超时或被新指令换掉。
                if is_command(user):
                    self.task = self._new_task(user)
                    self._log(f"任务：{user}")
                    want = self.task.get("want")
                    if want:
                        self._log(f"任务距离 {want:g} 米，按里程算进度")
                # 人说话就放弃正在进行的转向：他可能看到了我看不到的东西。
                if self._turn is not None:
                    # 打「还差多少」，不是「已经转了多少」。`_turn_elapsed` 这个方法
                    # 根本不存在 —— 2026-11-16 这条日志就是「说第二句话没反应」的
                    # 根因：AttributeError 把 `_loop` 线程打死，之后的话全进队列没人取。
                    left = (f"，还差 {abs(math.degrees(self._turn_error(p))):.0f}°"
                            if p is not None else "")
                    self._log(f"新指令打断转向{left}")
                    self._restore_gait(self._turn)
                    self._turn = None
                    # 人说的话优先：把位移冷却也清掉，否则他要掉头时会被「刚转过、
                    # 还没走够 0.5 m」挡回去。
                    self._free_move = 1e9
                    # 打断之后那条「需转 90 度」已经不准了（转了半截），
                    # 留着会让模型从当前朝向再补一个完整角度。
                    self.goal = _consume_turn_hint(self.goal)

                # 「过来」「找到我」这类要狗主动找人的话，交给扫视状态机。
                # 模型自己看不见人（视觉只判三块能不能过），让它答只会绕圈子
                # 或者把话推回去（见 `SCAN_STEP` 那段）。
                #
                # 三件事按顺序判，缺一个都会出问题：
                #   1. 说「停」就先停 —— 扫描期间模型不参与，不听这句就等于
                #      人喊停狗还在转。
                #   2. 已经在扫了就别重来，只报进度。原来是每次都覆盖 `_scan`，
                #      同一句话说两遍就清零重扫，日志里连着两条「我这就转一圈
                #      找你」，人看着像它没听见。
                #   3. 其余情况才新开任务。
                if p is not None and self._scan is not None and wants_stop(user):
                    self._scan = None
                    self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
                    self._reply("好，我停下")
                    user = None
                    last_user = None
                elif p is not None and wants_scan(user) and self._scan is not None:
                    if self._scan["mode"] == "watch" and not wants_look(user):
                        # 「看着我」期间人说「过来」，那是要它走过来，不是要它
                        # 继续站着看。回一句「我正看着你呢」等于把指令当成没听见。
                        self._scan = None
                        self._approach(p, "好")
                    elif self._scan["mode"] == "watch":
                        self._reply("我正看着你呢")
                    else:
                        self._reply(f"转过 "
                                    f"{self._scan['legs'] * SCAN_STEP:.0f} 度了，"
                                    f"还没看到你，看到就过去")
                    user = None
                    last_user = None
                elif p is not None and wants_scan(user):
                    look = wants_look(user)
                    self._scan = {
                        "kind": "look" if look else "find",
                        "mode": "sweep",
                        "legs": 0,
                        "report_leg": SCAN_REPORT_LEGS,
                        "target": p.ego.yaw + math.radians(SCAN_STEP),
                        # 先判**当前这一帧**再开始转。原来一起扫就转 30°，人明明
                        # 就在正前方，也被转到画面边缘去了（FOV 58°，转一档就走
                        # 到 ±29° 的边上）。
                        "phase": "hold",
                        "at": time.perf_counter(),
                        # 起扫时刻。判「这一帧是不是转完之后的」用它（见 `_run_scan`）。
                        "still_at": time.perf_counter(),
                        "centering": 0,      # 这一档转过去对了几次准
                        "look_back": False,  # 人丢了之后回头看过没有
                        "turn_at": 0.0,      # 上一次跟踪转向的时刻
                        "aim": None,         # 最近一次「人在中间」时的朝向
                    }
                    self._log(f"扫视：原地找一圈，步长 {SCAN_STEP:.0f}°，"
                              f"最多 {SCAN_LIMIT:.0f}°"
                              f"{'，找到之后一直看着' if look else ''}")
                    self._reply("我这就转一圈找你，看到你我就说" if not look
                                else "我这就转过去看你，找到了就一直盯着")
                    # 这句由扫描接管，不再交给模型 —— 否则它会再答一句
                    # 「我看不到你」，网页上出现两条互相打架的回话。
                    user = None
                    last_user = None

            # 转向闭环。放在这里而不是 `_think` 里：它每拍都要跑（0.5 s 一次
            # 够，策略自己会把命令保持 3 s），而 `_think` 一秒最多一次。
            if p is not None:
                self._spin_sample(p)
            if self._scan is not None and p is not None:
                self._run_scan(p)
            elif self._turn is not None and p is not None:
                self._run_turn(p)

            since = 0.0 if last_think is None else now - last_think
            why = self._should_think(p, user, reasons, since,
                                     first=last_think is None)
            if why:
                self._think(p, user or last_user, why, since)
                last_think = now
                self.thoughts += 1
                last_user = None
            elif self._turn is None and self._scan is None:
                # 转向时不能续期：`last_command` 里那条是 L 层原始命令（只有
                # vyaw，没有 pronk），续期就等于每 0.5 秒把 `_run_turn` 发的
                # 转向命令覆盖一次。实测这样 14 秒只转 45°（应该 4.5 秒转 90°）。
                self._renew()

            # 平均推理速率，不是窗口值。窗口只有两秒，静止时那两秒常常一次都
            # 没有，算出来是 0，看起来像「这一层没在跑」。
            span = now - window
            if span >= 3.0:
                self.think_rate = self.thoughts / max(1e-6, now - self.started_at)
                self._log(f"{window_frames / span:.1f} 拍/s，"
                          f"推理 {self.think_rate:.2f} 次/s")
                window = now
                window_frames = 0

            # 转向期间把节拍提到 20 Hz（见 `TURN_TICK`）：这一拍里真正要跑的是
            # 上面那个 `_run_turn`，`_should_think` 在转向期间直接返回 None，
            # 所以提频只加快闭环，不会多调一次模型。
            period = (TURN_TICK if (self._turn is not None or self._scan is not None)
                      else self.period)
            tick += period
            rest = period - (time.perf_counter() - tick)
            if rest > 0:
                self._wake.wait(rest)
                self._wake.clear()
            else:
                tick = time.perf_counter()      # 生成超时，不追帧

    # 距目标还差多少角度（带符号）。正数表示还要往左转。
    def _turn_error(self, p) -> float:
        if self._turn is None or p is None:
            return 0.0
        return _wrap(self._turn["target"] - p.ego.yaw)

    # 现在的步态名。`TextService` 这种没有步态的实现返回空串。
    def _gait_name(self) -> str:
        return str(getattr(self.motion, "gait", "") or "")

    # 转向不再改进步态，这里只是保证转完之后是走路步态。
    #
    # 现在转向全程用 trot（见 `TROT`），步态从走路到转向再到走路没变过，所以
    # 正常情况下这个函数什么都不用做。留它是因为 `set_command` 的语义是「只覆盖
    # 传进去的键」：万一有别处把步态改成了 pronk / custom，走路就会变成一直跳，
    # 而跳跃步态没有偏航修正，机头会自己漂。
    def _restore_gait(self, turn: dict | None) -> None:
        name = (turn or {}).get("gait") or ""
        if not name or name in ("pronk", "custom"):
            name = "trot"
        # 已经是这个步态就别再调一次：`set_gait()` 每次都打一行「步态切换到
        # trot」，看起来像每转一次就强制切了一次 trot，实际什么都没变。
        if name == self._gait_name():
            return
        try:
            self.motion.set_gait(name)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"

    # 原地一档一档地转，直到画面里出现人。返回 True 表示这一拍归扫描。
    #
    # 不和 `_run_turn` 共用状态：那个是「转到某个角度然后继续走」，这里转完一档
    # 要**停下来等视觉更新**（视觉 1 Hz，边转边判只会读到转之前的画面，人正好
    # 在某一档上就会被扫过去）。
    def _run_scan(self, p) -> bool:
        scan = self._scan
        if scan is None or p is None:
            return False

        scene = getattr(p, "scene", "") or ""
        age = float(getattr(p, "scene_age", 1e9))
        third = _person_third(scene) if scene else None
        now = time.perf_counter()
        # 只有「转到位之后拍的帧」才算数。`still_at` 是上一次转到位（或起扫）的时刻，
        # 比它新的描述才对应现在这个朝向 —— 否则 3 秒一帧的视觉总会慢一档，把上一档
        # 的画面当成「现在看到的人」，停下时朝向偏 30~60°。
        still = float(scan.get("still_at") or 0.0)
        fresh = age <= max(0.2, now - still)

        if scan["mode"] == "watch":
            return self._run_watch(p, scan, third, fresh, now)
        if scan["mode"] == "track":
            return self._run_track(p, scan, now)

        # 停稳等视觉。等够就进下一档，扫满一圈就放弃。
        if scan["phase"] == "hold":
            if third and fresh:
                return self._found(p, scan, third, now)
            if not fresh and now - scan["at"] < SCAN_HOLD_WAIT:
                # 这一档的画面还没刷新，接着等 —— 拿旧帧判「有没有人」会把方向报错一档。
                self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
                return True
            if scan["legs"] >= SCAN_LEGS:
                self._scan = None
                self._log("扫视：转满一圈没看到人")
                self._reply("我转了一圈没看到你，你出个声，或者站到我能看见的地方",
                            force=True)
                # 「看着我」是守状态任务，找不到人就没法守 —— 不撤的话它会在原地
                # 站满 900 秒（任务时限），期间还不许自己探索。
                if scan["kind"] == "look" and self.task is not None:
                    self._log(f"扫视没找到人，撤掉任务：{self.task['text']}")
                    self.task = None
                self._finish("刚扫完一圈没看到人，继续探路")
                return False
            # 每转 `SCAN_REPORT` 度报一次进度。不说的话人会以为它卡住了 ——
            # 24 秒一句话没有，屏幕上看着就是死机。
            while scan["legs"] >= scan["report_leg"]:
                scan["report_leg"] += SCAN_REPORT_LEGS
                self._reply(f"转了 {scan['legs'] * SCAN_STEP:.0f} 度，还没看到你")
            # 累计角在**开始转这一档时**就记上，不是在等完视觉之后。
            scan["target"] = p.ego.yaw + math.radians(SCAN_STEP)
            scan["legs"] += 1
            scan["phase"] = "turn"

        err = _wrap(scan["target"] - p.ego.yaw)
        speed = turn_command(err, float(getattr(p.ego, "wz", 0.0)))
        if speed is None:
            scan["phase"] = "hold"
            scan["at"] = now
            scan["still_at"] = now
            self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
            return True
        # 扫视时不带前进量：人站在附近，边走边转容易蹭到他。
        self.motion.set_command(vx=0.0, vy=0.0, vyaw=speed, **TROT)
        return True

    # 扫到了人。要不要就这样结束，看当初那句指令是「找我」还是「看着我」。
    #
    # 人在侧块先转过去对准再算数：相机视场 58°，人在边缘时它也会报「有人」，
    # 原来就此宣布「看到你了」，实际机头偏了 20~30°，人看到的是它对着旁边发呆。
    def _found(self, p, scan, third, now) -> bool:
        if third in ("左", "右"):
            tries = int(scan.get("centering") or 0)
            if tries < 2:
                scan["centering"] = tries + 1
                deg = SCAN_THIRD_DEG if third == "左" else -SCAN_THIRD_DEG
                scan["phase"] = "turn"
                scan["target"] = _wrap(p.ego.yaw + math.radians(deg))
                scan["at"] = now
                self._log(f"扫视：人在{third}边，转 {deg:+.0f}° 对准")
                return True
            self._log(f"扫视：转了 {tries} 次人还在{third}边，就按这个朝向看")

        swept = scan["legs"] * SCAN_STEP
        # 一档都没转就看到，说明画面里本来就有他 —— 写成「转过 0°」很傻。
        self._log("扫视：当前画面里就有人，不用转" if swept < 1.0
                  else f"扫视：转过 {swept:.0f}° 看到人了")

        if scan["kind"] == "look":
            # 持续任务：不结束，站在原地保持朝向，一直盯着。
            scan["mode"] = "watch"
            scan["until"] = now + SCAN_WATCH
            scan["report_at"] = now + SCAN_WATCH_REPORT
            scan["aim"] = p.ego.yaw
            scan["look_back"] = False
            self._reply("看到你了，我就站这儿看着你", force=True)
            self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
            return True

        # 一次性任务：看到了就交回去，让模型接着决定怎么走。
        self._scan = None
        self._approach(p, "看到你了")
        return False

    # 转向跟踪：转到 `scan["target"]` 之后回 `watch` 重新判。
    #
    # 和「扫视一档」的区别是转完不改 `legs`、不接着扫 —— 它只是把机头挪回人那边。
    def _run_track(self, p, scan, now) -> bool:
        err = _wrap(scan["target"] - p.ego.yaw)
        speed = turn_command(err, float(getattr(p.ego, "wz", 0.0)))
        if speed is None:
            scan["mode"] = "watch"
            scan["at"] = now
            scan["still_at"] = now
            scan["turn_at"] = now
            self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
            return True
        self.motion.set_command(vx=0.0, vy=0.0, vyaw=speed, **TROT)
        return True

    # 朝刚才看到的人走。**看到了不等于过得去。**
    #
    # 正前够宽才说「我朝你走」，否则那句话是空话 —— 走一步安全闸就把它压成 0，
    # 人在那儿看着狗一边说「朝你走」一边往后退。
    def _approach(self, p, saw: str = "看到你了") -> None:
        gap = float(p.front_gap())
        if gap >= SCAN_GO_GAP:
            self._reply(f"{saw}，我朝你那边走")
            try:
                self.motion.set_command(vx=STRAIGHT_VX, vy=0.0, vyaw=0.0, **TROT)
                self.last_command = {"vx": STRAIGHT_VX, "vy": 0.0, "vyaw": 0.0,
                                     **TROT}
            except Exception as err:                        # noqa: BLE001
                self.last_error = f"{type(err).__name__}: {err}"
        else:
            self._reply(f"{saw}，但正前 {gap:.1f} 米有东西挡着，我绕一下")
        self._finish("刚才扫到你，朝那个方向走")

    # 看着模式。人还在画面里就站着不动；走出画面就重新扫一圈。
    def _run_watch(self, p, scan, third, fresh, now) -> bool:
        if now > scan["until"]:
            self._scan = None
            self._log("看着：到时了，收工")
            self._reply("看了这么久，我先停在这儿，要接着看就再说一声", force=True)
            self._finish("刚看着人，现在可以接着探路")
            return False

        if third == "中" or (third is not None and not fresh):
            scan["aim"] = p.ego.yaw
            scan["look_back"] = False
            self.motion.set_command(vx=0.0, vy=0.0, vyaw=0.0)
            if now >= scan["report_at"]:
                scan["report_at"] = now + SCAN_WATCH_REPORT
                self._reply("我还在看着你")
            return True

        if third in ("左", "右") and fresh:
            # 人偏到侧块：把机头转过去跟。**这是「跟着他的位置转」，不是重扫一圈。**
            # 原来人往旁边挪一步就丢，然后从当前朝向扫 12 档、最坏 30 秒才回来 ——
            # 2026-11-18 用户的原话：「这个问题很不好」。
            if now - float(scan.get("turn_at") or 0.0) >= TRACK_TURN_GAP:
                deg = SCAN_THIRD_DEG if third == "左" else -SCAN_THIRD_DEG
                scan["mode"] = "track"
                scan["target"] = _wrap(p.ego.yaw + math.radians(deg))
                scan["turn_at"] = now
                self._log(f"看着：人偏到{third}边，转 {deg:+.0f}° 跟过去")
            return True

        if third is None and fresh:
            aim = scan.get("aim")
            if aim is not None and not scan.get("look_back"):
                # 先回头看他刚才在的方向。退半步、被柱子挡一下都会「消失」一次，
                # 为此扫一整圈不值 —— 回一次头就知道人是走了还是被挡住了。
                scan["look_back"] = True
                scan["mode"] = "track"
                scan["target"] = aim
                scan["turn_at"] = now
                self._log("看着：人出了画面，先回头看刚才那个方向")
                self._reply("你不在画面里了，我回头找一下", force=True)
                return True
            self._log("看着：回头也没人，重新扫一圈")
            self._reply("还是没看到你，我再扫一圈", force=True)
            scan["mode"] = "sweep"
            scan["legs"] = 0
            scan["centering"] = 0
            scan["report_leg"] = SCAN_REPORT_LEGS
            scan["phase"] = "hold"
            scan["at"] = now
            scan["still_at"] = now
            return True
        return True


    # 任务收尾：把控制权交回给模型，让它带着新目标重想一次。
    def _finish(self, goal: str) -> None:
        self.goal = goal
        self._goal_dirty = True
        self._wake.set()

    # 执行一次转向。每拍调一次，不调模型。
    #
    # 角度用 `ego.yaw` 闭环，角速度用 `ego.wz` 阻尼。`wz` 是 IMU 陀螺直接给的，
    # 不是 yaw 差分，噪声在 0.01 rad/s 量级 —— 这里最早的注释写「wz 噪声大、
    # 不看它」，代价就是收尾时多转 2° 再被模型补一次转向（见 `TURN_DAMP`）。
    def _run_turn(self, p) -> None:
        turn = self._turn
        if turn is None:
            return
        err = self._turn_error(p)
        elapsed = time.perf_counter() - turn["started"]
        speed = turn_command(err, float(getattr(p.ego, "wz", 0.0)))

        # 刹不住（顶墙、打滑、或者残余角速度一直下不去）时不能一直等：
        # 角度已经在容差内了，交回去让模型决定下一步。
        if speed is not None and abs(err) <= TURN_TOL:
            since = turn.setdefault("settle_at", time.perf_counter())
            if time.perf_counter() - since > TURN_SETTLE:
                speed = None

        if speed is None:
            # 角度到了、角速度也停了才算真的转完。
            self._turn = None
            self.turns_done += 1
            self._turn_done = "转到位"
            # 位移冷却归零（见 `TURN_MOVE`）：从这里起，必须真的走出去一段，
            # 才允许下一次转向。
            self._free_move = 0.0
            self._free_at = time.monotonic()
            self._log(f"转到位（{elapsed:.1f} s，累计 {self.turns_done} 次）")
            # 目标里那条转向提示已经用掉了，摘掉再交给下一拍。
            self.goal = _consume_turn_hint(self.goal)
            # 先把步态切回来，再恢复前进。转向全程是 trot，正常情况下
            # `_restore_gait` 什么都不做；留着是为了别处改过步态时不让它带着
            # 那个步态去走路（跳跃步态没有偏航修正）。
            #
            # `_resume` 会按正前剩余距离压住 vx：有空间就接着走，快贴墙就停。
            # 三版的取舍写在 `_resume` 上面。
            self._restore_gait(turn)
            self._resume(p)
            self._wake.set()                 # 叫醒下一拍去重想
            return

        if elapsed > TURN_TIMEOUT:
            self._turn = None
            self._turn_done = "转向没转到位"
            self._free_move = 0.0
            self._free_at = time.monotonic()
            self._log(f"转向超时 {elapsed:.0f} s（还差 {math.degrees(err):+.0f}°），"
                      f"交给下一拍重想")
            # 提示同样摘掉：没转到位是事实（触发原因里写了），但原样留着提示
            # 会让下一拍从「现在的朝向」再转一次同样的角度，越转越偏。
            self.goal = _consume_turn_hint(self.goal)
            self._restore_gait(turn)
            self._resume(p)       # 没转到位也一样：有空间就走，快贴墙就压住
            self._wake.set()
            return

        # 剩得少就转慢、带着角速度就减（都由 `turn_command` 给）。这里只管
        # 要不要顺带前进 —— trot 带 0.25 前进能把「原地转 0.74 rad/s、零位移」
        # 变成「转着也在挪」，这是打断连续转向的基础：转完那一下必须已经走出
        # 一点距离，否则下一拍又会因为「正前还是墙」再转一次（见 `TURN_MOVE`）。
        #
        # 闸刚停过车也不停转向。停车停的是「往前冲」，而原地转是脱身的唯一
        # 动作 —— 贴着墙的时候停了转向，狗就彻底不动了。闸那边对纯转向命令
        # 也是放行的（`guard.filter` 只清平移），两处口径一致。
        vx = TURN_VX if (speed and p.front_gap() > TURN_VX_CLEAR) else 0.0
        try:
            # 全程 trot，不切 pronk（见 `TROT`）。步态从头到尾没变过。
            self.motion.set_command(vx=vx, vy=0.0, vyaw=speed, **TROT)
        except Exception as err2:                            # noqa: BLE001
            self.last_error = f"{type(err2).__name__}: {err2}"

    # 这一拍要不要调模型。返回原因字符串或 None。
    #
    # 顺序有意义：先看人说了什么，再看危险，最后看环境变化。同一拍同时成立时，
    # 报第一个 —— 日志里只留一条原因，多了反而看不出这一拍主要是为什么。
    # 打断正在跑的那一轮推理。
    #
    # 视觉模型每给出一句新描述就调一次（`camera.VisionPercept.on_text`）。
    #
    # 只做标记，**不掐断正在跑的那一轮**。等它生成完，下一拍立刻用新描述重想。
    #
    # 掐断的代价实测过：一轮 1.0 s 的请求被打断后从零重来，日志里变成
    # 「打断 → 0.4 s 再打断 → 0.7 s 再打断」，主循环节拍全碎，而且每次打断都已经
    # 付过服务端的生成成本。排队不动节拍：一次完整 1.0 s 生成 + 紧接着一次重想。
    #
    # 不动正在进行的转向：转 90° 要 2~4 s，中途重想把命令覆盖掉就转不完
    # （实测那种情况四次「开始转向」才完成一次）。打断只作用于「还在等模型说话」
    # 这一段。
    def interrupt(self, text: str = "") -> None:
        if not self.self_drive:
            return
        # 记下这句话，免得 `_should_think` 里「视觉更新」那条再为同一句话触发
        # 一次 —— 打断一次、醒来一次就够，两次就是白烧一次请求。
        if text:
            self._last_scene = text
        self._interrupted = True
        self._wake.set()          # 主循环可能正在睡那 0.5 s 的节拍

    def _should_think(self, p, user: str | None, reasons: list[str],
                      since: float, *, first: bool = False) -> str | None:
        if user is not None:
            return "新指令"
        if reasons:
            # 转向期间「前方变近」不算危险：原地转 90° 时前向射线一定会扫过
            # 侧墙，那是几何必然，不是要撞了。实测不管这一条的话，每次转向都
            # 被自己打断，四次「开始转向」才完成一次，累计转了 462° 还在原地。
            # 倾角是真危险（转了会摔），照旧打断。
            hard = ([r for r in reasons if "倾斜" in r] if self._turn is not None
                    else reasons)
            if hard:
                return "危险：" + "；".join(hard)
        # 转向期间不重想。转 90° 要 4.5 秒，中途重想是这条路上最大的浪费：
        # 模型看到「还是被挡」会换个说法，`_think` 一执行就把转向命令覆盖掉，
        # 下次又从 0 开始转。转到位或超时之后才把控制权交回来。
        # 扫视期间模型不插手：转的过程由 `_run_scan` 逐档推进，模型一旦插话
        # 就会把转向命令覆盖掉。危险仍然优先 —— 上面的 `reasons` 已经返回了。
        if self._scan is not None:
            return None
        if self._turn is not None:
            return None
        if self._turn_done:
            done, self._turn_done = self._turn_done, None
            return str(done)

        if self._goal_dirty:
            # 目标变了要立刻重想：G 已经给了新目标点，狗还在按旧方向走。
            #
            # 排在转向判断**之后**才对。写在前面的话，G 每 0.1 Hz 换一次目标
            # 都会打断手上这次转向 —— 转半截重来，实测四次「开始转向」才完成
            # 一次，累计转了 462° 还在原地。转完再看新目标也不迟。
            return "目标更新"

        # 视觉更新：排在转向判断之后（不能打断转向），但排在 `min_think` 之前 ——
        # 上一轮已经生成完了，这一拍不等满最小间隔就直接用新描述重想。
        # `interrupt()` 只置标记，不掐正在跑的请求，所以走到这里时上一轮一定完整。
        if self._interrupted:
            self._interrupted = False
            return "视觉打断：画面变了，用新描述重看"

        # 首次必须排在 `since` 判断之前。`last_think` 只有在推理成功后才赋值，
        # 所以首拍 `since` 恒为 0，被下面的 `min_think` 拦掉之后就再也没有机会
        # —— L 层会一直不推理、vx 一直是 0，而且日志里看不出任何异常。
        if first:
            # 第一次进来时 `last_gap` 还是 inf。拿它比会算出「前方从 inf 变到
            # 3 m」，凭空触发一次「前方变近」，日志第一条原因就是假的。
            return "接管后首次观测"

        if since < self.min_think:
            return None

        if p is not None and p.rays:
            gap = p.front_gap()
            if gap < GAP_TRIGGER and self.last_gap >= GAP_TRIGGER:
                return f"前方出现障碍（{gap:.2f} m）"
            if abs(gap - self.last_gap) > GAP_DELTA and gap < self.last_gap:
                return f"前方变近（{self.last_gap:.2f} → {gap:.2f} m）"
            if p.ego.tilt_deg > TILT_TRIGGER:
                return f"倾角 {p.ego.tilt_deg:.0f}°"
        # 视觉模型给了新描述：立刻重想一次。
        #
        # V 层不是「每秒刷新」的，它只在画面变化超过阈值时才问一次模型（`VLM_HZ`
        # 是上限，实际约 0.3~0.4 次/s）。所以这里的文字变了，意味着场景真的变了
        # （多了个台阶、墙不见了、走进另一间屋），比任何定时器都值得打断。
        # 反过来，同一句话反复给不会命中 —— 那正是我们要的，画面没变就别叫它。
        if p is not None and p.scene and p.scene != self._last_scene:
            self._last_scene = p.scene
            return f"视觉更新：{p.scene[:24]}"

        # 自驱：上一轮想完就接着想下一轮，不等事件、不等定时器。
        #
        # 前面那些触发条件是「被叫醒」（有人说话、前方变近、目标更新）。没有人的
        # 时候事件几秒才来一次，中间它只是在 `_renew()` 重发上一条命令 —— 看起来
        # 就是「不推不动」。钩子放在这里（`min_think` 之后、事件之前）：想完立刻
        # 排下一轮，但间隔仍受 `min_think`（1 Hz）和生成耗时约束，不会打满 CPU。
        # 转向期间在上面就 return None 了，所以这个钩子不会打断转向。
        if self._chain:
            self._chain = False
            return "接着想"
        if since >= self.idle_think:
            return f"空闲 {since:.1f} s 复查"
        return None

    # 目标点相对当前机头的角度。**每次重想都重算**：G 给的是世界坐标，
    # 狗转到一半之后，写目标那一刻算的角度就已经不准了。
    #
    # 这个数由几何给出，不是模型猜的。模型要做的是「照着这个角度转、用多大速度、
    # 遇到台阶怎么办」，不是「从 8 个距离里推该往哪走」—— 后者它做不好，
    # 实测会给出 -135° 这种谁都没要求的数。
    def _aim_text(self, p) -> str:
        if self.target is None or p is None:
            return ""
        dx = self.target[0] - p.ego.x
        dy = self.target[1] - p.ego.y
        dist = math.hypot(dx, dy)
        if dist < 0.25:
            return "目标点：已经到了，等上层给下一个点"
        # 太近了不给**精确方位**，0.4 m 处横向差 0.1 m 就是 14°，那个数没有意义。
        #
        # 但「这个点在我身后」这件事必须说。原来这里只有一句「方向以它为准不准，
        # 按朝着走过的路继续往前走」，模型看到的就是「按机头直走」—— 而它完全
        # 可能正朝着反方向。
        #
        # 2026-11-17 的 VLA 日志：24~30 秒目标点一直落在 0.6~1.0 m 的盲区里，
        # 机头朝西（+163°）、真正没走过的格子在东北，六秒全走反，直到
        # `drive.AIM_BACK_DEG` 把它判掉才掉头。轨迹是「东 → 打转 12 秒 →
        # 退 3.7 m 回西 → 又往东」，走的是同一条走廊来回。
        if dist < AIM_BLIND:
            rel = math.degrees(_wrap(math.atan2(dy, dx) - p.ego.yaw))
            if abs(rel) > 90.0:
                return (f"目标点：它在我身后 {abs(rel):.0f} 度、{dist:.1f} 米外，"
                        f"当前朝向是反的。别朝这个方向继续走，"
                        f"先看视野里哪边空、哪边没走过")
            return ("目标点：就在脚边，方向以它为准不准。按「朝着走过的路继续往前」走，"
                    "不要为了对准它而转向")
        rel = math.degrees(_wrap(math.atan2(dy, dx) - p.ego.yaw))
        side = "左" if rel >= 0 else "右"

        # 超出测距量程就不给方位。
        #
        # `target` 是 G 层在 1.6 m 内挑的点，但狗可能一路走出去而它没作废
        # （`_aim_stale` 原来只判「到了」，不判「太远」）。日志里出现过
        # 「向左转 125 度，26.7 米外，这条路探过是通的」：26.7 m 早就超出
        # `range_max`（3 m），下面的 `trace.depth` 在量程处封顶，判据
        # `depth < min(dist,1.5)-0.2` 永远不成立 —— 于是它对着一个看不见的
        # 方向断言「通」。这套系统在 3 m 外没有任何测距数据。
        # `range_max > 0` 这一项不能少：`ProprioPercept`（只有本体遥测）给的
        # `range_max` 就是 0，不加判断的话任何正距离都满足 `dist > range_max`，
        # 每一轮都说「超出测距范围」。
        if p.range_max > 0 and dist > p.range_max:
            return (f"下一步方向：那个点在 {dist:.0f} 米外，已经超出测距范围，"
                    f"方位和距离都不可信。按视野里的实际情况走，"
                    f"不要为了对准它而转向")

        # 目标点可能在墙后面。
        #
        # G 层给点走的是 BFS：它证明了「沿这条格链绕过去能到」，但直线不一定通。
        # 原来这里无条件写「这条路探过是通的」，于是同一份观测里出现两句互相打架
        # 的话 —— 目标点「正前 0.9 米，探过是通的」，视野「正前 0.79 米墙」。
        # 模型只能自己判断（它转得没错），而转过去之后 G 又给一个新点，两秒一转。
        # 实测（2026-11-16 迷宫仿真）90 秒累计 1200°、净位移 3.8 m。
        #
        # 这里用射线校验一次直线方向：被挡就不给方位角，也不说「通」，直接说清
        # 「要绕」—— 让它按视野走，别再对着一堵墙调整朝向。
        trace = getattr(self.percept, "trace", None)
        verified = False
        if trace is not None and p.rays and dist > AIM_BLIND:
            depth = trace.depth(math.atan2(dy, dx))
            if depth < min(dist, 1.5) - 0.2:
                if abs(rel) < 1.0:
                    back = "正前"
                else:
                    back = f"{side}边 {abs(rel):.0f} 度"
                return (f"下一步方向（{dist:.1f} 米外）在墙后面，直线过不去。"
                        f"它大概在{back}的方向，但要沿着能走的路绕 —— "
                        f"别为了对准它而转向，先按视野里能走的那边走")
            verified = True

        # 要掉头的时候明说「这是回头」。
        #
        # 这种点通常只在**前面一片全走过了**的时候出现：`guide()` 打分里身后要罚
        # 4.7 m（`ANGLE_COST` 1.5 /rad × π），路径最多 4 m，所以它宁可给身后的
        # 前沿，也只可能因为前方真的没有候选。
        #
        # 原来这种情况一样写「向左转 176 度，1.5 米外，这条路我踩过」—— 模型
        # 不知道自己为什么要往回走，日志里读起来就是「无缘无故掉头」。写清
        # 「前面走完了、现在原路返回」，它的动作就有了上下文。
        #
        # **但「前面走完了」这个前提要当场验一次。** 2026-11-17 迷宫实测：
        # 八方向全 3.00 m 的开阔地里，日志照样写「前面这一片都走过了，往回走
        # —— 向我身后 168 度」，狗于是在空地中间掉头。原因是 G 层的点落后了
        # （已由 `drive.AIM_BACK_DEG` 修掉源头），而这句话本身也不该在前方
        # 空着的时候说出口 —— 那时让模型「继续往前探」才是对的。
        if abs(rel) > 120.0:
            # 没有测距就无从判断前面通不通 —— 这时候既不承诺「空着」，也不下
            # 「往回走」这种强指令。`front_gap()` 在没射线时返回 inf，直接拿来
            # 比会变成「正前还空着 inf 米」。
            if not p.rays:
                return (f"下一步方向：身后 {abs(rel):.0f} 度、{dist:.1f} 米外有个点，"
                        f"但这里没有测距数据。按视野里能走的那边走，"
                        f"别为了对准它而转向")
            front = p.front_gap()
            # 「正前还空着」不等于「正前没走过」。2026-11-17 那段 22~32 秒：
            # 正前空着 2.1 m，但那是刚走过来的走廊，日志却写「前面没走过，继续
            # 往前探，别掉头」—— 于是狗在同一条走廊里往西退到底。
            walked = trace is not None and bool(trace.been("正前"))
            if front >= 1.0 and not walked:
                return (f"下一步方向：身后 {abs(rel):.0f} 度有个点，但正前还空着 "
                        f"{front:.1f} 米、也没走过 —— 继续往前探，别掉头")
            return (f"下一步方向：前面这一片都走过了，往回走 —— 向我身后 "
                    f"{abs(rel):.0f} 度、{dist:.1f} 米外。那段我踩过，能走；"
                    f"不用急，转过去接着走")

        # 直接说「向左转 26 度」，不说「机头左侧 26 度」。模型要把方位换算成
        # vyaw 的正负号，这一步它经常弄反 —— 2026-11-15 实测：目标在左侧时
        # 它输出 -30，狗先朝墙上转一趟再回来。把动作和数值直接写出来，它抄就行。
        #
        # 措辞里的「我自己挑的，不是别人给的任务」是要紧的：这不是迷宫出口、
        # 也不是任何人下发的目标，是 G 层在**自己踩过的格子**里挑的「没去过、
        # 能到、后面还有余地」的那个，沿 BFS 路径取 1 米处的点。说成「上层建议」
        # 会让模型以为有个外部任务在，进而为了「完成目标」硬追一个看不见的点。
        if abs(rel) < 1.0:
            text = (f"下一步方向（我自己在走过的格子里挑的，不是别人给的任务）："
                    f"正前方，{dist:.1f} 米外")
        else:
            text = (f"下一步方向（我自己在走过的格子里挑的，不是别人给的任务）："
                    f"向{side}转 {abs(rel):.0f} 度，{dist:.1f} 米外")
        # 「通」这个结论只在真校验过的时候才说。没有射线、没有栅格时上面那段
        # 整段跳过，那时对这条直线一无所知 —— 原来照样写「这条路探过是通的」，
        # 是凭空断言。
        text += "，这条路我踩过，直线能走" if verified else "，直线通不通没验过，以视野为准"
        # 偏得不多就直接走。写进提示词是为了让它别每次都提「转」这个动作 ——
        # 执行层也会挡（`TURN_MIN_DEG`），但少一次被挡就少一次来回。
        if abs(rel) < TURN_MIN_DEG:
            text += "。偏得不多，直接走也行"
        return text

    # 八方位压成一行。提示词里是排成竖列的（模型好读），但日志里那样一次八行，
    # 一屏只剩三拍；排成一行才能对着「想法 + 命令」看它到底看错了哪一格。
    #
    # 加这个的直接原因：2026-11-15 在迷宫出口前，模型写「前方是墙角，需转向」
    # 然后转了 -70°，而日志里只有它自己复述的数字，没法核对是真墙角还是它看错。
    def _sector_line(self, p) -> str:
        try:
            secs = p.sectors()
        except Exception:                                   # noqa: BLE001
            return ""
        return "  ".join(f"{s.name} {s.gap:.2f}" for s in secs)

    # 结算一段时间的「命令兑现了没有」。每拍推理前调一次，结论留给这一拍用。
    #
    # 正在转向时不算：那一串命令的 vx 本来就小（`TURN_VX`，安全闸还会再压），
    # 位移也本来就小，判成「卡住」只会让它在墙角更不敢动。转向一结束就重新起算。
    def _progress(self, p) -> None:
        if p is None or self._turn is not None:
            self._prog_pose = None
            self._prog_vx.clear()
            return

        now = time.monotonic()
        self._prog_vx.append(float((self.last_command or {}).get("vx") or 0.0))
        if self._prog_pose is None:
            self._prog_pose = (p.ego.x, p.ego.y)
            self._prog_at = now
            return
        span = now - self._prog_at
        if span < PROGRESS_WINDOW:
            return

        moved = math.hypot(p.ego.x - self._prog_pose[0],
                           p.ego.y - self._prog_pose[1])
        want = sum(self._prog_vx) / len(self._prog_vx)
        self._prog_pose = (p.ego.x, p.ego.y)
        self._prog_at = now
        self._prog_vx.clear()

        # 位姿冻结：真机被 lowcmd 接管后 sportmodestate / robot_pose / robot_odom
        # / lf/sportmodestate 一起归零（2026-11-17 实测），`ego` 停在接管前那一帧，
        # `moved` 恒等于 0。这时候再说「位置只变了 0.00 m」是拿一个不更新的数
        # 下结论 —— 模型会以为自己被挡住，然后反复下同一条命令顶着墙推。
        if want >= PROGRESS_VX and moved < 1e-9:
            self._blind_windows += 1
        elif moved >= PROGRESS_MIN:
            self._blind_windows = 0

        if want >= PROGRESS_VX and moved < PROGRESS_MIN and self._blind_windows >= 3:
            self.stuck = ("这台机器没有里程计：接管后位姿话题全部停发，"
                          "位置一直停在起点。别用「走了多远」判断，"
                          "只看视野和画面决定下一步")
        elif want >= PROGRESS_VX and moved < PROGRESS_MIN:
            # 措辞按「估算」写，不下「前面顶着东西」这种因果结论。
            #
            # 这里两边都是估的：`want` 是这段时间**命令**的平均值，不是实际轮速；
            # `moved` 是里程计两次位置的差分，而里程计本身会漂（打滑、上坡、
            # 被推、刚起步那几拍）。原来写「命令没兑现，前面多半顶着东西」，
            # 在真机上会平白让它丢下一条本来能走的路。
            self.stuck = (f"按里程计估算：过去 {span:.0f} 秒命令 vx 平均 {want:.1f}，"
                          f"位置只变了 {moved:.2f} m。不一定是被挡住 —— 打滑、"
                          f"上坡、里程计漂都会这样，结合视野判断。"
                          f"真被挡住就换个方向，别在原地重复同一个命令")
        else:
            self.stuck = ""

    def _think(self, p, user: str | None, why: str, since: float) -> None:
        text = p.text() if p is not None else "没有任何观测"
        self._progress(p)
        self.last_gap = p.front_gap() if p is not None else float("inf")
        self._goal_dirty = False
        self._turn_done = None
        # 上一拍想转、被执行层按住改成直走。带进这一轮的触发原因：不带的话模型
        # 每秒都重新请求一次同样的转向，因为它不知道自己的命令被执行层改过。
        # 2026-11-15 第二段日志里 106 秒里 53 次推理，其中十几次是同一个请求。
        if self._blocked:
            why = f"{why}；{self._blocked}"
            self._blocked = ""

        t0 = time.perf_counter()
        try:
            # 记忆分两份：`memory` 是 M 层算出来的空间事实（走过的格子、里程），
            # `recent` 是这一层自己最近几轮说过什么。少了前者它不知道自己在绕圈，
            # 少了后者它每轮都当第一次看到这个世界。
            trace = getattr(self.percept, "trace", None)
            aim = self._aim_text(p)
            # 有目标点就不再给「目标」那句方向。
            #
            # 两者是同一个东西的两种精度：目标只有「左后」这种八方位词，目标点带
            # 具体角度和距离。都发出去时模型会各取一半 —— 实测在墙角它按「目标 左后
            # 151 度」转 -150°，走过去发现是墙，下一轮又按目标点转回来 +150°，
            # 一次掉头拆成两段 300°。少一句，它只能按角度走。
            self._cancel.clear()      # 上一轮的打断标记不能带到这一轮
            mem = trace.summary() if trace is not None else None
            task = self._task_arg()
            # 走够了就不再喂方向。G 层每拍都在写「目标 正前」、`_aim_text` 还在算
            # 「转多少度」，两句话和「vx 给 0」直接打架，模型会各听一半。
            finish = bool(task and task.get("done"))
            # 罚站期间不给方向、也不再提「往哪探索」。执行层反正会把 vx 压成 0，
            # 只给方向的话模型会写「按记忆往右前方走」，日志和动作对不上。
            left_hold = self._hold_until - time.monotonic()
            holding = left_hold > 0.5
            # 守状态的任务期间也不给 G 层方向。给了它就会「顺便往那边走两步」，
            # 状态当场丢 —— 而这条任务的全部内容就是这个状态。
            staying = bool(task and task.get("kind") == "stay")
            quiet = finish or holding or staying
            if task:
                # 进度是喂给模型的真值，也得留一行日志 —— 它说「还差两米」时，
                # 得能分清是没算出来，还是算出来了它没听。
                self._log(f"任务进度：{task['text'][:14]} 已走 {task['walked']:.1f} 米"
                          + (f" / 要 {float(task['want']):.1f} 米"
                             if task.get("want") else "")
                          + ("（已到，该停）" if task.get("done") else ""))
            raw = self._model.generate(
                build_think(text, user=user, task=task,
                            last_command=self.last_command,
                            last_thought=self.last_thought, reason=why,
                            goal=None if (aim or quiet) else self.goal,
                            aim=None if quiet else aim,
                            memory=mem,
                            progress=(f"任务刚做完，现在原地站着不动，还要站 "
                                      f"{left_hold:.0f} 秒；这一拍别给自己安排新的方向"
                                      if holding else (self.stuck or None)),
                            has_rays=bool(p is not None and p.rays),
                            recent=self.recent,
                            dialog=self.inbox.history()),
                self.max_tokens, cancel=self._cancel)
            # 记忆原来是只进提示词、不进日志的，于是「它记不记得左边那条路」
            # 只能靠猜。打出来一行，出问题时一眼能看出是记忆里没有，还是模型没听。
            if mem:
                self._log(f"记忆：{mem}")
            line = self._sector_line(p) if p is not None else ""
            if line:
                self._log(f"视野：{line}")
            if self.stuck:
                self._log(f"行进：{self.stuck}")
            if aim:
                self._log(f"{aim}")
        except Cancelled:
            # 不是错误：视觉模型给了新描述，这一轮的答案已经过时，丢掉重来。
            self.interrupts += 1
            self.gen_seconds = time.perf_counter() - t0
            self._log(f"推理被打断（第 {self.interrupts} 次），用新画面重想")
            self._chain = self.self_drive
            return
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            self._log(f"推理失败：{self.last_error}")
            self._renew()
            return
        self.gen_seconds = time.perf_counter() - t0
        # 这一轮成功了，预约下一轮（自驱）。失败（上面的 except）不预约，免得
        # 接口报错时变成死循环打请求。
        self._chain = self.self_drive

        thought, reply, fields, notes = parse_think(raw, user or "")
        if thought:
            self.last_thought = thought
        # 回话只在**这一轮有人开口**时收下。
        #
        # 模型在自驱轮次也可能写一句 reply（提示词里给了这个字段，它偶尔会用）。
        # 收下来的后果是它一边走路一边自言自语，而且这些话会进历史、挤掉真的
        # 对话。宁可丢掉：人在场的那一轮一定会被 `_should_think` 立刻触发。
        #
        # 回话先于执行层发出：模型只答话时命令是空的，下面会走 `if not cmd`
        # 那条路直接 return，晚了就发不出去。
        if user:
            # 这一轮有人开口，立刻回 —— 不参与限流。
            replied = self._reply(reply, force=True)
        elif self.task is not None:
            # 任务期间也要回话，只是限流（`REPLY_GAP`）。
            #
            # 原来这里只有 `if user`，任务那几十秒里 reply 全被丢掉 —— 网页上
            # 从头到尾只有第一句「我先左转绕开」，之后一片安静，人不知道它做到
            # 哪一步了。提示词里要求它每轮写 reply，写了却没地方去。
            replied = self._reply(reply)
        else:
            replied = False
        # 距离任务走够了、人也已经听到汇报，任务就撤掉 —— 留着的话下一拍提示词
        # 还在说「你还没完成」，它会原地再走两米。
        self._settle_task(replied)
        # 没有 thought 也走一遍：`_show` 同时负责打「触发原因」，跳过它日志里就
        # 只剩命令，看不出这一拍是撞墙了还是自驱接着想。
        self._show(why, thought, since)

        cmd = to_command(fields) if fields else {}
        hold_vyaw = 0.0                  # 守位时由 `_hold_fix` 给角速度

        # 转向是动作，不是速度：模型给多少度，这里闭环转到那个角度。
        # 角度完全来自模型（`fields["vyaw"]`，单位度）。执行层不自己定数，
        # 也不把「度」当成 rad/s 下发给策略。
        yaw_deg = float(fields["vyaw"]) if fields.get("vyaw") else 0.0

        # 任务刚做完就站住。这一步必须硬压，不能只靠提示词：G 层（`_aim_text`）
        # 下一拍照样给「下一步方向」，模型很听话地就往前走 —— 2026-11-18 实测
        # 「往前走两米」到点后，它先停住汇报，1 秒后又自己往右前方走了。
        if not user and time.monotonic() < self._hold_until:
            if yaw_deg or cmd.get("vx") or cmd.get("vy"):
                self._log("任务刚做完，原地站住，等人说话")
            yaw_deg = 0.0
            cmd = {"vx": 0.0, "vy": 0.0}

        # 守「别动」这类状态时，速度一直压成 0。模型偶尔想「先看看周围」就走两步，
        # 那两步就把状态丢了 —— 而这条任务的全部内容就是这个状态。
        #
        # 压成 0 还不够：小跑步态在 vx=0 时仍会往后漂，所以这里发的是守位修正量
        # （`_hold_fix`），不是死的 0。
        self._hold_vyaw = 0.0
        if (self.task is not None and self.task.get("kind") == "stay"
                and wants_stop(self.task["text"])):
            fix = self._hold_fix(p)
            quiet_fix = fix == (0.0, 0.0, 0.0)
            if not quiet_fix or yaw_deg or cmd.get("vx") or cmd.get("vy"):
                if not quiet_fix:
                    # 每拍都修，日志别每拍都写一行 —— 三秒一条够看出它在往回找。
                    now = time.monotonic()
                    if now - self._hold_log_at > 3.0:
                        self._hold_log_at = now
                        p0 = self.task.get("pos0")
                        where = (f"锚点 ({p0[0]:+.1f},{p0[1]:+.1f}) 现在 "
                                 f"({p.ego.x:+.1f},{p.ego.y:+.1f}) 朝向 "
                                 f"{math.degrees(p.ego.yaw):+.0f}°"
                                 if p0 is not None else "没有锚点")
                        self._log(f"守位修正 vx={fix[0]:+.2f} vy={fix[1]:+.2f} "
                                  f"vyaw={fix[2]:+.2f}"
                                  f"，离起点 {self._off_anchor(p):.2f} 米；{where}")
                else:
                    self._log("守状态：这条要求不动，速度压成 0")
            yaw_deg = 0.0
            # 抬脚高度压到最低（0.03 是训练范围下限）。守位时抬脚只是在制造侧滑。
            cmd = {"vx": fix[0], "vy": fix[1], "footswing": HOLD_FOOT}
            hold_vyaw = fix[2]
            self._hold_vyaw = fix[2]

        # 人开口的那一轮，下面四道「自动闸」全部让路。
        #
        # 那四道闸的存在理由是「模型自己看不到全局，会反复微调式地转」。人说话时
        # 这个前提不成立：人能看见狗看不见的东西，他说转就是要转。不豁免的话
        # 表现是「同一句指令说第二遍就没反应」—— 先说「往右转」，转完刚走 0.2 m，
        # 再说「再往右」被位移冷却按住；或者正前 1.3 m 通着，被「前面能走就别转」
        # 按住。2026-11-16 网页上实测：连发五句，只有第一句有效果。
        #
        # 保留的底线：`guard.filter` 照旧（前方太近就压速度），`_run_turn` 的
        # 转向前进速度照旧，撞不撞仍由向量场决定。人说的话不是「关掉安全闸」，
        # 只是不让「建议性规则」覆盖它。
        free = bool(user)
        if free:
            self._free_move = 1e9
            # 人说话了就不再罚站 —— 他可能就是让狗接着走。
            self._hold_until = 0.0

        # 小角度不转（见 `TURN_MIN_DEG`）。不是否决模型的判断 —— 它想转的方向
        # 照样往前走就有了，只是不值得为八度把步态切成 pronk、停两秒。
        if yaw_deg and abs(yaw_deg) < TURN_MIN_DEG:
            self._log(f"只偏 {yaw_deg:+.0f}°，不值得转，直接走")
            yaw_deg = 0.0
        # 左右写反了就按目标点纠正 —— 但**只在「像是抄反了」的时候**。
        #
        # 判据是幅度：模型给 -32 而目标是 +32，那是把「向左转 32 度」的正负号
        # 抄错了；模型给 +60 而目标是 -35，那是它自己看到左边有条通道、决定不走
        # 目标点 —— 必须放行。
        #
        # 2026-11-15 的教训：第一版只比符号，结果把「看到左边能拐、目标点却在
        # 右边」的判断全部改回目标点，表现就是**明明看见了左拐角还是右转**。
        # 幅度差到 20° 以上就不当口误。
        # 目标点够远、且直线能到，才拿它当「抄反了」的依据。
        #
        # 两个前提是 2026-11-16 迷宫仿真加上的。原来只看符号和幅度差（≤20°），
        # 于是出现了这种日志：
        #
        #     方向抄反了：目标在左侧 11°，模型给 -20°，改按目标点转 +11°
        #     正前 2.3 m 能走，不转 +11°，直接走
        #
        # 模型想右转（左侧 0.58 m 有墙、右后 3.0 m 空），被改成 +11°，再被
        # `TURN_FREE_GAP` 按住 —— 一步没转也一步没走。下一拍目标点跳到别处，
        # 同样的事再来一次。118 秒里 21 次转向、累计 1579°、净位移 2.0 m。
        #
        # 两个条件分别挡两种误判：
        #   点在脚边（< AIM_BLIND）  方位是噪声，`_aim_text` 自己都说「以它为准不准」
        #   点在墙后                直线方位根本走不过去，照着它转就是朝墙转
        if (yaw_deg and p is not None and self.target is not None
                and abs(yaw_deg) >= TURN_MIN_DEG):
            dx = self.target[0] - p.ego.x
            dy = self.target[1] - p.ego.y
            far = math.hypot(dx, dy) >= AIM_BLIND
            trace = getattr(self.percept, "trace", None)
            blocked = (trace is not None and p.rays
                       and trace.depth(math.atan2(dy, dx))
                       < min(math.hypot(dx, dy), 1.5) - 0.2)
            rel = math.degrees(_wrap(math.atan2(dy, dx) - p.ego.yaw))
            if (far and not blocked and abs(rel) < 90
                    and (yaw_deg > 0) != (rel > 0)
                    and abs(abs(yaw_deg) - abs(rel)) <= 20):
                self._log(f"方向抄反了：目标在{'左' if rel > 0 else '右'}侧 "
                          f"{abs(rel):.0f}°，模型给 {yaw_deg:+.0f}°，"
                          f"改按目标点转 {rel:+.0f}°")
                yaw_deg = rel
        # 目标点到机头的方位。后面几处都要用，先算一次。
        aim_rel = None
        if p is not None and self.target is not None:
            aim_rel = math.degrees(_wrap(
                math.atan2(self.target[1] - p.ego.y,
                           self.target[0] - p.ego.x) - p.ego.yaw))

        # 前面能走就别转（见 `TURN_FREE_GAP`）。两个条件：
        #
        #   只拦 15°~90°         掉头（> 90°）是 G 的要求，放行
        #   目标点在正前 ±15° 内  目标点在侧面时，转过去是 G 要它横移，放行
        #
        # 第二条是补的。原来只看正前一个维度，2026-11-15 第二段日志里狗贴着
        # 北墙走（y 从 2.9 涨到 4.1，墙内表面 4.44），G 的目标点一直在走廊中心
        # y=3.6~3.8，也就是右前方 -23°~-51°。模型想右转是对的，而正前是**沿
        # 走廊**的 1.9 m —— 规则读作「前面能走」，连拦 5 次，13 秒只挪 1.3 m。
        # `p.rays` 这一项不能少。没有几何视野时 `front_gap()` 返回 inf（那是给
        # 安全闸用的「看不见就别停」），这条规则会把它读成「前方无限远，能走」，
        # 于是**每一次转向都被拦掉**。实测（2026-11-16，关掉射线、只有画面）：
        # 100 秒里模型每拍都说「画面显示正前有墙，先减速左转扫视」，转向 0 次，
        # 狗一路直走到东墙。模型想转，代码不让。
        if (yaw_deg and p is not None and p.rays and abs(yaw_deg) <= 90.0
                and p.front_gap() >= TURN_FREE_GAP
                and (aim_rel is None
                     or abs(aim_rel) <= TURN_AIM_NEAR
                     or (yaw_deg > 0) != (aim_rel > 0))
                and not free):
            self._log(f"正前 {p.front_gap():.1f} m 能走，不转 {yaw_deg:+.0f}°，直接走")
            self._blocked = (f"你上一拍想转 {yaw_deg:+.0f}°，但正前 "
                             f"{p.front_gap():.1f} m 能走，被按住了，这一拍已经在直走")
            yaw_deg = 0.0

        if (yaw_deg and p is not None and not free and self._spinning(p)):
            # 转过 `SPIN_DEG` 以上却一步没挪，这一串转向是白转的。禁掉这一拍，
            # 并且保证有前进速度 —— 「只走不转」比「只转不走」先动起来才有新信息。
            self._log(f"原地打转：{SPIN_WINDOW:.0f} s 内转了 {self._spin_tot:.0f}°、"
                      f"只挪 {self._spin_move:.1f} m，这一拍只走不转")
            self._blocked = (f"你上一拍想转 {yaw_deg:+.0f}°，但最近 "
                             f"{SPIN_WINDOW:.0f} 秒转了 {self._spin_tot:.0f}° 只挪 "
                             f"{self._spin_move:.1f} m，被按住改成直走了")
            yaw_deg = 0.0
            if float(fields.get("vx") or 0.0) < SPIN_VX:
                fields["vx"] = SPIN_VX
                cmd = to_command(fields)

        # 单次转向幅度上限（见 `TURN_MAX_DEG`）。
        #
        # 目标点在身后时放行 —— 那是 G 让它掉头，该转。目标点在正前方、或者根本
        # 没有目标点时收到 90°：模型看不见身后，它转 120° 的依据只是某条射线够长，
        # 而转身途中那条射线就不再朝着它了。
        if yaw_deg and abs(yaw_deg) > TURN_MAX_DEG:
            ahead = not free
            if free:
                # 人说「掉头」就该转 180°。那条上限的依据是「模型看不见身后」，
                # 而人看得见。
                self._log(f"人要求转 {yaw_deg:+.0f}°，超过单次上限也放行")
            elif p is not None and self.target is not None:
                aim_rel = math.degrees(_wrap(
                    math.atan2(self.target[1] - p.ego.y,
                               self.target[0] - p.ego.x) - p.ego.yaw))
                ahead = abs(aim_rel) <= 90.0
            if ahead:
                capped = math.copysign(TURN_MAX_DEG, yaw_deg)
                self._log(f"转 {yaw_deg:+.0f}° 超过单次上限"
                          f"（目标点在正前方），收到 {capped:+.0f}°")
                yaw_deg = capped

        if yaw_deg:
            if self._turn is not None:
                self._log(f"已经在转，忽略这次的 {yaw_deg:+.0f}°")
            elif (p is not None and p.rays and not free
                  and self._free_move < TURN_MOVE
                  and time.monotonic() - self._free_at < TURN_MOVE_TIME
                  and p.front_gap() >= TURN_CLEAR_OUT):
                # 转完还没走出去，又要在原地转 —— 这是「转圈」的直接形态（见
                # `TURN_MOVE`）。这一拍只走不转，并且保证有前进速度：模型给的
                # 转向命令往往不带 vx，光把 vyaw 清零是站着不动。
                self._log(f"上一转之后只走了 {self._free_move:.2f} m"
                          f"（< {TURN_MOVE:.1f} m），先往前走，这一拍不转 "
                          f"{yaw_deg:+.0f}°")
                self._blocked = (f"你上一拍想转 {yaw_deg:+.0f}°，但刚转完只走了 "
                                 f"{self._free_move:.2f} m，被按住先往前走了")
                yaw_deg = 0.0
                if float(fields.get("vx") or 0.0) < STRAIGHT_VX:
                    fields["vx"] = STRAIGHT_VX
                    cmd = to_command(fields)
            elif p is not None:
                self._turn = {"target": _wrap(p.ego.yaw + math.radians(yaw_deg)),
                              "started": time.perf_counter(),
                              "gait": self._gait_name()}
                self._log(f"开始转向 {yaw_deg:+.0f}°"
                          f"（目标 yaw {math.degrees(self._turn['target']):+.0f}°）")
                self._run_turn(p)        # 立刻发第一拍，不等下一轮

        if self.guard is not None:
            cmd, veto = self.guard.filter(cmd) if cmd else ({}, "")
            if veto:
                self._log(veto)

        if not cmd:
            # 只答话的轮次命令是空的。`parse_think` 对空 `command` 直接返回空
            # fields、不带备注，所以这里通常什么都不做；真正会走到 `notes` 的是
            # 「给了字段但一个都用不了」—— 那种照旧记错误，除非这一轮有回话
            # （那是聊天，不是故障，别让状态栏一直红着）。
            if notes and not reply:
                self.last_error = "；".join(notes)
            elif reply:
                self.last_error = None
            # 转向刚开时 `_run_turn` 已经在发命令，别再续期上一条速度把它盖掉。
            if self._turn is None:
                self._renew()
            return

        # 每条走路命令都显式带 `vyaw=0`。
        #
        # `MotionService.set_command()` 只覆盖传进去的键，不传的保持原值。而
        # `to_command` 特意把 vyaw 排掉（它现在是「角度」不是角速度），于是转向
        # 时设下的那个 0.35 谁都不清 —— 狗转到位之后继续以约 0.1 rad/s 慢慢转。
        # 实测停稳后 8 秒多转 28°，模型看到偏了 18° 就往回转，这就是「转过头
        # 来回转」的真正来源。控制器那条路（`_renew`）同样要带，见那个函数。
        #
        # `last_command` 里仍不放 vyaw：它是给 `_renew` 重发走路速度用的，
        # 放进去会让 `_renew` 的 `vyaw=0.0` 和它撞成重复关键字。
        # `last_command` 只留速度两项。步态是**状态**，不是每拍都要重新确认的
        # 命令：一旦被记进 `last_command`，`_renew` 会每 0.5 秒把它重发一遍，
        # 狗就用那个步态一直走 —— 曾经这样一直 pronk 往前跳，跳跃步态没有偏航
        # 修正，机头自己漂了 28°。现在转向不切步态，这条约束仍然要守。
        # `STRAIGHT_FREQ` 只是兜底：模型给了 freq 会被后面的 `**cmd` 盖掉。
        # 顺序不能反 —— 反了就等于执行层强行改模型的判断。
        send = {"freq": STRAIGHT_FREQ, **cmd, "vyaw": hold_vyaw}
        self.last_command = {**self.last_command,
                             **{k: v for k, v in cmd.items() if k in ("vx", "vy")}}
        # 转向期间由 `_run_turn` 独占命令。这一轮的速度只记下来，转完 `_hold()`
        # 会让狗站住等下一拍 —— 直接下发会把转向命令覆盖掉（之前就是这样转了
        # 45° 就停住）。
        if self._turn is not None:
            self._remember(thought, cmd)
            return

        self.last_error = None
        try:
            got = self.motion.set_command(**send)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"
            self._log(f"下发失败：{self.last_error}")
            return

        # 自己这一轮的结论也进记忆。带命令一起记：「想了什么」和「因此发了
        # 什么」分开看没有意义，下一轮要对照的是「上次说右转，转了吗、转到了吗」。
        self._remember(thought, cmd)

        clamped = got.get("clamped") or {}
        if self.verbose:
            shown = " ".join(f"{k}={v:g}" for k, v in cmd.items())
            extra = f"  （{'；'.join(notes)}）" if notes else ""
            if clamped:
                extra += "  [被压住：" + ", ".join(clamped) + "]"
            print(f"     命令 {shown}  {self.gen_seconds:.2f}s{extra}", flush=True)

    # 记一轮「想了什么 + 因此发了什么」。下一轮要对照的是「上次说右转，转到了吗」，
    # 把想法和命令分开记没有意义。
    def _remember(self, thought: str, cmd: dict) -> None:
        self.recent.append((time.perf_counter() - self.started_at, thought or "",
                            dict(cmd)))
        if len(self.recent) > 8:
            del self.recent[:-8]

    # 把思考贴出来。这是这一层唯一能被外面看见的内部状态 ——
    # 决策对不对只能靠它判断，日志里只留命令的话，看到「vx=0」分不清是
    # 因为看见墙、还是因为模型胡说。
    def _show(self, why: str, thought: str, since: float) -> None:
        self.last_why = why
        # 只有真拿到一句话才进窗口历史。关掉 thought（`prompt.EMIT_THOUGHT`）后
        # 窗口右栏保持上一句，不塞空行 —— 空白会被读成「模型没想出东西」，而实际
        # 情况是我们没让它说。
        if thought:
            self.history.append((time.perf_counter() - self.started_at, thought))
            if len(self.history) > 12:
                del self.history[:-12]
        if not self.verbose:
            return
        print(f"\n[L] 触发：{why}（距上次 {since:.1f} s）" if since else
              f"\n[L] 触发：{why}", flush=True)
        if thought:
            print(f"    想法：{thought}", flush=True)

    # 没有新决策时把上一条命令再发一次。`MotionService` 有 3 秒超时，
    # 不续期狗会走两步就停。
    def _renew(self) -> None:
        if not self.last_command:
            return
        try:
            # 显式 `vyaw=0`：转完之后不清它，狗会一直以转向时那个角速度慢慢转
            # （见 `_think` 里同名注释）。`last_command` 里不会有 vyaw
            # （`to_command` 排掉了），所以这里不会传重。
            self.motion.set_command(vyaw=0.0, **self.last_command)
        except TypeError:
            # 万一哪天 `last_command` 里真的带了 vyaw，别在这里抛。
            self.motion.set_command(**self.last_command)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"

    # 转完之后原地站住，等下一拍重新想。
    #
    # 原来那两处调的是 `_renew()`：把转向前的 vx 原封不动重新发下去。可转完那
    # 一刻机头指的是哪儿，没人知道 —— 这个项目的转向是开环的「大概这么多度」，
    # 而且 `_renew()` 之后要等模型生成（1.0~1.3 s）才可能有新命令，0.8 m/s 乘
    # 这 1 秒就是 0.8 m。
    #
    # 实测（2026-11-15，迷宫东端）：转 +119° 之后正前 0.3 m 是东墙，速度没停，
    # 鼻子直接顶上去 —— 截图里狗贴在墙上就是这么来的。`[guard] 停车：前方
    # 0.45 m` 能拦住，但激光是 10 Hz、反射是 20 Hz，留出的余量比一步小。
    # 转完之后接着走，但前进速度按正前剩余的距离压住。
    #
    # 这里换过三版，每版都有实测教训：
    #
    # 1. 原先调 `_renew()`：把转向前的 vx 原样重发。转完机头指向哪儿没人知道
    #    （转向是开环的「大概这么多度」），而下一句模型要 1.0~1.3 s 才回来 ——
    #    0.8 m/s × 1 s = 0.8 m，正前 0.3 m 是墙就全撞上（2026-11-15 撞墙截图）。
    # 2. 改成 `_hold()`：vx 清零站住。不撞了，但**狗转完就不走了** —— 模型的
    #    很多转向指令只带 vyaw 不带 vx，它不发 vx，这位就一直保持 0，日志里一片
    #    `vx=+0.00`，转到位之后要等下一次决策才动。用户当场反馈「没法正常转向」。
    # 3. 第 3 版仍然重发旧速度，但按 `front_gap` 又线性压了一遍。
    # 4. 现在这个：压缩交给 `Guard.filter` 一家（见 `STRAIGHT_VX` 上面的账），
    #    这里只管给速度。正前真的贴上了，`Guard.check()` 会停车。
    #
    # 第 2 版留下的坑没修干净：`last_command` 里那个 vx 是「模型最后一次显式给
    # 前进速度」，而转向命令只带 `vyaw` —— 所以它可能停在很早的某一拍，甚至是
    # 开局那句「先原地不动」的 0。2026-11-15 无视觉迷宫实测，转到位之后
    # `_free_move` 一直是 0.00 m，狗站在新朝向上等下一次决策。所以没有可续的
    # 前进速度时补一个：正前有空间，转完就该走。
    def _resume(self, p) -> None:
        if not self.last_command:
            return
        rest = {k: v for k, v in self.last_command.items()
                if k not in ("vx", "vy", "vyaw")}
        vx = float(self.last_command.get("vx") or 0.0)
        if vx <= 0.0:
            vx = STRAIGHT_VX
        try:
            self.motion.set_command(vx=vx, vy=0.0, vyaw=0.0, **rest)
        except Exception as err:                            # noqa: BLE001
            self.last_error = f"{type(err).__name__}: {err}"

    # 每拍记一次位姿，顺手把窗口内的「转了多少度 / 挪了多少米」算出来。
    #
    # 在主循环里调，不在 `_think` 里：转向那几秒 `_think` 根本不跑，而打转正好
    # 发生在连续转向的时候 —— 只在 `_think` 采样会把转动全漏掉。
    def _spin_sample(self, p) -> None:
        now = time.monotonic()
        e = p.ego
        # 上一次转向结束以来走了多远。累加每拍的位移，不是「起点到现在的直线」：
        # 顶墙时来回蹭、绕圈走，直线距离都不涨，而这两种都不是「走出来了」。
        if self._last_pose is not None:
            self._free_move += math.hypot(e.x - self._last_pose[0],
                                          e.y - self._last_pose[1])
        self._last_pose = (e.x, e.y)
        self._pose_hist.append((now, e.x, e.y, e.yaw))
        while self._pose_hist and now - self._pose_hist[0][0] > SPIN_WINDOW:
            self._pose_hist.popleft()
        if len(self._pose_hist) < 2:
            self._spin_tot = 0.0
            self._spin_move = 0.0
            return
        _, x0, y0, yaw0 = self._pose_hist[0]
        turn = 0.0
        prev = yaw0
        for _, _, _, yaw in self._pose_hist:
            turn += abs(_wrap(yaw - prev))
            prev = yaw
        self._spin_tot = math.degrees(turn)
        self._spin_move = math.hypot(e.x - x0, e.y - y0)

    # 这一拍要不要禁转（见 `SPIN_WINDOW` 上面那段）。
    def _spinning(self, p) -> bool:
        if self._spin_tot < SPIN_DEG or self._spin_move >= SPIN_MOVE:
            return False
        # 前面是墙的时候转是唯一出路，拦了就死锁。
        return p is not None and p.front_gap() >= SPIN_CLEAR


# 解析 L 层的输出：`{"thought": ..., "command": {...}}`。
#
# 模型经常不按格式来：thought 写成数组、command 直接提到顶层、或者干脆只给
# 一个纯文本理由。三种都接住 —— L 层每 0.5 秒调用一次，抛异常等于整个循环停摆。
def parse_think(raw: str, text: str = "") -> tuple[str, str, dict, list[str]]:
    from quadruped.llm.schema import extract_json

    data = extract_json(raw or "")
    if data is None:
        line = (raw or "").strip().splitlines()
        return (line[0][:80] if line else ""), "", {}, ["没有 JSON"]

    notes: list[str] = []
    thought = data.get("thought") or data.get("思考") or data.get("reason") or ""
    if isinstance(thought, (list, tuple)):
        thought = "；".join(str(x) for x in thought)
    thought = str(thought).strip()[:120]

    inner = None
    for key in ("command", "cmd", "命令", "action"):
        if isinstance(data.get(key), dict):
            inner = data[key]
            break
    if inner is None:
        # 顶层直接写字段的情况。`thought` 不是命令字段，不会被误收。
        inner = {k: v for k, v in data.items()
                 if k in ("vx", "vy", "vyaw", "gait", "freq", "footswing",
                          "height", "pitch", "width", "length")}
        if inner:
            notes.append("command 提到顶层了，已按命令处理")

    # 回话。小模型常见的三种写法都要接住：顶层 `reply`、塞进 `command` 里、
    # 或者写成「无 / null」表示不想说。
    reply = data.get("reply") or data.get("回复") or data.get("说") or ""
    if not reply and isinstance(inner, dict):
        reply = inner.get("reply") or ""
    if isinstance(reply, (list, tuple)):
        reply = "；".join(str(x) for x in reply)
    reply = " ".join(str(reply).split())[:60]
    if reply.lower() in ("无", "没有", "null", "none", "-", "空", ""):
        reply = ""

    # `_guard` 那条「原文没提转向就丢掉 vyaw」，防的是「用户没说转、模型自己
    # 编一个」。它需要的是**用户原话**。自驱时没有原话，而 thought 和 command
    # 是模型同一轮的输出 —— 拿 thought 否决 command 等于替它改主意：它写
    # 「这堵墙过不去，往空的一侧走」，不含「转」字，vyaw 就被吞掉。实测这份
    # 提示词下想法措辞很随意，命中率不到一半。
    # 所以自驱（`text` 为空）传 `None`：没有可对照的原文，就不做这项纠错。
    fields, why = validate(inner, text or None) if inner else ({}, [])
    return thought, reply, fields, notes + why


# 和 `schema.to_command` 同一件事，但这里要容忍空结果（模型说「维持」）。
def to_command(fields: dict) -> dict:
    from quadruped.llm.schema import to_command as _to_command

    return _to_command(fields) if fields else {}
