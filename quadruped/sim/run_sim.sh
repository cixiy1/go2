#!/bin/zsh
# 启动 Go2 MuJoCo 仿真器（带图形窗口）。
#
# 为什么用 mjpython 而不是 python：
#   macOS 要求 GUI 必须跑在主线程，mujoco.viewer.launch_passive() 在普通
#   python 下会崩。mjpython 是官方给的外壳，内部 exec 一个 .app 再跑你的脚本。
#
# 为什么必须 cd 到 simulate_python：
#   config.py 里 ROBOT_SCENE 是相对路径 "../unitree_robots/..."，不在该目录下
#   启动会找不到模型。
#
# 用法：
#   ./quadruped/sim/run_sim.sh              # 默认 go2
#   ./quadruped/sim/run_sim.sh b2           # 换机器人（改 config.py 的 ROBOT）
#
# 跑起来之后，另开终端跑控制程序（domain_id=1、interface="lo0"）：
#   uv run python -m quadruped.cli.main stand
#   uv run python -m quadruped.cli.main act squat --depth 0.08
#   uv run python -m quadruped.cli.main forward 1.0
#   uv run python -m quadruped.cli.main trot --vx 0.3 --duration 8

set -e

# 本文件在 quadruped/sim/ 下，项目根是上两级
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM_DIR="$ROOT/quadruped/sim/unitree_mujoco/simulate_python"

if [ ! -x "$ROOT/.venv/bin/mjpython" ]; then
    echo "找不到 .venv/bin/mjpython，先装依赖：" >&2
    echo "  cd $ROOT && uv sync --group sim" >&2
    exit 1
fi

cd "$SIM_DIR"

# 允许从命令行临时换机器人，不用手改 config.py
if [ -n "${1:-}" ]; then
    sed -i '' "s/^ROBOT = \".*\"/ROBOT = \"$1\"/" config.py
    echo "已切换机器人 -> $1"
fi

exec "$ROOT/.venv/bin/mjpython" unitree_mujoco.py
