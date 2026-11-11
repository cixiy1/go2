"""强化学习支线：离线训练与参数搜索，不参与运行时代码。

`control/` 下的东西负责 200 Hz 闭环与真机，这里负责在仿真里反复跑、搜参数、
训策略。两边只共享 `control/model.py` 的几何常量与 `go2.xml`。

    gait.py    步态参数向量与足端轨迹（与 controller.py 的 foot_target 同构）
    env.py     进程内 MuJoCo 环境，不经 DDS，无 GUI，快于实时
    search.py  CMA-ES 搜索 + 验收指标，输出与 gait_speed.py 同名的量

不依赖 controller.py 的实例：`_tick` 里挂着 RecurrentThread 与 MPC/WBC 线程，
在快于实时的仿真里跑不起来。步态数学单独实现一份，改动时两边一起对。
"""
