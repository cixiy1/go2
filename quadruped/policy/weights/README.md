# 策略权重

两个文件是一份 50 Hz 步态策略的 actor 路径：

    body.jit                    actor_body      2102 → 512 → 256 → 128 → 12
    adaptation.jit              adaptation      2100 → 256 → 128 → 2

TorchScript 导出，只有前向权重，不含训练代码与优化器状态。`mlp.py` 直接按 zip
读里面的裸 float32 张量做前向，所以运行时不依赖 torch。

## 推理时的两段

```
obs_history(2100) → adaptation.jit → latent(2)
obs_history(2100) + latent(2) = 2102 → body.jit → 12 个关节偏移
```

`latent` 是 2 维，训练时对应的特权观测量是摩擦系数与恢复系数，部署时只能从 30 帧
历史里估出来。少掉这一步，body 的输入少 2 维，直接报 shape 错。

## 训练参数（部署必须一致）

下表是给人读的摘要。运行时取值只在 `quadruped/policy/training.py` 里定义一份，
`walker.py` 与 `motion/service.py` 都从那里取，`test/policy_audit.py` 负责核对
「训练值 / 运行时值」是否逐项相等 —— 改这里记得同步那边，否则审计会报不一致。

    num_observations         70       每帧 70 维
    num_observation_history  30       30 帧 → 2100
    num_privileged_obs       2        隐变量 2 维
    num_actions              12
    control.action_scale     0.25
    control.hip_scale_reduction 0.5
    control.stiffness        {'joint': 20.0}
    control.damping          {'joint': 0.5}
    control.decimation       4        × sim.dt 0.005 = 0.02 s（50 Hz）
    clip_actions             10

部署默认命令（参考仓库 `go2_gym_deploy` 的 command_profile）：freq 3.0、footswing 0.08、
width 0.33、length 0.40、height 0；`phase` 我们用 0.5（trot），部署侧 0（pronk）——
理由见 `training.py` 顶部。
