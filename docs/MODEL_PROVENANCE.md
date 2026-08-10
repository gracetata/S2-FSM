# 当前部署模型来源

本页记录状态机仓库中两份可直接替换的最新策略。状态机仍使用原来的模型名称和
路由，不需要修改 ROS 2 接口或启动命令。

| 状态机功能 | 部署文件 | 训练产物 | ONNX SHA256 |
| --- | --- | --- | --- |
| High 1 / Low 1 非零速度跟踪 | `models/free_walk.onnx` | `future5090_model9996_two_goal_robust_v3_20260809/model_final.pt` | `5de4f2852286919395a3579e106ccdee0164cc220e9bd1d9865a64912e1a0dcd` |
| High 4 及内部站立恢复 | `models/extreme_stand_recovery.onnx` | `2026-08-07_16-37-50_g1_extreme_stand_v4_jerk_limited_from_model2999_full_20260807/model_20.pt` | `eb2e993220d2e4a343602dfa1556064fce440ce230580803930f7b82151eab6e` |

## Velocity tracking

- 来源会话：`019fd708-6fe7-7943-9cd2-53dae063ce9c`
- checkpoint SHA256：
  `12428d0a2312de12f49e43eeade0c3bd73cdb09634e0ad5a2c0fa9f1f55ca9af`
- 使用位置：仅在 High Mode 1、Low Mode 1 收到新鲜非零速度时选择
  `free_walk.onnx`。
- 零速度、速度缺失或超时仍按状态机设计切换到站立恢复模型。

## Extreme stand

- 来源会话：`019fdb54-8fcb-7842-abad-f25b46de8999`
- 正式 checkpoint：`model_20.pt`
- checkpoint SHA256：
  `afc4f5a520152c99f1f58e803261a422153505474f54dfcf0bed3a7d58b5068d`
- 使用位置：显式 High Mode 4，以及初始化、零速度、安全回退和
  Low Mode 1→2 过渡等内部站立阶段。
- 会话中 2026-08-09 生成的 `model_0.pt` 是启动 smoke 模型，不是正式交付模型，
  本仓库没有采用它。

## 共同部署合同

两份模型均满足：

```text
input:  obs, float32, [1,96]
output: actions, float32, [1,29]
frequency: 50 Hz
action_scale: 0.25
joint order: controller.policy_joint_names
```

逐模型的机器可读合同位于：

- `models/free_walk.contract.json`
- `models/extreme_stand_recovery.contract.json`
