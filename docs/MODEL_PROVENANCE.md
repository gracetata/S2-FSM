# 当前部署模型来源

本页记录状态机仓库中三份已更新的部署策略。状态机仍使用原来的模型名称和
路由，不需要修改 ROS 2 接口或启动命令。

| 状态机功能 | 部署文件 | 训练产物 | ONNX SHA256 |
| --- | --- | --- | --- |
| High 1 / Low 1 非零速度跟踪 | `models/free_walk.onnx` | `future5090_model9996_two_goal_robust_v3_20260809/model_final.pt` | `5de4f2852286919395a3579e106ccdee0164cc220e9bd1d9865a64912e1a0dcd` |
| High 3 / Low 1 双臂截持行走 | `models/walk_with_object.onnx` | `checkpoint/walk/armhack_two_goal_20260811/model_armhack_walk_two_goal_robust.pt` | `011c4bbf47846285328045967e45b78274d3c81c7cff315fc19b5c9aae095d5b` |
| High 4 及内部站立恢复 | `models/extreme_stand_recovery.onnx` | `2026-08-07_16-37-50_g1_extreme_stand_v4_jerk_limited_from_model2999_full_20260807/model_20.pt` | `eb2e993220d2e4a343602dfa1556064fce440ce230580803930f7b82151eab6e` |

## Velocity tracking

- 来源会话：`019fd708-6fe7-7943-9cd2-53dae063ce9c`
- checkpoint SHA256：
  `12428d0a2312de12f49e43eeade0c3bd73cdb09634e0ad5a2c0fa9f1f55ca9af`
- 使用位置：仅在 High Mode 1、Low Mode 1 收到新鲜非零速度时选择
  `free_walk.onnx`。
- 零速度、速度缺失或超时仍按状态机设计切换到站立恢复模型。

## ArmHack walk

- 来源会话：`019f5fd3-0663-7142-9a9e-ade03c8480a7`
- 正式 checkpoint：`model_armhack_walk_two_goal_robust.pt`
- checkpoint SHA256：
  `00a36bd58ce63c39e7c441d507e4111df92281ea1f5b59fe6869fb28830a00ce`
- 使用位置：High Mode 3；只有导航 Low Mode 1 的 `[vx,vy,yaw_rate]` 进入模型。
- 模型保留原 `model_10990` 主 actor，并为严格侧移和严格纯 yaw 命令加入稳健专家。
- 双臂动作仍不是当前帧模型输入，只在推理后覆盖 14DoF 双臂输出；覆盖后的完整
  29DoF action 作为下一帧 `previous_action`。
- 三种推荐双臂姿态的 12 个专项 MuJoCo 场景，以及高/低被动关节参数压力测试均已通过。

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

三份模型均满足：

```text
input:  obs, float32, [1,96]
output: actions, float32, [1,29]
frequency: 50 Hz
action_scale: 0.25
joint order: controller.policy_joint_names
```

逐模型的机器可读合同位于：

- `models/free_walk.contract.json`
- `models/walk_with_object.contract.json`
- `models/extreme_stand_recovery.contract.json`
