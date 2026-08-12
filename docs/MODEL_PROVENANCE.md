# 当前部署模型来源

本页记录状态机仓库中四份已确认的部署策略。状态机仍使用原来的模型名称和
路由，不需要修改 ROS 2 接口或启动命令。

| 状态机功能 | 部署文件 | 训练产物 | ONNX SHA256 |
| --- | --- | --- | --- |
| High 1 / Low 1 速度跟踪（含零速） | `models/free_walk.onnx` | `future5090_model9996_two_goal_robust_v3_20260809/model_final.pt` | `5de4f2852286919395a3579e106ccdee0164cc220e9bd1d9865a64912e1a0dcd` |
| High 1 / Low 2 精确到点 | `models/accurate_arrival.onnx` | 2026-08-12 提供的 `accurate_arrival.onnx`；checkpoint 未提供 | `ad4673888f9a82652ef11380ee00bd131cf2a8dfa56e734a3ede193799130edb` |
| High 3 / Low 1 双臂截持行走 | `models/walk_with_object.onnx` | `checkpoint/walk/armhack_two_goal_20260811/model_armhack_walk_two_goal_robust.pt` | `011c4bbf47846285328045967e45b78274d3c81c7cff315fc19b5c9aae095d5b` |
| 仅 High 4 鲁棒站立恢复 | `models/extreme_stand_recovery.onnx` | `2026-08-07_16-37-50_g1_extreme_stand_v4_jerk_limited_from_model2999_full_20260807/model_20.pt` | `eb2e993220d2e4a343602dfa1556064fce440ce230580803930f7b82151eab6e` |

## Velocity tracking

- 来源会话：`019fd708-6fe7-7943-9cd2-53dae063ce9c`
- checkpoint SHA256：
  `12428d0a2312de12f49e43eeade0c3bd73cdb09634e0ad5a2c0fa9f1f55ca9af`
- 使用位置：High Mode 1、Low Mode 1 的速度跟踪，以及初始化、等待、模式过渡、
  导航速度缺失/超时和安全回退等所有内部站立；站立 command 固定为 `[0,0,0]`。

## Accurate arrival

- 使用位置：High Mode 1 / Low Mode 2。
- command：导航层持续发送的机体坐标系闭环误差
  `[dx_body,dy_body,dyaw]`。
- 2026-08-12 提供的替换文件与仓库中的部署文件逐字节 SHA256 相同；仓库当前模型
  已经是该版本，因此没有对 ONNX 二进制制造无意义改动。
- 已重新验证 ONNX 输入为 `obs / tensor(float) / [1,96]`，输出为
  `actions / tensor(float) / [1,29]`。
- 训练 checkpoint 和训练会话未随文件提供，文档不推测未知来源。

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
- 使用位置：仅显式 High Mode 4。初始化、速度缺失/超时、安全回退和
  Low Mode 1→2 过渡均不再使用该模型。
- 会话中 2026-08-09 生成的 `model_0.pt` 是启动 smoke 模型，不是正式交付模型，
  本仓库没有采用它。

## 共同部署合同

四份模型均满足：

```text
input:  obs, float32, [1,96]
output: actions, float32, [1,29]
frequency: 50 Hz
action_scale: 0.25
joint order: controller.policy_joint_names
```

逐模型的机器可读合同位于：

- `models/free_walk.contract.json`
- `models/accurate_arrival.contract.json`
- `models/walk_with_object.contract.json`
- `models/extreme_stand_recovery.contract.json`
