# 29DoF 关节顺序与模型转换

项目同时保留两套 29DoF 顺序：

- `policy_joint_names`：ONNX 模型、observation、action 和阻抗 YAML 使用；
- `motor_joint_names`：Unitree LowState/LowCmd 和 `/hecbot/whole_body_state` 使用。

两套列表包含完全相同的 29 个关节，只是排列不同。控制器按关节名称构造置换，
不依赖人工猜测下标。

## 1. 两套完整顺序

| 下标 | `policy_joint_names`（模型顺序） | `motor_joint_names`（电机/对外顺序） |
| ---: | --- | --- |
| 0 | `left_hip_pitch_joint` | `left_hip_pitch_joint` |
| 1 | `right_hip_pitch_joint` | `left_hip_roll_joint` |
| 2 | `waist_yaw_joint` | `left_hip_yaw_joint` |
| 3 | `left_hip_roll_joint` | `left_knee_joint` |
| 4 | `right_hip_roll_joint` | `left_ankle_pitch_joint` |
| 5 | `waist_roll_joint` | `left_ankle_roll_joint` |
| 6 | `left_hip_yaw_joint` | `right_hip_pitch_joint` |
| 7 | `right_hip_yaw_joint` | `right_hip_roll_joint` |
| 8 | `waist_pitch_joint` | `right_hip_yaw_joint` |
| 9 | `left_knee_joint` | `right_knee_joint` |
| 10 | `right_knee_joint` | `right_ankle_pitch_joint` |
| 11 | `left_shoulder_pitch_joint` | `right_ankle_roll_joint` |
| 12 | `right_shoulder_pitch_joint` | `waist_yaw_joint` |
| 13 | `left_ankle_pitch_joint` | `waist_roll_joint` |
| 14 | `right_ankle_pitch_joint` | `waist_pitch_joint` |
| 15 | `left_shoulder_roll_joint` | `left_shoulder_pitch_joint` |
| 16 | `right_shoulder_roll_joint` | `left_shoulder_roll_joint` |
| 17 | `left_ankle_roll_joint` | `left_shoulder_yaw_joint` |
| 18 | `right_ankle_roll_joint` | `left_elbow_joint` |
| 19 | `left_shoulder_yaw_joint` | `left_wrist_roll_joint` |
| 20 | `right_shoulder_yaw_joint` | `left_wrist_pitch_joint` |
| 21 | `left_elbow_joint` | `left_wrist_yaw_joint` |
| 22 | `right_elbow_joint` | `right_shoulder_pitch_joint` |
| 23 | `left_wrist_roll_joint` | `right_shoulder_roll_joint` |
| 24 | `right_wrist_roll_joint` | `right_shoulder_yaw_joint` |
| 25 | `left_wrist_pitch_joint` | `right_elbow_joint` |
| 26 | `right_wrist_pitch_joint` | `right_wrist_roll_joint` |
| 27 | `left_wrist_yaw_joint` | `right_wrist_pitch_joint` |
| 28 | `right_wrist_yaw_joint` | `right_wrist_yaw_joint` |

例如右踝：

| 关节 | 模型顺序下标 | 电机/whole-body-state 下标 |
| --- | ---: | ---: |
| `right_ankle_pitch_joint` | 14 | 10 |
| `right_ankle_roll_joint` | 18 | 11 |

因此查看 `/hecbot/whole_body_state` 时右踝使用下标 `10/11`；查看模型 action、
observation 或 `impedancepara*.yaml` 时右踝使用下标 `14/18`。

## 2. 两个固定置换

当前配置生成以下置换：

```text
motor → policy:
[0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]

policy → motor:
[0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28]
```

含义不是修改数值，而是按下标重新排列：

```python
policy_values = motor_values[motor_to_policy]
motor_values = policy_values[policy_to_motor]
```

代码中的置换由两套关节名称实时生成：

```python
motor_to_policy = [
    motor_joint_names.index(name)
    for name in policy_joint_names
]

policy_to_motor = [
    policy_joint_names.index(name)
    for name in motor_joint_names
]
```

如果修改任何一套名称或顺序，必须保证两边仍是同一组 29 个唯一关节。不要手工修改
一个置换数组而不修改关节名称合同。

## 3. LowState 到模型的转换

```text
Unitree LowState（motor 顺序）
        │ motor_to_policy
        ▼
关节角、关节速度（policy 顺序）
        ▼
96 维 observation
        ▼
ONNX 模型
```

96 维 observation 的布局：

| observation 下标 | 内容 | 内部关节顺序 |
| --- | --- | --- |
| `0:3` | 机体角速度 | 不适用 |
| `3:6` | 重力方向 | 不适用 |
| `6:9` | 导航 command | 不适用 |
| `9:38` | 29DoF 关节位置 | policy |
| `38:67` | 29DoF 关节速度 | policy |
| `67:96` | 上一帧实际 action | policy |

## 4. 模型输出到 LowCmd 的转换

模型输出的 29 维 action 是 policy 顺序。控制器在 policy 顺序下完成：

1. `target = default_angles + action * action_scale`；
2. mode 2/3 推理后覆盖双臂目标；
3. 计算下一帧 `previous_action`；
4. 选择本模式的 policy 顺序 Kp/Kd；
5. 用 `policy_to_motor` 同时重排目标角、目标速度、Kp 和 Kd；
6. 写入 Unitree LowCmd。

```text
ONNX action、default angles、Kp/Kd（policy 顺序）
        │ 双臂输出覆盖仍在 policy 顺序完成
        │ policy_to_motor
        ▼
Unitree LowCmd（motor 顺序）
```

`impedancepara.yaml` 和 `impedancepara_default.yaml` 的所有 29 维数组必须使用
policy 顺序。错误地按 motor 顺序填写会把增益和默认角施加到其他关节。

## 5. 双臂 14DoF 接口

`/hecbot/upper_body_cmd` 使用左臂 7DoF 后接右臂 7DoF：

```text
left shoulder pitch/roll/yaw, left elbow,
left wrist roll/pitch/yaw,
right shoulder pitch/roll/yaw, right elbow,
right wrist roll/pitch/yaw
```

它等于 `motor_joint_names[15:29]`，但不等于 policy 数组中的连续切片。控制器仍按
关节名称找到 policy 下标，再在推理后覆盖相应的 14 个 action 分量。
