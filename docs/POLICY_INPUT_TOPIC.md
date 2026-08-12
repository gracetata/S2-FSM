# 模型推理前输入 Topic

## 1. 接口

| 方向 | topic | 类型 | 产生时机 |
| --- | --- | --- | --- |
| 输出 | `/hecbot/locomotion/policy_input` | `std_msgs/msg/String` | 每个 50 Hz 控制帧、ONNX `infer()` 调用前 |

五个模型使用同一个 topic。每条消息只对应一个实际推理帧，`model` 字段用于区分：

- `free_walk`：high 1 / low 1 非零速度行走；
- `accurate_arrival`：high 1 / low 2 精确到点；
- `arm_stand`：high 2 原地双臂；
- `arm_walk`：high 3 持物行走；
- `stand_recovery`：初始化、安全等待、模式过渡、零速站立或 high 4。

## 2. JSON 合同

`String.data` 是严格 JSON 对象，顶层字段如下：

| 字段 | 含义 |
| --- | --- |
| `schema` | 固定为 `hecbot.policy_input.v1` |
| `stage` | 固定为 `pre_inference`，表示数据在推理前捕获 |
| `frame` | 从控制进程启动开始递增的推理帧号 |
| `wall_time` | 带时区的 ISO 8601 实际时间 |
| `monotonic_time_s` | 控制循环单调时钟时间 |
| `model` | 本帧实际运行的模型名 |
| `high_mode` / `low_mode` | 状态机业务模式；尚未收到 high mode 时可为 `null` |
| `standing_transition` | 是否处于内部站立恢复过渡 |
| `navigation_input` | 状态机选中值、语义和真正写入 observation 的三元组 |
| `input` | ONNX 输入张量的完整描述和值 |

`input` 固定包含：

```text
name:  obs
dtype: float32
shape: [1, 96]
```

`input.observation` 的布局：

| observation 下标 | 维数 | 内容 |
| --- | ---: | --- |
| `0:3` | 3 | 缩放后的角速度 |
| `3:6` | 3 | 投影重力方向 |
| `6:9` | 3 | 本帧模型 command |
| `9:38` | 29 | 相对默认角并缩放后的关节位置，policy 顺序 |
| `38:67` | 29 | 缩放后的关节速度，policy 顺序 |
| `67:96` | 29 | 上一帧实际执行 action，policy 顺序 |

`input.policy_joint_names` 在每个包内给出上述三个 29DoF 切片使用的确切顺序，避免
消费者误用 `/hecbot/whole_body_state` 的 motor 顺序。

双臂消息始终只在推理后覆盖 `arm_stand/arm_walk` 输出，不会作为本帧额外模型
输入。覆盖后的 action 是机器人实际采用的 action，因此下一帧
`previous_action` 一定包含覆盖结果。

## 3. 查看和录包

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash

ros2 topic hz /hecbot/locomotion/policy_input
ros2 topic echo --once /hecbot/locomotion/policy_input std_msgs/msg/String
```

只录模型输入：

```bash
ros2 bag record -o policy_input_bag \
  /hecbot/locomotion/policy_input
```

同时录制实测关节角，便于离线对齐：

```bash
ros2 bag record -o policy_input_and_joint_state_bag \
  /hecbot/locomotion/policy_input \
  /hecbot/whole_body_state
```

停止录制按 `Ctrl+C`。查看包信息或回放：

```bash
ros2 bag info policy_input_bag
ros2 bag play policy_input_bag
```

## 4. 连续性判断

消费者以 `frame` 判断连续性，不要仅依赖 ROS 接收时间。正常运行时帧号连续递增。
ROS 节点会缓存并顺序补发短暂调度延迟期间积累的输入；如果 ROS 进程长时间无法读取，
导致运行时 256 帧历史缓存被覆盖，节点会明确打印跳帧范围，不会把不连续数据伪装成
连续记录。需要完整实验数据时，应在动作开始前先启动 `ros2 bag record`，并确认
终端没有 `policy-input topic skipped runtime frames` 警告。
