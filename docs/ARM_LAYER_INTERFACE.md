# 双臂层接入说明

双臂层负责在 high mode 2 和 high mode 3 下，向状态机发送左右双臂共 14DoF 的
目标位置、目标速度和融合权重。双臂层不发布 high mode，也不直接发布 Unitree
`rt/lowcmd`。

## 1. 启动握手

双臂层启动后先订阅：

| topic | 类型 | 含义 |
| --- | --- | --- |
| `/hecbot/locomotion/initialized` | `std_msgs/msg/Bool` | 收到 `data: true` 后状态机可接收双臂命令 |

该 topic 使用 reliable + transient-local QoS，双臂层晚启动也能收到最近一次
`true`。状态机重启后，双臂层应重新等待初始化完成，再恢复发送。

## 2. 双臂层发布接口

| topic | 类型 | 推荐频率 |
| --- | --- | ---: |
| `/hecbot/upper_body_cmd` | `std_msgs/msg/String` | 20 Hz |

`String.data` 必须是严格 JSON，只允许以下五个字段：

```json
{
  "schema": "hecbot.upper_body_command.v1",
  "seq": 123456789,
  "arm_q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "arm_dq": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "weight": 1.0
}
```

字段合同：

| 字段 | 类型 | 要求 |
| --- | --- | --- |
| `schema` | string | 必须严格等于 `hecbot.upper_body_command.v1` |
| `seq` | integer | 非负；每条消息必须比上一条严格增大 |
| `arm_q` | float[14] | 目标关节角，单位 rad；必须都是有限数值 |
| `arm_dq` | float[14] | 目标关节速度，单位 rad/s；必须都是有限数值 |
| `weight` | float | 必须位于 `[0,1]` |

不允许缺字段、额外字段、数组长度错误、`NaN` 或无穷值。

## 3. 14DoF 数组顺序

`arm_q` 和 `arm_dq` 使用完全相同的顺序：

| 下标 | 关节名称 |
| ---: | --- |
| 0 | `left_shoulder_pitch_joint` |
| 1 | `left_shoulder_roll_joint` |
| 2 | `left_shoulder_yaw_joint` |
| 3 | `left_elbow_joint` |
| 4 | `left_wrist_roll_joint` |
| 5 | `left_wrist_pitch_joint` |
| 6 | `left_wrist_yaw_joint` |
| 7 | `right_shoulder_pitch_joint` |
| 8 | `right_shoulder_roll_joint` |
| 9 | `right_shoulder_yaw_joint` |
| 10 | `right_elbow_joint` |
| 11 | `right_wrist_roll_joint` |
| 12 | `right_wrist_pitch_joint` |
| 13 | `right_wrist_yaw_joint` |

## 4. high mode 对双臂命令的解释

high mode 由应用层发布：

| high mode | 状态机功能 | 双臂命令 |
| --- | --- | --- |
| `1` | 导航 | 不使用双臂层输入 |
| `2` | 原地站立双臂操作 | 使用 `/hecbot/upper_body_cmd` |
| `3` | 双臂/持物行走 | 使用 `/hecbot/upper_body_cmd` |
| `4` | 鲁棒站立恢复独立测试 | 不使用双臂层输入 |

每次进入 mode 2 或 mode 3，状态机会作废切换前收到的双臂消息。因此双臂层应持续
以 20 Hz 发布；应用层切换 high mode 后，下一条新消息会自动成为当前模式的有效
输出覆盖命令。不要只在状态机切模前发布一次目标。

mode 2 会先运行配置时长的 `stand_recovery + [0,0,0]`，再进入双臂站立模型；
mode 3 直接切换。双臂层可以持续发布，不需要自行实现这段等待。

mode 3 的行走速度由导航层通过 low mode 1 和
`/hecbot/locomotion/navigation_command` 提供。双臂层不发送导航速度。

mode 2 和 mode 3 使用完全相同的 standing-grasp 专用 Kp/Kd。双臂层不需要在
消息中发送增益。

### high mode 3 推荐双臂姿态

持物行走（high mode 3）优先使用
[`models/walk_with_object_arm_pose_set.json`](../models/walk_with_object_arm_pose_set.json)
中的三个模型预设姿态。文件中的 `left` 后拼接 `right`，即为
`/hecbot/upper_body_cmd` 的 14 维 `arm_q`：

| 键盘模拟键 | JSON 姿态名 | 含义 | high mode 3 建议 |
| --- | --- | --- | --- |
| `z` | `pos1_back` | 双臂靠后 | 推荐预设 |
| `x` | `pos2_down` | 双臂下垂 | 推荐预设 |
| `c` | `pos3_front` | 双臂靠前 | 推荐预设 |
| `b` | 不在该 JSON 中 | 额外的对称持物联调姿态 | 仅联调，非模型预设 |

因此，当前键盘模拟器的 `z`、`x`、`c`、`b` 都能设置双臂姿态，但在 high mode 3
持物行走时，建议优先使用 `z`、`x`、`c` 对应的三个模型预设。`b` 来自
`config/simulator_presets.json`，用于额外接口联调，不应被理解成
`walk_with_object_arm_pose_set.json` 的第四个预设。

正式双臂层不接收键盘字符，而应读取上述 JSON 的 `left`、`right` 数组，按本文件
第 3 节的顺序拼成 `arm_q` 后持续发布。切换姿态时仍须由双臂层生成平滑、限速且
通过碰撞检查的过渡轨迹；不要将预设关节角作为单帧跳变命令直接发送到实机。
通常将 `arm_dq` 填为轨迹规划器给出的关节速度，并根据所需融合程度设置
`weight`。

## 5. `seq` 生成规则

`seq` 在同一次状态机运行期间必须全局严格递增，包括 mode 2 和 mode 3 之间切换。
双臂节点重启后也不能从零重新计数，否则状态机会拒绝新消息。

推荐直接用单调时钟纳秒值：

```python
from time import monotonic_ns

message["seq"] = monotonic_ns()
```

同一进程内如果可能在一个纳秒内生成多条消息，应保存上一序号并使用：

```python
sequence = max(monotonic_ns(), previous_sequence + 1)
```

## 6. 位置、速度和权重如何生效

双臂消息不是当前帧 ONNX 的独立输入。每帧固定顺序是：

1. 模型只根据 observation 完成推理；high mode 3 的 observation 包含导航 low 1
   速度；
2. 推理结束后，控制器才读取双臂命令；
3. 控制器用 `arm_q/arm_dq` 覆盖模型输出的 14 个双臂关节。

覆盖后的完整实际 action 会写入下一帧 observation 的 `previous_action`，这是
策略原有的历史 action 输入，并不是把 `arm_q/arm_dq` 作为当前帧额外输入。
机器人实际运动后产生的实测关节角也属于正常状态反馈。

- `weight=1.0`：完全使用外部 `arm_q` 和 `arm_dq`。
- `weight=0.0`：保持切入当前双臂模型时的双臂位置基线，外部速度为零。
- `0<weight<1`：目标位置在切模基线和 `arm_q` 之间线性融合；`arm_dq` 同样乘以
  `weight`。
- 双臂目标收到后在下一控制帧直接生效，状态机不生成中间轨迹，也不做位置、速度或
  加速度限幅。平滑、限速、限位和碰撞保护必须由双臂层在发送前完成。

## 7. 超时行为

当前配置的双臂超时 `arm_timeout_s` 为 0.20 秒，因此推荐 20 Hz 持续发送，消息间隔
不要接近或超过 0.20 秒。

命令超时后：

- 双臂目标位置保持上一控制帧已经执行的目标；
- 外部目标速度变为零；
- high mode 不会自动改变。

恢复发送一条序号更大的合法消息后，外部双臂控制继续生效。

## 8. 获取实测双臂关节角

状态机以 50 Hz 发布：

| topic | 类型 | 内容 |
| --- | --- | --- |
| `/hecbot/whole_body_state` | `std_msgs/msg/String` | 29DoF 实测关节角 JSON 数组，单位 rad |

双臂关节对应该数组下标 `15:29`，顺序与本文件的 14DoF 顺序完全相同。该 topic
来自 Unitree `rt/lowstate`，是反馈角，不是双臂层发送的目标角。

## 9. ROS 2 命令行示例

以下示例仅用于接口联调，`seq` 必须比状态机已经收到的上一条序号更大：

```bash
ros2 topic pub --once /hecbot/upper_body_cmd std_msgs/msg/String \
  "{data: '{\"schema\":\"hecbot.upper_body_command.v1\",\"seq\":123456789,\"arm_q\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],\"arm_dq\":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],\"weight\":1.0}'}"
```

真实双臂节点必须为每条消息生成新序号，不能把上述固定 `seq` 示例直接改为
`--rate 20`，否则第二条开始会因为序号没有增加而被拒绝。

正式双臂层接入后，键盘模拟器保持默认静默状态即可，不要按 `k` 开启其双臂参数
发布。
