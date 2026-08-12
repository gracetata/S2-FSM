# 应用层 / 翻译层接入说明

应用层只负责告诉状态机“当前要做什么”，不要向状态机发送底层关节命令。

状态机另以 `/hecbot/locomotion/policy_input`（`std_msgs/msg/String`）输出五个
模型每次 ONNX 推理前的完整 96 维输入包。应用层无需发布或回传该 topic；需要记录
实验时可直接订阅或使用 `ros2 bag record`。

## 1. 启动握手

订阅：

| topic | 类型 | 含义 |
| --- | --- | --- |
| `/hecbot/locomotion/initialized` | `std_msgs/msg/Bool` | 收到 `data: true` 后状态机才可接收业务模式 |

该 topic 使用 reliable + transient-local QoS。应用层晚启动也能收到最近一次
`true`。状态机或整机重启后，应用层必须重新等待本次启动的 `true`，然后重新发送
当前 high mode。

## 2. 应用层唯一需要发送的消息

发布：

| topic | 类型 | 合法值 |
| --- | --- | --- |
| `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `1`、`2`、`3`、`4` |

值的含义：

| high mode | 功能 | 还需要哪个模块发数据 |
| --- | --- | --- |
| `1` | 导航：速度行走或精确到点 | 导航层发送 low mode 和导航三元组 |
| `2` | 原地站立双臂操作 | 双臂操作层发送 14DoF 双臂命令 |
| `3` | 持物/双臂行走 | 导航层发送 low 1 速度；双臂层发送 14DoF 输出覆盖命令 |
| `4` | 鲁棒站立恢复独立测试 | 不需要其他模块；command 固定 `[0,0,0]` |

ROS 2 示例：

```bash
ros2 topic pub --once /hecbot/locomotion/high_level_mode \
  std_msgs/msg/UInt8 "{data: 1}"
```

## 3. 切换行为

- mode 1 和 mode 2 切入后，状态机先运行配置时长的
  `stand_recovery + [0,0,0]`，再进入目标模型。
- mode 3 和 mode 4 直接切入目标模型。
- 重复发送当前 high mode 不会重新开始站立计时，可用于上游状态重发。
- 切入 mode 1 后，在导航层的新 low mode 和导航参数到达前保持零速。
- 切入 mode 3 后，只有导航层的 low mode 1 速度会进入 `arm_walk` 模型；缺少
  low 1 或速度消息时模型速度输入为零。
- 切入 mode 2/3 后，在双臂操作层的新消息到达前保持切换前最后一帧双臂目标。
  双臂命令不作为当前帧独立模型输入，只在推理后覆盖输出；覆盖后实际 action 会
  进入下一帧 `previous_action`。
- high 1/low 1 收到新鲜速度后使用 `free_walk`，明确的 `[0,0,0]` 也不切换模型；
  速度尚未收到或超时时才使用恢复模型。
- high 1 从 low 1 切到 low 2 时，第一次识别切换就运行 `stand_recovery`；
  `stand_duration_s` 结束后才进入 `accurate_arrival`。
- mode 4 不读取 low mode、导航或双臂输入；恢复模型的完整 29DoF 输出直接控制
  机器人。同一模型用于内部站立，但内部切换不会把 high mode 改写成 `4`。
- 初始化后未收到任何 high mode 时，状态机保持
  `high_mode=None + stand_recovery + [0,0,0]`。
- 非法值会被拒绝，并使状态机回到 `stand_recovery + [0,0,0]` 安全等待。

## 4. 应用层不需要发送的内容

应用层不发送以下消息：

- `/hecbot/locomotion/low_level_mode`：由导航层发送；
- `/hecbot/locomotion/navigation_command`：由导航层发送；
- `/hecbot/upper_body_cmd`：由双臂操作层发送；
- 任何 `rt/lowcmd`：只能由本状态机发布。

联调时可运行键盘模拟器代替应用层切换 high mode。正式接入应用层后不需要启动
键盘模拟器。
