# 人形机器人运动控制器

本项目是 Unitree G1 29DoF 的四模式有限状态机。入口为
`scripts/locomotion_controller_node`。程序启动时一次性加载并预热五个 ONNX
模型，然后启动唯一的 50 Hz 推理/`LowCmd` 控制线程。首帧发送后，控制器继续以
`stand_recovery + [0,0,0]` 站立 2 秒；全程健康后节点才发布初始化完成消息，并开始
以 50 Hz 发布实测 29DoF 关节角。

实机接管前，程序会读取当前 MotionSwitcher 状态。无论当前 Loco FSM ID 是什么，
只要仍有高层模式，程序就调用 `ReleaseMode()`，并确认 MotionSwitcher 模式名严格
变为空，即进入低层调试模式；已经处于空模式时直接继续。查询失败、返回格式异常、
释放失败或超时都会终止初始化，且不会进入默认姿态运动。

## 模式

| high-level mode | low-level mode | 模型 | 有效上游输入 |
| --- | --- | --- | --- |
| `1` 导航 | `1` 非零速度 | `free_walk.onnx` | 导航三元组 `[vx, vy, yaw_rate]` |
| `1` 导航 | `1` 零速/缺失/超时 | `extreme_stand_recovery.onnx` | command 固定 `[0,0,0]` |
| `1` 导航 | `2` 位置 | `accurate_arrival.onnx` | 导航三元组 `[dx_body, dy_body, dyaw]` |
| `2` 双臂站立 | 不使用 | `standing_grasp.onnx` | 14DoF 双臂输出覆盖 |
| `3` 双臂行走 | `1` 速度 | `walk_with_object.onnx` | 导航 `[vx,vy,yaw_rate]` + 14DoF 双臂输出覆盖 |
| `4` 鲁棒站立恢复 | 不使用 | `extreme_stand_recovery.onnx` | 无；command 固定 `[0,0,0]` |

当前 velocity tracking、ArmHack walk 和 extreme stand 模型的训练来源、checkpoint 与 ONNX 哈希
见 [`docs/MODEL_PROVENANCE.md`](docs/MODEL_PROVENANCE.md)。

mode 2 不读取导航输入，模型运动命令固定为 `[0,0,0]`。mode 3 只接受 low mode 1
的导航速度三元组，并把它写入 `arm_walk` 模型的 command observation；未收到
low mode 1、速度尚未到达或速度超时时使用 `[0,0,0]`。

`stand_recovery` 是统一站立策略：初始化等待、未收到 high mode、非法模式安全
回退、mode 1/2 切入等待、high 1/low 1 的零速或超时，以及 low 1→low 2 的切换
等待都使用该模型，不再用 `free_walk + [0,0,0]` 实现站立。

双臂消息不是当前帧 ONNX 的独立输入。控制器先完成模型推理，再用外部 14DoF
双臂位置和速度覆盖模型输出中的双臂关节。覆盖后实际执行的完整 action 会按策略
合同写入下一帧 observation 的 `previous_action`。

应用层切换到 mode 1 或 mode 2 时，控制器先选择 `stand_recovery`，向模型输入严格
的零 command 并保持配置的 `stand_duration_s`，然后进入目标模式。切换到 mode 3/4
不执行这一步。mode 4 不读取 low mode、导航或双臂命令，直接运行恢复模型自身的
29DoF 输出。重复发布当前 high-level mode 不会重新开始站立计时。

high mode 1 内从 low mode 1 切换到 low mode 2 时也执行同一段
`stand_duration_s` 的 `stand_recovery` 推理。切换瞬间旧速度被清除；等待结束后
才进入 `accurate_arrival`。如果位置参数尚未到达，位置模型以 `[0,0,0]` 启动，
不会把旧速度解释成位置误差。

关节默认角、Kp 和 Kd 从根目录
[`impedancepara.yaml`](impedancepara.yaml) 严格加载。mode 2 使用
`standing_grasp` 专用默认角和增益。mode 3 使用通用默认角，但 Kp/Kd 与 mode 2
完全相同；初始化、站立过渡、mode 1 和 mode 4 使用通用 Kp/Kd。
文件必须保留六个 29 维数组，缺字段、额外字段、长度错误、非有限值或负增益都会
使启动失败。

## 启动

目标环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Python 3.12。ONNX Runtime、
CycloneDDS 与 Unitree SDK2 安装在配置指定的 Conda 环境中。

当前 NUC 的已验证路径：

- SSH：`hecbot@192.168.50.113`
- 代码：`/home/wenduo/locomotion_controller`
- ROS 2：`/opt/ros/jazzy`
- 控制运行时：`/home/hecbot/miniconda3/envs/locomotion_controller`
- 机器人网卡：`enp86s0`
- 机器人地址：`192.168.123.161`

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller

chmod +x scripts/locomotion_controller_*
colcon build --symlink-install --packages-select locomotion_controller
source install/setup.bash

ros2 launch locomotion_controller locomotion_controller.launch.py
```

### 修改代码后重新编译

先停止正在运行的控制器和键盘模拟器。每次修改代码、主配置、
`impedancepara.yaml`、模型、launch 文件、`CMakeLists.txt` 或 `package.xml` 后，
在 NUC 上执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller

colcon build --symlink-install --packages-select locomotion_controller
source /home/wenduo/locomotion_controller/install/setup.bash
```

必须看到 `Summary: 1 package finished` 后再启动程序。即使仍在同一个终端，
编译完成后也必须重新执行 `source install/setup.bash`。

以后每打开一个新终端，只需重新加载 ROS 2 和本项目环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash
```

然后启动：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

使用其他配置：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py \
  config_file:=/absolute/path/to/locomotion_controller.yaml
```

启动前必须确保没有其他程序发布 `rt/lowcmd`。实机运行还必须核对
`network_interface`、`robot_ip` 和 `confirm_real_robot`。控制器初始化时会主动
释放当前 MotionSwitcher 高层模式，并严格确认进入低层调试模式，不要求启动前的
Loco FSM ID 为 `0`。

面向不同接入方的说明：

- [应用层 / 翻译层接入](docs/APPLICATION_LAYER_INTERFACE.md)
- [双臂层接入](docs/ARM_LAYER_INTERFACE.md)
- [整体启动脚本接入](docs/SYSTEM_STARTUP_INTEGRATION.md)
- [状态机极简使用方法](docs/QUICK_START.md)
- [极简 Kp/Kd YAML 替换](docs/IMPEDANCE_YAML_REPLACEMENT.md)
- [29DoF 两套顺序与模型转换](docs/JOINT_ORDER_AND_MAPPING.md)
- [High Mode 4 鲁棒站立恢复测试](docs/STAND_RECOVERY_MODE.md)
- [Mode 4 极简真机测试](docs/MODE4_QUICK_TEST.md)

### NUC 运行前检查

以下命令不会发布 `LowCmd`：

```bash
cd /home/wenduo/locomotion_controller

PYTHONPATH=src \
  /home/hecbot/miniconda3/envs/locomotion_controller/bin/python \
  -m unittest discover -s tests -v

PYTHONNOUSERSITE=1 \
  /home/hecbot/miniconda3/envs/locomotion_controller/bin/python \
  -c "import numpy, onnxruntime, cyclonedds, unitree_sdk2py; print('runtime imports: OK')"

ping -c 1 192.168.123.161
ip -br address show enp86s0
```

五个模型预热检查：

```bash
cd /home/wenduo/locomotion_controller

PYTHONNOUSERSITE=1 \
  /home/hecbot/miniconda3/envs/locomotion_controller/bin/python \
  -c "import glob, onnxruntime as ort; paths=glob.glob('models/*.onnx'); [ort.InferenceSession(path, providers=['CPUExecutionProvider']) for path in paths]; print('loaded models:', len(paths))"
```

输出必须为 `loaded models: 5`。

## ROS 2 接口

程序只接收以下四个输入 topic：

| 方向 | topic | 类型 | 合同 |
| --- | --- | --- | --- |
| 输入 | `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `1`、`2`、`3` 或 `4` |
| 输入 | `/hecbot/locomotion/low_level_mode` | `std_msgs/msg/UInt8` | high mode 1 支持 `1/2`；high mode 3 只使用 `1` |
| 输入 | `/hecbot/locomotion/navigation_command` | `std_msgs/msg/Float32MultiArray` | 恰好 3 个有限数值；high 1/low 1 和 high 3/low 1 表示速度 |
| 输入 | `/hecbot/upper_body_cmd` | `std_msgs/msg/String` | 严格 JSON；只在推理后覆盖 mode 2/3 的双臂输出 |

程序输出：

| 方向 | topic | 类型 | 合同 |
| --- | --- | --- | --- |
| 输出 | `/hecbot/locomotion/initialized` | `std_msgs/msg/Bool` | 完成五模型预热、实机接管、首帧发送和 2 秒恢复策略站立后发布 `true` |
| 输出 | `/hecbot/whole_body_state` | `std_msgs/msg/String` | 初始化后以 50 Hz 发布实测 29DoF 关节角；`data` 是恰好 29 个有限数值的紧凑 JSON 数组，单位 rad |

初始化 topic 使用 reliable + transient-local QoS，初始化完成后启动的订阅者也能收到
最近一次 `true`。

`/hecbot/whole_body_state` 发布的是 `rt/lowstate` 反馈角，不是策略目标角。
`String.data` 示例：

```json
[0.01,-0.02,0.0,0.31,-0.18,0.01,0.02,-0.01,0.0,0.30,-0.19,-0.01,0.0,0.0,0.0,0.2,-0.2,0.0,0.9,0.0,0.0,0.0,-0.2,0.2,0.0,0.9,0.0,0.0,0.0]
```

数组下标固定如下，配置加载器会强制校验该顺序：

| 下标 | 关节名称 | 下标 | 关节名称 |
| ---: | --- | ---: | --- |
| 0 | `left_hip_pitch_joint` | 15 | `left_shoulder_pitch_joint` |
| 1 | `left_hip_roll_joint` | 16 | `left_shoulder_roll_joint` |
| 2 | `left_hip_yaw_joint` | 17 | `left_shoulder_yaw_joint` |
| 3 | `left_knee_joint` | 18 | `left_elbow_joint` |
| 4 | `left_ankle_pitch_joint` | 19 | `left_wrist_roll_joint` |
| 5 | `left_ankle_roll_joint` | 20 | `left_wrist_pitch_joint` |
| 6 | `right_hip_pitch_joint` | 21 | `left_wrist_yaw_joint` |
| 7 | `right_hip_roll_joint` | 22 | `right_shoulder_pitch_joint` |
| 8 | `right_hip_yaw_joint` | 23 | `right_shoulder_roll_joint` |
| 9 | `right_knee_joint` | 24 | `right_shoulder_yaw_joint` |
| 10 | `right_ankle_pitch_joint` | 25 | `right_elbow_joint` |
| 11 | `right_ankle_roll_joint` | 26 | `right_wrist_roll_joint` |
| 12 | `waist_yaw_joint` | 27 | `right_wrist_pitch_joint` |
| 13 | `waist_roll_joint` | 28 | `right_wrist_yaw_joint` |
| 14 | `waist_pitch_joint` |  |  |

检查频率和内容：

```bash
ros2 topic hz /hecbot/whole_body_state
ros2 topic echo --once /hecbot/whole_body_state std_msgs/msg/String
```

### 导航示例

先选择 high mode 1 和速度 submode：

```bash
ros2 topic pub --once /hecbot/locomotion/low_level_mode \
  std_msgs/msg/UInt8 "{data: 1}"

ros2 topic pub --once /hecbot/locomotion/high_level_mode \
  std_msgs/msg/UInt8 "{data: 1}"

ros2 topic pub --rate 20 /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray "{data: [0.3, 0.0, 0.0]}"
```

切换到位置 submode 后，同一个三元组 topic 的语义变成机体坐标系目标误差：

```bash
ros2 topic pub --once /hecbot/locomotion/low_level_mode \
  std_msgs/msg/UInt8 "{data: 2}"

ros2 topic pub --rate 20 /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray "{data: [0.5, 0.0, 0.0]}"
```

控制器不会根据三元组内容猜测速度或位置语义。语义只由 low-level mode 决定。
速度三元组在收到后的下一控制帧直接写入模型 observation，不做时间插值或加速度
限幅；仅保留 `max_velocity_command` 的逐分量最大速度保护。

位置 submode 要求导航持续发送骨盆坐标系闭环误差，不接受全局目标坐标，也不会在
控制器内部计算或累计误差。完整合同、变换公式和数据示例见
[`docs/NAVIGATION_ADAPTER.md`](docs/NAVIGATION_ADAPTER.md)。

high mode 3 使用同一个 low mode 1 速度接口：

```bash
ros2 topic pub --once /hecbot/locomotion/low_level_mode \
  std_msgs/msg/UInt8 "{data: 1}"

ros2 topic pub --once /hecbot/locomotion/high_level_mode \
  std_msgs/msg/UInt8 "{data: 3}"

ros2 topic pub --rate 20 /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray "{data: [0.2, 0.0, 0.0]}"
```

该速度三元组是 `walk_with_object.onnx` 的模型输入。双臂层同时持续发送
`/hecbot/upper_body_cmd`，但双臂消息只覆盖推理后的 14DoF 双臂输出。

### 持续查看订阅值

已加载 ROS 2 和本项目环境后，用下面一个命令打开 Topic Monitor 窗口：

```bash
ros2 run rqt_topic rqt_topic
```

在窗口中展开并勾选以下各项的 value 列，即可持续刷新控制器的输入和关节角输出：
`/hecbot/locomotion/high_level_mode`、
`/hecbot/locomotion/low_level_mode`、
`/hecbot/locomotion/navigation_command`、`/hecbot/upper_body_cmd` 和
`/hecbot/whole_body_state`。
若命令不存在，安装 ROS Jazzy 的 `ros-jazzy-rqt-topic` 包。

## 键盘整机测试

测试节点会等待 `/hecbot/locomotion/initialized=true`。它默认不发布导航和双臂
参数，相当于启动前已经按过 `k`，只用键盘发送 high/low mode；按 `k` 开启后才以
20 Hz 持续发布导航和双臂输入，频率高于两类输入的超时要求。预设位于
[`config/simulator_presets.json`](config/simulator_presets.json)，包含四个双臂
姿态、三条速度轨迹和三个位置目标。键盘也支持三轴速度增减。按下双臂姿态键后，测试节点从下一次发布开始
直接发送该目标，不生成中间姿态，也不做加速度限幅。

`z/x/c` 分别对应
[`models/walk_with_object_arm_pose_set.json`](models/walk_with_object_arm_pose_set.json)
中的 `pos1_back`、`pos2_down`、`pos3_front`，是 high mode 3 持物行走的推荐
预设。`b` 是额外的对称持物联调姿态，不属于该模型姿态文件。

确保机器人周围无人、急停可用且没有其他 `rt/lowcmd` 发布者。终端 1：

```bash
ssh hecbot@192.168.50.113
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

看到 `stand-recovery initialization is complete` 后，在终端 2：

```bash
ssh -t hecbot@192.168.50.113
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 run locomotion_controller locomotion_controller_simulator
```

必须使用交互终端；通过 SSH 启动时保留 `-t`。键位：

| 按键 | 动作 |
| --- | --- |
| `1` / `2` / `3` / `4` | high-level mode 1 / 2 / 3 / 4 |
| `v` | low mode 1，high mode 1/3 的速度模式，同时清零当前导航输入 |
| `p` | low mode 2，仅 high mode 1 的位置模式，同时清零当前导航输入 |
| `0` | 取消轨迹；参数发布开启时持续发送 `[0,0,0]` |
| `k` | 开始/停止导航与双臂参数发布；启动时默认停止，high/low mode 按键始终有效 |
| `W` / `S` | `vx` 每次增加 / 减少 `0.05 m/s`；自动选择 low mode 1 |
| `A` / `D` | `vy` 每次增加 / 减少 `0.05 m/s`；自动选择 low mode 1 |
| `Q` / `E` | `yaw_rate` 每次增加 / 减少 `0.05 rad/s`；自动选择 low mode 1 |
| `f` | 固定轨迹：前进 3 秒后停止 |
| `5` | 左右横移后停止 |
| `6` | 左右原地转向后停止 |
| `7` | 位置目标：前方 0.3 m |
| `8` | 位置目标：左侧 0.2 m |
| `9` | 位置目标：左转约 20° |
| `z` / `x` / `c` | 双臂靠后 / 下垂 / 靠前；对应持物行走模型的三个推荐预设 |
| `b` | 额外的对称持物联调姿态；不是持物行走模型预设 |
| `Space` | 仅在 high mode 2/3 中循环切换 `z → x → c → z` |
| `h` | 重新显示帮助 |
| `Esc` / `Ctrl+C` | 退出测试节点 |

速度增减结果按 `max_velocity_command=[0.8,0.5,1.57]` 逐轴限幅。大小写按键效果
相同；按住按键可以利用终端按键重复连续调整。`Q/E` 已用于偏航速度，因此不再用
`q` 退出。

每次修改速度、High/Low Mode、双臂姿态、位置目标、固定轨迹或参数发布开关时，
键盘模拟器都会打印一行完整状态，例如：

```text
[KEYBOARD_STATE] event=velocity key -> W | publishing=on | high_mode=3 | low_mode=1 | navigation=[0.25, -0.10, 0.35] | arm_pose=down | trajectory=none
```

其中 `publishing=off` 表示当前值只保存在模拟器内部，不会发给状态机；此时先按
`k` 开启参数发布。

让真实导航或双臂节点独占发布参数时保持默认状态，不要按 `k`；`1/2/3/4`、`v/p`
仍可切换 high/low mode。完全使用模拟输入测试时按一次 `k` 开启参数发布。再次按
`k` 会立即停止 `/hecbot/locomotion/navigation_command` 和
`/hecbot/upper_body_cmd`，并取消内部速度轨迹。停止时不额外发布零值；最后一条
模拟参数分别在 `navigation_timeout_s` 和 `arm_timeout_s` 到期后失效。

### 终端调试日志

控制进程在每次 50 Hz ONNX 推理后，向控制器终端输出一行 `[INFERENCE]` JSON：

```text
[INFERENCE] {"event":"policy_inference","frame":125,"model":"free_walk","high_mode":1,"low_mode":1,"standing_transition":false,"navigation_input":{"semantics":"velocity","selected":[0.25,0.0,0.0],"model_input":[0.25,0.0,0.0]},"arm_output_override":null,"model_output":[...29 values...]}
```

- `selected`：状态机选择的速度或骨盆坐标系位置误差。
- `model_input`：实际写入 observation 的三元组；速度模式只做最大速度幅值保护，
  不做插值或加速度限幅，位置模式不做变换。
- `arm_output_override`：推理后用于覆盖双臂输出的位置、速度、权重和序号；它不是
  当前帧独立模型输入，但覆盖后 action 会进入下一帧 `previous_action`。无有效
  双臂消息时为 `null`。
- `model_output`：ONNX 本帧返回的原始 29 维 action，记录发生在双臂覆盖和模型切换插值之前。

每次启动控制器时，运行子进程的 stdout/stderr 还会完整保存到：

```text
/home/wenduo/locomotion_controller/log/runtime/runtime_<实际时间>.log
```

屏幕输出不受影响。日志根目录由严格配置项 `runtime.log_root` 指定。

### ToTarget 复现日志

每次控制线程真正切入 `accurate_arrival`（ToTarget）模型时，会在
`log/ToTarget/` 新建一个 JSONL 文件。文件名格式为：

```text
dx_<初始dx>_dy_<初始dy>_yaw_<初始dyaw>_<实际时间>.jsonl
```

这里的“初始目标”是进入模型第一帧状态机实际选中的三元组；如果该帧尚未收到新位置
误差，文件名会如实记录 `[0,0,0]`。JSONL 第一行是 `session_start`，包含模型路径和
SHA256、50 Hz 周期、policy/motor 关节顺序、默认角、增益、缩放参数和 observation
切片定义。之后每个 ToTarget 控制帧严格写一条 `frame`，包括：

- 墙上时间、单调时钟、全局推理帧号和 LowState tick/新鲜度；
- 当帧 high/low mode 以及状态机选中的实时 `[dx,dy,dyaw]`；
- policy order 的实际 29DoF 关节位置、速度和策略使用的 IMU；
- 完整 96 维 observation、原始 29 维 ONNX 输出和推理耗时；
- policy order 与 motor order 的最终目标位置、速度、Kp 和 Kd。

这些字段足以把同一份 ONNX 模型的每帧输入输出精确重算，并对齐实机关节反馈与最终
控制目标。日志不包含机器人接口没有提供、策略也没有读取的世界坐标系绝对基座位置、
接触力或地面扰动，因此它能复现策略和控制器现象，但仅靠该日志不能保证 MuJoCo
物理轨迹与实机逐点完全一致。文件由独立线程按顺序写入；队列积压时控制器会报错，
不会静默丢帧。

high-level 或 low-level mode 真正改变后，控制器终端会输出：

```text
[locomotion_controller_node-1] 模式切换为 high mode 1
[locomotion_controller_node-1] 模式切换为 low mode 1
```

重复发送相同 mode 不重复输出切换日志。按 `4` 直接发送 high mode 4；固定的
“前进三秒后停止”轨迹使用 `f`。自由速度导航的完整操作顺序为
`k → 1 → W/S、A/D、Q/E`，速度增量键会自动选择 low mode 1。

推荐测试顺序：

1. 确认真实导航和双臂节点均未运行，按 `k` 开启模拟参数发布。
2. 按 `1`，再用 `W/S`、`A/D`、`Q/E` 自由调整三轴速度：验证 mode 1 low mode 1
   和速度模型；增量键会自动发送 low mode 1。
3. 按 `p`：观察 low mode `1 → 2` 时先运行 `stand_recovery`，等待
   `stand_duration_s` 后再进入位置模型；随后按 `7`、`8` 或 `9` 验证位置控制。
4. 按 `2`，再按空格循环 `z/x/c`，或直接按 `z`、`x`、`c`、`b`：验证站立
   双臂模型和姿态直接切换。
5. 按 `3`，优先用空格循环或用 `z/x/c` 直接选择模型预设姿态，再按
   `W/S/A/D/Q/E`：同时验证 `arm_walk` 的导航速度模型输入和双臂输出覆盖；
   `b` 仅用于额外接口联调。
6. 保持模拟参数发布开启或关闭均可，直接按 `4`：验证零 command 的
   `stand_recovery` 模型。
7. 任意阶段按 `0` 验证导航零速回退，最后按 `Esc` 或 `Ctrl+C` 退出输入节点。

编辑预设 JSON 时字段不可缺失或增加；数组长度、有限数值、名称唯一性和持续时间
都会在测试节点启动前严格校验。使用其他预设文件：

```bash
ros2 run locomotion_controller locomotion_controller_simulator --ros-args \
  -p preset_file:=/absolute/path/to/presets.json
```

### 双臂 JSON

双臂消息必须只包含以下五个字段，不允许缺失或附加字段：

```json
{
  "schema": "hecbot.upper_body_command.v1",
  "seq": 42,
  "arm_q": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "arm_dq": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "weight": 1.0
}
```

- `seq`：非负且严格递增的整数。
- `arm_q`：左右双臂共 14 个目标关节角，单位 rad。
- `arm_dq`：对应的 14 个目标关节速度，单位 rad/s。
- `weight`：`[0, 1]`。`0` 保持切入该模型时的双臂基线，`1` 完全使用外部目标。
- 顺序：左臂 7 关节，随后右臂 7 关节；每侧依次为肩
  pitch/roll/yaw、肘、腕 roll/pitch/yaw。

## 严格行为

- 主配置和阻抗参数文件的每个字段都是必填项；未知字段、缺失字段、非法模式和
  非有限数值直接拒绝。
- 初始化后但尚未收到 high-level mode 时，`high_mode=None`；控制器仅用
  `stand_recovery + [0,0,0]` 保持站立。它使用与显式 mode 4 相同的模型，但
  `high_mode` 仍是 `None`。
- high mode 1 尚未收到 low-level mode 时，不选择默认 submode；保持
  `stand_recovery + [0,0,0]`。
- high 1/low 1 只有非零且未超时的速度才使用 `free_walk`；速度为零、尚未收到或
  超时时自动使用 `stand_recovery + [0,0,0]`。
- high mode 3 只有 low mode 1 才读取导航速度；low mode 2、未收到 low mode 或
  导航超时时，`arm_walk` 的 command observation 为 `[0,0,0]`。
- high mode 4 不读取 low mode、导航或双臂输入，直接使用
  `stand_recovery + [0,0,0]`。同一模型也用于上述内部站立阶段，但不会把业务
  high mode 数值改写为 `4`。
- 非法 high/low mode 数值会被拒绝并立即落入 `stand_recovery + [0,0,0]`。合法的
  high 3 + low 2 不属于非法模式，行为是 `arm_walk + [0,0,0]`。
- 模式与参数可以不同步到达。导航模式发生变化时先使用 `[0,0,0]`；mode 2/3
  会作废进入该 mode 之前缓存的双臂参数；收到切换后的第一条有效双臂参数前，
  每一帧都保持模式切换前最后一帧的双臂目标。
- 导航或双臂消息超时后不复用陈旧输入。导航变为零，双臂保持上一控制帧的实际目标。
- 双臂位置/速度和导航速度都不做时间插值或加速度限幅。模型切换动作融合只作用于
  腿和腰；模型 `previous_action` 在切换时清零。mode 2/3 的外部双臂命令不写入
  当前帧 command observation，只在推理后覆盖输出；覆盖后实际执行的 action 会
  写入下一帧 `previous_action`。
- 控制线程异常或程序退出时发送 `Kp=0, Kd=8` 阻尼命令。

完整运行时结构、时序和配置说明见
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 测试

不连接 ROS 或实机即可运行协议和纯状态机测试：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

这些测试同时覆盖模拟器预设格式、速度轨迹边界、mode 3 导航速度路由、mode 2/3
的双臂输出覆盖时序，以及 mode 4 的零 command 恢复模型路由与模型文件合同。
