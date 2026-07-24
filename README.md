# 人形机器人运动控制器

本项目是 Unitree G1 29DoF 的三模式有限状态机。入口为
`scripts/locomotion_controller_node`。程序启动时一次性加载并预热四个 ONNX
模型，然后启动唯一的 50 Hz 推理/`LowCmd` 控制线程。首帧发送后，控制器继续以
`free_walk + [0,0,0]` 站立 2 秒；全程健康后节点才发布初始化完成消息。

实机接管前，程序会读取当前 MotionSwitcher 状态。无论当前 Loco FSM ID 是什么，
只要仍有高层模式，程序就调用 `ReleaseMode()`，并确认 MotionSwitcher 模式名严格
变为空，即进入低层调试模式；已经处于空模式时直接继续。查询失败、返回格式异常、
释放失败或超时都会终止初始化，且不会进入默认姿态运动。

## 模式

| high-level mode | low-level mode | 模型 | 有效上游输入 |
| --- | --- | --- | --- |
| `1` 导航 | `1` 速度 | `free_walk.onnx` | 导航三元组 `[vx, vy, yaw_rate]` |
| `1` 导航 | `2` 位置 | `accurate_arrival.onnx` | 导航三元组 `[dx_body, dy_body, dyaw]` |
| `2` 双臂站立 | 不使用 | `standing_grasp.onnx` | 14DoF 双臂位置、速度、权重 |
| `3` 双臂行走 | 不使用 | `walk_with_object.onnx` | 14DoF 双臂位置、速度、权重 |

按当前接口合同，mode 2 和 mode 3 不读取导航输入；两者给模型的运动命令均为
`[0, 0, 0]`。`arm_walk` 模型负责腿腰策略，双臂关节由操作侧命令覆盖。

应用层切换到 mode 1 或 mode 2 时，控制器先选择 `free_walk`，向模型输入严格的
零速度并保持配置的 `stand_duration_s`，然后进入目标模式。切换到 mode 3
不执行这一步。重复发布当前 high-level mode 不会重新开始站立计时。

high mode 1 内从 low mode 1 切换到 low mode 2 时也执行同一段
`stand_duration_s` 零速站立。切换瞬间旧的速度三元组会被清除；如果位置参数尚未
到达，位置模型以 `[0,0,0]` 启动，不会把旧速度解释成位置误差。

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

先停止正在运行的控制器和键盘模拟器。每次修改代码、配置、模型、launch 文件、
`CMakeLists.txt` 或 `package.xml` 后，在 NUC 上执行：

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

四个模型预热检查：

```bash
cd /home/wenduo/locomotion_controller

PYTHONNOUSERSITE=1 \
  /home/hecbot/miniconda3/envs/locomotion_controller/bin/python \
  -c "import glob, onnxruntime as ort; paths=glob.glob('models/*.onnx'); [ort.InferenceSession(path, providers=['CPUExecutionProvider']) for path in paths]; print('loaded models:', len(paths))"
```

输出必须为 `loaded models: 4`。

## ROS 2 接口

程序只接收以下四个输入 topic：

| 方向 | topic | 类型 | 合同 |
| --- | --- | --- | --- |
| 输入 | `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `1`、`2` 或 `3` |
| 输入 | `/hecbot/locomotion/low_level_mode` | `std_msgs/msg/UInt8` | `1` 或 `2`，仅 high mode 1 使用 |
| 输入 | `/hecbot/locomotion/navigation_command` | `std_msgs/msg/Float32MultiArray` | 必须恰好 3 个有限数值 |
| 输入 | `/hecbot/upper_body_cmd` | `std_msgs/msg/String` | 严格 JSON，格式见下文 |

初始化输出：

| 方向 | topic | 类型 | 合同 |
| --- | --- | --- | --- |
| 输出 | `/hecbot/locomotion/initialized` | `std_msgs/msg/Bool` | 完成四模型预热、实机接管、首帧发送和 2 秒零速站立后发布 `true` |

初始化 topic 使用 reliable + transient-local QoS，初始化完成后启动的订阅者也能收到
最近一次 `true`。

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

### 持续查看订阅值

已加载 ROS 2 和本项目环境后，用下面一个命令打开 Topic Monitor 窗口：

```bash
ros2 run rqt_topic rqt_topic
```

在窗口中展开并勾选以下四项的 value 列，即可持续刷新控制器当前订阅的输入：
`/hecbot/locomotion/high_level_mode`、
`/hecbot/locomotion/low_level_mode`、
`/hecbot/locomotion/navigation_command` 和 `/hecbot/upper_body_cmd`。
若命令不存在，安装 ROS Jazzy 的 `ros-jazzy-rqt-topic` 包。

## 键盘整机测试

测试节点会等待 `/hecbot/locomotion/initialized=true`，然后以 20 Hz 持续发布导航
和双臂输入，频率高于两类输入的超时要求。预设位于
[`config/simulator_presets.json`](config/simulator_presets.json)，包含四个双臂
姿态、三条速度轨迹和三个位置目标。按下双臂姿态键后，测试节点从下一次发布开始
直接发送该目标，不生成中间姿态，也不做加速度限幅。

确保机器人周围无人、急停可用且没有其他 `rt/lowcmd` 发布者。终端 1：

```bash
ssh hecbot@192.168.50.113
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

看到 `initialization stand is complete` 后，在终端 2：

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
| `1` / `2` / `3` | high-level mode 1 / 2 / 3 |
| `v` | low mode 1，速度模式，同时清零当前导航输入 |
| `p` | low mode 2，位置模式，同时清零当前导航输入 |
| `0` | 取消轨迹并持续发布 `[0,0,0]` |
| `k` | 停止/恢复导航与双臂参数发布；high/low mode 按键始终有效 |
| `4` | 前进 3 秒后停止 |
| `5` | 左右横移后停止 |
| `6` | 左右原地转向后停止 |
| `7` | 位置目标：前方 0.3 m |
| `8` | 位置目标：左侧 0.2 m |
| `9` | 位置目标：左转约 20° |
| `z` / `x` / `c` / `b` | 双臂靠后 / 下垂 / 靠前 / 持物 |
| `h` | 重新显示帮助 |
| `q` | 退出测试节点 |

需要让另一个节点独占发布真实导航指令时，先按一次 `k`。模拟器会立即停止发布
`/hecbot/locomotion/navigation_command` 和 `/hecbot/upper_body_cmd`，并取消其内部
正在执行的速度轨迹，但 `1/2/3`、`v/p` 仍可用于切换 high/low mode。停止时不会
额外发布零值；状态机中最后一条模拟参数分别在 `navigation_timeout_s` 和
`arm_timeout_s` 到期后失效。再次按 `k` 会恢复模拟参数的 20 Hz 发布。

### 终端调试日志

控制进程在每次 50 Hz ONNX 推理后，向控制器终端输出一行 `[INFERENCE]` JSON：

```text
[INFERENCE] {"event":"policy_inference","frame":125,"model":"free_walk","high_mode":1,"low_mode":1,"standing_transition":false,"navigation_input":{"semantics":"velocity","selected":[0.25,0.0,0.0],"model_input":[0.25,0.0,0.0]},"arm_input":null,"model_output":[...29 values...]}
```

- `selected`：状态机选择的速度或骨盆坐标系位置误差。
- `model_input`：实际写入 observation 的三元组；速度模式只做最大速度幅值保护，
  不做插值或加速度限幅，位置模式不做变换。
- `arm_input`：当前有效的双臂位置、速度、权重和序号；无有效双臂消息时为 `null`。
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

重复发送相同 mode 不重复输出切换日志。按 `4` 本身只启动速度轨迹，不会自动选择
模式；完整操作顺序为 `v → 1 → 4`。

推荐测试顺序：

1. 按 `v`、`1`、`4`：验证 mode 1 low mode 1 和速度模型。
2. 按 `p`：观察 low mode `1 → 2` 的 `stand_duration_s` 零速站立；随后按
   `7`、`8` 或 `9` 验证位置模型。
3. 按 `2`，再按 `z`、`x`、`c`、`b`：验证站立双臂模型和姿态直接切换。
4. 按 `3`，再选择双臂姿态：验证行走双臂模型。
5. 任意阶段按 `0` 验证零速回退，最后按 `q` 退出输入节点。

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

- 配置文件的每个字段都是必填项；未知字段、缺失字段、非法模式和非有限数值直接拒绝。
- 初始化后但尚未收到 high-level mode 时，不猜测业务模式；控制器仅用
  `free_walk` 零速度保持站立。
- high mode 1 尚未收到 low-level mode 时，不选择默认 submode；保持
  `free_walk` 零速度。
- high mode 或 low mode 无法匹配时，立即落入 `free_walk + [0,0,0]`，不会继续
  执行上一业务模式。
- 模式与参数可以不同步到达。导航模式发生变化时先使用 `[0,0,0]`；mode 2/3
  会作废进入该 mode 之前缓存的双臂参数；收到切换后的第一条有效双臂参数前，
  每一帧都保持模式切换前最后一帧的双臂目标。
- 导航或双臂消息超时后不复用陈旧输入。导航变为零，双臂保持上一控制帧的实际目标。
- 双臂位置/速度和导航速度都不做时间插值或加速度限幅。模型切换动作融合只作用于
  腿和腰；模型 `previous_action` 在切换时清零。
- 控制线程异常或程序退出时发送 `Kp=0, Kd=8` 阻尼命令。

完整运行时结构、时序和配置说明见
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 测试

不连接 ROS 或实机即可运行协议和纯状态机测试：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

这些测试同时覆盖模拟器预设格式、速度轨迹边界和 mode 2/3 的双臂输入时序。
