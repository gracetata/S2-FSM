# 人形机器人运动控制器

本项目是 Unitree G1 29DoF 的三模式有限状态机。入口为
`scripts/locomotion_controller_node`。程序启动时一次性加载并预热四个 ONNX
模型，然后启动唯一的 50 Hz 推理/`LowCmd` 控制线程。首帧发送后，控制器继续以
`free_walk + [0,0,0]` 站立 2 秒；全程健康后节点才发布初始化完成消息。

实机接管前存在硬性安全门：程序必须先读到
`MotionSwitcher mode=ai` 和 `Loco FSM=0 (ZeroTorque)`，随后才调用
`ReleaseMode()`；释放后还必须确认 MotionSwitcher 模式名变为空。任何查询失败、
返回格式异常或状态不匹配都会终止初始化，且不会进入默认姿态运动。

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
`network_interface`、`robot_ip` 和 `confirm_real_robot`。启动控制器之前必须先
通过遥控器将 G1 原生运控置于 `FSM 0 / ZeroTorque`；如果当前 MotionSwitcher
不是 `ai` 或 FSM 不是 `0`，控制器会拒绝接管。

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

位置 submode 要求导航持续发送骨盆坐标系闭环误差，不接受全局目标坐标，也不会在
控制器内部计算或累计误差。完整合同、变换公式和数据示例见
[`docs/NAVIGATION_ADAPTER.md`](docs/NAVIGATION_ADAPTER.md)。

## 键盘整机测试

测试节点会等待 `/hecbot/locomotion/initialized=true`，然后以 20 Hz 持续发布导航
和双臂输入，频率高于两类输入的超时要求。预设位于
[`config/simulator_presets.json`](config/simulator_presets.json)，包含四个双臂
姿态、三条速度轨迹和三个位置目标。双臂姿态之间默认使用 2 秒 minimum-jerk
轨迹，不会直接跳变。

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
| `4` | 前进 3 秒后停止 |
| `5` | 左右横移后停止 |
| `6` | 左右原地转向后停止 |
| `7` | 位置目标：前方 0.3 m |
| `8` | 位置目标：左侧 0.2 m |
| `9` | 位置目标：左转约 20° |
| `z` / `x` / `c` / `b` | 双臂靠后 / 下垂 / 靠前 / 持物 |
| `h` | 重新显示帮助 |
| `q` | 退出测试节点 |

推荐测试顺序：

1. 按 `v`、`1`、`4`：验证 mode 1 low mode 1 和速度模型。
2. 按 `p`：观察 low mode `1 → 2` 的 `stand_duration_s` 零速站立；随后按
   `7`、`8` 或 `9` 验证位置模型。
3. 按 `2`，再按 `z`、`x`、`c`、`b`：验证站立双臂模型和姿态平滑切换。
4. 按 `3`，再选择双臂姿态：验证行走双臂模型。
5. 任意阶段按 `0` 验证零速回退，最后按 `q` 退出输入节点。

编辑预设 JSON 时字段不可缺失或增加；数组长度、有限数值、名称唯一性和正持续时间
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
  尚未收到有效双臂参数时使用模型切换前最后一帧双臂目标。
- 导航或双臂消息超时后不复用陈旧输入。导航变为零，双臂保持上一控制帧的实际目标。
- 模型切换使用配置的动作融合时间，模型 `previous_action` 在切换时清零。
- 控制线程异常或程序退出时发送 `Kp=0, Kd=8` 阻尼命令。

完整运行时结构、时序和配置说明见
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 测试

不连接 ROS 或实机即可运行协议和纯状态机测试：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

这些测试同时覆盖模拟器预设格式、速度轨迹边界和双臂 minimum-jerk 轨迹端点。
