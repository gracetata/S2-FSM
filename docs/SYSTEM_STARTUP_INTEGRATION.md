# 整体启动脚本接入说明

整体启动脚本负责启动状态机、等待初始化完成，并在整机退出时把状态机正常关闭。

## 1. 启动前条件

- ROS 2 Jazzy 已安装；
- 项目已经 `colcon build`；
- 配置中的 Conda Python、网卡、机器人 IP、模型和日志路径存在；
- `confirm_real_robot: true`；
- 没有其他进程发布 `rt/lowcmd`；
- 机器人周围安全，急停可用。

## 2. 启动命令

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash

ros2 launch locomotion_controller locomotion_controller.launch.py
```

如需指定配置：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py \
  config_file:=/absolute/path/to/locomotion_controller.yaml
```

启动进程必须常驻。整体启动脚本不要在看到进程创建后立刻启动业务，应等待：

```bash
ros2 topic echo --once /hecbot/locomotion/initialized std_msgs/msg/Bool
```

只有输出 `data: true` 才表示初始化成功。该 topic 是 transient-local，等待程序晚于
初始化完成也不会错过结果。

## 3. 可直接嵌入的 Bash 示例

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash

ros2 launch locomotion_controller locomotion_controller.launch.py &
fsm_pid=$!

stop_fsm() {
  kill -INT "${fsm_pid}" 2>/dev/null || true
  wait "${fsm_pid}" 2>/dev/null || true
}
trap stop_fsm EXIT INT TERM

timeout 90 ros2 topic echo --once \
  /hecbot/locomotion/initialized std_msgs/msg/Bool |
  grep -q "data: true"

# 到这里才能启动应用层、导航层和双臂操作层。
wait "${fsm_pid}"
```

如果 90 秒内没有收到 `true`，或者 launch 进程提前退出，整体启动应判定失败，不要
继续启动会向机器人发业务命令的模块。实际整机脚本还应把 launch 输出保存到统一日志。

## 4. 状态机启动时机器人的反应

启动后程序依次：

1. 加载并预热五个 ONNX 模型；
2. 连接 Unitree DDS，等待 `rt/lowstate`；
3. 主动释放当前 Unitree MotionSwitcher 高层模式，进入低层调试模式；
4. 用配置的 `startup_move_s` 从当前关节位置平滑移动到默认姿态；
5. 启动唯一的 50 Hz `LowCmd`/推理线程；
6. 继续零速站立 `initialization_stand_duration_s`；
7. 发布 `/hecbot/locomotion/initialized = true`，同时开始以 50 Hz 发布
   `/hecbot/whole_body_state`。

初始化完成后，状态机不会自行选择 high mode，也不会自行行走；它保持
`high_mode=None` 和 `free_walk + [0,0,0]`，等待应用层发送 high mode。它不会
自动进入 NAV 2，也不会自动进入 high mode 4。

整机需要使用 high mode 3 时，整体启动必须同时保证导航层和双臂层常驻：

- 导航层持续发送 low mode 1 和速度三元组，速度作为 `arm_walk` 模型输入；
- 双臂层持续发送 14DoF 命令，在推理后覆盖双臂输出；
- 状态机自动为 mode 3 使用与 mode 2 相同的 Kp/Kd；
- 应用层应在上述模块就绪后再发布 high mode 3。

整机需要测试 high mode 4 时，应用层只需发布数值 `4`。mode 4 直接进入
`extreme_stand_recovery.onnx + [0,0,0]`，不依赖导航层和双臂层输入。

## 5. 关闭

整体退出时给 launch 进程发送 `SIGINT`，并等待它退出。状态机停止 50 Hz 控制线程
后，会按 `fault_damping_duration_s` 发送 `Kp=0, Kd=8` 的阻尼命令。不要直接
`SIGKILL`，否则正常阻尼收尾无法执行。

正式整体启动接入后，不需要启动 `locomotion_controller_simulator`。
