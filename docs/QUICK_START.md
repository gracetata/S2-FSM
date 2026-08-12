# 状态机极简使用方法

以下步骤按终端操作顺序执行。实机前先确认周围无人、急停可用，并确保没有其他
`rt/lowcmd` 发布者。

## 1. 终端 1：启动状态机

```bash
ssh -t hecbot@192.168.50.113
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

等待终端出现：

```text
five ONNX models are ready; stand-recovery initialization is complete
```

也可以在另一个已加载 ROS 环境的终端确认：

```bash
ros2 topic echo --once /hecbot/locomotion/initialized std_msgs/msg/Bool
```

看到 `data: true` 才能继续。

初始化由 launch 自动完成，没有额外的初始化命令。

如需录制所有模型真正送入 ONNX 前的 96 维输入，可在任意已加载 ROS 环境的新终端
先运行下面命令，再开始切换模式：

```bash
ros2 bag record -o policy_input_bag \
  /hecbot/locomotion/policy_input \
  /hecbot/whole_body_state
```

录制终端最后按 `Ctrl+C` 保存包。仅查看一帧可用：

```bash
ros2 topic echo --once /hecbot/locomotion/policy_input std_msgs/msg/String
```

## 2. 终端 2：启动键盘 high mode 模拟

```bash
ssh -t hecbot@192.168.50.113
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 run locomotion_controller locomotion_controller_simulator
```

模拟器默认就是“按过 `k`”的静默状态：不发导航速度、位置参数和双臂参数，只允许
用键盘切换 high/low mode，因此不会和真实导航、双臂操作节点抢输入。

## 3. 选择功能并等待业务模块发指令

- 按 `1`：进入导航 high mode。随后等待导航层发布
  `/hecbot/locomotion/low_level_mode` 和
  `/hecbot/locomotion/navigation_command`。如果导航层暂时不能发布 low mode，
  可按 `v` 代替速度模式，或按 `p` 代替位置模式。
- 按 `2`：进入原地双臂操作，等待双臂操作层发布
  `/hecbot/upper_body_cmd`。
- 按 `3`：进入 high mode 3 双臂行走，同时等待导航层发布 low mode 1 速度和
  双臂操作层发布
  `/hecbot/upper_body_cmd`。导航速度进入 `arm_walk` 模型；双臂命令只覆盖模型
  输出，不是当前帧独立输入；覆盖后 action 会进入下一帧 `previous_action`。
- 按 `4`：直接进入鲁棒站立恢复模型。固定使用 `[0,0,0]` command，不需要 low
  mode、导航或双臂输入，也不需要按 `k`。

初始化后如果不按任何 high mode，状态机持续运行
`stand_recovery + [0,0,0]`，但 `high_mode` 仍是 `None`。切入 mode 1/2 后先短暂
运行同一恢复模型属于正常行为；尤其 low 1→low 2 会先恢复站立，等待结束后才进入
位置模型。high 1/low 1 的速度为零或超时时也自动使用恢复模型。

## 4. 完全使用键盘模拟输入（可选）

只有在真实导航层和双臂操作层都没有运行时才按一次 `k`，开启模拟参数发布。常用
顺序：

```text
速度导航：k → 1 → W/S、A/D、Q/E（速度增量键自动选择 low mode 1）
位置导航：p → 1 → 7/8/9
站立双臂：2 → Space 循环 z/x/c，或直接按 z/x/c/b
双臂行走：3 → Space 循环 z/x/c，并用 W/S、A/D、Q/E 调整速度
鲁棒站立恢复：4
停止导航：0
```

每次 `W/S`、`A/D` 分别增减 `0.05 m/s`，`Q/E` 分别增减
`0.05 rad/s`；按 `0` 严格归零。固定测试轨迹仍可用 `f/5/6`。空格仅在 high
mode 2/3 中生效，只循环三个推荐姿态，不包含额外姿态 `b`。

每次按键改变模式、速度或双臂姿态后，终端都会打印 `[KEYBOARD_STATE]`，其中包含
当前 High Mode、Low Mode、`[vx,vy,yaw_rate]`、双臂姿态和参数发布开关。看到
`publishing=off` 时，速度和双臂值尚未发布，需要先按 `k`。

`z/x/c/b` 都能设置双臂姿态；其中 `z/x/c` 对应
`models/walk_with_object_arm_pose_set.json` 的三个模型预设，是 high mode 3
持物行走的推荐选择。`b` 是额外联调姿态，不属于该文件的预设。

再次按 `k` 会停止模拟器的导航和双臂参数发布，但 high/low mode 键仍有效。

## 5. 退出

1. 在终端 2 按 `Esc` 或 `Ctrl+C`，只退出键盘模拟器；
2. 在终端 1 按 `Ctrl+C`，等待状态机完成阻尼收尾并退出。

正式的应用层负责发布 high mode、整体启动脚本负责启动和等待 initialized 后，就
无需启动键盘模拟器。
