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
four ONNX models are ready; initialization stand is complete
```

也可以在另一个已加载 ROS 环境的终端确认：

```bash
ros2 topic echo --once /hecbot/locomotion/initialized std_msgs/msg/Bool
```

看到 `data: true` 才能继续。

初始化由 launch 自动完成，没有额外的初始化命令。

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

切入 mode 1/2 后先短暂零速站立属于正常行为。缺少对应业务输入或输入超时，状态机
不会猜测参数：导航回零，双臂保持上一实际目标。

## 4. 完全使用键盘模拟输入（可选）

只有在真实导航层和双臂操作层都没有运行时才按一次 `k`，开启模拟参数发布。常用
顺序：

```text
速度导航：k → v → 1 → 4/5/6
位置导航：p → 1 → 7/8/9
站立双臂：2 → z/x/c/b
双臂行走：v → 3 → z/x/c，并用 4/5/6 发送速度
停止导航：0
```

`z/x/c/b` 都能设置双臂姿态；其中 `z/x/c` 对应
`models/walk_with_object_arm_pose_set.json` 的三个模型预设，是 high mode 3
持物行走的推荐选择。`b` 是额外联调姿态，不属于该文件的预设。

再次按 `k` 会停止模拟器的导航和双臂参数发布，但 high/low mode 键仍有效。

## 5. 退出

1. 在终端 2 按 `q`，只退出键盘模拟器；
2. 在终端 1 按 `Ctrl+C`，等待状态机完成阻尼收尾并退出。

正式的应用层负责发布 high mode、整体启动脚本负责启动和等待 initialized 后，就
无需启动键盘模拟器。
