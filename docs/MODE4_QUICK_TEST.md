# Mode 4 极简真机测试

Mode 4 独立运行鲁棒站立恢复模型，不需要 low mode、导航、双臂命令，也不需要按
`k`。实机前确保周围无人、急停可用且没有其他 `rt/lowcmd` 发布者。

## 1. 拉取并编译

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
git pull
colcon build --symlink-install --packages-select locomotion_controller
source install/setup.bash
```

## 2. 启动状态机

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

等待：

```text
five ONNX models are ready; initialization stand is complete
```

此时如果不发送 high mode，机器人保持
`high_mode=None + free_walk + [0,0,0]`，不会自动进入 mode 4。

## 3. 按键进入 Mode 4

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source /home/wenduo/locomotion_controller/install/setup.bash
ros2 run locomotion_controller locomotion_controller_simulator
```

直接按数字：

```text
4
```

不需要按 `k`、`v` 或 `p`。终端 1 应看到：

```text
模式切换为 high mode 4
```

推理日志应出现：

```text
"model":"stand_recovery"
```

从可控的小扰动开始测试恢复效果。数字 `4` 已专用于 mode 4；速度导航的前进轨迹
改用 `w`。

## 4. 退出

1. 终端 2 按 `q`；
2. 终端 1 按 `Ctrl+C`；
3. 等待控制器完成阻尼收尾后再断开。
