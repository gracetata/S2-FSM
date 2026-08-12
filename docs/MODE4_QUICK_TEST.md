# Mode 4 极简真机测试

Mode 4 独立运行鲁棒站立恢复模型，不需要 low mode、导航、双臂命令，也不需要按
`k`。实机前确保周围无人、急停可用且没有其他 `rt/lowcmd` 发布者。

## 1. 拉取并编译

```bash
source /opt/ros/jazzy/setup.bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
git pull
colcon build --symlink-install --packages-select locomotion_controller
source "$FSM_ROOT/install/setup.bash"
```

## 2. 启动状态机

终端 1：

```bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source /opt/ros/jazzy/setup.bash
source "$FSM_ROOT/install/setup.bash"
ros2 launch locomotion_controller locomotion_controller.launch.py
```

等待：

```text
five ONNX models are ready; zero-command free-walk initialization is complete
```

此时如果不发送 high mode，机器人保持
`high_mode=None + free_walk + [0,0,0]`。恢复模型尚未运行，业务 high mode 仍是
`None`。

## 3. 按键进入 Mode 4

终端 2：

```bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source /opt/ros/jazzy/setup.bash
source "$FSM_ROOT/install/setup.bash"
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

控制器不打印逐帧 `[INFERENCE]`。直接检查推理前的完整 96 维输入：

```bash
ros2 topic echo /hecbot/locomotion/policy_input std_msgs/msg/String
```

按 `4` 后消息中的 `model` 应为 `stand_recovery`、`high_mode` 应为 `4`，
`input.observation` 必须恰好包含 96 个数值。

从可控的小扰动开始测试恢复效果。数字 `4` 已专用于 mode 4；速度导航的前进轨迹
改用 `f`。

## 4. 退出

1. 终端 2 按 `Esc` 或 `Ctrl+C`；
2. 终端 1 按 `Ctrl+C`；
3. 等待控制器完成阻尼收尾后再断开。
