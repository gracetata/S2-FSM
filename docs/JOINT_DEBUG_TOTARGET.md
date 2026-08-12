# S2-FSM 与 Kairui ToTarget 联合调试操作手册

本文按实际操作顺序编写。不要跳步；每个“终端”都代表一个独立 SSH 窗口。

## 一、先明确安全边界

联合调试时：

- S2-FSM 是唯一允许发送 Unitree `LowCmd` 的项目；
- Kairui 只运行 Airy、ego-motion 和 `kairui_navigation_shadow_probe`；
- 严禁同时运行 `/home/kairui/run_totarget.sh run`；
- 键盘模拟器只负责选择 FSM 的 high/low mode，不负责发布导航三元组。

`kairui_navigation_shadow_probe` 使用 Kairui 正式 ToTarget 的相同算法，把启动瞬间的
相对目标冻结为地面目标，再根据实时 pelvis odom 持续计算
`[dx_body,dy_body,dyaw]`。它不连接 Unitree SDK，也不发送 `LowCmd`。

消息链如下：

```text
S2-FSM /hecbot/whole_body_state
  → totarget_navigation_bridge
  → /kairui/whole_body_state
  → Kairui ego-motion
  → /ego_motion/plevis
  → Kairui shadow probe
  → /kairui/totarget/shadow_pelvis_error
  → totarget_navigation_bridge
  → /hecbot/locomotion/navigation_command
  → S2-FSM accurate_arrival.onnx
```

## 二、严格按顺序启动

### 第 0 步：现场安全确认

开始前必须满足：

1. 机器人已架起，或位于有安全员和急停的空旷场地；
2. 操作人员知道如何触发硬件急停；
3. 不存在另一套正在控制机器人的程序。

### 第 1 步：终端 0，检查旧进程

打开第一个 SSH 窗口，依次输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
source /opt/ros/jazzy/setup.bash
pgrep -af 'kairui_totarget_console|kairui_totarget.runtime_process|locomotion_controller_node'
ros2 node list | sort
```

第一条 `pgrep` 在启动本次 S2-FSM 前应没有输出。如果看到旧控制器，只能在它原来的
终端按 `Ctrl+C` 正常退出，并等待阻尼收尾。不要直接启动第二套控制器。

`ros2 node list` 如果已经出现 Airy 或 `ego_motion_node`，也不要重复启动。应先在旧
启动终端正常退出，保证这次调试的所有进程来自下面的新终端。

### 第 2 步：终端 1，启动 S2-FSM 控制器

新开 SSH 窗口，完整输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
cd "$HOME/wenduo/S2-FSM"
export FSM_ROOT="$(pwd -P)"
set -a
source "$FSM_ROOT/config/nuc.env"
set +a
source /opt/ros/jazzy/setup.bash
source "$FSM_ROOT/install/setup.bash"
ros2 launch locomotion_controller locomotion_controller.launch.py \
  config_file:="$FSM_ROOT/config/locomotion_controller.yaml"
```

保持这个终端运行，不要按 `Ctrl+C`。

### 第 3 步：终端 0，确认 FSM 初始化完成

回到终端 0，输入：

```bash
ros2 topic echo --once /hecbot/locomotion/initialized std_msgs/msg/Bool
```

必须看到：

```text
data: true
```

如果不是 `true`，不要继续。

### 第 4 步：终端 2，启动联合调试桥

新开 SSH 窗口，输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
cd "$HOME/wenduo/S2-FSM"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run locomotion_controller totarget_navigation_bridge
```

必须看到包含以下内容的日志：

```text
ROS-only bridge ready (never publishes LowCmd)
```

随后应看到它开始转发 `/hecbot/whole_body_state`。

### 第 5 步：终端 3，启动 Airy

新开 SSH 窗口，输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
source /opt/ros/jazzy/setup.bash
source /home/kairui/navigation_ws/install/setup.bash
ros2 launch rslidar_sdk bringup.launch.py
```

保持运行。若出现 UDP 端口已占用，说明旧 Airy 未退出；停止本次命令，处理旧进程后
重新从第 1 步开始。

### 第 6 步：终端 4，启动 Kairui live pelvis ego-motion

新开 SSH 窗口，输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
source /opt/ros/jazzy/setup.bash
source /home/kairui/navigation_ws/install/setup.bash
ros2 launch robot_navigation ego_motion_robosense_airy.launch.py \
  use_sim_time:=false debug_vis:=true pelvis_joint_mode:=live
```

保持运行。

### 第 7 步：终端 0，检查定位和关节状态频率

回到终端 0，先输入：

```bash
ros2 topic hz /kairui/whole_body_state
```

看到稳定频率后按 `Ctrl+C`，再输入：

```bash
ros2 topic hz /ego_motion/plevis
```

两者都必须持续更新。`/ego_motion/plevis` 没有数据时不要启动目标。

### 第 8 步：终端 5，启动第一个 ToTarget 目标

首次只测试机器人前方 `0.30 m`，航向不变。新开 SSH 窗口，输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
source /opt/ros/jazzy/setup.bash
source /home/kairui/navigation_ws/install/setup.bash
source /home/kairui/install/setup.bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.30,0.00,0.00 --samples 100000 --timeout 3600
```

此时目标已经冻结，但 FSM 尚未切入位置模式。

### 第 9 步：终端 6，启动 FSM 键盘模拟器

新开 SSH 窗口，输入：

```bash
ssh -t nuc-005
export ROS_DOMAIN_ID=22
cd "$HOME/wenduo/S2-FSM"
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source /opt/ros/jazzy/setup.bash
source "$FSM_ROOT/install/setup.bash"
ros2 run locomotion_controller locomotion_controller_simulator --ros-args \
  -p preset_file:="$FSM_ROOT/config/simulator_presets.json"
```

看到 `controller initialized; keyboard commands are now active` 后，按下一节的完整
场景操作。速度键现在会自动切到 low mode 1 并开启键盘导航发布；按 `p` 会先停止
键盘导航，再切到 low mode 2，bridge 只在 low mode 2 时转发 ToTarget。

### 第 10 步：完整两轮联调场景

终端 5 中的第一目标已经用 `0.30,0.00,0.00` 冻结。回到终端 6，严格按顺序操作：

1. 按 `1`：选择 high mode 1；
2. 按 `W` 若干次：每次增加 `vx=0.05 m/s`；第一次速度键会自动发布 low mode 1
   并开启键盘导航；
3. 可用 `S/A/D/Q/E` 调整或抵消速度，观察
   `delivery=PUBLISHING_20HZ`；
4. 准备交给 ToTarget 时只按一次 `p`；
5. `p` 会按代码保证的顺序先停止键盘导航发布，再发布 low mode 2；bridge 收到
   low mode 2 后开始转发第一个目标；
6. 确认键盘日志包含 `low_mode=2` 和 `navigation_publishing=off`；
7. 等待 `stand_duration_s` 的零速 `free_walk` 过渡结束，随后 FSM 进入
   `accurate_arrival` 到达第一目标；
8. 第一目标完成后按 `5`：直接进入 high mode 5，即
   `free_walk + [0,0,0]` 站立；该键也保持键盘导航发布关闭；
9. 在终端 5 按 `Ctrl+C` 停止第一目标，等待 `0.5 s`，再启动第二目标：

   ```bash
   ros2 run kairui_totarget kairui_navigation_shadow_probe \
     0.00,0.20,0.00 --samples 100000 --timeout 3600
   ```

10. 回到终端 6 按 `1`，然后按 `W/A/D/Q/E` 中需要的速度键遥控走动；速度键会
    自动切回 low mode 1 并取得导航发布权；
11. 再按一次 `p`，一键停止键盘速度并切到 low mode 2，开始接收第二个 ToTarget；
12. 到达后按 `5` 回到显式零速站立。

不要按 `k` 或 `m`。本场景也不需要按 `n`：速度键会自动启用键盘导航，`p` 和
`5` 会自动关闭它。

## 三、按顺序验证消息是否正常

### 检查 1：发布者数量

在终端 0 输入：

```bash
ros2 topic info -v /hecbot/locomotion/navigation_command
```

由于键盘模拟器和 bridge 都预先创建 ROS publisher，这里会看到两个 publisher
端点：

```text
/totarget_navigation_bridge
/locomotion_controller_simulator
```

然后输入：

```bash
ros2 topic info -v /hecbot/locomotion/high_level_mode
ros2 topic info -v /hecbot/locomotion/low_level_mode
```

这两个 mode 话题应只有键盘模拟器一个发布者。真正的数据互斥由 low mode 门控：

- low mode 1：键盘速度发布，bridge 不转发 ToTarget；
- 按 `p` 后的 low mode 2：键盘停止导航发布，bridge 转发 ToTarget。

bridge 终端应对应打印 `ToTarget forwarding disabled/enabled by low mode`。如果在
low mode 2 时键盘日志不是 `navigation_publishing=off`，立即按 `5` 停止本轮。

### 检查 2：Kairui 输出与 FSM 输入频率

终端 0 依次执行，每条观察数秒后按 `Ctrl+C`：

```bash
ros2 topic hz /kairui/totarget/shadow_pelvis_error
ros2 topic hz /hecbot/locomotion/navigation_command
```

两边都必须稳定高于 `4 Hz`，推荐约 `10–20 Hz`。FSM 导航输入超时为 `0.25 s`。

### 检查 3：三元组数值

终端 0 先输入：

```bash
ros2 topic echo --once /kairui/totarget/shadow_pelvis_error \
  std_msgs/msg/Float32MultiArray
```

记下 `[dx,dy,dyaw]`，再立即输入：

```bash
ros2 topic echo --once /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray
```

两个三元组应基本一致。机器人向目标运动时，前进测试的 `dx` 应总体向零减小。

### 检查 4：FSM 实际送入 ONNX 的值

终端 0 输入：

```bash
ros2 topic echo --once /hecbot/locomotion/policy_input std_msgs/msg/String
```

消息中的 JSON 必须满足：

- `model` 是 `accurate_arrival`；
- `high_mode` 是 `1`；
- `low_mode` 是 `2`；
- `navigation_input.semantics` 是 `target_pose`；
- `navigation_input.selected` 和 `navigation_input.model_input` 与
  `/hecbot/locomotion/navigation_command` 基本一致。

这一步通过，才说明信息不仅到达 ROS topic，而且已真正进入 FSM 和 ONNX 输入。

### 检查 5：超时保护

在终端 5 按 `Ctrl+C` 停止 shadow probe，等待 `0.5 s`，然后在终端 0 输入：

```bash
ros2 topic echo --once /hecbot/locomotion/policy_input std_msgs/msg/String
```

`navigation_input.selected` 和 `model_input` 应变成 `[0,0,0]`。high 1/low 2
仍会显示 `accurate_arrival`，但不会继续使用最后一帧旧目标。

## 四、键盘能否增减 ToTarget 目标位置

### 当前结论：不能直接用现有键盘导航键修改 Kairui ToTarget 目标

现有键盘模拟器中的按键含义是：

- `W/S/A/D/Q/E`：修改键盘模拟器自己的速度命令，自动切到 low mode 1 并开启
  键盘导航发布；
- `7/8/9`：选择键盘模拟器内部的固定位置三元组；
- `n` 或 `k`：开启键盘自己的导航发布。

这些按键不会修改 Kairui shadow probe 已冻结的地面目标。它们只在 low mode 1
期间遥控速度；按 `p` 后键盘立即停止导航发布，由 bridge 在 low mode 2 转发目标。

### 不使用键盘时，如何切换目标

推荐先在终端 6 按 `5` 进入 high mode 5 零速站立，再在 **终端 5** 换目标。
S2-FSM、bridge、Airy、ego-motion 和键盘模拟器都保持运行。新目标冻结后，如果要
直接到点，按 `1` 再按 `p`；如果要先速度遥控，按 `1` 后直接使用速度键，最后按
一次 `p` 交给 ToTarget。

严格执行下面四步：

1. 在终端 6 按 `5`，确认 `high_mode=5` 且 `navigation_publishing=off`；
2. 在终端 5 按 `Ctrl+C`，停止当前 `kairui_navigation_shadow_probe`；
3. 等待至少 `0.5 s`，让 FSM 的 `0.25 s` 导航超时保护生效；
4. 可选但推荐：在终端 0 执行下面的命令，确认
   `navigation_input.selected/model_input` 已变为 `[0,0,0]`：

   ```bash
   ros2 topic echo --once /hecbot/locomotion/policy_input std_msgs/msg/String
   ```

5. 回到终端 5，用新的 `[dx,dy,dyaw]` 重新启动 shadow probe；
6. 回到终端 6，按 `1`，速度遥控后按 `p`，或直接按 `p` 开始新目标。

如果终端 5 仍保留之前加载的环境，换目标命令模板是：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  DX,DY,DYAW --samples 100000 --timeout 3600
```

如果关闭过终端 5，则重新打开 SSH 后完整输入：

```bash
ssh nuc-005
export ROS_DOMAIN_ID=22
source /opt/ros/jazzy/setup.bash
source /home/kairui/navigation_ws/install/setup.bash
source /home/kairui/install/setup.bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  DX,DY,DYAW --samples 100000 --timeout 3600
```

`DX,DY,DYAW` 是以**执行新命令时的当前 pelvis**为原点的新相对目标：

- `DX`：前后距离，单位 m；前方为正，后方为负；
- `DY`：左右距离，单位 m；左侧为正，右侧为负；
- `DYAW`：航向变化，单位 rad；从上方看逆时针/左转为正，右转为负。

每次重新运行命令都会根据当时的 pelvis 位姿冻结一个新的地面目标。因此，连续执行
两次 `0.30,0,0` 表示“从每次命令启动时的位置再向前 0.30 m”，并不是回到同一个
绝对地图坐标。

常用目标命令如下。

向前 `0.30 m`：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.30,0.00,0.00 --samples 100000 --timeout 3600
```

向后 `0.20 m`：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  -0.20,0.00,0.00 --samples 100000 --timeout 3600
```

向左 `0.20 m`：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.00,0.20,0.00 --samples 100000 --timeout 3600
```

向右 `0.20 m`：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.00,-0.20,0.00 --samples 100000 --timeout 3600
```

原地左转 `0.30 rad`（约 `17.2°`）：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.00,0.00,0.30 --samples 100000 --timeout 3600
```

原地右转 `0.30 rad`（约 `17.2°`）：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.00,0.00,-0.30 --samples 100000 --timeout 3600
```

组合目标：向前 `0.30 m`、向左 `0.10 m`、同时左转 `0.20 rad`：

```bash
ros2 run kairui_totarget kairui_navigation_shadow_probe \
  0.30,0.10,0.20 --samples 100000 --timeout 3600
```

新命令启动后，在终端 0 执行以下命令确认新目标已经生效：

```bash
ros2 topic echo --once /kairui/totarget/shadow_pelvis_error \
  std_msgs/msg/Float32MultiArray
ros2 topic echo --once /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray
```

两边的新三元组应基本一致。首次联调每次只改一个维度，并使用上述小目标。

如果需要真正通过按键实时增减 Kairui 目标，应另加一个专用的“目标调整 topic”，由
Kairui shadow/ToTarget 节点负责冻结新地面目标。不能复用 FSM 的
`navigation_command`，否则会破坏单发布者约束。该专用按键功能当前尚未实现。

## 五、建议测试顺序

每个用例都先停止旧 shadow probe，确认 FSM 输入归零，再启动新目标：

| 顺序 | 输入目标 `[dx,dy,dyaw]` | 预期 |
| --- | --- | --- |
| 1 | `[0.30,0,0]` | `dx` 向零收敛 |
| 2 | `[0,0.20,0]` | `dy` 向零收敛，左为正 |
| 3 | `[0,0,0.30]` | `dyaw` 向零收敛，单位 rad |
| 4 | `[0.30,0.10,0.20]` | 三轴连续更新且不超时 |

首次联调不要使用大目标或连续随机目标。

## 六、停止顺序

严格按以下顺序停止：

1. 终端 5：`Ctrl+C` 停止 shadow probe；
2. 等待至少 `0.5 s`，确认 `policy_input` 的导航输入已归零；
3. 终端 6：按 `Esc` 退出键盘模拟器；
4. 终端 4：`Ctrl+C` 停止 ego-motion；
5. 终端 3：`Ctrl+C` 停止 Airy；
6. 终端 2：`Ctrl+C` 停止 bridge；
7. 最后在终端 1 按 `Ctrl+C` 停止 S2-FSM，并等待阻尼收尾。

不要用 `kill -9` 停止 S2-FSM，除非进程完全失去响应且现场已经执行硬件急停。
