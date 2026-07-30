# High Mode 4：鲁棒站立恢复测试

High Mode 4 只用于独立测试 `extreme_stand_recovery.onnx` 的真机恢复效果，不替换
mode 1/2/3 中的任何现有策略。

只需要最短操作步骤时，直接阅读
[`MODE4_QUICK_TEST.md`](MODE4_QUICK_TEST.md)。

## 初始化后没有 high mode 时

状态机启动、五个模型预热、默认姿态移动和 2 秒初始化站立完成后，会发布
`/hecbot/locomotion/initialized = true`。

如果此后没有任何节点发送 high mode：

```text
high_mode = None
model = free_walk.onnx
command = [0,0,0]
arm override = none
```

50 Hz LowCmd 和推理线程仍持续运行。状态机不会自动进入 NAV 2、不会自动进入
High Mode 4，也不会自行行走；它只是用原 `free_walk` 零速度策略安全等待。

## High Mode 4 行为

收到 `/hecbot/locomotion/high_level_mode = 4` 后：

```text
model = extreme_stand_recovery.onnx
command observation = [0,0,0]
low mode = 不使用
navigation_command = 不使用
upper_body_cmd = 不使用
双臂输出 = 恢复模型自身的 29DoF 输出
default angles / Kp / Kd = 通用组
```

切入 mode 4 不执行额外的 `stand_duration_s` 等待，下一控制帧直接切换到恢复模型。
模型切换时 `previous_action` 清零，并继续使用现有 `model_switch_blend_s` 对腿腰目标
做短时融合；mode 4 不执行外部双臂覆盖。

新模型合同：

| 项目 | 值 |
| --- | --- |
| 文件 | `models/extreme_stand_recovery.onnx` |
| SHA256 | `81bc3c1a1744e5549a8209f3e46a8b46863ff2fe68a38f3f50719a7f0f25784e` |
| 输入 | `obs`, float32, `[1,96]` |
| 输出 | `actions`, float32, `[1,29]` |
| 控制频率 | 50 Hz |
| action scale | 0.25 |
| 关节顺序 | `controller.policy_joint_names` |

模型的 observation 布局、29DoF 顺序、默认角和通用 Kp/Kd 已逐项核对，与当前
控制器合同一致。

## 键盘真机测试

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

等待：

```text
five ONNX models are ready; initialization stand is complete
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
source install/setup.bash
ros2 run locomotion_controller locomotion_controller_simulator
```

直接按：

```text
4
```

数字 `4` 只发送 High Mode 4，不需要按 `k`、`v`、`p`，也不需要导航或双臂节点。
模拟器启动后默认静默不发布导航/双臂参数，但 high mode 按键始终有效。

控制器终端应出现：

```text
模式切换为 high mode 4
```

随后 `[INFERENCE]` 日志应包含：

```json
"model":"stand_recovery","high_mode":4,"navigation_input":{"model_input":[0.0,0.0,0.0]}
```

原键盘的“前进三秒后停止”从数字 `4` 改为 `w`；`5/6` 仍是横移和转向轨迹。

测试结束先在键盘模拟器按 `q`，再在控制器终端按 `Ctrl+C`，等待阻尼收尾。真机
恢复测试必须确保周围无人、急停可用，并从可控的小扰动开始。
