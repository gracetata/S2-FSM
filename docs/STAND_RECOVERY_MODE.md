# High Mode 4：鲁棒站立恢复测试

High Mode 4 用于显式进入 `extreme_stand_recovery.onnx`。同一个模型用于初始化、
安全回退、模式过渡以及导航缺失/超时，但 High 1 / Low 1 明确收到 `[0,0,0]` 时
仍使用 `free_walk.onnx`。它不会替换新鲜速度跟踪、`accurate_arrival`、
`standing_grasp` 或 `walk_with_object`。

只需要最短操作步骤时，直接阅读
[`MODE4_QUICK_TEST.md`](MODE4_QUICK_TEST.md)。

## 初始化后没有 high mode 时

状态机启动、五个模型预热、默认姿态移动和 2 秒初始化站立完成后，会发布
`/hecbot/locomotion/initialized = true`。

如果此后没有任何节点发送 high mode：

```text
high_mode = None
model = extreme_stand_recovery.onnx
command = [0,0,0]
arm override = none
```

50 Hz LowCmd 和推理线程仍持续运行。恢复模型已经生效，但业务状态仍是
`high_mode=None`，不是自动发送了 high mode 4，也不会自动进入 NAV 2 或自行行走。

## 内部站立使用位置

以下情况统一选择 `stand_recovery + [0,0,0]`：

- 初始化首帧后的健康站立；
- 初始化完成后尚未收到 high mode；
- mode 1 或 mode 2 切入后的 `stand_duration_s`；
- high 1/low 1 的速度尚未收到或已经超时；明确收到 `[0,0,0]` 不在此列；
- high 1 从 low 1 切换到 low 2 的等待期；
- high mode 或 low mode 非法后的安全回退。

low 1→low 2 时不会把 high mode 改成 `4`。状态仍是 high 1/low 2，并标记
`standing_transition=true`；等待结束后才选择 `accurate_arrival`。

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
| SHA256 | `eb2e993220d2e4a343602dfa1556064fce440ce230580803930f7b82151eab6e` |
| 来源 checkpoint | `model_20.pt`（2026-08-07 jerk-limited V4 正式模型） |
| 输入 | `obs`, float32, `[1,96]` |
| 输出 | `actions`, float32, `[1,29]` |
| 控制频率 | 50 Hz |
| action scale | 0.25 |
| 关节顺序 | `controller.policy_joint_names` |

模型的 observation 布局、29DoF 顺序、默认角和通用 Kp/Kd 已逐项核对，与当前
控制器合同一致。

运行时每个恢复推理帧都会发布到 `/hecbot/locomotion/policy_input`，消息中
`model=stand_recovery`，并携带推理前捕获的完整 `obs`。初始化、安全等待、内部
过渡和显式 high mode 4 都可通过 `high_mode` 与 `standing_transition` 字段区分。

完整来源、checkpoint 哈希和 velocity tracking 模型版本见
[`MODEL_PROVENANCE.md`](MODEL_PROVENANCE.md)。

## 键盘真机测试

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source "$FSM_ROOT/install/setup.bash"
ros2 launch locomotion_controller locomotion_controller.launch.py
```

等待：

```text
five ONNX models are ready; stand-recovery initialization is complete
```

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source "$FSM_ROOT/install/setup.bash"
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

固定的“前进三秒后停止”轨迹使用 `f`；`5/6` 仍是横移和转向轨迹。

测试结束先在键盘模拟器按 `Esc` 或 `Ctrl+C`，再在控制器终端按 `Ctrl+C`，等待阻尼收尾。真机
恢复测试必须确保周围无人、急停可用，并从可控的小扰动开始。
