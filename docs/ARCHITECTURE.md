# 运行架构与接口合同

## 1. 设计边界

系统只有两个进程角色：

```text
ROS 2 进程
  4 个订阅者 + initialized 发布者
                 │ 本机 Unix socket
                 ▼
ONNX/Unitree 运行进程
  纯状态机 + 4 个预热 Session + 唯一 50 Hz 控制线程
                 │ Unitree DDS
                 ▼
        rt/lowstate / rt/lowcmd
```

保留进程隔离只有一个原因：ROS 2 节点运行在系统 Python，ONNX Runtime、
CycloneDDS 和 Unitree SDK2 运行在现有 Conda Python。业务状态、输入缓存、
模型选择、推理和 Unitree 输出全部在控制进程内，不存在第二套状态机或第二个
`LowCmd` 发布者。

## 2. 初始化时序

`locomotion_controller_node` 按以下固定顺序启动：

1. 严格读取 YAML。缺少字段、出现未知字段或模型文件不存在时立即失败。
2. 启动配置指定的 Conda Python 子进程。
3. 创建四个 ONNX Runtime Session，并分别执行一次 96 维零 observation 推理。
4. 初始化 Unitree DDS，等待第一帧有效 `rt/lowstate`。
5. 确认 `confirm_real_robot: true`，并依次验证
   `MotionSwitcher mode=ai` 与 `Loco FSM=0 (ZeroTorque)`。任一查询失败、结构
   异常或状态不匹配都会在发送策略控制前终止初始化。
6. 调用 `MotionSwitcher.ReleaseMode()`，并再次查询直到模式名严格变为空；出现
   其他模式或超时立即失败。
7. 从当前实机关节位置按 minimum-jerk 轨迹移动到策略默认姿态。
8. 启动唯一的 50 Hz 线程。此时尚无业务模式，线程执行
   `free_walk + [0,0,0]` 安全等待状态。
9. 确认第一帧 `rt/lowcmd` 已经发送，并继续保持
   `initialization_stand_duration_s`；当前配置严格为 2 秒。期间持续检查控制线程
   和 LowState，任何故障都会终止初始化。
10. 完成初始化站立后创建 Unix socket。
11. ROS 节点收到运行时健康响应后创建四个订阅者，最后发布
   `/hecbot/locomotion/initialized = true`。

任何一步失败都不会发布初始化完成。

## 3. 状态机

### 3.1 状态变量

状态机仅保存：

- application 指定的 `high_mode`；
- navigation 指定的 `low_mode`；
- 最新导航三元组及接收时间；
- 最新双臂命令及接收时间；
- mode 1/2 切换前的站立截止时间。

初始 `high_mode` 和 `low_mode` 都是“尚未提供”，不是某个默认业务模式。

### 3.2 选择表

| 条件 | 模型 | command observation | 双臂 |
| --- | --- | --- | --- |
| 未收到 high mode | `free_walk` | `[0,0,0]` velocity | 模型输出 |
| mode 1/2 的站立过渡期 | `free_walk` | `[0,0,0]` velocity | 模型输出 |
| high 1 + low 1 | `free_walk` | 最新导航 `[vx,vy,yaw_rate]` | 模型输出 |
| high 1 + low 2 | `accurate_arrival` | 最新导航 `[dx,dy,dyaw]` | 模型输出 |
| high 1 + 未收到 low | `free_walk` | `[0,0,0]` velocity | 模型输出 |
| high 2 | `arm_stand` | `[0,0,0]` velocity | 外部 14DoF 覆盖 |
| high 3 | `arm_walk` | `[0,0,0]` velocity | 外部 14DoF 覆盖 |

mode 1 与 mode 2 的 high-level 请求都会开始一次明确的站立事务。站立期间
`command_ramp` 被绕过，模型 observation 的 command 三元组是严格零值。
`stand_duration_s` 到期后，50 Hz 线程自然选择目标模型。

low-level mode 只在 high mode 1 下解释；在其他 high mode 下收到的值仅被保存，
不影响当前模型。high mode 1 内从 low mode 1 切换到 low mode 2 时，状态机清除
旧速度参数，先执行 `stand_duration_s` 的 `free_walk + [0,0,0]`，再进入
`accurate_arrival`。从 low mode 2 切回 low mode 1 不增加等待，但同样先清除旧
位置参数，因此新速度尚未到达时使用 `[0,0,0]`。

任何未匹配的 high/low mode 分支都选择 `free_walk + [0,0,0]`。非法模式消息会
返回拒绝结果，同时先把状态机置于这一安全回退状态，而不是保留上一模式。

### 3.3 参数不同步时的确定值

模式和对应参数不要求在同一时刻到达，控制帧也不会使用空参数：

- 进入或切换导航语义后，尚未收到对应新参数时使用 `[0,0,0]`。
- 进入 mode 2/3 时，尚未收到新双臂参数则使用上一控制帧的实际双臂目标；控制器
  每帧更新该值，因此输入为空或不同步时不会发生关节目标跳变。
- 收到有效新双臂参数后，按消息 `weight` 在切入当前模型时保存的基线和新目标之间
  融合。

### 3.4 超时

- 导航输入超过 `navigation_timeout_s`：三元组替换为零，不改变模式。
- 双臂输入超过 `arm_timeout_s`：外部覆盖移除，14DoF 双臂目标保持上一控制帧的
  实际目标。
- `rt/lowstate` 超过 `lowstate_runtime_timeout_s`：50 Hz 线程进入故障，停止推理并
  连续发送配置时长的阻尼命令。

这些行为全部由显式 YAML 参数控制，没有隐藏缺省时间。

## 4. 每个 50 Hz 控制帧

每帧只执行一条路径：

1. 检查 LowState 新鲜度。
2. 从状态机取得一个不可变的模型/命令/双臂选择结果。
3. 如果模型发生变化，保存上一帧目标用于动作融合，并清空新模型的
   `previous_action`。
4. 按 policy joint order 读取 29DoF 关节位置、速度和 pelvis IMU。
5. 形成 96 维 observation：
   - `0:3` 机体角速度；
   - `3:6` 重力方向；
   - `6:9` 当前三元组命令；
   - `9:38` 相对默认角的 29DoF 位置；
   - `38:67` 29DoF 速度；
   - `67:96` 上一帧实际执行 action。
6. 调用当前预热 ONNX Session，得到 29 维 action。
7. mode 2/3 下按 `weight` 将外部双臂位置与切入基线融合，覆盖 action 中的
   14 个双臂分量；外部双臂速度同样乘以 `weight`。
8. 对模型切换前后的目标关节位置做线性融合。
9. 转为 Unitree motor order，写入位置、速度、Kp、Kd 和 CRC。
10. 发布唯一一帧 `rt/lowcmd`，等待下一个单调时钟 deadline。

mode 2 使用 `arm_stand_*` 的默认角和增益；其他三个模型使用通用参数。

## 5. ROS 接口

### 5.1 high-level mode

- Topic：`topics.high_level_mode`
- 类型：`std_msgs/msg/UInt8`
- 合法值：`1`、`2`、`3`
- 发布者：应用层

只有值变化才触发切换。重复值用于上游重发时不会重置站立计时。

### 5.2 low-level mode

- Topic：`topics.low_level_mode`
- 类型：`std_msgs/msg/UInt8`
- 合法值：`1` 速度、`2` 位置
- 发布者：导航脚本

状态机不从导航数值字段推断 submode。

### 5.3 导航输入

- Topic：`topics.navigation_command`
- 类型：`std_msgs/msg/Float32MultiArray`
- 长度：恰好 3
- 发布者：导航脚本

low mode 1 时单位为 `[m/s, m/s, rad/s]`；low mode 2 时单位为
`[m, m, rad]`。速度命令最终按 `max_velocity_command` 逐分量限制；位置误差不套用
速度限制。

### 5.4 双臂输入

- Topic：`topics.arm_command`
- 类型：`std_msgs/msg/String`
- 发布者：双臂操作脚本

JSON 合同：

| 字段 | 类型 | 约束 |
| --- | --- | --- |
| `schema` | string | 必须是 `hecbot.upper_body_command.v1` |
| `seq` | integer | 非负且相对上一条严格增加 |
| `arm_q` | float[14] | 有限数，rad |
| `arm_dq` | float[14] | 有限数，rad/s |
| `weight` | float | `[0,1]` |

所有字段必填，不接受别名、缺省值或额外字段。

### 5.5 初始化输出

- Topic：`topics.initialized`
- 类型：`std_msgs/msg/Bool`
- QoS：reliable、transient-local、depth 1
- 值：只有完整初始化成功后才发布 `true`

“完整初始化”包含首帧 LowCmd 后的 2 秒 `free_walk + [0,0,0]` 健康站立。

## 6. 配置原则

YAML 是唯一运行参数来源，launch 只暴露 `config_file`。加载器要求五个顶层段：

- `topics`
- `runtime`
- `state_machine`
- `models`
- `controller`

每个段都执行精确 key 集合检查。增加新字段时必须同步修改加载器、本文档和测试；
拼错字段不会被静默忽略。

## 7. 关闭与故障

正常关闭：

1. ROS 节点向控制进程发送 `shutdown`；
2. 50 Hz 线程停止并 join；
3. 控制进程按 `fault_damping_duration_s` 发送 `Kp=0, Kd=8`；
4. 删除权限为 `0600` 的本机 Unix socket。

推理、DDS 或 LowState 检查抛出异常时，控制线程执行相同阻尼路径并记录故障。
本版本不会在退出时自动重新选择 Unitree 原生运动模式。

## 8. 键盘测试节点

`locomotion_controller_simulator` 是独立 ROS 2 输入节点，不接触 Unitree DDS，也不
发布 `LowCmd`。它订阅 transient-local 的初始化 topic，只有控制器完成模型预热、
首帧发送和初始化站立后才开始发布四类上游输入。

测试节点以 20 Hz 刷新导航三元组和严格双臂 JSON。high/low mode 只在键盘选择时
发布；速度轨迹结束后自动保持 `[0,0,0]`。双臂姿态使用
`config/simulator_presets.json` 中的持续时间执行 minimum-jerk 插值，发布位置和
解析速度。进程内双臂序号从系统 monotonic nanosecond 起始，测试节点重启后仍高于
同一次系统启动中的旧序号。
