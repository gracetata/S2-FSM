# 运行架构与接口合同

## 1. 设计边界

系统只有两个进程角色：

```text
ROS 2 进程
  4 个订阅者 + initialized/whole_body_state 发布者
                 │ 本机 Unix socket
                 ▼
ONNX/Unitree 运行进程
  纯状态机 + 5 个预热 Session + 唯一 50 Hz 控制线程
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
3. 创建五个 ONNX Runtime Session，并分别执行一次 96 维零 observation 推理。
4. 初始化 Unitree DDS，等待第一帧有效 `rt/lowstate`。
5. 确认 `confirm_real_robot: true`，读取当前 MotionSwitcher 模式。初始化不检查
   Loco FSM ID，也不要求机器人预先处于 FSM 0。
6. 当前模式名非空时调用 `MotionSwitcher.ReleaseMode()`，并再次查询直到模式名
   严格变为空，即确认进入低层调试模式；已经是空模式时直接继续。查询结构异常、
   释放失败或超时会在发送策略控制前终止初始化。
7. 从当前实机关节位置按 minimum-jerk 轨迹移动到策略默认姿态。
8. 启动唯一的 50 Hz 线程。此时尚无业务模式，线程执行
   `stand_recovery + [0,0,0]` 安全等待状态。
9. 确认第一帧 `rt/lowcmd` 已经发送，并继续保持
   `initialization_stand_duration_s`；当前配置严格为 2 秒。期间持续检查控制线程
   和 LowState，任何故障都会终止初始化。
10. 完成初始化站立后创建 Unix socket。
11. ROS 节点收到运行时健康响应后创建四个订阅者和 50 Hz 状态反馈定时器，最后
   发布 `/hecbot/locomotion/initialized = true`。定时器从控制进程读取最新实测
   LowState 关节角并发布 `/hecbot/whole_body_state`。

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
| 未收到 high mode | `stand_recovery` | `[0,0,0]` velocity | 模型输出 |
| mode 1/2 的站立过渡期 | `stand_recovery` | `[0,0,0]` velocity | 模型输出 |
| high 1 + low 1 + 非零新鲜速度 | `free_walk` | 最新导航 `[vx,vy,yaw_rate]` | 模型输出 |
| high 1 + low 1 + 零速/缺失/超时 | `stand_recovery` | `[0,0,0]` velocity | 模型输出 |
| high 1 + low 2 | `accurate_arrival` | 最新导航 `[dx,dy,dyaw]` | 模型输出 |
| high 1 + 未收到 low | `stand_recovery` | `[0,0,0]` velocity | 模型输出 |
| high 2 | `arm_stand` | `[0,0,0]` velocity | 推理后外部 14DoF 覆盖 |
| high 3 + low 1 | `arm_walk` | 最新导航 `[vx,vy,yaw_rate]` | 推理后外部 14DoF 覆盖 |
| high 3 + 其他/未收到 low | `arm_walk` | `[0,0,0]` velocity | 推理后外部 14DoF 覆盖 |
| high 4 | `stand_recovery` | `[0,0,0]` velocity | 模型完整 29DoF 输出 |

mode 1 与 mode 2 的 high-level 请求都会开始一次明确的站立事务。站立期间
模型固定为 `stand_recovery`，command 三元组严格为零。
`stand_duration_s` 到期后，50 Hz 线程自然选择目标模型。
high mode 4 与 mode 3 一样直接切入，不执行这段站立等待。

high mode 1 解释 low mode 1/2；high mode 3 只解释 low mode 1，并把最新速度命令
路由到 `arm_walk`。high mode 3 收到 low mode 2 时仍保持 `arm_walk`，但 command
严格为 `[0,0,0]`。high mode 1 内从 low mode 1 切换到 low mode 2 时，状态机清除
旧速度参数，先执行 `stand_duration_s` 的 `stand_recovery + [0,0,0]`，再进入
`accurate_arrival`。切换等待期间 high mode 保持 `1`、low mode 已是 `2`，不会
伪造 high mode 4。从 low mode 2 切回 low mode 1 不增加等待，但同样先清除旧位置
参数；新非零速度尚未到达时使用 `stand_recovery`。

high mode 1 的未匹配 low 分支选择 `stand_recovery + [0,0,0]`；high mode 3 的
非 low-1 分支保持 `arm_walk + [0,0,0]`。非法 high/low mode 数值会返回拒绝结果，
同时先把状态机置于 `stand_recovery + [0,0,0]` 安全回退，而不是保留上一模式。
high mode 4 忽略 low mode、导航缓存和双臂缓存，固定选择
`stand_recovery + [0,0,0]`。

### 3.3 参数不同步时的确定值

模式和对应参数不要求在同一时刻到达，控制帧也不会使用空参数：

- 进入或切换导航语义后，尚未收到对应新参数时三元组使用 `[0,0,0]`。high 1/low 1
  同时切换到 `stand_recovery`；high 1/low 2 保持 `accurate_arrival`；high 3/low 1
  保持 `arm_walk`。
- 每次进入 mode 2/3 都作废进入该 mode 之前缓存的双臂消息。尚未收到切换后的
  新双臂参数时使用上一控制帧的实际双臂目标；控制器每帧更新该值，因此输入为空
  或不同步时不会发生关节目标跳变。
- 收到有效新双臂参数后，按消息 `weight` 在切入当前模型时保存的基线和新目标之间
  融合，并在当前帧直接输出结果，不生成时间插值轨迹。

### 3.4 超时

- 导航输入超过 `navigation_timeout_s`：三元组替换为零，不改变 high/low mode；
  high 1/low 1 的模型切换到 `stand_recovery`。
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
7. ONNX 推理完成后，mode 2/3 才按 `weight` 将外部双臂位置与切入基线融合，
   覆盖模型输出 action 中的 14 个双臂分量；外部双臂速度同样乘以 `weight`。
   外部双臂消息不是当前帧模型的独立输入。双臂分量绕过模型切换融合，在收到消息
   的当前帧直接生效。
8. 仅对模型切换前后的腿、腰目标关节位置做线性融合。
9. 用覆盖后最终目标反算实际执行 action，保存为下一帧 observation 的
   `previous_action`；因此该切片包含双臂覆盖后的实际 action。
10. 转为 Unitree motor order，写入位置、速度、Kp、Kd 和 CRC。
11. 发布唯一一帧 `rt/lowcmd`，等待下一个单调时钟 deadline。

mode 2 使用 `impedancepara.yaml` 中的 `*_standing_grasp` 默认角和 Kp/Kd。
mode 3 使用通用默认角，同时使用与 mode 2 完全相同的
`kps_standing_grasp/kds_standing_grasp`。初始化、过渡、mode 1 和 mode 4 使用
通用默认角与通用 Kp/Kd。

## 5. ROS 接口

### 5.1 high-level mode

- Topic：`topics.high_level_mode`
- 类型：`std_msgs/msg/UInt8`
- 合法值：`1`、`2`、`3`、`4`
- 发布者：应用层

只有值变化才触发切换。重复值用于上游重发时不会重置站立计时。

### 5.2 low-level mode

- Topic：`topics.low_level_mode`
- 类型：`std_msgs/msg/UInt8`
- 合法值：`1` 速度、`2` 位置
- 发布者：导航脚本

high mode 1 支持 low mode 1/2；high mode 3 只使用 low mode 1。状态机不从导航
数值字段推断 submode。

### 5.3 导航输入

- Topic：`topics.navigation_command`
- 类型：`std_msgs/msg/Float32MultiArray`
- 长度：恰好 3
- 发布者：导航脚本

high 1/low 1 和 high 3/low 1 时单位为 `[m/s, m/s, rad/s]`；high 1/low 2 时
单位为 `[m, m, rad]`。速度命令最终按 `max_velocity_command` 逐分量限制；位置
误差不套用速度限制。速度命令没有时间插值或加速度限幅，收到后在下一控制帧直接
写入当前模型 observation 的 command 切片。

### 5.4 双臂输出覆盖命令

- Topic：`topics.arm_command`
- 类型：`std_msgs/msg/String`
- 发布者：双臂操作脚本

JSON 合同：

| 字段 | 类型 | 约束 |
| --- | --- | --- |
| `schema` | string | 必须是 `hecbot.upper_body_command.v1` |
| `seq` | integer | 非负；新 payload 严格增加，相同 payload 可同序号心跳 |
| `arm_q` | float[14] | 有限数，rad |
| `arm_dq` | float[14] | 有限数，rad/s |
| `weight` | float | `[0,1]` |

所有字段必填，不接受别名、缺省值或额外字段。

该消息不是当前帧 ONNX 的独立输入。控制器在推理结束后才读取它并覆盖 14 个双臂
输出。覆盖后的实际 action 会按策略合同进入下一帧 observation 的
`previous_action`；机器人运动后产生的实测关节反馈也会正常出现在
joint-position observation 中。

### 5.5 初始化输出

- Topic：`topics.initialized`
- 类型：`std_msgs/msg/Bool`
- QoS：reliable、transient-local、depth 1
- 值：只有完整初始化成功后才发布 `true`

“完整初始化”包含首帧 LowCmd 后的 2 秒
`stand_recovery + [0,0,0]` 健康站立。

### 5.6 全身关节角输出

- Topic：`topics.whole_body_state`
- 类型：`std_msgs/msg/String`
- 频率：50 Hz（ROS 定时器周期与 `controller.control_dt=0.02` 相同）
- 内容：恰好 29 个有限浮点数的紧凑 JSON 数组，单位 rad
- 数据源：Unitree `rt/lowstate` 的实测 `motor_state[].q`
- 顺序：严格等于 `controller.motor_joint_names`

该输出是关节反馈角，不是本帧 LowCmd 的目标角。配置加载器要求
`motor_joint_names` 与公开接口的 29 关节顺序完全一致，避免配置变化静默改变数组
语义。ROS 进程通过现有本机 Unix socket 读取控制进程中的最新快照；返回维度错误或
出现非有限数值时该帧不发布，并记录错误。

### 5.7 推理前模型输入输出

- Topic：`topics.policy_input`
- 类型：`std_msgs/msg/String`
- 产生位置：控制线程组装完 96 维 observation 后、调用 ONNX `infer()` 前
- 内容：`hecbot.policy_input.v1` 自描述紧凑 JSON
- 覆盖模型：五个预加载模型使用完全相同的消息合同

控制线程先复制本帧 observation，再开始 ONNX 推理；因此 topic 数据不是根据输出
或 LowCmd 反推。每包包含全局 `frame`、时间、实际模型、high/low mode、导航命令
语义、policy 关节顺序、observation 切片定义和完整 96 个 float32 值。

控制进程保存最近 256 帧不可变输入包。本地 IPC 每次最多顺序取 16 帧，首次连接只
取当前最新帧，避免初始化期间的历史数据在 ROS 节点启动时突发发布。正常情况下 ROS
定时器每 20 ms 取到一个新帧；短暂延迟时会按 `frame` 补发。缓存覆盖导致跳帧时 ROS
节点会打印明确警告。详细合同见 `POLICY_INPUT_TOPIC.md`。

## 6. 配置原则

YAML 定义完整运行参数，launch 只暴露 `config_file`。仅 `runtime` 中明确写成
`${VAR}` 或 `${VAR:-default}` 的部署值可由每台 NUC 的环境覆盖；状态机、模型路由、
关节合同和控制参数不会被环境变量隐式修改。加载器要求五个顶层段：

- `topics`
- `runtime`
- `state_machine`
- `models`
- `controller`

每个段都执行精确 key 集合检查。`controller.impedance_file` 指向第二个严格 YAML；
当前为包根目录的 `impedancepara.yaml`。它必须只包含通用和 standing-grasp 两组
default angles、Kp、Kd，共六个 29 维数组。mode 2 使用完整 standing-grasp 组；
mode 3 使用通用 default angles 和 standing-grasp Kp/Kd；其他模式和初始化使用
通用组。增加新字段时必须同步修改加载器、本文档和测试；拼错字段不会被静默忽略。

`runtime` 部署字符串支持 `${VAR}` 和 `${VAR:-default}`；相对路径按安装包 share
目录解析，因而仓库和安装目录不绑定某台 NUC。每次运行的控制子进程 stdout/stderr
保存到 `runtime.log_root/runtime/`；每次进入 `accurate_arrival` 的 50 Hz 结构化
复现日志保存到 `runtime.log_root/ToTarget/`。正式部署推荐由每台 NUC 的
`config/nuc.env` 指定可写日志目录和运行环境。

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
首帧发送和初始化站立后才允许键盘选择结果发往状态机。

测试节点默认关闭导航和双臂参数发布，只允许 high/low mode 键生效；`n` 单独开关
20 Hz 导航，`m` 单独开关双臂，`k` 兼容地同时开关两者。high/low mode 只在键盘选择时发布；
速度轨迹结束后自动保持 `[0,0,0]`。双臂姿态使用
`config/simulator_presets.json` 中的目标值直接切换，不生成中间姿态，发布速度为
零。其中 `z/x/c` 与 `models/walk_with_object_arm_pose_set.json` 的三个姿态一致，
是 high mode 3 的推荐持物行走预设；`b` 只是额外的接口联调姿态。进程内双臂序号
使用 Unix epoch 纳秒并保证递增，以适配时钟同步的多 NUC 部署。按 `k` 可同时
开始/停止导航和双臂参数的周期发布；停止不会
影响 high/low mode 发布，因此默认即可只作为模式切换终端，与外部真实导航节点
并行使用。

## 9. ToTarget 结构化日志

控制线程检测到模型从其他模型切换为 `accurate_arrival` 时，以进入该模型第一帧的
`[dx,dy,dyaw]` 和墙上时间创建一个独立 JSONL 会话。切出模型或关闭控制器时写入
`session_end`。每个模型控制帧写入完整 96 维 observation、原始 29 维 action、
LowState 实际关节位置/速度、策略使用的 IMU、实时位置误差以及最终 policy/motor
order LowCmd 目标和增益。

JSON 序列化和磁盘 I/O 位于独立有界队列线程，50 Hz 线程只复制当帧不可变数据并
入队。队列最多缓存 4096 帧；写盘无法跟上或写线程失败时控制线程显式进入故障，
不会生成看似完整但实际缺帧的日志。
