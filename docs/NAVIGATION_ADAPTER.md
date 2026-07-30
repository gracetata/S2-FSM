# 导航适配手册

## 结论

当前代码支持两类导航输入：

| high mode | low mode | 模型 | 三元组语义 |
| --- | --- | --- | --- |
| `1` | `1` | `free_walk.onnx` | 速度 `[vx,vy,yaw_rate]` |
| `1` | `2` | `accurate_arrival.onnx` | 骨盆坐标系闭环误差 `[dx_body,dy_body,dyaw]` |
| `3` | `1` | `walk_with_object.onnx` | 速度 `[vx,vy,yaw_rate]` |

high mode 3 只接受 low mode 1。导航速度会进入 `walk_with_object.onnx` 的 command
observation；双臂消息不作为当前帧独立模型输入，只在推理后覆盖双臂输出，覆盖后
实际 action 会进入下一帧 `previous_action`。

位置控制只允许 high mode 1 + low mode 2。
控制器不读取全局目标，不做坐标变换，不累计位移，也不保存第一次误差作为固定目标。
导航必须根据最新定位结果持续重算并发布误差。

high mode 4 不属于导航模式。状态机会忽略 low mode 和导航三元组，并固定向
`extreme_stand_recovery.onnx` 输入 `[0,0,0]` command。

## Topic

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `data: 1` 或 `data: 3` |
| `/hecbot/locomotion/low_level_mode` | `std_msgs/msg/UInt8` | 速度为 `1`；high mode 1 的位置控制为 `2` |
| `/hecbot/locomotion/navigation_command` | `std_msgs/msg/Float32MultiArray` | 恰好 3 个有限数值 |

发布器使用 reliable QoS。导航三元组推荐以 `20 Hz` 持续发布，消息间隔不得超过
`0.25 s`。

## 速度模式

high 1/low 1 和 high 3/low 1 使用相同速度定义：

- `vx`：骨盆前向速度，单位 m/s；
- `vy`：骨盆左向速度，单位 m/s；
- `yaw_rate`：从上方观察逆时针角速度，单位 rad/s。

速度三元组收到后在下一控制帧直接进入当前模型 observation，不做时间插值或加速度
限幅，只按 `max_velocity_command` 逐分量裁剪。

high mode 3 未收到 low mode 1、收到 low mode 2、导航消息尚未到达或消息超时时，
仍保持 `arm_walk` 模型，但其 command observation 为 `[0,0,0]`。

high mode 3 的发送顺序：

1. 持续发布 low mode `1`；
2. 应用层发布 high mode `3`；
3. 以 20 Hz 持续发布 `[vx,vy,yaw_rate]`；
4. 停止时持续发布 `[0,0,0]`。

## 位置模式坐标定义

骨盆坐标系约定：

- `+x`：骨盆前方
- `+y`：骨盆左方
- `+yaw`：从上方观察逆时针
- `dx_body`、`dy_body`：单位 `m`
- `dyaw`：单位 `rad`，归一化到 `(-π, π]`

如果目标和当前骨盆位姿都在全局坐标系：

```text
delta_x = goal_x - pelvis_x
delta_y = goal_y - pelvis_y

dx_body =  cos(pelvis_yaw) * delta_x + sin(pelvis_yaw) * delta_y
dy_body = -sin(pelvis_yaw) * delta_x + cos(pelvis_yaw) * delta_y
dyaw    = normalize(goal_yaw - pelvis_yaw)
```

必须使用骨盆位姿，不要使用相机、雷达、躯干或地图坐标系中的未变换误差。

## 位置模式发送逻辑

1. 发布 low mode `2`。
2. 发布 high mode `1`。
3. 每次定位更新后，用最新骨盆位姿重新计算误差。
4. 持续发布最新 `[dx_body, dy_body, dyaw]`。
5. 到达目标后持续发布 `[0,0,0]`，直到切换到其他模式或收到新目标。

从 low mode `1` 切换到 `2` 时，控制器先执行 `stand_duration_s` 的
`free_walk + [0,0,0]`。旧速度命令会被清除。站立期间导航仍应持续发布最新位置
误差；结束后控制器直接使用当时最新的一帧。

导航消息超时后，输入自动变为 `[0,0,0]`，但 low mode 不会自动改变。该行为同时
适用于 high 1/low 1、high 1/low 2 和 high 3/low 1。

## 数值示例

全局目标：

```text
goal = [x=2.0 m, y=2.5 m, yaw=45°]
```

第一帧骨盆位姿：

```text
pelvis = [x=1.0 m, y=2.0 m, yaw=30°]
command = [1.116, -0.067, 0.262]
```

机器人运动后的下一帧：

```text
pelvis = [x=1.4 m, y=2.2 m, yaw=35°]
command = [0.664, -0.098, 0.175]
```

到达目标：

```text
command = [0.0, 0.0, 0.0]
```

ROS 测试消息：

```bash
ros2 topic pub --rate 20 /hecbot/locomotion/navigation_command \
  std_msgs/msg/Float32MultiArray "{data: [0.664, -0.098, 0.175]}"
```

## 禁止行为

- 不要只发送第一帧误差。
- 不要发送全局坐标 `[goal_x, goal_y, goal_yaw]`。
- 不要发送速度 `[vx, vy, yaw_rate]` 给 low mode `2`。
- 不要在 high mode 3 使用 low mode `2`；它不会启动位置模型，只会使
  `arm_walk` 速度输入归零。
- 不要停止发布并依赖控制器自行完成闭环。
- 不要发送 `NaN`、无穷值或长度不是 3 的数组。
- 不要依赖控制器裁剪位置误差；位置三元组会直接进入模型 observation。
