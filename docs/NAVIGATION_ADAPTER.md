# 导航适配手册

## 结论

当前代码支持导航持续发送闭环位置误差。

位置控制使用：

- high mode：`1`
- low mode：`2`
- 模型：`accurate_arrival.onnx`
- 输入：骨盆坐标系实时误差 `[dx_body, dy_body, dyaw]`

控制器不读取全局目标，不做坐标变换，不累计位移，也不保存第一次误差作为固定目标。
导航必须根据最新定位结果持续重算并发布误差。

## Topic

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `data: 1` |
| `/hecbot/locomotion/low_level_mode` | `std_msgs/msg/UInt8` | `data: 2` |
| `/hecbot/locomotion/navigation_command` | `std_msgs/msg/Float32MultiArray` | `data: [dx_body, dy_body, dyaw]` |

发布器使用 reliable QoS。误差推荐以 `20 Hz` 持续发布，消息间隔不得超过
`0.25 s`。

## 坐标定义

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

## 发送逻辑

1. 发布 low mode `2`。
2. 发布 high mode `1`。
3. 每次定位更新后，用最新骨盆位姿重新计算误差。
4. 持续发布最新 `[dx_body, dy_body, dyaw]`。
5. 到达目标后持续发布 `[0,0,0]`，直到切换到其他模式或收到新目标。

从 low mode `1` 切换到 `2` 时，控制器先执行 `stand_duration_s` 的
`free_walk + [0,0,0]`。旧速度命令会被清除。站立期间导航仍应持续发布最新位置
误差；结束后控制器直接使用当时最新的一帧。

导航消息超时后，输入自动变为 `[0,0,0]`，但 low mode 仍保持 `2`。

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
- 不要停止发布并依赖控制器自行完成闭环。
- 不要发送 `NaN`、无穷值或长度不是 3 的数组。
- 不要依赖控制器裁剪位置误差；位置三元组会直接进入模型 observation。
