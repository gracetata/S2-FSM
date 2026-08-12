# 导航适配（极简）

## 1. 最小接口

导航侧只需对齐以下 ROS 2 topic：

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/hecbot/locomotion/high_level_mode` | `std_msgs/msg/UInt8` | `1`：普通导航；`3`：双臂行走 |
| `/hecbot/locomotion/low_level_mode` | `std_msgs/msg/UInt8` | `1`：速度；`2`：到点误差 |
| `/hecbot/locomotion/navigation_command` | `std_msgs/msg/Float32MultiArray` | 恰好 3 个有限数值 |

推荐 reliable QoS、`10~20 Hz` 持续发布。FSM 的导航超时为 `0.25 s`，超时后输入
自动归零。模式切换会清除旧导航数据，因此切换后必须继续发布新命令。

## 2. 速度导航

适用组合：

- high `1` + low `1`：普通行走；
- high `3` + low `1`：双臂行走。

`navigation_command` 定义为：

```text
[vx, vy, yaw_rate]
```

- `vx`：骨盆前方为正，单位 `m/s`；
- `vy`：骨盆左方为正，单位 `m/s`；
- `yaw_rate`：俯视逆时针为正，单位 `rad/s`。

发送顺序：发布 high mode，发布 low mode `1`，然后持续发布速度。停车时持续发布
`[0,0,0]`。high `3` 不支持 low `2`。

## 3. ToTarget 到点导航

只使用 high `1` + low `2`。发送：

```text
[dx_body, dy_body, dyaw]
```

它表示目标相对**当前骨盆坐标系**的闭环误差，不是全局目标坐标，也不是速度：

- `dx_body`：目标在骨盆前方为正，单位 `m`；
- `dy_body`：目标在骨盆左方为正，单位 `m`；
- `dyaw`：俯视逆时针为正，单位 `rad`，归一化到 `(-π,π]`。

若目标与骨盆位姿均在全局坐标系：

```text
delta_x = goal_x - pelvis_x
delta_y = goal_y - pelvis_y
dx_body =  cos(pelvis_yaw) * delta_x + sin(pelvis_yaw) * delta_y
dy_body = -sin(pelvis_yaw) * delta_x + cos(pelvis_yaw) * delta_y
dyaw    = normalize(goal_yaw - pelvis_yaw)
```

每次定位更新都要重新计算并持续发布，不能只发送第一帧。low `1 → 2` 时 FSM 会先
短暂零速站立；导航侧在此期间仍应持续发布最新误差。到达后持续发布 `[0,0,0]`。

## 4. 接入规则

- `/navigation_command` 同一时刻只能有一个实际数据源；键盘模拟和真实导航不要同时发送。
- 不得发送 `NaN`、无穷值或长度不为 3 的数组。
- 不得把 `[goal_x,goal_y,goal_yaw]` 直接发给 low `2`。
- 如使用项目自带 `totarget_navigation_bridge`，导航只发布 bridge 的输入 topic，
  不要再直接发布 `/hecbot/locomotion/navigation_command`。
- 需要普通零速站立时，由应用层切 high mode `5`。

## 5. 最小验证

观察 FSM 实际收到的模式和模型输入：

```bash
ros2 topic echo /hecbot/locomotion/policy_input std_msgs/msg/String
```

速度模式应看到 `semantics=velocity`；到点模式应看到
`model=accurate_arrival`、`semantics=target_pose`，且 `model_input` 与导航最新发布的
三元组一致。
