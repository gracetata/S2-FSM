# 极简：替换 Kp/Kd YAML

`impedancepara_default.yaml` 是全默认诊断文件：mode 1、mode 2、mode 3 使用相同的
29DoF 默认角、Kp 和 Kd。数组顺序是 `policy_joint_names`，不要按
`whole_body_state` 顺序重排。

两套 29DoF 顺序、右踝在两套顺序中的下标，以及 LowState→模型→LowCmd 的完整
置换见 [`JOINT_ORDER_AND_MAPPING.md`](JOINT_ORDER_AND_MAPPING.md)。

右踝过热时先停止控制器并等待电机冷却，不要带故障反复测试。

## 使用全默认文件

```bash
cd /home/wenduo/locomotion_controller
git pull
```

打开 `config/locomotion_controller.yaml`，只修改这一行：

```yaml
impedance_file: impedancepara_default.yaml
```

重新编译并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/wenduo/locomotion_controller
colcon build --symlink-install --packages-select locomotion_controller
source install/setup.bash
```

确认安装后的配置和文件：

```bash
grep impedance_file install/locomotion_controller/share/locomotion_controller/config/locomotion_controller.yaml
test -f install/locomotion_controller/share/locomotion_controller/impedancepara_default.yaml && echo OK
```

两个命令应分别显示 `impedancepara_default.yaml` 和 `OK`。然后按正常命令启动：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

## 恢复当前分模式文件

停止控制器，把配置改回：

```yaml
impedance_file: impedancepara.yaml
```

再次执行同一条 `colcon build`，然后重新 `source install/setup.bash` 并启动。

不要直接覆盖或删除 `impedancepara.yaml`，这样可以随时恢复并对比两次测试结果。
