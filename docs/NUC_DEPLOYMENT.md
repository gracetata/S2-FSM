# 多 NUC 可移植部署

仓库、构建目录、运行环境和日志可以位于任意路径。代码不依赖固定用户名、固定 SSH
地址或某个固定代码目录。每台 NUC 只需保存自己的本地环境
文件，Git 中的公共配置无需按机器反复修改。

## 1. 首次准备本机配置

在仓库根目录执行：

```bash
export FSM_ROOT="$(pwd -P)"
cp config/nuc.env.example config/nuc.env
```

编辑 `config/nuc.env`，逐项核对：

- `LOCOMOTION_RUNTIME_PYTHON`：包含 `onnxruntime`、`cyclonedds` 和
  `unitree_sdk2py` 的 Python；
- `LOCOMOTION_RUNTIME_HOME`：上述 Python 环境前缀；
- `LOCOMOTION_NETWORK_INTERFACE`：本 NUC 连接机器人的网卡名称；
- `LOCOMOTION_ROBOT_IP`：机器人地址；
- `LOCOMOTION_LOG_ROOT`：本 NUC 可写的日志目录。
- `LOCOMOTION_SOCKET_PATH`：本机 Unix socket 的绝对路径。

该文件已被 `.gitignore` 排除，不会把一台 NUC 的绝对路径提交给其他机器。

统一加载器会检查文件存在、六个变量非空、Python/环境目录有效、日志目录可写，并
用选中的 Python 导入四个控制依赖。运行：

```bash
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1

ip link show "$LOCOMOTION_NETWORK_INTERFACE"
ping -c 1 "$LOCOMOTION_ROBOT_IP"
```

必须看到 `[S2-FSM] NUC environment ready`。任何一步失败都不要启动实机控制器。
主 YAML 不再回退到系统 `python3`；即使调用者忽略加载器的失败，ROS 节点也会因
缺少强制环境变量而在接管机器人前退出。

## 2. 编译

```bash
source /opt/ros/jazzy/setup.bash
cd "$FSM_ROOT"
colcon build --symlink-install --packages-select locomotion_controller
source "$FSM_ROOT/install/setup.bash"
```

必须看到 `Summary: 1 package finished`。每次修改代码、模型、YAML 或文档安装内容后
重新执行 build，并重新 source 安装环境。

## 3. 每个新终端

```bash
cd <本机仓库目录>
export FSM_ROOT="$(pwd -P)"
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source /opt/ros/jazzy/setup.bash
source "$FSM_ROOT/install/setup.bash"
```

`<本机仓库目录>` 是占位符，不要求仓库放在任何固定绝对路径。也可以由整体启动
脚本自身目录计算 `FSM_ROOT`，不要依赖调用者当前工作目录。

## 4. 启动

确认没有其他 `rt/lowcmd` 发布者，机器人周围安全且急停可用后：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py
```

默认 launch 会使用安装包内的配置和模型。只有确实需要独立配置文件时才传：

```bash
ros2 launch locomotion_controller locomotion_controller.launch.py \
  config_file:="$FSM_ROOT/config/locomotion_controller.yaml"
```

配置中的六个部署变量没有隐式默认值，全部来自 `config/nuc.env`。模型和阻抗文件
等相对路径始终相对于 ROS 安装包 share 目录解析，因此项目目录可以不同；运行时
Python、环境前缀、网卡、机器人 IP、日志和 socket 则由每台 NUC 独立指定。

## 5. 更新代码后

```bash
cd "$FSM_ROOT"
git pull --ff-only
source "$FSM_ROOT/config/load_nuc_env.sh" || exit 1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select locomotion_controller
source "$FSM_ROOT/install/setup.bash"
```

不要复制另一台 NUC 的 `config/nuc.env`；新机器从 example 建立并逐项核对。模型、
ROS topic 和状态机行为在各 NUC 上相同，只有部署环境变量不同。

## 6. 为什么 build 成功仍可能无法启动

项目有两个 Python 进程：ROS 2 系统 Python 负责 topic 和 Unix socket；
`LOCOMOTION_RUNTIME_PYTHON` 指定的 Conda Python 负责 ONNX Runtime、CycloneDDS、
Unitree SDK2、五模型状态机和唯一的 50 Hz LowCmd。`colcon build` 只证明 ROS 包可
安装，不证明控制环境包含 `unitree_sdk2py`。因此每次启动前都必须成功 source
`load_nuc_env.sh`，不能直接依赖系统 `python3`。
