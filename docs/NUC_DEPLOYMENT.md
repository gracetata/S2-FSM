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

编辑 `config/nuc.env`，至少核对：

- `LOCOMOTION_RUNTIME_PYTHON`：包含 `onnxruntime`、`cyclonedds` 和
  `unitree_sdk2py` 的 Python；
- `LOCOMOTION_RUNTIME_HOME`：上述 Python 环境前缀；
- `LOCOMOTION_NETWORK_INTERFACE`：本 NUC 连接机器人的网卡名称；
- `LOCOMOTION_ROBOT_IP`：机器人地址；
- `LOCOMOTION_LOG_ROOT`：本 NUC 可写的日志目录。

该文件已被 `.gitignore` 排除，不会把一台 NUC 的绝对路径提交给其他机器。

加载并检查本机参数：

```bash
set -a
source "$FSM_ROOT/config/nuc.env"
set +a

test -x "$LOCOMOTION_RUNTIME_PYTHON"
test -d "$LOCOMOTION_RUNTIME_HOME"
mkdir -p "$LOCOMOTION_LOG_ROOT"
ip link show "$LOCOMOTION_NETWORK_INTERFACE"
ping -c 1 "$LOCOMOTION_ROBOT_IP"

PYTHONNOUSERSITE=1 "$LOCOMOTION_RUNTIME_PYTHON" -c \
  "import numpy, onnxruntime, cyclonedds, unitree_sdk2py; print('runtime imports: OK')"
```

任何一步失败都不要启动实机控制器。

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
set -a
source "$FSM_ROOT/config/nuc.env"
set +a
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

配置中的部署字符串支持 `${VAR}` 和 `${VAR:-default}`。模型和阻抗文件等相对路径
始终相对于 ROS 安装包 share 目录解析；日志和 socket 可使用绝对路径，也可使用
相对 share 目录的路径。正式多 NUC 部署推荐通过 `config/nuc.env` 给出每台机器的
可写日志目录和运行环境。

## 5. 更新代码后

```bash
cd "$FSM_ROOT"
git pull --ff-only
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select locomotion_controller
source "$FSM_ROOT/install/setup.bash"
```

不要复制另一台 NUC 的 `config/nuc.env`；新机器从 example 建立并逐项核对。模型、
ROS topic 和状态机行为在各 NUC 上相同，只有部署环境变量不同。
