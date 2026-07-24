# 控制运行时依赖

`locomotion_controller_node` 是 ROS 2 系统 Python 进程。ONNX 推理和 Unitree DDS
位于配置指定的 Conda Python 子进程，因此目标系统需要提前准备以下环境。

## 创建环境

```bash
conda create -y -n locomotion_controller python=3.12
conda activate locomotion_controller

python -m pip install --upgrade pip setuptools wheel
python -m pip install -r python/requirements.txt
python -m pip install ./unitree_sdk2_python
python -m pip check
```

`python/requirements.txt` 固定 NumPy、SciPy、PyYAML、ONNX Runtime 和
CycloneDDS Python binding 版本。`unitree_sdk2_python/` 是随项目保存的 Unitree
SDK2 Python 源码。

## 安装 CycloneDDS C 运行库

仓库中的 `cyclonedds/install/` 面向 Ubuntu 24.04 x86_64：

```bash
cp -a cyclonedds/install/. "${CONDA_PREFIX}/"

mkdir -p "${CONDA_PREFIX}/etc/conda/activate.d"
cat >"${CONDA_PREFIX}/etc/conda/activate.d/locomotion_controller.sh" <<'EOF'
export CYCLONEDDS_HOME="${CONDA_PREFIX}"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
EOF
```

控制节点启动子进程时也会根据 YAML 中的 `cyclonedds_home` 显式设置
`CYCLONEDDS_HOME` 和 `LD_LIBRARY_PATH`。

## 验证

```bash
conda activate locomotion_controller

python -c \
  "import cyclonedds, numpy, onnxruntime, scipy, unitree_sdk2py, yaml; print('runtime OK')"

test -f "${CONDA_PREFIX}/lib/libddsc.so.0.10.5"
ldd "${CONDA_PREFIX}/lib/libddsc.so.0.10.5"
```

配置文件中的 `runtime.python_executable` 和 `runtime.cyclonedds_home` 必须指向
同一个已验证环境。程序不会联网安装依赖，也不会自动创建虚拟环境。
