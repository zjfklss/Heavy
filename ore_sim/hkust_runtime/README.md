# 运行时工作区

本工作区把公开的感知、规划算法接到 ROS 2 Host 和 MuJoCo 仿真后端。图像、TF、关节状态使用标准 ROS 2 消息；任务命令和模拟下位机命令使用仓库自带的接口包。

当前版本只包含仿真后端，不含实机驱动和启动文件。以下命令默认在 `hkust_runtime` 根目录执行。

## 包结构

```text
hkust_runtime/
├── interfaces/    # Host 与仿真共用的 ROS 2 消息
├── core/          # 机器人模型、PnP、碰撞检查、运动规划
├── host/          # 感知、规划、任务编排节点
├── sim/
│   ├── mujoco_engine/       # 可复用的 MuJoCo 引擎与插件生命周期
│   └── arm_exchange_sim/    # 装配场景、插件、控制与资产
└── scripts/       # 构建、环境和测试脚本
```

详细说明：

- [数值核心与算法结构](core/README.md)
- [Host 节点与数据流](host/README.md)
- [装配 MuJoCo 应用](sim/arm_exchange_sim/README.md)
- [可复用 MuJoCo 引擎](sim/mujoco_engine/README.md)

## ROS 2 架构

```mermaid
flowchart LR
  input["operator_input（可选）"]:::optional
  panel["station_panel（可选调试）"]:::optional
  sim["mujoco_simulator<br/>物理、相机、TF、模拟下位机"]:::backend
  perception["arm_exchange_perception<br/>YOLO + LiteHRNet + PnP"]:::host
  task["arm_exchange_task<br/>半自动状态机"]:::host
  planning["arm_exchange_planning<br/>II 型 / III 型 / 恢复"]:::host

  input -->|"/operator/input_state"| sim
  panel -->|"/debug/scene/exchange_station/set_state"| sim
  sim -->|"Image + CameraInfo"| perception
  sim -->|"/tf 与 /tf_static"| perception
  sim -->|"/mcu/gimbal/control"| perception
  perception -->|"/host/perception/exchange_station_pose"| task
  sim -->|"/joint_states 与 /mcu/arm/state"| task
  sim -->|"模拟控制事件"| task
  task -->|"接近 / III 型 / 恢复请求"| planning
  planning -->|"规划结果"| task
  task -->|"/host/arm/host_output<br/>/host/arm/feedforward_wrench"| sim

  classDef backend fill:#eef6ff,stroke:#3973ac,color:#111;
  classDef host fill:#f3faef,stroke:#4f8a3f,color:#111;
  classDef optional fill:#fff8e8,stroke:#ad7a22,color:#111,stroke-dasharray:5 4;
```

`mujoco_simulator` 是一个 ROS 节点。机械臂控制器、操作逻辑、TF 发布、相机安装和装配站模型都是该节点内的应用插件，不是额外的 ROS 节点。`arm_exchange_perception` 仅在 `enable_perception:=true` 时启动；`operator_input` 仅在 `enable_operator_input:=true` 时启动。

`/host/arm/feedforward_wrench` 是预留的接触力前馈研究接口，默认关闭，不提供力传感器反馈或闭环力控。

### 后端边界

Host 节点只依赖 ROS 接口，不直接依赖 MuJoCo。仿真后端提供相机图像与内参、TF、关节与下位机反馈、操作事件和轨迹执行端点。公开版本只用 `mujoco_simulator` 实现这套接口；实机相机驱动、虚拟云台、编码器/MCU 传输和真实执行器接口未包含。

实机接入时，保留感知、规划、任务节点，替换仿真侧即可；但必须保持话题类型、坐标系、单位、时间戳和控制权限语义一致，并提供标定后的相机与并联机构变换。

## 本机说明

上游按 Ubuntu 22.04、ROS 2 Humble、Python 3.10 开发。本机是：

- Ubuntu 26.04
- ROS 2 Lyrical
- Python 3.14
- NVIDIA RTX 4060 Laptop，8 GB
- 30 GB 内存

当前拷贝把上游 `runtime/` 的内容拍平到了 `hkust_runtime/` 根目录，但 `scripts/` 仍按 `runtime/...` 查找路径，直接执行会找不到包。依赖也尚未装齐：`mujoco`、`hpp-fcl`、`scipy`、`openvino`、`glfw`、`readerwriterlock` 缺失，`mujoco==3.4.0` 和 `hpp-fcl==2.4.4` 在 Python 3.14 下没有现成轮子。需要先解决这组依赖。

## 本地配置

首次构建前生成本机配置：

```bash
cp core/arm_exchange_core/system_config.example.yaml \
  core/arm_exchange_core/system_config.yaml
```

模型权重需从 Hugging Face 下载，放到配置指定的 `weights/` 目录：

- `weights/yolo_v10.12_openvino_model`
- `weights/litehrnet30_v11.0_openvino_model`

也可修改本机 `system_config.yaml` 中的路径。`yolo_device` 和 `hrnet_device` 可分别设为 `GPU` 或 `CPU`；在本机上先用 `CPU` 验证流程，再切换显卡。

检查 OpenVINO 设备：

```bash
python3 - <<'PY'
from openvino import Core

core = Core()
for device in core.available_devices:
    print(device, core.get_property(device, "FULL_DEVICE_NAME"))
PY
```

## 构建

上游命令：

```bash
scripts/build.sh
source scripts/setup_env.sh
```

当前脚本仍按 `runtime/` 和 Humble 找路径，需先修正或手动执行 `colcon build`。构建包含五个包：

- `arm_exchange_interfaces`
- `arm_exchange_core`
- `mujoco_engine`
- `arm_exchange_sim`
- `arm_exchange_host`

## 启动

正常交互流程同时启动感知、操作输入和仿真。需要可读的 Linux 键盘事件设备。

查找键盘：

```bash
ls -l /dev/input/by-id /dev/input/by-path
sudo apt install evtest
sudo evtest
```

优先使用稳定的 `by-id` 或 `by-path` 路径。当前用户需要读权限；加入 `input` 组后必须重新登录：

```bash
sudo usermod -aG input "$USER"
export KEYBOARD_DEVICE=/path/to/keyboard-event-device
id -nG | tr ' ' '\n' | grep '^input$'
test -r "$KEYBOARD_DEVICE" && echo "键盘可读"
```

启动完整仿真：

```bash
ros2 launch arm_exchange_host sim_host.launch.py \
  enable_perception:=true \
  enable_operator_input:=true \
  keyboard_device:="$KEYBOARD_DEVICE"
```

无感知、无操作输入的核心冒烟测试：

```bash
ros2 launch arm_exchange_host sim_host.launch.py
```

只测感知：

```bash
ros2 launch arm_exchange_host sim_host.launch.py \
  enable_perception:=true \
  camera_view:=true
```

启动参数：

- `enable_perception`：默认 `false`，启动 YOLO、LiteHRNet 和 PnP。
- `camera_view`：默认 `true`，显示感知叠加窗口。
- `enable_operator_input`：默认 `false`，启动原始键盘输入节点。
- `keyboard_device`：启用操作输入时必填。
- `sim_config_path`：覆盖 MuJoCo 应用 YAML。

装配站状态面板单独启动：

```bash
source scripts/setup_env.sh
ros2 run arm_exchange_sim station_panel
```

## 测试

```bash
scripts/test.sh
```

脚本会加载工作区、关闭无关的 pytest 插件，运行五个包的测试并输出 colcon 结果。

接口检查和仿真冒烟命令见 [装配仿真说明](sim/arm_exchange_sim/README.md)。
