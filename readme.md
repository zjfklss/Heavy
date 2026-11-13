# Heavy 重装自动对矿

本分支只做 RoboMaster 2027 重装机器人自动对矿。

## 目录

- 技术路线：`docs/重装自动对矿技术路线.md`
- 仿真运行时：`ore_sim/hkust_runtime/`（港科 RM2026 ROS 2 + MuJoCo，已可在 Heavy 内构建，Host 尚未接入）
- 复用内容：`tools/`、`calibration/`、`io/camera.*`、`io/hikrobot/`、标定配置与圆点板

## 标定工具构建

```bash
cmake -S . -B build -DBUILD_CALIBRATION=ON
cmake --build build -j4
```

产物：`build/calibrate_camera`、`build/calibrate_handeye`、`build/calibrate_robotworld_handeye`。

## 仿真构建

仿真基于港科 RM2026 的 ROS 2 + MuJoCo 运行时，在 `ore_sim/hkust_runtime` 内单独用 colcon 构建。

仿真依赖已放在 `ore_sim/hkust_runtime/.venv`，约 953 MB，已被 `.gitignore` 忽略。

如果 venv 缺失或换机器，用可用环境覆盖：

```bash
cd ore_sim/hkust_runtime
VENV=/path/to/venv ./scripts/build.sh
```

`VENV` 必须能导入 `mujoco`、`hppfcl`、`scipy`、`openvino`、`glfw`、`readerwriterlock`、`rclpy`。

构建：

```bash
./scripts/build.sh
source scripts/setup_env.sh
```

脚本会加载 ROS 2 Lyrical，使用 venv 的 Python，产物在：

- `ore_sim/hkust_runtime/build`
- `ore_sim/hkust_runtime/install`

只启动 MuJoCo 场景：

```bash
ros2 launch arm_exchange_sim arm_exchange_sim.launch.py
```

带 Host、不带感知：

```bash
ros2 launch arm_exchange_host sim_host.launch.py
```

感知需要先把权重放到 `ore_sim/hkust_runtime/weights`，再：

```bash
ros2 launch arm_exchange_host sim_host.launch.py \
  enable_perception:=true \
  camera_view:=true
```

## 现状

- 五个仿真包已用 Heavy 内的脚本构建通过。
- 最小仿真已验证：20 个 body、5 个插件、两路 640×640 相机、`/joint_states` 可读。
- Host 与感知尚未启动验证；`weights` 仍未拷贝。
- 对矿业务代码尚未实现。
- 本分支暂不推送。
