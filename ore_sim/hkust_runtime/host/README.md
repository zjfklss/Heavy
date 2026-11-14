# 装配 Host

半自动装配流程的 Host 端 ROS 2 节点。工作区搭建、配置和完整启动命令见[运行时说明](../README.md)。

## 节点

| 节点 | 职责 | 主要输出 |
| --- | --- | --- |
| `perception_node` | YOLO 检测、LiteHRNet 关键点、PnP、TF 转换 | `/host/perception/exchange_station_pose` |
| `planning_node` | II 型接近、III 型约束路径、恢复规划 | 规划结果话题 |
| `task_node` | 人机状态机、规划请求、轨迹播放 | `/host/arm/host_output` |

本包配合公开的 MuJoCo 后端运行。数值感知、运动学、碰撞检查和规划由工作区包 `arm_exchange_core` 提供。

## 数据流

感知节点读取当前第二相机的 `sensor_msgs/Image` 和 `CameraInfo`，输出机械臂基座系下的装配站位姿。任务节点把该位姿、操作命令和当前关节状态组合起来，向规划节点请求接近、约束装配或恢复轨迹，再把选中的轨迹发给模拟下位机。

插入仍由操作手完成。操作手确认插入后，任务节点用当前 FK 重建五自由度装配基准，再进入后续约束自动段。

## 感知

Host 使用本机 `system_config.yaml` 中配置的 YOLO 和 LiteHRNet OpenVINO 模型。两者有独立的 OpenVINO 设备字段。节点可显示 OpenCV 叠加窗口，包含 YOLO 检测框、选中关键点、可见性分数和感知状态。

相机传输使用标准 `sensor_msgs/Image`，不需要 `shm-tools`、`shm_msgs` 或共享内存桥。位姿估计使用 `CameraInfo` 中的相机标定和配置的 12 点物体 schema；RANSAC/SQPnP 选初值，再用 Levenberg-Marquardt 精修。

## 规划与任务编排

规划节点持有机器人模型、碰撞环境和 II/III 型规划器。任务节点持有流程状态，是把操作事件转成规划或执行请求的唯一组件。这样几何规划器不依赖确认、手动插入和恢复语义。

任务节点还发布默认关闭的 `/host/arm/feedforward_wrench` 研究接口；力投影、仿真侧力矩映射和验证边界见[仿真说明](../sim/arm_exchange_sim/README.md)。

公开 launch 文件按选项启动所需 Host 节点。启动参数见[运行时说明](../README.md)，操作按键和 ROS 话题检查见[仿真说明](../sim/arm_exchange_sim/README.md)。
