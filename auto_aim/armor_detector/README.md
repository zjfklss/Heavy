# 👁️ Armor Detector | 装甲板视觉识别节点

![ROS2](https://img.shields.io/badge/ROS2-Foxy%2FHumble-blue.svg)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)
![OpenVINO](https://img.shields.io/badge/Inference-OpenVINO-blueviolet)

> 本功能包是军临战队 (JunLin) 自瞄系统的**视觉前端**。
> 负责从相机读取图像，利用 `YOLOv5` 或者 `传统视觉匹配` 找出装甲板，通过如果是YOLOv5则还要经过**传统视觉**精修角点，最后使用 `PnP` 算法解算出装甲板在相机坐标系下的 3D 位姿。

---

## ✨ 核心特性

- **⚡ 极致推理**：采用 Intel OpenVINO 框架对 YOLOv5 模型进行加速，保证极高的检测帧率。
- **🎯 混合定位 (AI + CV)**：单纯的深度学习找角点容易抖动。本包在 YOLO 框出大区域 (ROI) 后，利用传统视觉（颜色差分、轮廓提取、灯条匹配成本函数）进行二次精修，大幅提高 PnP 解算精度。
- **🚀 零拷贝支持**：采用 `std::make_unique` 构建 ROS2 消息并配合特定的 QoS，当上下游节点都在同一进程时，可实现真正的进程内零拷贝通信。
- **🐛 完善的 Debug 机制**：内置丰富的可视化话题（二值图、灯条图、延迟显示、RViz 3D Marker），并且可通过 ROS 参数**热重载**动态开关，不影响比赛时的绝对性能。

---

## 🧠 算法流水线 (Pipeline)

当一帧图像进入回调函数后，会经历以下 4 个核心步骤：

1. **YOLO 粗检测 (`yolodetect`)**
   - 将图片缩放并输入 OpenVINO 推理引擎。
   - 输出候选框 (Bounding Box)、置信度、装甲板颜色、数字 ID 以及**粗略的4个角点**。
2. **ROI 传统视觉精修 (`refineArmors`)**
   - 根据 YOLO 框，向外扩张一定比例截取 ROI 区域。
   - 在 ROI 内进行颜色阈值与亮度二值化，寻找所有可能的灯条 (`yoloFindLights`)。
   - 计算灯条对的 **成本代价 (Armor Cost)**（考虑平行度、长度差、Y轴高度差等），选出最优灯条对，覆盖掉原有的粗略角点。
3. **PnP 姿态解算 (`solvePnP`)**
   - 订阅 `/camera_info` 获取相机内参矩阵与畸变系数。
   - 使用 `cv::solvePnP` 将 2D 像素角点转换为 3D 相机坐标系下的位姿（平移向量与旋转向量/四元数）。
4. **数据发布**
   - 将包含 3D 姿态的 `jlcv_interfaces::msg::Armors` 消息发布给后端的 EKF 预测节点。

---

## 🔄 节点输入输出 (I/O)

### 📥 订阅话题 (Subscriptions)
| 话题名称 | 消息类型 | 说明 |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` | 原始相机图像（使用 KeepLast(1) 极致 QoS 避免积压） |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参及畸变系数，用于 PnP 解算 |
| `/game_status` | `jlcv_interfaces/msg/GameStatus` | 裁判系统/主控发来的比赛状态（用于同步敌方颜色、切换相机等） |

### 📤 发布话题 (Publishers)
| 话题名称 | 消息类型 | 说明 |
| --- | --- | --- |
| `/detector/armors` | `jlcv_interfaces/msg/Armors` | 最终输出的装甲板 3D 位姿及属性 |
| `/detector/marker` | `visualization_msgs/msg/MarkerArray`| RViz 可视化标记（彩色 3D 框） |

*注：开启 `debug` 参数后，还会额外发布 `/detector/debug_lights`、`/detector/final_img` 等调试图像话题。*

---

## ⚙️ 参数与配置 (配置陷阱 ⚠️ 必看)

本节点的配置分为**两部分**，新人极易混淆，请仔细阅读：

### 1. ROS 2 节点参数 (由 Launch 文件或动态调参控制)
这些参数在代码中通过 `this->declare_parameter()` 声明。
- `debug` (bool): 开启后会发布调试图像，消耗 CPU，赛场上务必设为 `false`。
- `flip` (bool): 是否将图像旋转 180 度。
- `subscribe_compressed` (bool): 是否订阅压缩图像。

### 2. 算法核心配置文件 (`config/hero_yolo.yaml`)
存放模型路径、推理设备、灯条长宽比阈值等算法参数。
> 💡 **避坑指南**：
> `hero_yolo.yaml` 是本包**内部通过 `yaml-cpp` 自行读取**的文件，**它不是 ROS 2 标准的参数文件！** 
> ❌ **千万不要**在 launch 文件的 `parameters=[...]` 中传入这个 yaml，否则会报 `Cannot have a value before ros__parameters` 的严重报错。
> ✅ ROS 参数的修改请去 `vision_bringup/config/node_params.yaml` 中进行。

---

## 🚀 快速运行与单测

如果你只想单独启动视觉节点测试模型识别效果（例如接上摄像头调试）：

```bash
# 1. 编译并 source
colcon build --packages-select armor_detector
source install/setup.bash

# 2. 启动单测 Launch 文件
ros2 launch armor_detector armor_detector.launch.py
