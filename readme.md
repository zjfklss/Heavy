# 英雄视觉：快速上手、工程说明与相机标定

本分支（`feature/tongji-hero`）是不依赖 ROS 的英雄自瞄程序，目录分层与 `sp_jlcv_aimer` 一致：`src` 放入口，`io` 放硬件适配，`tasks` 放算法，`tools` 放公共工具，`configs` 和 `assets` 放配置及模型。上一代 ROS2 版本在 `feature/hero-ros2` 分支，两者差异见 [docs/hero.md](docs/hero.md) 第 7 节。

## 1. 快速上手

### 1.1 第一次装环境（Ubuntu 22.04）

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libopencv-dev libeigen3-dev \
  libfmt-dev libspdlog-dev libyaml-cpp-dev \
  nlohmann-json3-dev libusb-1.0-0-dev \
  qtbase5-dev libgtest-dev
```

另外两项需要手动安装：

| 依赖 | 用途 | 车上 `jlcv-pc` 的位置 |
| --- | --- | --- |
| OpenVINO Runtime | YOLOv5 推理 | `/opt/intel/openvino_2024.6.0` |
| 海康 MVS SDK | 相机采集 | `/opt/MVS`（CMake 只在这里找） |

**MVS 没装或不在 `/opt/MVS` 时，编译仍然成功**，只是 CMake 打印 `MVS not found`，程序打不开相机；视频回放、仿真、标定不受影响。

### 1.2 编译和自检

```bash
cd ~/Heavy            # 换成自己的克隆目录
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

`OpenVINO_DIR` 填包含 `OpenVINOConfig.cmake` 的目录，按本机安装位置改（例如 2026.4 装在 `/opt/openvino/2026.4.1/openvino/cmake`）。`ctest` 不需要相机、串口或仿真器，全部通过再上车。

可选开关：`-DHERO_WITH_SHOW_UI=OFF`（不要 Qt 调试窗）、`-DHERO_WITH_SIMULATION=OFF`（不要仿真入口）、`-DBUILD_TESTING=OFF`（不编测试）。

### 1.3 上车运行

程序不再要求在仓库根目录运行；不写配置时默认用 `configs/hero.yaml`。

```bash
./build/hero --check-config          # 只检查配置和模型文件
./build/hero --camera-test           # 只测相机采集帧率，不开串口、不加载模型
./build/hero --no-fire --show        # 实车跟踪 + 调试窗，但永不发射
./build/hero                         # 正式运行
```

建议顺序：`--check-config` → `--camera-test` → `--no-fire --show` 确认跟踪和角度方向正确 → 去掉 `--no-fire`。

| 参数 | 作用 |
| --- | --- |
| 第一个位置参数 | 配置文件；默认 `configs/hero.yaml`，`--simulation` 时默认 `configs/simulation/hero.yaml` |
| `--check-config` | 校验矩阵、模型路径、串口参数后退出，不开任何外设 |
| `--smoke-test` | 三张空图跑完整算法，不开外设 |
| `--camera-test [--frames=N]` | 只测相机采集 FPS |
| `--video=文件 [--yaw= --pitch=]` | 离线视频回放，固定云台姿态（弧度，抬头为正） |
| `--no-serial` | 实车相机 + 固定姿态，不打开串口 |
| `--no-fire` | 正常瞄准，但发射位始终为 0 |
| `--show` | Qt 调试窗：画面、装甲板世界坐标、yaw/pitch/发射曲线，见 [docs/SHOW_UI.md](docs/SHOW_UI.md) |
| `--simulation` | 接 rm_simulation 仿真，见 [docs/SIMULATION.md](docs/SIMULATION.md) |

运行前须知：

- 串口默认 `/dev/ttyUSB0` @ 921600（`configs/hero.yaml` 的 `serial`）。当前用户需在 `dialout` 组（`groups` 查看；不在就 `sudo usermod -aG dialout $USER` 后重新登录）。
- 敌方颜色由电控每帧下发；`enemy_color` 只在没有串口反馈时作为后备。
- 关闭占用同一相机的 MVS 客户端或其他程序。日志里 `5000 Mbit/s` 是 USB3，`480 Mbit/s` 是 USB2，165 FPS 需要 USB3。
- 反馈超时（`serial.timeout`）或电控要求重置时，程序只发“不控制”帧，并重建跟踪器。

### 1.4 文档导航

| 想做什么 | 看哪里 |
| --- | --- |
| 架构、配置项、与 ROS2 分支的差异 | [docs/hero.md](docs/hero.md) |
| 调试窗怎么看 | [docs/SHOW_UI.md](docs/SHOW_UI.md) |
| 仿真联调 | [docs/SIMULATION.md](docs/SIMULATION.md) |
| 选标定板 | 本文第 5 节 |
| 内参拍摄、解算、检查 | 本文第 6 节；[详细教程](docs/相机标定操作流程.md) |
| 手眼标定（相机安装外参） | 本文第 7 节；详细教程第 9 节 |

## 2. 目录与模块

```text
Heavy/
├── CMakeLists.txt                 根构建入口
├── src/
│   ├── hero.cpp                   英雄程序入口
│   ├── hero_show.hpp              把算法输出交给调试窗（只读）
│   └── simulation/                仿真启动器与仿真自瞄循环
├── io/
│   ├── CMakeLists.txt             相机模块
│   ├── camera.*                   相机接口
│   ├── hikrobot/                  海康驱动
│   ├── hero/                      英雄串口协议与配置校验
│   └── simulation/                仿真协议桥接；protocol/ 为随仓库的仿真协议 SDK
├── tasks/
│   ├── auto_aim/                  检测、解算、跟踪
│   └── auto_outpost/              前哨站 / 反陀螺射击决策
├── tools/
│   └── jlcv_show/                 Qt 调试窗（与 sp_jlcv_aimer 同一份代码）
├── configs/
│   ├── hero.yaml                  实车配置
│   ├── simulation/hero.yaml       仿真配置（实车配置去掉零偏和延迟补偿）
│   ├── calibration.yaml           小板 A：圆点 7×7、25 mm
│   ├── calibration_chessboard_11x8.yaml  大板 B：棋盘格 9×12 格、8×11 交叉点、35 mm
│   └── camera_info.yaml           历史参考内参
├── assets/                        模型
├── calibration/                   capture 采图、内参与两套手眼离线求解
├── tests/
│   ├── hero/                      协议、配置、冒烟、仿真回归
│   └── calibration/               合成图片的两种板内参与手眼回归
└── docs/
    ├── hero.md
    ├── SHOW_UI.md
    ├── SIMULATION.md
    ├── 相机标定操作流程.md
    └── calibration_boards/        小板 A 的 SVG 与 PDF
```

与参考仓库保持的是分层和模块职责，没有复制步兵、哨兵等英雄用不到的入口。`io/hero/` 对应参考仓库的 `io/jlcv/`；算法是英雄自己的版本，没有换成步兵算法。

`configs/camera_info.yaml` 带有 `source: remote_hero_untuned`，只是历史相机数据。新内参结果另存实验目录，不会自动覆盖 `configs/hero.yaml`。

## 3. 环境准备

完整依赖见第 1.1 节。离线标定只需要第 1.1 节 apt 列表里的公共依赖，不需要 OpenVINO、MVS、Qt、ROS 或串口。

## 4. 构建方式

### 4.1 默认构建

见第 1.2 节。默认 `BUILD_HERO=ON`、`BUILD_CALIBRATION=ON`、`HERO_WITH_SHOW_UI=ON`、`HERO_WITH_SIMULATION=ON`，测试由标准的 `BUILD_TESTING` 控制。

主要产物为 `build/hero`、`build/capture`、`build/calibrate_camera`、`build/calibrate_handeye` 与 `build/calibrate_robotworld_handeye`。

### 4.2 只构建离线标定工具

```bash
cmake -S . -B build/calibration \
  -DBUILD_HERO=OFF \
  -DBUILD_CALIBRATION=ON
cmake --build build/calibration -j4 --target calibrate_camera
./build/calibration/calibrate_camera --help
```

该模式不查找 OpenVINO 或 MVS。两套已有 handeye 工具仍保留，按需构建对应目标即可。它们的输入假设和限制见详细教程。

使用新构建目录，避免沿用旧工程的 CMake 缓存。内存不足时将 `-j4` 改为 `-j2` 或 `-j1`。

## 5. 两张标定板

| 项目     | 小板 A                                                                                                               | 大板 B                                                                      |
| ------ | ------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------- |
| 类型     | 白底黑圆点，对称圆点阵                                                                                                        | **棋盘格**                                                                   |
| 实物     | 7 × 7 个圆点                                                                                                          | **9 × 12 个方格**                                                            |
| 工具识别的点 | 49 个圆心                                                                                                             | **8 × 11 = 88 个中间交叉点**                                                    |
| 间距     | 圆心距 25 mm                                                                                                          | 方格边长 35 mm                                                                |
| 配置     | `configs/calibration.yaml`                                                                                         | `configs/calibration_chessboard_11x8.yaml`                                |
| 关键字段   | `pattern_type: circles`，`pattern_cols/rows: 7/7`，`center_distance_mm: 25`                                          | `pattern_type: chessboard`，`pattern_cols/rows: 11/8`，`square_size_mm: 35` |
| 打印文件   | [PDF](docs/calibration_boards/circles_7x7_25mm_A4.pdf) / [SVG](docs/calibration_boards/circles_7x7_25mm_A4.svg)，A4 | 已有实物板                                                                     |

棋盘格的 `pattern_cols/rows` 填中间交叉点数（11 × 8），不是方格数（12 × 9）。大板 B 与 ROS `camera_calibration --size 11x8 --square 0.035` 是同一块板。

小板 A 打印选择 **100% / 实际大小**，关闭“适合页面”，打印后用尺检查底部 100 mm 标尺和多组圆心距。大板 B 用尺量跨 10 格应为 350 mm，不是就改配置副本里的 `square_size_mm`。两块板都要平整贴在硬板上，避免翘曲和反光。

![小板 A：7×7 对称圆点阵](docs/calibration_boards/circles_7x7_25mm_A4.svg)

一次解算只用一种板，两种板的照片不能放在同一输入目录。

## 6. 内参标定：从拍照到结果

### 6.1 固定相机设置

确认镜头、焦距、对焦和分辨率，记录 ROI、裁剪、像素合并等设置。拍摄期间固定这些设置，关闭会改变焦距或对焦位置的自动功能。

保存未缩放、未去畸变的原始分辨率图片，不要使用远程桌面截图。所有图片必须具有相同宽高。

### 6.2 拍摄有差异的图片

将完整板分别放在画面中心、边缘和四角，改变距离及板面倾斜方向。每次停稳、图像清晰后再保存。每张都应看见全部圆点或全部 8 × 11 个交叉点，避免反光、运动模糊或遮挡。

先采 20～30 张作为起点，关键是位置与角度覆盖，不是连续拍很多相似照片。不同批次、不同板、不同分辨率分别建目录。

### 6.3 整理连续编号

以小板 A 为例：

```bash
mkdir -p datasets/intrinsic_7x7/images
mkdir -p datasets/intrinsic_7x7/results
cp configs/calibration.yaml datasets/intrinsic_7x7/board.yaml
```

将图片整理为：

```text
datasets/intrinsic_7x7/
├── board.yaml
├── images/
│   ├── 1.jpg
│   ├── 2.jpg
│   ├── 3.jpg
│   └── ...
└── results/
```

工具从 `1.jpg` 起逐张读取，遇到第一张缺失或无法读取的图片就停止。不能跳号，不能只改扩展名把 PNG 当作 JPEG。**内参工具不读取姿态 TXT**，无需串口反馈。

### 6.4 运行解算

```bash
./build/calibration/calibrate_camera datasets/intrinsic_7x7/images \
  --config-path=datasets/intrinsic_7x7/board.yaml \
  --camera-info=datasets/intrinsic_7x7/results/camera_info.yaml \
  --output=datasets/intrinsic_7x7/results/intrinsics_flat.yaml \
  --preview=none
```

使用 `--参数=值`。这里显式指定输出路径，防止默认覆盖仓库中的 `configs/camera_info.yaml`。程序不会创建输出父目录，因此必须提前 `mkdir -p`。

有桌面显示时，可将 `none` 改成 `auto` 快速预览，或 `manual` 逐张按键查看；SSH 没有图形显示时使用 `none`。

大板 B 把目录换成自己的、`--config-path` 换成 `configs/calibration_chessboard_11x8.yaml` 的副本即可，终端会打印 `标定板 棋盘格(内角点) 11x8 @ 35.0mm`。完整命令、图片整理脚本和参数说明见[详细教程](docs/相机标定操作流程.md)。

### 6.5 阅读与检查输出

终端逐张打印 `[success]` 或 `[failure]`，失败样本跳过。现有程序只要有有效样本就可能继续求解，**退出码为 0 不等于样本充分或结果可靠**。

输出包括：

- `camera_info.yaml`：图像宽高、3×3 内参矩阵、五项畸变系数；采用 ROS 风格格式，但不需要 ROS。
- `intrinsics_flat.yaml`：`camera_matrix` 与 `distort_coeffs` 两个扁平字段。
- 注释中的重投影误差：所有有效圆心或角点的二维欧氏误差算术平均，单位像素；不是 OpenCV 的 RMS 返回值。

```text
K = [ fx   0  cx ]
    [  0  fy  cy ]
    [  0   0   1 ]

D = [k1, k2, p1, p2, k3]
```

`fx/fy` 是以像素为单位的焦距，`cx/cy` 是主点。当前工具用 `CALIB_FIX_K3` 固定 `k3`，不代表镜头没有更高阶畸变。

检查宽高与照片一致、数值有限、`fx/fy` 为正、独立重拍时结果可重复，并用未参与拟合的照片检查去畸变表现。平均误差小只是其中一项指标。

## 7. handeye 与内参有什么关系

**内参**描述相机如何把三维点投到图像；**手眼标定**描述相机与另一个刚体坐标系之间的固定变换。

| 项目 | 相机内参 | 通用手眼标定 |
| --- | --- | --- |
| 典型输入 | 板图片、圆心或角点几何 | 图片、已知内参、与图片同步的云台姿态（`capture` 采集） |
| 典型输出 | `K`、`D`、图像尺寸 | 坐标系间的旋转 `R` 与平移 `t` |
| 是否需要姿态文件 | 本仓库内参工具不需要 | 现有手眼实现需要配对姿态 |
| 标定板数量 | 一种板、多种视角 | 不因叫 handeye 就必须换第二块板 |

两张板不分别绑定“内参”和“手眼”。通用手眼还需明确 eye-in-hand / eye-to-hand、变换方向、单位和时间同步，不能把任意 `R/t` 当作可互换结果。

### 7.1 英雄手眼标定速查

采集要开电控（每张图要配一组串口回传的云台姿态），求解不用：

```bash
# 1. 板竖直固定在云台前方，电控上电，手动转云台；s 保存 / a 自动 / q 退出
./build/capture datasets/handeye_1 --board=configs/calibration_chessboard_11x8.yaml
# 2. 求解（主路径 + 交叉对照）
./build/calibrate_robotworld_handeye datasets/handeye_1 --config-path=datasets/handeye_1/board.yaml \
  --output=datasets/handeye_1/result.yaml --preview=none
./build/calibrate_handeye datasets/handeye_1 --config-path=datasets/handeye_1/board.yaml \
  --output=datasets/handeye_1/result_handeye.yaml --preview=none
# 3. 把 result.yaml 的 R_camera2gimbal、t_camera2gimbal、camera_rpy 换进 configs/hero.yaml
```

看最后一行“链路重投影”：平均 < 1 px 可用。完整步骤、保存条件和排查见[详细教程](docs/相机标定操作流程.md)第 9 节。

## 8. 常见问题

| 现象 | 先检查什么 |
| --- | --- |
| 无有效内参样本 | 路径、`1.jpg`、图片是否可读、板规格是否一致 |
| 有很多图片却只读几张 | 编号断档或中间图片损坏 |
| 所有图片检测失败 | `pattern_type` 与实物是否一致；点数（棋盘格填 11×8 交叉点，不是 12×9 格）；黑白方向、反光和清晰度 |
| 远处或近处圆点检测失败 | 圆点像素大小是否落在 blob 面积范围内 |
| 无法连接图形显示 | 改用 `--preview=none` |
| 参数被当作 `true` | 使用 `--config-path=...` 等等号写法 |
| 输出不存在或未更新 | 输出父目录与写权限；写文件辅助函数未检查流错误 |
| 新内参与旧值不同 | 镜头、焦距、分辨率是否不同；旧值是否只是参考 |
| 小误差但参数不稳定 | 图片是否相似、覆盖不足、板是否弯曲 |

## 9. 历史记录

2026-11-05 整理时移除了旧 Heavy 的吊射、钉头、打符、哨兵等入口，并把英雄程序移到根目录；没有改相机设置、串口协议或算法。当时的记录：1000 帧相机测试采集约 165 FPS，完整视觉处理约 100～106 FPS，仍有跟踪重建等未解决问题。

2026-11 加入调试窗（`--show`）、仿真入口（`--simulation`）和棋盘格标定支持，同样没有改检测、跟踪、射击算法。离线回归不能替代实车验收。
