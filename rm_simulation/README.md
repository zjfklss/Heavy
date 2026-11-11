# RoboMaster 独立仿真

本项目实现目标真值、运动、弹丸和命中事件。`rm_sim_world` 只依赖 C++17 标准库，
不引用 `tasks/`、`io/`、OpenCV、Eigen、ROS、相机 SDK 或具体自瞄算法。
JSON 场景程序额外依赖 nlohmann-json，图像 PNG 导出依赖 libpng，原生可视化界面依赖 Qt 5/6 Widgets。
场景协议测试需要 Python 标准库。

项目目录：`/home/jlcv/rm_simulation`。源码、配置、测试和文档均独立于视觉工程；编译产物集中在项目内的 `build/`，工作产物位于项目外。

相机默认输出 BGR 图像，按实际相机位姿采样并延迟交付；UI 实时显示，CLI 可导出 PNG。
相机配置与边界见 [相机文档](docs/CAMERA.md)。新增独立进程协议适配，可在 UI 选择通用或 SP Vision 协议；
自瞄本体在外部单独运行，不链接进仿真。接入方法见 [协议适配文档](docs/ADAPTERS.md)。

本机 SP 自瞄已支持双向仿真通信，一条命令启动两端：
`scripts/start_sp_vision.sh --robot=infantry4 --simulation --show=true`。
自瞄接收图像/反馈并回传控制及状态；默认监听 `127.0.0.1:17001`，UI 可选择协议。
`--simulation` 使用 `configs/jlcv/simulation/` 下同兵种 YAML，并自动选择
`configs/scenarios/sensors/` 下同兵种 JSON，生成启用 SP 连接的运行副本；
支持 infantry3、infantry4、sentry，原配置文件不会被覆盖。
不传 `--simulation` 时使用 `configs/jlcv/` 下同兵种 YAML，以实机模式运行，不打开仿真。
执行器响应和无 GUI 调度继续可用，原几何特征模式显式保留。完成 test 构建后可运行：

```bash
mkdir -p ../rm_simulation-artifacts/runs
build/test/bin/rm_sim_loop configs/scenarios/camera/image.json ../rm_simulation-artifacts/runs/camera.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/camera.json
```

配置、算法适配接口及模型边界见 [闭环使用文档](docs/CLOSED_LOOP.md)。
[交接文档](docs/handoff/SENSOR_ACTUATOR_HANDOFF.md) 汇总当前交付、代码入口、UI 与配置约定、验证记录和后续接手边界。
`rm_sim_world` 仍保持独立；原 `rm_sim` 场景程序和 Qt 目标配置流程继续可用。

## 项目目录

| 目录 | 内容 |
| --- | --- |
| `modules/world` | 世界真值、目标运动、弹道、命中与机关状态机 |
| `modules/protocol` | 位姿、旋转、整数时钟与时间量化 |
| `modules/camera` | 相机采样、图像渲染、几何特征兼容模式及私有纹理 |
| `modules/actuator` | 云台响应、反馈、出膛 |
| `modules/runner` | 调度、延迟队列与算法接口 |
| `modules/scenario` | JSON 场景解析和世界帧序列化 |
| `modules/recording` | PNG 图像记录 |
| `modules/adapters` | 独立算法进程的协议、按算法配置、TCP 传输与算法端 SDK |
| `apps/cli`、`apps/viewer` | 命令行入口、Qt 参数配置与实时监测 |
| `tests`、`configs`、`docs` | 集成/协议测试、场景配置、使用和参考文档 |
| `cmake`、`scripts` | 构建规则和启动脚本 |

每个功能库自带 `include/`、`src/` 和 `CMakeLists.txt`；公共 C++ include 名称仍为 `rm_sim/*.hpp`。
编译目录统一为 `/home/jlcv/rm_simulation/build/{release,test,core}/`，程序在其中的 `bin/`。
截图、轨迹、测试输出、临时工作脚本和旧产物归档位于 `/home/jlcv/rm_simulation-artifacts/`。
完整约定见 [目录说明](docs/PROJECT_LAYOUT.md)。

## 一键启动与界面配置

在仿真项目根目录运行 `./start_simulation.sh`，也可从任意目录执行该脚本的绝对路径。
脚本自动增量构建 `build/release/` 下的 Release 程序并打开 Qt，无需先生成 JSON。
首次构建需要 CMake（预设命令需 3.21+）、C++17 编译器、Qt 5/6 Widgets、nlohmann-json 和 libpng 开发包。
`./start_simulation.sh --build-only` 只构建；`RM_SIM_JOBS=2 ./start_simulation.sh` 可限制构建并发。
已有轨迹可作为第一个参数传入。可通过 `RM_SIM_BUILD_DIR` 覆盖构建目录，`RM_SIM_ARTIFACT_DIR` 覆盖项目外的工作产物目录。

右侧顶部是“目标、相机、云台、调度、监测”。画面随仿真运行即时更新，持续到点击“停止”。
“应用并重启”使用修改后的参数重新开始；配置中的时长只安排预置事件，不再自动结束实时连接。
进度条、播放、单帧、复位和倍速控件已移除。完整行为见 [实时运行说明](docs/REALTIME.md)。

相机页可调整内参、分辨率、采样率、延迟、灯条外观和噪声。当前自瞄 infantry3、infantry4、sentry
预设在 [传感器配置目录](configs/scenarios/sensors/README.md)，可用“打开配置”载入。
云台默认在启动或重启时朝向可见装甲板（包含前哨站高度），之后接受控制指令；可在“云台 → 运动”关闭并手动设置初始角度。
云台页配置响应、安装、发射和反馈；调度页选择外部自瞄及 SP Vision 协议；监测页显示当前状态和最近命中。
UI 角度为 °、延迟为 ms，保存时转换为 rad、s。保存采用原子写入并校验参数。

“目标 → 车运动状态”可选“左右3m平移并转动靶”：车辆中心沿世界 Y 轴相对初始位置
左右各 3 m 往返（总行程 6 m），周期为 2π s（约 6.28 s），同时以 7 rad/s 匀速转动。
X 和高度保持固定，初始朝向仍由表单设置；选择后点击“应用并重启”生效。
切回“自定义”可分别配置 X/Y 平移和旋转。

相机图像生成和独立自瞄收发都在后台处理。处理落后时丢弃旧帧，世界时钟继续推进；
底部显示实际相机频率和丢帧数。UI 直接接收 BGR 像素，不等待整轮 JSON 或 PNG 归档。

```bash
./start_simulation.sh --config configs/scenarios/adapters/sp_vision.json
```

## 构建与运行

在仿真项目根目录执行；以下预设需要 CMake 3.21+：

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
mkdir -p ../rm_simulation-artifacts/runs ../rm_simulation-artifacts/previews

build/test/bin/rm_sim configs/scenarios/world_demo.json ../rm_simulation-artifacts/runs/world_demo.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/world_demo.json
```

Qt 正常启动显示实时画面，可旋转、缩放、切换视角和选择目标。CLI 导出的 JSON 仍可读取最后状态用于检查。
三维观察窗口使用 Qt Widgets + QImage/QPainter，支持透视投影、CPU 深度缓冲和面光照。
算法相机由独立渲染线程使用 EGL/OpenGL 硬件 GPU 离屏生成，直接使用标定分辨率输出 BGR8；
无需 Qt 图形上下文或窗口显示。相机“成像”页可选自动（优先 GPU）、强制 GPU 或 CPU，均按原生尺寸渲染。
GPU 原生帧缓冲使用最多 4 样本 MSAA，已移除先放大两倍宽高再缩小的相机路径。
界面只读核心输出，不计算目标运动或命中。
“目标”页可选择 2026 标识：英雄 1、工程 2、步兵 3/4、哨兵炮塔图案；前哨站自动显示塔形图案。
车辆、前哨站仅显示浮空装甲板，能量机关仅显示浮空圆靶；车体、轮组、塔身和支架均已移除，相机输出同步简化。
选择单个目标会自动调整近景距离。建模参考、模型近似与 CAD 获取状态见 [MODELS.md](docs/reference/MODELS.md)。
“导入场景”使用所选配置启动实时后端；正常运行不会生成完整回放轨迹。

`motion_catalog.json` 展示全部运动模式。修改场景后重新运行命令生成轨迹。
命令只处理虚拟场景，不连接真实相机、串口或机器人。

仅构建无第三方依赖的核心和测试：

```bash
cmake --preset core
cmake --build --preset core
ctest --preset core
```

没有显示设备时可验证界面交互并保存截图：

```bash
QT_QPA_PLATFORM=offscreen build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/world_demo.json \
  --self-test --screenshot ../rm_simulation-artifacts/previews/qt_preview.png
```

截图可追加 `--target car_stationary`、`--target outpost` 或 `--target small_rune` 查看模型近景。

默认启用 CLI 和 Qt。只需无界面场景程序时使用 `-DRM_SIM_BUILD_QT=OFF`。

## 规则版本和几何

按当前任务约定，**车辆装甲沿用 2026，前哨站与能量机关使用 2027**。
来源、图号、推定值和近似边界记录在 [RULES.md](docs/reference/RULES.md)。

| 目标 | 已实现几何与运动 |
| --- | --- |
| 四装甲板车 | 四板间隔 90°，两对半径可分别配置；相邻板高差可配；正面法线向外上倾 15°；支持小/大装甲外形参数 |
| 前哨站 | 三板间隔 120°，旋转直径 550 mm；三层中心高度 1.114/1.216/1.318 m（相对于局部底座原点）；正面法线向外下倾 15°；5 s 线性起转至 ±0.8π rad/s，默认 180 s 停转 |
| 小能量机关 | 五圆靶间隔 72°，靶心旋转半径 0.7 m，有效直径 0.3 m；恒速 ±π/3 rad/s |
| 大能量机关 | 相同五圆靶；正在激活时速度 ±[a sin(ωt)+(2.090−a)]；其他状态恒速 ±π/3 rad/s |

车辆半径、初始位置和运动强度是调试场景参数，不是比赛统一规格。
所有目标均可配置位置和初始相位，场地道具的中心固定，不跟随车辆运动模式。

## 坐标、时间与公共接口

入口为 `modules/world/include/rm_sim/world.hpp`，公共数据是 `Vec3`、`TargetConfig`、`Surface`、
`Shot`、`Hit`、`Snapshot` 等普通结构体。

- SI 单位：m、s、rad；右手世界坐标 x 前、y 左、z 上。
- 车辆/前哨站相位 0 的第 0 板位于中心的 +x 方向，其余按 +z 右手方向排列。
- 装甲面 `normal` 指向受击面外侧，`right × up = normal`。
- 能量机关法线由 `axis_yaw` 指定；相位 0 的第 0 圆靶在旋转中心正上方。
  正转按绕法线的右手方向定义，从正面观察为逆时针。两侧共轴可用相反法线与相反方向建立。
- `World::step(n)` 推进 n 个固定物理 tick，时间由整数 tick 推导，不读取系统时钟。
- 目标平移与转动采用解析函数；相同程序、配置和种子可复现结果，不承诺跨平台浮点逐位一致。
- `add_target()` 只允许在开始步进前调用；重新构造 `World` 即可开始新回合。
- `launch()` 使用当前位置、初速度和寿命创建弹丸；每次运行内 target/shot ID 各自唯一。
- `activate_rune(id)` 开始一次练习激活；`stop_outpost(id)` 可提前停止前哨站。
- `snapshot()` 提供真值，`hits()` 提供累计接触记录。传感器模拟从这里生成观测；
  不应把整个真值快照直接送给完整自瞄算法。

最小接入示例：

```cpp
#include "rm_sim/world.hpp"

rm_sim::World world;
rm_sim::TargetConfig car;
car.id = "car";
car.rotation.kind = rm_sim::MotionKind::velocity;
car.rotation.velocity = 7.0;
world.add_target(car);
world.step(20); // 默认 20 ms
auto truth = world.snapshot();
```

## 场景协议 v1

顶层必需字段为 `schema_version: 1`、`duration`、`targets`；可选 `name`、
`sample_period`、`world`、`shots`、`events`。未知字段会报错，避免参数拼写错误静默失效。

`world` 可配置 `timestep`（默认 0.001，最大 0.001 s）、`gravity`（默认 `[0,0,-9.81]`）、
`ground_z`、`minimum_normal_speed`（默认 12 m/s）和 `detection_interval`（默认 0.05 s）。
时间必须与物理 tick 对齐；采样周期不能为 0；运行长度上限 600 s。

每个目标必需 `id`、`kind`、`position`。`kind` 为 `car/outpost/small_rune/big_rune`。
三个 `translation` 分量分别控制世界 x/y/z，`rotation` 控制车辆 yaw。
目标 `position` 加各轴运动的 `initial` 得到初始中心；车辆的中心高度为第 0、2 板的高度。
`outpost.position` 则是尺寸图中前哨站自身底座的局部原点；示例另外加 0.3 m 场地底座。

| motion.kind | 参数与定义 |
| --- | --- |
| `stationary` | 固定在 `initial` |
| `velocity` | `initial + velocity*t` |
| `acceleration` | 增加 `acceleration*t²/2` |
| `jerk` | 增加 `jerk*t³/6`，表示恒定加加速度 |
| `sinusoidal` | 加速度为 `acceleration + amplitude*sin(frequency*t+phase)`；对其解析积分，保持指定的初始位置和速度 |
| `random` | 三个由 `seed` 决定频率与相位的正弦分量；相对初始位置的绝对偏移不超过 `amplitude`，速度、加速度、加加速度均连续 |

`frequency` 是角频率 rad/s，不是 Hz。随机模式忽略 `velocity/acceleration/jerk/phase`，
其初始导数由种子决定；三个轴、转动可分别选择模式和种子。随机模式是可复现的平滑不规则运动，
不是轮胎接地、导航或避障模型。多项式运动不会被自动限制在场地内。

几何/道具参数支持 `armor_width/armor_height/radius/second_radius/height_difference/tilt`、
`outpost_heights/ramp_seconds/stop_time/direction`、
`axis_yaw/rune_radius/disk_radius/rune_a/rune_omega/seed/initially_active`。
参数默认值见公共头文件。角度参数统一为 rad；`direction` 必须为 ±1。
前哨站 `outpost_heights` 默认 `[1.318,1.216,1.114]`，分别对应递增几何相位的高、中、低板。
改变转动方向不改变装配顺序；显式配置的高度数组仍按原顺序使用。

发弹字段为 `time/id/origin/velocity` 和可选 `lifetime`。
不提供自动朝目标发射的内置自瞄；场景中的初速度是外部输入。
事件为 `time/type/target_id`，支持 `activate_rune`、`stop_outpost`。
同一 tick 先执行事件、再发弹、再输出快照并推进；同 tick 输入保持数组顺序。

## 命中与能量机关练习

弹丸采用点质量、无阻力重力弹道。物理 tick 内对运动装甲面求交，并二分细化接触时刻；
旋转过快时进一步细分角度区间。测试覆盖单 tick 穿越整车的高速弹丸。
只接受从受击面正面进入的交点；最早接触消耗弹丸，地面也会吸收弹丸。
当前没有弹丸半径、机身/支架遮挡、反弹和空气阻力。

`Hit.effective` 与几何接触分开。有效命中要求接触相对法向速度大于 12 m/s，
同一装甲模块两次检测至少间隔 50 ms；所有参数对应 17 mm 弹丸练习。
圆靶按 15 mm 径向分带输出 1–10 环，边界归外侧环（中心为 10 环）。
记录中包含 shot/target/plate ID、接触时刻、位置、相对法向速度、环数和原因。

能量机关提供单次练习的状态机：

- 小符随机点亮一板，击中后从尚未命中的板中继续选择，五板完成后激活。
- 大符每组点亮两板；先命中一板算该组完成，随后保留 1 s 第二板窗口；五组完成后激活。
- 首次命中窗口 2.5 s；超时或有效速度击中非激活板时重置序列进度。
- 单次尝试 20 s 到期结束；可通过事件重新开始。
- 切换激活状态保持相位连续；大符每次开始使用配置的 a、ω，并重置函数时间原点。

**范围限制：**这不是完整比赛裁判系统。未实现比赛机会发放、增益/血量、阵地归属、双方共享轴、
大符累计激活后缩圈、最多 200 ms 灯效切换黑屏，以及实际电机对速度目标的滞后。
这些规则可在核心外的比赛调度层继续添加。前哨站停转当前立即保持停止相位，未模拟停机回位过程。

## 验证和数值边界

测试使用真实核心函数和独立的解析期望，覆盖运动导数、随机轨迹、有界性、几何基底、官方转速、
中心/擦边/脱靶/背面接触、移动目标、高速扫掠、最近接触、弹道重力、地面吸收、有效速度、
检测间隔、能量机关时序、事件、非法配置及重复运行。

求交以最多 1 ms 的时间括区与最多约 0.01 rad 的旋转括区近似连续碰撞；
切换状态的定时器量化到物理 tick。它不是任意高频运动下的精确刚体接触求解器。
支持通过减小 timestep 做步长收敛检查；实际云台、目标运动和命中率还需用实测数据校准。

首版矩形装甲按外形包络近似，**不能把模拟有效命中率直接视为裁判系统实测命中率**。
回放文件记录原场景、核心/协议版本、规则版本、模型限制、逐帧真值与命中事件。
