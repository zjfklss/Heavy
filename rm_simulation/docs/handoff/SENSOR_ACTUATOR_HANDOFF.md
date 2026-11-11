# 传感器、执行器与闭环调度交接

**实时更新（2026-11-07）：**UI 已改为墙钟驱动、持续运行、即时显示 BGR 和世界状态，去掉播放/进度条。相机和协议各自使用后台线程；配置时长不再结束实时会话。当前实现以 [REALTIME.md](../REALTIME.md) 为准。

交接日期：2026-11-06。项目位置：`/home/jlcv/rm_simulation`。

**协议适配更新（2026-11-07）：**新增 `modules/adapters/`，按算法划分 generic 与 sp_vision 协议配置。
UI 的“调度 → 算法”可选择外部进程及协议，仿真主动连接先启动的算法 TCP 监听端。
适配代码只负责协议和符号转换，不链接自瞄本体；提供算法端可独立构建的 C++ SDK。
完整测试 **15/15**、核心 **3/3** 通过，算法端 SDK 已独立构建。
**真实自瞄通信补充（2026-11-07）：**`/home/jlcv/sp_vision_jlcv` 已增加 `io/simulation/` 和
`jlcv --simulation=true` 模式。当前用 `scripts/start_sp_vision.sh --simulation` 自动启动自瞄与对应兵种仿真；
图像/反馈进入独立自瞄的原有流程，指令和 telemetry 回到仿真。真实进程与 Qt 已双向验证，
18 帧图像、60 条反馈产生 18 条控制指令。自瞄编译位于 `build/sp-vision/`，工作产物在项目外。
用法、限制和完整协议见 [ADAPTERS.md](../ADAPTERS.md)。

**目录整理更新（2026-11-06）：**源码现按 `modules/`、`apps/`、`tests/`、`configs/` 分包；
编译产物按用户最终约定集中于项目内 `/home/jlcv/rm_simulation/build/`，工作产物移到 `/home/jlcv/rm_simulation-artifacts/`。
旧生成目录整体保存在后者的 `archive/layout-before-2026-11-06/`，旧 CMake 缓存仅作历史记录。
当前使用和重新构建方式见 [PROJECT_LAYOUT.md](../PROJECT_LAYOUT.md)。

**相机后续更新（2026-11-06）：**用户已将本轮范围收敛为“先解决相机，暂不接入自瞄”。
默认相机现为 BGR8 图像，提供 PNG 序列、采样/就绪时间、只读像素接口和 Qt 相机画面；
原几何特征模式保留。新增代码入口为 `modules/camera/include/rm_sim/image.hpp`、`modules/camera/src/image.cpp`、
`modules/recording/src/image_io.cpp`，详细用法与成像边界见 [CAMERA.md](../CAMERA.md)。
当时完整构建为 **14/14**，核心构建 **3/3** 通过；核心仍无需第三方库，CLI 新增 libpng。
相机阶段当时未修改 `/home/jlcv/sp_vision_jlcv`；之后的实际通信接入见本文最上方更新。
下文原交接中的“几何观测”说明适用于兼容特征模式；涉及相机的新状态以此更新和 CAMERA.md 为准。


**本轮任务已完成：传感器几何观测、云台执行器、闭环调度，以及对应 Qt 参数编辑和配置文件保存/载入均已交付。** 本文更新为当前实现交接，替代首次交接时的待实现方案。详细参数和协议说明见 [CLOSED_LOOP.md](../CLOSED_LOOP.md)，启动说明见 [README.md](../../README.md)。

## 1 交付范围与代码入口

闭环已贯通：世界快照 → 相机几何观测 → 算法 → 云台实际运动 → 实际出膛 → 世界弹道和命中。算法输入与评估真值分开，更换算法不需要修改世界模块。

| 模块 / 构建 target | 入口文件 | 当前职责 |
| --- | --- | --- |
| 世界 `rm_sim_world` | [world.hpp](../../modules/world/include/rm_sim/world.hpp)、[world.cpp](../../modules/world/src/world.cpp) | 原有目标运动、受击面、弹道、命中与机关状态机 |
| 公共协议 | [protocol.hpp](../../modules/protocol/include/rm_sim/protocol.hpp) | 位姿、旋转、整数 tick、延迟量化、周期时钟 |
| 传感器 `rm_sim_sensors` | [sensors.hpp](../../modules/camera/include/rm_sim/sensors.hpp)、[sensors.cpp](../../modules/camera/src/sensors.cpp) | 针孔特征投影、可见性、噪声、观测与真值标签 |
| 执行器 `rm_sim_actuators` | [actuators.hpp](../../modules/actuator/include/rm_sim/actuators.hpp)、[actuators.cpp](../../modules/actuator/src/actuators.cpp) | yaw/pitch 响应、相机/枪口实际位姿、反馈、实际出膛 |
| 调度 `rm_sim_runner` | [runner.hpp](../../modules/runner/include/rm_sim/runner.hpp)、[runner.cpp](../../modules/runner/src/runner.cpp) | 时钟、延迟队列、算法接口、运行日志、示例 PixelServo |
| 闭环 CLI `rm_sim_loop` | [loop_main.cpp](../../apps/cli/loop_main.cpp) | 配置校验、算法选择、运行和 JSON 日志 |
| 共享场景 `rm_sim_scenario` | [scenario.hpp](../../modules/scenario/include/rm_sim/scenario.hpp)、[scenario.cpp](../../modules/scenario/src/scenario.cpp) | 从原 CLI 提取的目标解析、时间和世界帧序列化 |
| Qt 配置 | [configuration_editor.hpp](../../apps/viewer/configuration/configuration_editor.hpp)、[json_fields.hpp](../../apps/viewer/configuration/json_fields.hpp) | 分页表单、单位转换、保留未编辑字段 |
| Qt 入口 `rm_sim_qt` | [qt_viewer.cpp](../../apps/viewer/qt_viewer.cpp)、[scenario_editor.hpp](../../apps/viewer/configuration/scenario_editor.hpp) | 文件载入/保存、后端校验与运行、目标表单、轨迹回放 |

世界、传感器、执行器、调度器仅依赖 C++17 标准库。JSON 层依赖 nlohmann-json，查看器依赖 Qt Widgets。世界不反向依赖传感器、执行器或算法。原 `rm_sim` 和旧场景 v1 保持可用。

项目已与 `/home/jlcv/sp_vision_jlcv` 分离；后续仿真代码继续放在本目录。交接时目录未建立 Git 仓库，无交接提交号。外观显示仍由 `apps/viewer/rendering/models.hpp` 和 `apps/viewer/rendering/mesh_renderer.hpp` 负责，不是相机成像或实体遮挡接口。

## 2 用户已确认的 UI 与配置流程

右侧**顶部直接显示“目标｜相机｜云台｜调度｜回放”五个标签**。这是本轮明确要求的布局，后续不要再把三个模块塞回同一个展开栏或模块下拉选择框。

| 顶部页面 | 已交付的参数分组 |
| --- | --- |
| 目标 | 类型、位置、运动、仿真时长、固定方向发弹 |
| 相机 | 采样与误差；分辨率和内参；特征模板 |
| 云台 | 运动响应；基座/关节/相机/枪口安装；发射；反馈 |
| 调度 | 时间与频率；算法；物理参数；预置命令和场景事件 |
| 回放 | 运行结果播放、观察视角与命中查看 |

前四页共用“打开配置、保存、另存为、应用并运行”。文件行为如下：

- 打开完整闭环 JSON 后填入表单；旧场景 v1 自动包装到 `scene`，算法设为 `none`。
- 保存写回当前文件，首次保存和另存为选择路径；输出可直接交给 `rm_sim_loop`。
- 保存和运行前调用 `rm_sim_loop --validate`。保存使用 `QSaveFile` 原子提交，无效输入不会覆盖原文件；打开失败不会部分替换现有表单。
- “应用并运行”使用临时配置，完成计算后回放，**不覆盖当前磁盘文件**；持久化到该文件需点保存。
- Qt 设置恢复最近成功打开、保存或应用的配置及文件路径；设置组织/应用为 `SPVision/RMSimulation`，键为 `closed_loop/configuration`、`closed_loop/path`。
- ° 转换成 rad，ms 转换成 s；安装旋转按三个列向量编辑，后端校验正交、单位长度和右手性。
- 多目标文件只在目标表单编辑第一个目标，其他目标保留。未显示字段和未修改的高精度值保留；预置场景发弹默认保留，取消勾选才由表单替换。
- 特征模板、预置命令和事件使用各自 JSON 编辑框。Qt 拒绝绝对值超过 `2^53−1` 的数值，避免大整数精度丢失；此类配置可走 CLI。

当前是离线计算后回放，不支持仿真运行中在线调参。没有己方云台三维外观；现已新增相机图像窗口，按就绪时间显示。既有显示风格继续保持白底、红蓝灯条和简洁控件；不添加无关大标题、赛季或资料来源文字。

## 3 启动、示例与验证入口

```bash
cd /home/jlcv/rm_simulation
./start_simulation.sh
# 直接载入闭环配置
./start_simulation.sh --config configs/scenarios/closed_loop/ideal.json
```

脚本增量构建 `build/release/` 下的 Release 程序且关闭测试；程序位于 `bin/`，已打开的旧进程需重启才能显示新版。

完整构建与测试：

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
mkdir -p ../rm_simulation-artifacts/runs

build/test/bin/rm_sim_loop --validate configs/scenarios/closed_loop/ideal.json
build/test/bin/rm_sim_loop configs/scenarios/closed_loop/ideal.json ../rm_simulation-artifacts/runs/loop.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/loop.json
build/test/bin/rm_sim_loop configs/scenarios/closed_loop/delayed.json ../rm_simulation-artifacts/runs/delayed-loop.json
```

[ideal.json](../../configs/scenarios/closed_loop/ideal.json) 为理想基准，[delayed.json](../../configs/scenarios/closed_loop/delayed.json) 加入运动、延迟、噪声和非理想云台。两者显式设为零重力，用于验证闭环连接，不代表已有弹道补偿算法。

不构建 CLI/Qt 的核心验证：

```bash
cmake --preset core
cmake --build --preset core
ctest --preset core
```

## 4 坐标和世界接口约定

世界 W 和固定基座 B 均为右手系：x 前、y 左、z 上；单位 m、s、rad。`T_A_B` 将 B 系坐标转换到 A 系；`Pose.rotation` 为三个列向量。相机 C 为 x 右、y 下、z 前；枪口 M 的 x 为发射方向。yaw 正方向绕 +z，pitch 正方向抬头，绕局部 **−y**。

实际位姿链：

```text
T_W_B × [Rz(yaw), yaw_offset] × [Ry(-pitch), pitch_offset] × T_P_C 或 T_P_M
```

默认基座平移 `(0,0,0.3)`，关节偏置为零，枪口安装为单位变换；相机安装旋转列为 `(0,-1,0)`、`(0,0,-1)`、`(1,0,0)`。无安装偏差时枪口方向为 `(cos(pitch)cos(yaw), cos(pitch)sin(yaw), sin(pitch))`。基座是己方执行器，不是场景中被击打的车辆。

世界接口仍遵循以下契约：

- `World::add_target()` 只在步进前调用；`snapshot()` 返回当前快照，不推进时间，也不提供历史查询。
- `time()` 为 tick × timestep；默认步长 1 ms，范围 `(0, 1 ms]`。无公共 reset 或线程安全承诺，新回合重建对象。
- `Surface.right × up = normal`；车辆相位零时第 0 板在中心 +x，0/2 板使用中心高度，1/3 板叠加高差。前哨站 position 是底座原点；机关 position 是转轴中心，正转绕正面法线遵循右手规则。
- `World::launch(Shot)` 在当前时刻发弹；ID 一轮内不得复用，出膛点高于地面，速度非零且有限，寿命为正。
- `hits()` 是累计接触数组，步进可能使引用失效。有效法向速度默认严格大于 12 m/s，同模块检测间隔默认 50 ms。无效计分接触也消耗弹丸；落地、超寿命和未命中不生成 Hit。

规则保持车辆装甲 2026、前哨站和能量机关 2027，依据见 [RULES.md](../reference/RULES.md)、[MODELS.md](../reference/MODELS.md)。世界采用解析运动、点弹丸与受击面求交，没有整车刚体、空气阻力或轮胎接触；装甲外形包络不等于真实裁判有效检测区。

## 5 传感器实际实现

当前默认传感器输出 **BGR8 合成图像**；设置 `sensor.output=features` 时使用下面的几何观测兼容模式。
图像模式不向算法附带真值角点，完整成像与文件约定见 [CAMERA.md](../CAMERA.md)。默认内参为合成针孔参数：1280×1024、`fx=fy=900`、`cx=640`、`cy=512`，版本 `synthetic-pinhole-v1`；它们可在 UI/JSON 修改，没有声称来自某台实机标定。

`Sensor::capture(snapshot, camera_pose, tick)` 使用同刻实际相机位姿，过滤背面、相机后方和视野外目标；要求整组特征可见。支持采样率、延迟、独立高斯像素噪声、按面漏检、每帧至多一个假装甲观测，以及独立随机种子。可选遮挡只考虑受击面，没有车身和支架遮挡。

默认矩形特征是四个外形角点 `synthetic-armor-envelope-v1`；圆靶是中心和四个圆周点 `synthetic-disk-cardinals-v1`，仅观测亮起的靶面。真实 PnP 接入应配置 `appearances`，局部点单位 m，x/y/z 分别沿受击面的 right/up/normal，并与算法物点定义一致。分类来自模板，尚无分类混淆模型。

`SensorPacket` 包含序号、采样/就绪 tick 与秒、传感器/标定标识、模型与测量类别、像素和协方差。它不包含真实目标 ID、装甲 ID、世界速度。真实相机姿态、理想投影、目标与装甲编号、可见性原因放在独立 `TruthAnnotation`，仅供评估。延迟消息保留原始采样内容，不在交付时重取真值。

## 6 云台与发射实际实现

云台为**固定基座的 yaw/pitch 两自由度角度位置响应模型**。`ideal=true` 时到期命令立即改变实际角度，跳变角速度记 0；`ideal=false` 时按角度误差求期望速度，结合制动速度、速度/加速度上限和梯形积分更新。默认最大速度 6 rad/s、最大加速度 60 rad/s²、响应时间 0.04 s。它不是扭矩、电流、惯量或实测电机模型。

yaw 默认连续回绕并使用最短角误差，关闭回绕才使用 yaw 限位；pitch 默认 ±π/3。无新命令继续跟踪最后目标角；机械硬限位处直接停止，该瞬间不受加速度上限约束。

`ControlRequest` 含 yaw、pitch、fire、request_id。每个 `fire=true` 是一发，要求非空 request ID；同回合重复请求不重复发弹，重复命令序号被忽略。独立请求按准备延迟和最小发射间隔排队，没有持续电平连发或取消已排队请求接口。

实际出膛使用出膛 tick 的实际枪口姿态；弹速每发采样一次，可叠加关节转动引起的偏置枪口线速度。默认弹速 23 m/s、最小间隔 0.1 s、寿命 3 s。`LaunchEvent` 关联命令、请求与 `Shot`，调度器再调用世界发弹。`next_launch_tick` 是新请求可预约的最早槽位，不一定是队首待发时间。

反馈默认 200 Hz，采样、延迟和角度噪声独立；角速度反馈当前为理想值。弹速和编码器分别使用随机流。相机投影始终使用实际姿态，不能改为有噪声的反馈或尚未生效的目标角。

## 7 调度、算法接入与日志

每个整数 tick 按以下顺序执行：

1. 到期场景事件和已排队命令。
2. 预置场景发弹、执行器实际出膛。
3. 读取同刻世界与相机实际姿态进行观测采样，独立采样反馈。
4. 交付就绪消息，运行到期算法，将新命令入队。
5. 记录到期真值帧，世界和执行器各推进一步。

算法新命令至少下一 tick 生效，即 `max(1, ceil(command_latency/dt))`；预置命令可在 tick 0 生效，其 JSON `time` 已是生效时间，不再叠加命令延迟。命令按 `(due_tick, sequence)` 排序；同 tick 场景输入保持原数组顺序。延迟向上量化，60/144 Hz 等周期按 `ceil(n/(frequency*dt))` 计算，避免累计漂移。

运行区间 `[0,duration)`，末尾仅补终点真值帧，不执行终点采样或命令、不强制排空队列。摘要报告未消费观测、反馈、未生效命令和待发请求。每个 Runner 只运行一次，最大 600 s/1000 万 tick，日志驻留内存。

接入新算法时继承 `Algorithm`，实现 `version()` 和 `update(const AlgorithmInput&)`，返回 `vector<ControlRequest>`，然后传入 `Runner::run()`。输入只有当前时间、步长、就绪观测和反馈；标定与特征模型通过适配器初始化传入。不要给适配器开放世界快照、真实 ID 或未来状态。

内置 `PixelServo` 只是最近图像特征组的比例角度控制示例，使用最近编码器反馈并拒绝过期数据；没有 PnP、跟踪、运动预测、弹道或延迟补偿，假设标准安装。真实自瞄工程尚未接入。

闭环 JSON 顶层使用 `closed_loop_version: 1`，包括 `scene`（原场景 v1）、`sensor`、`actuator`、`runner`、`algorithm`、`commands`。未知字段严格拒绝，`algorithm.kind` 为 `pixel_servo`、`none` 或新增的 `external`；外部协议配置见 ADAPTERS.md。原 `rm_sim` 不接受闭环包装，使用 `rm_sim_loop`。场景弹丸 ID 禁用保留前缀 `actuator-shot-`。

日志保留 Qt 使用的 `frames/hits`，新增原始及补全配置、版本、时间语义、`algorithm_channels`、命令、实际出膛、`evaluation` 和摘要。整个日志包含评估真值，不能直接作为算法输入。评估命中率以实际 launches 为分母，并处理终点仍在飞的弹丸。同程序和相同种子可复现，不承诺跨标准库逐位一致。

## 8 已完成验证

2026-11-06 本轮实现验证记录：

- 完整 旧完整构建测试曾通过 **10/10**：`rm_sim_loop`、`rm_sim_world`、`rm_sim_scenarios`、`rm_sim_loop_protocol`、`rm_sim_loop_trace`、`rm_sim_loop_qt`、`rm_sim_qt_trace`、`rm_sim_qt_ui`、`rm_sim_qt_loop_config`、`rm_sim_qt_config`。
- 最后将模块选择改成顶部五个标签后，重新执行相关 Qt 测试及轨迹 fixture，**6/6 通过**。当次记录见 `/home/jlcv/rm_simulation-artifacts/archive/layout-before-2026-11-06/build-test/Testing/Temporary/LastTest.log`，包含配置保存/载入、保留字段、无效输入保护和运行参数传递。
- 无 CLI/Qt 的 旧核心构建 **2/2 通过**，包括闭环 4253 项检查和世界 2118 项检查；记录见 `/home/jlcv/rm_simulation-artifacts/archive/layout-before-2026-11-06/build-core/Testing/Temporary/LastTest.log`。
- 当时界面构建到原 `build/`；旧构建和顶部标签预览现一并归档于 `/home/jlcv/rm_simulation-artifacts/archive/layout-before-2026-11-06/`。Qt 验证使用 offscreen，不代表真实硬件或视觉真实性验证。
- 理想示例最近轨迹产生 15 发、14 次有效命中；回合终点仍有在飞弹丸，不将其描述为全部命中或全部已结算。

本次收尾只更新交接和文档入口，上述为已完成实现的验证记录，没有因文档编辑重复执行功能测试。构建目录和截图是可再生成产物，不作为长期协议依据。

## 9 后续接手边界

本轮三个模块和 UI 配置保存任务已完成；以下是后续扩展条件，不是未完成的本轮实现：

- 实机接入前需提供相机内参、真实特征尺寸和编号、安装外参、云台响应、发射延迟/弹速分布及算法 I/O，再校准配置和编写适配器。
- 已有合成图像和简化实体遮挡，尚无真实感成像、镜头畸变、曝光积分/运动模糊、完整场景遮挡、多相机或分类混淆。
- 当前没有运动基座、扭矩/电流控制、真实设备通信或在线运行中调参。需要历史曝光时应新增带时戳缓存，不能倒退世界或用低频回放帧替代物理采样。
- 长时间实验需另行设计流式日志和生命周期；不要把一次性 Runner 当成无限时长服务。

后续修改保持世界与算法解耦、观测与真值分离、同刻实际姿态采样、单发请求去重和顶部模块标签布局，并继续验证旧场景、闭环配置及 Qt 保存/载入的兼容性。
