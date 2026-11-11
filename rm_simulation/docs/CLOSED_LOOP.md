# 传感器、执行器与闭环调度

已实现版本：2026-11-06，协议 `closed_loop_version: 1`。交付状态和接手约定见
[SENSOR_ACTUATOR_HANDOFF.md](handoff/SENSOR_ACTUATOR_HANDOFF.md)。本文说明当前可运行实现。

相机更新：默认输出已改为 BGR 图像，可保存 PNG 并在 Qt 中回放，详见 [CAMERA.md](CAMERA.md)。
新增 [独立进程协议适配](ADAPTERS.md)，支持 UI 切换通用/SP Vision 协议；适配层不链接算法本体。
下面的 PixelServo 示例继续显式使用 `sensor.output=features`。

## 构建与运行

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
mkdir -p ../rm_simulation-artifacts/runs

build/test/bin/rm_sim_loop configs/scenarios/closed_loop/ideal.json ../rm_simulation-artifacts/runs/loop.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/loop.json

build/test/bin/rm_sim_loop configs/scenarios/closed_loop/delayed.json ../rm_simulation-artifacts/runs/delayed-loop.json
```

`rm_sim_loop` 是独立的无 GUI 闭环入口。Qt 提供完整参数编辑、配置保存/载入和运行入口，
并读取世界帧与命中结果进行离线回放。目前支持相机图像回放，仍没有运行中在线修改参数的接口。
现有 `rm_sim`、场景 JSON v1 和一键启动脚本继续可用。

关闭 CLI 和 Qt 时，世界、传感器、执行器、调度器及其 C++ 测试仍可构建，全部只依赖 C++17 标准库。
JSON 配置与日志层依赖 nlohmann-json，CLI 的 PNG 导出另依赖 libpng。

## Qt 参数配置与文件保存

右侧顶部直接切换“目标、相机、云台、调度、回放”标签，前四页用于配置：

| 页面 | 可调整参数 |
| --- | --- |
| 目标 | 原目标、运动、时长与固定方向发弹表单 |
| 相机 | 采样、延迟、误差、随机种子；分辨率、fx/fy/cx/cy、标定版本；特征模板 JSON |
| 云台 | 理想/动态响应、初始角度、回绕和限位、速度/加速度；基座、关节和安装外参；发射；反馈 |
| 调度 | 步长、算法和日志频率、场景采样周期；算法增益/开火；重力和命中参数；预置命令与事件 |

角度输入标为 ° 的控件转换为 rad 保存，延迟输入标为 ms 的控件转换为 s 保存。
安装旋转展开后按三列输入 3×3 旋转矩阵，校验要求正交、单位长度、右手性。
相机默认光学安装旋转保持不变。动态参数在理想模式下禁用，yaw 限位在连续回绕模式下禁用。
禁用的动力学参数仍保存在文件中，以便切换模式；不使用算法时文件中只写 `kind: none`。

“打开配置”加载到表单；“保存”写回当前文件，“另存为”写到新路径。
保存采用 `QSaveFile` 原子提交；保存和应用前调用 `rm_sim_loop --validate`，不需要运行整个仿真即可
检查字段、频率与步长关系、关节限位、旋转矩阵、事件、命令和目标配置。
输入错误不会覆盖原文件，打开错误文件也不会部分替换现有表单。
启动时恢复最近成功载入、保存或运行的配置；“应用并运行”不覆盖当前磁盘配置，写文件需点击“保存”。

保存的是完整的闭环运行 JSON，可直接被 CLI 运行。旧场景 v1 文件打开后包装为 `scene`，
自动选择“不使用算法”。多目标场景只在原表单中编辑第一个目标，其他目标保留。
未在目标表单中显示的字段和未改动的高精度值保留；预置场景发弹默认保留，可取消勾选后由表单替换。
自定义特征、预置云台命令、场景事件可在各自 JSON 编辑框中修改。
Qt JSON 数值使用双精度，界面拒绝超过 `2^53−1` 的数值以避免大整数种子精度丢失；此类配置仍可直接使用 CLI。

```bash
build/test/bin/rm_sim_qt --config configs/scenarios/closed_loop/ideal.json
build/test/bin/rm_sim_loop --validate configs/scenarios/closed_loop/ideal.json
QT_QPA_PLATFORM=offscreen build/test/bin/rm_sim_qt --self-test-loop-config
```

## 模块和依赖

| Target / 接口 | 职责 |
| --- | --- |
| `rm_sim_world` / `world.hpp` | 目标、弹道和命中，保持原实现，不依赖新增模块 |
| `rm_sim_sensors` / `sensors.hpp` | 针孔特征投影、可见性、噪声及观测/真值分离 |
| `rm_sim_actuators` / `actuators.hpp` | 实际角度、关节响应、相机和枪口位姿、编码器反馈、出膛事件 |
| `rm_sim_runner` / `runner.hpp` | 整数 tick 调度、延迟队列、算法接口、记录 |
| `protocol.hpp` | `Pose`、旋转、时间量化、无累计漂移的周期采样 |
| `rm_sim_scenario` / `scenario.hpp` | 从原 CLI 提取的目标解析和世界帧序列化，供两个 CLI 复用 |
| `rm_sim_loop` | 配置解析、选择示例算法、运行、写出 JSON 日志 |

每回合构造新的 `Runner` 和算法对象，`run()` 只能调用一次。配置与目标在开始前固定，
当前无运行中增删目标、多线程或无限时长服务接口。最大回合 600 s、最多 10,000,000 tick；
日志全部保存在内存，应使用有限长度实验，较高采样率及多目标会增加内存占用。

## 传感器

默认 `output: image`，输出 BGR8 像素与 PNG 引用，图像采样和时序见 [CAMERA.md](CAMERA.md)。
本节以下特征投影、漏检与误检说明仅适用于显式 `output: features` 的兼容模式。

默认相机为 1280×1024，`fx=fy=900`，`cx=640, cy=512`；这些是可配置的合成参数，
不代表实际相机标定。传感器配置字段见 `SensorConfig`：

| 配置 | 默认值与含义 |
| --- | --- |
| `enabled` | true；false 时停止相机采样 |
| `id` / `calibration.id` | camera / synthetic-pinhole-v1；更改标定时应更新版本标识 |
| `frequency` / `latency` | 60 Hz / 0 s；延迟从采样时刻开始，向上量化 |
| `pixel_sigma` | 0 pixel；各坐标独立高斯噪声，输出对应协方差 |
| `dropout` | 0；每个可见受击面的独立漏检概率 |
| `false_positive_probability` | 0；每帧增加一个随机假装甲观测的概率 |
| `surface_occlusion` | false；开启后对已有受击面做双面射线遮挡检查 |
| `seed` | 1；独立于目标运动、弹速和编码器随机流 |
| `appearances` | 按目标 ID 配置特征模板，只用于仿真侧匹配 |

默认装甲投影四个外形角点，模型标识为 `synthetic-armor-envelope-v1`；
圆靶投影中心与四个圆周基准点，标识为 `synthetic-disk-cardinals-v1`。
两者都是几何观测代理，不能作为已标定的真实灯条或真实检测器输出。
圆靶只输出当前亮起的靶面，模拟可见灯效，不向算法输出机关内部进度。

需要与 PnP 物点一致时，显式配置特征模板，例如：

```json
"appearances": {
  "target": {
    "model_id": "my-calibrated-feature-layout-v1",
    "measured_class": "armor",
    "points": [[-0.06,0.025,0], [0.06,0.025,0], [0.06,-0.025,0], [-0.06,-0.025,0]]
  }
}
```

这里的数值只是配置语法示例；应替换为真实测量值。每个点的单位为 m，局部 x 沿
`Surface.right`，y 沿 `Surface.up`，z 沿 `Surface.normal`。同一目标的各块受击面使用同一模板。

投影先转换到相机光学系，再过滤背面、相机后方和视野外目标。首版要求一个观测的全部特征
位于图像内；不输出部分可见的特征组。加噪后超出边界同样丢弃。可选遮挡仅检查受击面，
没有车身、立柱或支架遮挡。假阳性使用合成装甲包络模型标识，不携带“这是误检”的算法可见标志。
当前分类标签由模板提供，没有分类混淆模型。

`SensorPacket` 只包含序号、采样和就绪 tick/时间、传感器和标定标识、特征像素、类别与协方差。
世界目标 ID、装甲编号、理想投影、实际相机位姿和漏检原因放在独立 `TruthAnnotation` 中。
算法不接收后者。延迟队列保存完整的原始采样内容，不在交付时重新读取世界。

## 执行器

当前控制模式为 yaw/pitch **角度位置指令**，单位 rad；基座固定。
默认 yaw 正转绕 +z，pitch 抬头绕局部 −y。无安装偏差时枪口方向为：

```text
(cos(pitch) cos(yaw), cos(pitch) sin(yaw), sin(pitch))
```

所有 `Pose` 都采用 `T_A_B` 的 B→A 变换。JSON `rotation` 是三个列向量。
位姿链为：

```text
T_W_B × [Rz(yaw), yaw_offset] × [Ry(-pitch), pitch_offset] × T_P_C 或 T_P_M
```

`base_pose` 默认平移 `[0,0,0.3]`；关节偏置默认为零。相机光学系 x 右、y 下、z 前，
默认 `camera_mount.rotation` 三列为 `[0,-1,0]`、`[0,0,-1]`、`[1,0,0]`。
枪口系 x 为出膛方向，`muzzle_mount` 默认为单位变换。安装平移和旋转都可配置。

| 配置 | 默认值与行为 |
| --- | --- |
| `ideal` | true；命令生效时立即到达目标角，理想跳变角速度记 0 |
| `wrap_yaw` | true；yaw 在 ±π 回绕并按最短角误差控制；此模式不使用 yaw 机械限位 |
| `yaw_min/yaw_max` | −π / π；关闭回绕时生效 |
| `pitch_min/pitch_max` | −π/3 / π/3；目标指令裁剪到范围内 |
| `initial_yaw/initial_pitch` | 0 / 0；初始 pitch、非回绕 yaw 必须在限位内 |
| `align_on_start` | UI 默认开启，实时 CLI 缺省开启，离线 CLI/C++ 缺省关闭；仅初始化时对准首个目标的可见装甲板，机关对准靶组中心，遵守安装外参和限位；关闭后使用手动初始角 |
| `max_speed/max_acceleration` | 6 rad/s / 60 rad/s²；`ideal=false` 时使用 |
| `response_time` | 0.04 s；误差到期望速度的响应时间，可设 0 |
| `command_latency` | 0 s；算法命令仍至少下一 tick 生效 |
| `fire_latency/fire_interval` | 0 s / 0.1 s；准备延迟与最小出膛间隔 |
| `muzzle_speed/muzzle_speed_sigma` | 23 / 0 m/s；每次实际出膛独立采样，极小或负样本裁剪到 1e-6 m/s |
| `shot_lifetime` | 3 s |
| `inherit_muzzle_velocity` | true；叠加关节转动造成的偏置枪口线速度 |
| `feedback_enabled` | true；false 时不产生反馈消息 |
| `feedback_frequency/feedback_latency` | 200 Hz / 0 s |
| `encoder_sigma` | 0 rad；仅角度测量加噪，角速度反馈当前为理想值 |
| `seed/feedback_seed` | 2 / 3；弹速和反馈采用独立随机流 |

非理想执行器将角度误差转换成期望速度，结合制动速度、速度和加速度上限，使用梯形积分更新角度。
它是可调的响应模型，没有扭矩、电流、齿隙或实测电机拟合。硬限位处按完全非弹性停止处理，
该瞬间速度归零不受加速度上限约束。无新指令时继续跟踪最近目标角；没有自动断流回零。
理想角度跳变不推导无限角速度，因此不会给弹丸增加跳变的继承速度。

每条 `fire=true` 指令表示**一发**，必须提供非空 `request_id`。同一回合重复 request ID
只允许首次请求入队；重复命令 sequence 被忽略。独立请求按到达顺序排队，按准备时间和最小间隔
分配出膛 tick。首版没有持续电平开火或取消已排队请求接口。
实际出膛使用当时实际枪口位姿与速度，而非发出请求时的目标角。执行器生成 `LaunchEvent`，
调度器调用 `World::launch()`；世界继续负责弹道和命中。`next_launch_tick` 表示新请求可预订的
最早出膛槽位，未必是队首待发弹的时间。

## 时间与算法接口

每 tick 的顺序：

1. 执行到期场景事件、此前排队的命令。
2. 执行预置场景发弹，再兑现执行器实际出膛。
3. 根据同刻世界和实际相机姿态采样；按独立频率采样反馈。
4. 交付已就绪消息，运行到期算法，将新命令排入队列。
5. 记录到期真值帧，世界与执行器各前进一步。

命令按 `(due_tick, sequence)` 排序，同 tick 的场景输入保持原数组顺序。
执行器、传感器及反馈延迟向上量化到 tick。算法命令延迟为 `max(1, ceil(command_latency/dt))`。
例如 0 或 0.5 ms 延迟在 1 ms 步长下均为 1 tick。

60/144 Hz 等频率使用 `ceil(n/(frequency*dt))` 计算每个采样期限，避免固定周期取整产生累计漂移。
频率必须大于 0 且不超过物理频率。算法未更新的间隔内，到达消息在收件箱累积；
`ready_tick` 是允许读取时刻，实际算法处理可能更晚。

运行区间为 `[0, duration)`；末尾额外保存 `duration` 真值帧，不再在末尾采样或执行命令。
结束时不强制清空延迟/开火队列；摘要分别报告未消费观测、反馈、未生效命令和待出膛请求数。
评估实际命中率应以 `launches` 为发弹分母，注意回合结束时可能仍有在飞弹丸。

算法继承 `Algorithm`，实现 `version()` 和 `update(const AlgorithmInput&)`。
`AlgorithmInput` 包含当前 tick/秒、步长，以及已就绪的 `SensorPacket` 和 `ActuatorFeedback`；
返回 `std::vector<ControlRequest>`。该接口不提供世界、真值、未来状态或真实目标 ID。
将自己的算法实例传给 `Runner::run()` 即可替换，不需修改世界模块。
算法所需标定及特征布局应通过适配器初始化配置提供。

内置 `PixelServo` 只根据距图像中心最近的特征组做比例角度修正，并在图像误差较小时请求开火。
它只作接口示例，没有 PnP、目标跟踪、延迟补偿、弹道补偿或运动预测，假设标准相机安装且使用最近反馈。
因此两个示例显式采用**零重力**作闭环基准；恢复重力后要由实际算法负责弹道补偿。
任意安装外参、较大俯仰或高延迟也需要完整算法适配器处理。

## JSON 与记录

闭环使用独立顶层配置，不改变旧场景协议：

```json
{
  "closed_loop_version": 1,
  "scene": {"schema_version": 1, "duration": 3,
    "targets": [{"id": "target", "kind": "car", "position": [4,0,0.3]}]},
  "sensor": {"output": "features"},
  "actuator": {},
  "runner": {"algorithm_frequency": 200, "log_frequency": 50},
  "algorithm": {"kind": "pixel_servo"},
  "commands": []
}
```

顶层、场景及各配置对象拒绝未知字段。`scene` 使用原目标、运动、事件和发弹格式；
`sample_period` 默认 20 ms，作为日志频率的默认值，`runner.log_frequency` 可覆盖。
场景 duration、事件、预置发弹及预置命令 time 仍要求与物理 tick 对齐。
旧 CLI 不接受此包装配置，应使用 `rm_sim_loop`。

`commands` 可预置 `time/yaw/pitch/fire/request_id`；预置命令的 time 是**生效时间**，
不再额外叠加 command_latency，允许在 tick 0 执行。`algorithm.kind="none"` 可关闭算法，
用这些指令单独测试执行器；`none` 不接受其他算法参数。
场景预置弹丸 ID 不允许使用执行器保留前缀 `actuator-shot-`。

输出保留 Qt 所需的 `schema_version=1`、`scenario`、`frames`、`hits`，同时包含：

- `configuration` 和 `resolved`：原始配置及外围模块完整默认参数、世界参数。
- `versions` 和 `timing`：各模块、算法版本、时间量化和终点语义。
- `algorithm_channels.observations/feedback`：算法可见消息，含采样与就绪时间。
- `commands/launches`：命令发出与生效时间、请求关联、实际出膛点与速度。
- `evaluation.annotations/actuator_states`：独立真值标签及按日志频率采样的实际执行器状态。
- `summary`：实际发弹、接触、有效命中与未处理队列统计。

日志整体包含评估真值，不能把整个日志作为算法输入。重放适配器应按消息就绪时间交付
`algorithm_channels`，并按算法频率执行；Qt 仅做世界轨迹回放，不重跑算法。
同程序、相同配置和种子可复现结果，不承诺跨标准库或平台的浮点/正态分布逐位一致。

## 验收与范围

C++ 测试覆盖投影数值、背面/相机后方/视野外过滤、特征模板、可选受击面遮挡、噪声重复性、
漏检/误检、60/144 Hz 时钟、姿态变换、正 yaw/pitch、角回绕和限位、速度/加速度限制、枪口偏置速度、
指令与反馈延迟、重复请求、发射间隔、延迟帧保存、同刻状态、随机流独立、关闭观测和理想闭环命中。
协议测试覆盖完整日志可复现、真值字段隔离、配置拒绝和纯校验模式；Qt 测试验证日志回放，以及参数编辑、单位转换、文件往返、高级字段保留、无效输入保护和运行后端参数一致性。

已实现简化实体遮挡的合成图像；尚未实现真实感成像、镜头畸变、曝光积分和运动模糊、完整场景实体遮挡、分类混淆、多相机、
运动基座、真实设备通信以及具体自瞄工程适配。相机和执行器参数仍需用实际设备校准。
