# 独立进程算法协议适配

更新：2026-11-07。UI 已改为持续实时运行，停止前不按场景时长断开；见 [REALTIME.md](REALTIME.md)。适配包位于 `modules/adapters/`，**只负责协议、字段和单位转换**。
仿真不链接或启动自瞄本体，自瞄作为单独的程序监听 TCP，由仿真主动连接。

本机 `/home/jlcv/sp_vision_jlcv` 已增加独立的仿真通信模式，实际收取图像/反馈，运行自己的 SP 流程，
将指令和状态返回仿真。其 `io/simulation/` 负责协议 IO，`src/simulation/runner.cpp` 调用原有算法。
仿真适配包没有复制算法本体；其他算法仍需要在自己的 IO 入口对接 SDK。

## 启动真实 SP 自瞄与 UI

在 `/home/jlcv/rm_simulation` 启动：

```bash
scripts/start_sp_vision.sh --simulation
```

它使用 SP 的 `configs/jlcv/simulation/infantry4.yaml`（瞄准零偏和额外通信延迟补偿归零），
构建并独立运行 `build/sp-vision/jlcv`，自动打开对应兵种配置的仿真 UI，默认监听 `127.0.0.1:17001`；
不会打开实体相机或串口，运行 JSON 写到项目外 `rm_simulation-artifacts/runs/sp-vision/`。
UI 运行场景时自动握手，发送 BGR 图像及云台反馈，读取自瞄的角度、控制及射击请求。
实时模式可先启动仿真，再启动自瞄；关闭自瞄修改代码时，仿真继续运行并等待自动重连。
UI 底部显示收发数量、目标数量、控制状态和自瞄角度。点击停止后，自瞄继续等待下次 UI 运行；
UI 中断或错误后，自瞄也重新监听。自瞄空闲时 Ctrl+C 可退出。
需要显示自瞄收到的图像时使用 `scripts/start_sp_vision.sh --simulation --show=true`。
按兵种选择 `scripts/start_sp_vision.sh --robot=infantry3 --simulation --show=true`；
`--robot` 支持 `infantry3`、`infantry4`（默认）、`sentry`，分别使用 `configs/jlcv/simulation/` 下同名 YAML。
本机工程可通过 `RM_SIM_SP_VISION_DIR` 指定；编译集中在 `build/sp-vision/`。

也可以在启动自瞄时一起打开仿真：

```bash
scripts/start_sp_vision.sh --robot=infantry4 --simulation --show=true
```

`jlcv` 程序统一处理 `--simulation`（也支持 `--simulation=true/false`），按 `--robot` 选择
`configs/scenarios/sensors/{infantry3,infantry4,sentry}.json`，构建并后台打开仿真 UI。
自瞄的 `src/simulation/launcher.cpp/.hpp` 在项目外的 `rm_simulation-artifacts/runs/sp-vision/` 生成本次联调 JSON，保留兵种场景、
相机和云台参数，启用图像、反馈及外部 SP 控制器，不覆盖原 JSON。
自瞄的 `--sim-host`、`--sim-port` 同步用于仿真连接；监听 `0.0.0.0` 时 UI 连接本机 `127.0.0.1`。
启动逻辑已整合到 C++，不需要 Python 3；脚本只构建自瞄并转发参数。
`--build-only` 只构建自瞄，不启动程序或仿真；正常仿真启动时由 C++ 调用仿真构建脚本。
不传此开关或传 `--simulation=false` 时，改用 `configs/jlcv/` 下同兵种 YAML，按实机模式启动，
不打开仿真。关闭仿真窗口后自瞄继续等待连接；自瞄退出时会停止自己自动启动的仿真进程组。
需要仿真独立常驻或无界面运行时，传 `--sim-ui=false` 并单独启动 UI。

已经构建后，可直接运行：

```bash
build/sp-vision/jlcv --robot=infantry4 --simulation
```

支持 `--sim-host=127.0.0.1`、`--sim-port=17001`、`--sim-once=true` 和 `--sim-project=目录`；同一端口同时只运行一个自瞄监听端。
普通硬件、视频和 smoke 模式保留；仿真模式不能与这些模式、ROS、`--check-config` 或 `--frames` 混用。

SP 输入入口用握手标定替换运行时相机 K、无畸变 D、相机安装位姿、图像尺寸、敌方颜色及标称弹速，
临时 YAML 置于系统临时目录并在会话结束时删除，不覆盖自瞄配置。算法调参（如瞄准偏置）沿用所选 YAML。
姿态只由已就绪反馈按图像采样时间插值，反馈稍晚时缓存图像等待；不读取相机位姿真值。
实时会话将模拟秒映射到本地 steady_clock，调用 SP 现有实时 Aimer 时钟模式；CLI 批处理保留逻辑时钟。
连续 `shoot` 状态只在新的算法结果到达时按配置发射间隔映射为唯一单发请求，无图像时不重放旧射击状态。

当前 SP Solver 入口要求关节偏移为零、枪口安装为单位位姿、标称弹速至少 14 m/s；不支持时明确报错。
静态底座方向会用于姿态和指令坐标转换。示例使用 1440×1080、fx/fy=2400，匹配当前 SP 的固定图像中心；
640×480 示例曾在真实检测器上未检出目标，连接成功并不保证任意相机参数下都能识别。

## UI 使用

1. 先独立运行带协议入口的自瞄程序，监听 IPv4 地址及端口。
2. 仿真“调度 → 算法 → 控制器”选择“外部自瞄（协议连接）”。相机会自动切到图像模式。
3. 在“算法协议”选择通用或 SP Vision，填写自瞄端 IPv4、端口、连接和单步超时。
4. 点击“启动实时仿真”。界面实时显示图像、世界和自瞄状态；“停止”结束会话，“应用并重启”使用新参数运行。

保存/载入会保留外部配置。`--validate` 和 UI 保存仅校验参数，**不连接算法进程**。
切换协议在下次运行时生效；运行期间不替换已经建立的会话。

实时模式由墙钟驱动物理世界，自瞄的逐条请求/应答在独立线程完成，不阻塞世界或 UI。
图像过载时丢旧帧；实际计算/网络时间会影响控制到达时间，配置延迟继续作用。
原 CLI 批处理仍使用确定的逻辑时间，仅供导出和回归验证。

## 先验证协议链路

完成 Release 构建：

```bash
./start_simulation.sh --build-only
```

终端一独立启动协议测试端（它只应答，不做自瞄或开火）：

```bash
build/release/bin/rm_sim_adapter_peer sp_vision 17001
```

终端二打开仿真：

```bash
./start_simulation.sh --config configs/scenarios/adapters/sp_vision.json
```

通用协议对应 `rm_sim_adapter_peer generic 17001` 和 `configs/scenarios/adapters/generic.json`。
通用算法接入时，用自己的独立自瞄程序替换测试端；本机 SP 可直接使用前面的真实启动方式。
SDK 默认一直监听等待首次连接，支持设置接受连接超时。

## 模块与算法目录

```text
modules/adapters/
├── algorithms/
│   ├── generic/profile.cpp        # rad，yaw 左正、pitch 抬头为正
│   └── sp_vision/profile.cpp      # rad，yaw 左正、pitch 抬头为负
├── include/rm_sim/adapters/
│   ├── profile.hpp                # 算法协议描述/注册表，UI 与运行端共享
│   ├── external.hpp               # 仿真端配置与 Algorithm 协议代理
│   ├── wire.hpp                   # TCP 定长帧头与 JSON/二进制载荷
│   └── server.hpp                 # 算法端 C++ 协议 SDK
├── src/                           # 注册、传输和仿真端收发
├── sdk/cpp/                       # 可单独构建的算法端 SDK
└── examples/protocol_peer.cpp     # 无算法的独立测试进程
```

SP Vision 配置只约定角度符号及标准字段，不复用其硬件串口包，也不包含 SP 的算法实现。
算法端接收和返回自己的角度符号，仿真侧在反馈发出与控制指令进入时做对称转换。
静态旋转矩阵仍遵循右手坐标变换，矩阵列向量不因为 pitch 标量符号变化而被逐项翻转。

增加算法时，在 `algorithms/<name>/` 定义协议描述，并加入注册表和构建列表；UI 自动读取同一注册表。
需要不同字段布局时，应把映射放在对应算法目录，共用传输层。不要将检测、解算或预测放进协议适配层。

## 配置

```json
"algorithm": {
  "kind": "external",
  "adapter": "sp_vision",
  "host": "127.0.0.1",
  "port": 17001,
  "connect_timeout": 5,
  "step_timeout": 1
}
```

JSON 超时单位为秒，UI 为 ms；范围 0.001–60 s。地址为数字 IPv4，端口为 1–65535。
外部模式要求相机启用且 `sensor.output=image`，并启用执行器反馈。
单消息 JSON 头最多 1 MiB、二进制载荷最多 64 MiB、指令最多 64 条；
配置校验限制估算的单步图像/反馈批次不超过各 1024 条，图像批次不超过载荷上限。
消息过大时应降低分辨率或提高协议交换频率 `runner.algorithm_frequency`。

`none`、`pixel_servo` 继续可用，UI 会只保存所选模式的字段。
相机几何特征模式不能用于外部图像连接。

## 算法端 C++ 接入

算法端 SDK 只依赖 C++17、Linux socket 和 nlohmann-json，不依赖 OpenCV、世界模块、相机 SDK 或自瞄工程。
在独立自瞄工程的 CMake 中加入（将目标名替换为自己的目标）：

```cmake
add_subdirectory(/home/jlcv/rm_simulation/modules/adapters/sdk/cpp
                 ${CMAKE_BINARY_DIR}/sim_protocol)
target_link_libraries(your_auto_aim PRIVATE rm_sim_adapter_wire)
```

也可以单独构建 SDK：

```bash
cmake -S modules/adapters/sdk/cpp -B build/adapter-sdk -DCMAKE_BUILD_TYPE=Release
cmake --build build/adapter-sdk -j4
```

IO 入口的基本结构如下，`run_your_algorithm` 代表自己的算法调用，适配包中没有这个函数：

```cpp
#include "rm_sim/adapters/server.hpp"

rm_sim::adapters::ServerOptions options;
options.adapter = "sp_vision";
options.port = 17001;
options.peer_version = "my-auto-aim/1";
rm_sim::adapters::ProtocolServer server(options);
server.accept();
auto nominal_configuration = server.hello().at("configuration");
while (auto step = server.next()) {
  // step->header["images"] 给出尺寸、stride、采样/就绪时间、offset 和 bytes。
  // 对应原始 BGR 像素位于 step->payload，step->header["feedback"] 为已就绪反馈。
  // 在这里调用自己已有的算法，返回协议 Command；没有新指令时返回空数组。
  auto commands = run_your_algorithm(*step, nominal_configuration);
  server.reply(commands);
}
```

原始图像跨度连续，`stride = width*3`，对应 OpenCV `CV_8UC3`。
像素缓冲生命周期与 `step` 相同；异步使用必须自行保存或复制。
每次 `next()` 后必须应答一次；`next()` 返回空表示已完成结束握手。
发生超时或断线会抛异常；异常会话应销毁后重新建立，正常结束的 server 可以再次 `accept()`。实时会话只在用户停止、重启配置或错误时结束。

算法端按**模拟时间**处理数据，不要把模拟秒和系统 `steady_clock::now()` 直接相减。
图像与反馈分别带采样时间，算法输入入口需要按采样时间同步反馈；协议不代替算法插值或延迟补偿。
所有帧都来自实际采样位姿，但协议不会发送该实际位姿真值。
SP 的 `control/yaw/pitch/shoot` 可在外部 IO 入口映射为 SDK `Command.control/yaw/pitch/fire`。

## 线上协议 rm-sim/1

每条 TCP 消息：

| 区域 | 格式 |
| --- | --- |
| 固定头 12 字节 | ASCII `RMS1`，随后两个网络字节序 uint32：JSON 长度、二进制长度 |
| JSON | UTF-8 对象，始终含 `protocol: "rm-sim/1"` 和 `type` |
| 二进制 | 图像 BGR8 字节依 JSON 中的 offset/bytes 连续排列；应答不允许二进制 |

TCP 可分包/合包，接收端按长度完整读取。每次交换共享一个绝对墙钟超时，部分字节到达不会重新计时。
此协议没有复用串口、ROS 或算法工程私有结构体的内存布局。

连接流程：

1. 仿真发送 `hello`：会话 ID、adapter、单位、模拟步长/时长/频率、重力、相机内参、静态安装外参、标称弹速和配置延迟。
2. 算法回复 `hello_ack`，回显 session_id、adapter，并提供 peer_version。
3. 每次算法时钟到期，仿真发送 `step`，算法回复 `commands`，包括无图像或无新指令的时刻。
4. 回合结束发送 `finish`，算法回复 `finished`。不强制排空相机/反馈/出膛队列。

`hello` 的配置只包含静态、可公开的标定/设备配置，不包含场景目标、种子、目标类型或真值。

`step` 的 JSON 结构：

```json
{
  "protocol": "rm-sim/1", "type": "step", "session_id": "...",
  "sequence": 3, "tick": 15, "time": 0.015, "timestep": 0.001,
  "images": [{
    "sequence": 0, "sensor_id": "camera", "calibration_id": "synthetic-pinhole-v1",
    "capture_tick": 0, "ready_tick": 12, "capture_time": 0, "ready_time": 0.012,
    "width": 640, "height": 480, "stride": 1920,
    "encoding": "bgr8", "offset": 0, "bytes": 921600
  }],
  "feedback": [{
    "sequence": 3, "capture_tick": 15, "ready_tick": 15,
    "capture_time": 0.015, "ready_time": 0.015,
    "yaw": 0, "pitch": 0, "yaw_velocity": 0, "pitch_velocity": 0
  }]
}
```

响应 `commands` 必须回显同一 session_id、sequence 和 tick，未知字段、过期/重复应答、非有限角度均拒绝。
可选 `telemetry` 是最多 32 个标量字段的对象；键不超过 64 字节，字符串不超过 128 字节，数值必须有限。
它仅用于状态显示与记录，不参与执行器控制。SDK 使用 `server.reply(commands, telemetry)` 发送；
不传 telemetry 的已有通用算法保持兼容。
单个会话仅有一个在途 step。响应例子：

```json
{
  "protocol": "rm-sim/1", "type": "commands", "session_id": "...",
  "sequence": 3, "tick": 15,
  "commands": [{"control": true, "yaw": 0.1, "pitch": -0.02,
    "fire": true, "request_id": "shot-17"}]
}
```

空数组表示无新指令；`control=false` 同样不产生新执行器命令，并要求 `fire=false`。
它不会清除先前的目标角度，执行器仍按既有模型保持/跟踪最近目标角。
`fire=true` 是一发，必须给一个非空、最多 128 字节的请求 ID。
**同一逻辑发弹请求重传时保持 ID 不变，新的一发使用新 ID**；适配层不生成持续连发或补造请求。
指令进入原执行器队列，仍至少下一物理 tick 生效，按配置量化命令、准备和发射间隔。

CLI 离线批处理遇到版本不符、超时、断线或无效报文会终止本回合，不自动重连或重放应答。
实时模式下，连接失败、超时和断线进入正常等待状态，后台每隔 0.5 秒重试连接；世界和相机继续推进，UI 不弹断线错误。
重连使用新 session_id，丢弃旧会话尚未交付的控制与 telemetry，按新会话区分发弹请求 ID；不会重放旧应答。
协议版本不符、会话不匹配和无效报文仍终止实时运行。
失败不会覆盖已有轨迹文件；已经写出的 PNG 可能保留。网络耗时不写作虚构的模拟延迟。

## 记录和验证

运行轨迹的 `resolved.algorithm` 保存协议和连接配置，`adapter` 摘要保存 profile、peer_version、完成步数、已发送图像/反馈数、接收指令数、结束状态及最后的自瞄 telemetry。
实际指令、出膛与评估仍分别写原有通道。协议不会将整个含真值的轨迹文件传给外部算法。
外部算法自身可能不确定；会话 ID 随机，不承诺外部运行与内部示例同样的逐字节可复现性。

新增协议测试使用真实独立进程和 TCP，覆盖两种符号配置、原始 BGR/反馈、分包、握手、错误版本、
超时/断线、消息大小、旧会话/旧应答拒绝、非法数值/字段、重复单发去重、SDK 独立进程、
UI 实际连接以及不连接情况下的配置校验/保存/切换。

真实 SP 端到端验证脚本（输出目录必须放到源码外）：

```bash
python3 tests/protocol/sp_vision_test.py \
  build/test/bin/rm_sim_loop build/sp-vision/jlcv \
  /home/jlcv/sp_vision_jlcv/configs/jlcv/simulation/infantry4.yaml \
  build/test/bin/rm_sim_qt /home/jlcv/rm_simulation-artifacts/tests/sp-vision-link
```

脚本调用真实检测/跟踪/瞄准流程，验证 BGR、反馈、非零角度指令、延迟反馈、同进程连续回合、
错误会话后恢复、Qt 实际连接及原硬件配置校验；不是协议假端。
2026-11-07，1440×1080 的 0.3 s 回合处理 18 帧图像、60 条反馈，返回 18 条控制指令。
