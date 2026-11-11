# SP Vision 协议配置

只约定 SP 的数据习惯：m、s、rad，yaw 左转为正，pitch 抬头为负。
仿真发出反馈时翻转 pitch 和 pitch_velocity；收到控制指令时将 pitch 翻回仿真符号。
相机与枪口安装矩阵保持原右手坐标变换定义。

SP 侧 IO 入口可将已有 `io::Command.control/yaw/pitch/shoot` 对应到 SDK
`Command.control/yaw/pitch/fire`，并为每一发提供独立请求 ID；相同请求重传复用 ID。
图像和反馈通过 SDK 读取，算法时间需采用模拟时钟，图像采样姿态同步仍由外部输入入口处理。

`/home/jlcv/sp_vision_jlcv` 已增加 `jlcv --simulation=true` 入口，通信代码位于其 `io/simulation/`，
算法执行入口位于其 `src/jlcv_simulation.cpp`。可从仿真项目运行 `scripts/start_sp_vision.sh` 单独启动。
相机内参、安装外参、颜色和标称弹速从握手读取，反馈按图像采样时间同步，控制及状态返回 UI。
本包没有复制、调用或链接检测、PnP、Tracker、Aimer 或 Shooter；这些仍在独立自瞄进程运行。
示例使用 1440×1080、fx/fy=2400；SP 入口当前要求零关节偏移、单位枪口安装及标称弹速至少 14 m/s。
用法与可独立构建的 SDK 见 [ADAPTERS.md](../../../../docs/ADAPTERS.md)。
