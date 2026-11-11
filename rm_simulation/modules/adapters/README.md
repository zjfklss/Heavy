# 协议适配包

本包不包含任何自瞄算法。仿真作为 TCP 客户端连接独立算法进程，发送图像/反馈，接收控制指令。

- `algorithms/generic/`：统一 SI 单位，pitch 抬头为正。
- `algorithms/sp_vision/`：统一 SI 单位，pitch 抬头为负；不链接 SP 算法或硬件通信。
- `src/`：共用注册表、TCP 帧传输和仿真端协议代理。
- `sdk/cpp/`：独立算法程序可链接的协议端 SDK。
- `examples/`：仅用于通信验证、不会瞄准或开火的独立测试端。

使用、协议字段和 SDK 接入见 [ADAPTERS.md](../../docs/ADAPTERS.md)。
编译输出统一进入项目根目录的 `build/`，测试/运行产物按项目约定写到项目外。
