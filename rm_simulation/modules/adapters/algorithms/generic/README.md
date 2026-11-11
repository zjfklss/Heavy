# 通用协议配置

只约定协议字段及坐标符号：m、s、rad，yaw 左转为正，pitch 抬头为正。
图像为 BGR8，反馈与图像按采样/就绪时间分别传递。

算法端使用 `ProtocolServer`，`ServerOptions.adapter="generic"`，返回 SDK `Command`。
无需链接世界模块、渲染器或模拟执行器，详见 [协议文档](../../../../docs/ADAPTERS.md)。
