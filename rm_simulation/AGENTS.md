# 项目目录约定

- 源码按功能维护在 `modules/`，每个库自带 `include/`、`src/` 和 `CMakeLists.txt`；公共 include 名为 `rm_sim/*.hpp`。
- CLI 和 Qt 入口分别位于 `apps/cli/`、`apps/viewer/`；配置位于 `configs/scenarios/`。
- 编译输出集中放在本目录的 `build/{release,test,core}/`；程序在 `bin/`，库在 `lib/`。
- 助手临时脚本、工作笔记、运行报告、截图和生成轨迹放在同级的 `rm_simulation-artifacts/` 对应子目录。不要在 `build/` 以外散放编译文件，也不要在仿真源码树内新建 outputs 或工作记录目录。
- 项目使用、接口和交接文档保留在 `docs/`；不要把项目必需的资源当作临时产物移走。
- 常用验证：`cmake --preset test && cmake --build --preset test && ctest --preset test`。无第三方核心验证使用 `core` 预设。
- 保持世界与算法解耦、图像/观测与评估真值隔离，以及顶部五个模块标签布局（目标、相机、云台、调度、监测）。UI 必须显示运行中的实时状态，不提供播放/进度条，不按配置时长自动结束连接。CLI 离线轨迹仅供导出和回归检查。自瞄必须保持独立进程；`modules/adapters/algorithms/<name>/` 只放协议、字段和符号转换，不放算法本体。协议、SDK 和 UI 选择见 `docs/ADAPTERS.md`。
- 完整布局和手动构建方式见 `docs/PROJECT_LAYOUT.md`。
