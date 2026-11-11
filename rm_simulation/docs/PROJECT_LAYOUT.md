# 源码、编译和工作产物目录

源码根目录为 `/home/jlcv/rm_simulation`。编译产物集中放在项目的 `build/`，工作产物放在项目外；
`cmake/ProjectLayout.cmake` 会拒绝在源码根目录或 `build/` 以外的源码子目录构建，并拒绝源码树内的工作产物目录。

```text
/home/jlcv/
├── rm_simulation/                  # 源码与项目资源
│   ├── build/                      # 编译产物集中目录
│   │   ├── release/{bin,lib}/      # 日常启动用程序及库
│   │   ├── test/{bin,lib}/         # 完整测试构建
│   │   ├── core/{bin,lib}/         # 无第三方依赖核心构建
│   │   └── sp-vision/             # 外部自瞄工程的仿真模式构建
│   ├── modules/
│   │   ├── world/                  # 真值、目标运动、弹道、命中
│   │   ├── protocol/               # 公共位姿与时钟（仅头文件）
│   │   ├── camera/                 # 采样、图像生成、兼容特征与私有纹理
│   │   ├── actuator/               # 云台响应、反馈与实际出膛
│   │   ├── runner/                 # 调度、消息队列与算法接口
│   │   ├── scenario/               # JSON 场景与帧序列化
│   │   ├── recording/              # PNG 图像记录
│   │   └── adapters/               # 协议及按算法配置、独立进程 SDK
│   ├── apps/
│   │   ├── cli/                    # rm_sim、rm_sim_loop
│   │   └── viewer/                 # rm_sim_qt
│   │       ├── configuration/      # 参数表单与 JSON 字段
│   │       ├── rendering/          # 观察视角模型与光栅渲染
│   │       └── resources/          # Qt 图标及资源清单
│   ├── tests/{integration,protocol}/
│   ├── configs/scenarios/          # 可复用场景输入
│   ├── docs/{reference,handoff}/   # 使用说明、参考与交接
│   ├── cmake/                      # 构建布局约束
│   ├── scripts/                    # 启动脚本实现
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   └── start_simulation.sh         # 兼容的一键启动入口
└── rm_simulation-artifacts/
    ├── tests/{test,core}/          # 测试轨迹、PNG、界面截图
    ├── runs/                      # 手动运行示例和实验输出
    ├── previews/                  # 手动预览截图
    ├── work/                      # 助手临时脚本、整理记录
    └── archive/                   # 整理前的构建和输出，保留以便恢复
```

各实现库在自己的目录维护 `include/rm_sim/`、`src/` 和 `CMakeLists.txt`。
`world` 的单元测试随模块保存；跨相机、执行器、调度器的集成测试位于 `tests/integration`，
CLI 协议测试位于 `tests/protocol`。模块通过 CMake target 声明依赖和头文件搜索路径，
不使用全局 include 目录。公共 include 写法保持 `#include "rm_sim/world.hpp"` 等形式，
现有 target 名和程序名保留。

模块依赖保持单向：protocol 使用 world，camera 和 actuator 使用 protocol，runner 使用 camera 和 actuator。
camera 可选链接 EGL/OpenGL，在捕获线程内离屏渲染，GPU 实现位于 `modules/camera/src/gpu_renderer.cpp`。
`scenario` 依赖世界和 JSON 库，`recording` 依赖相机像素类型和 libpng，CLI 组合这些模块。
Qt 通过 Runner 配置和协议注册表提供选项；独立算法通过 TCP 连接，不链接算法工程。
协议包结构及算法端 SDK 见 [ADAPTERS.md](ADAPTERS.md)。

`modules/camera/assets/armor_markings_2026.hpp` 是程序编译所需的私有纹理资源，属于源码；
生成它的临时工具、运行图像和截图属于工作产物，存放在外部产物目录。
使用文档与交接约定属于项目文档；临时工作笔记和运行报告放在 `work/`。

## 构建

从源码根目录执行（预设要求 CMake 3.21+，程序统一放到 `build/<配置>/bin/`）：

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test

cmake --preset core
cmake --build --preset core
ctest --preset core

./start_simulation.sh --build-only
./start_simulation.sh --config configs/scenarios/camera/image.json
```

不使用预设时，CMake 3.16+ 可直接指定 `build/` 下的配置目录：

```bash
cmake -S /home/jlcv/rm_simulation -B /home/jlcv/rm_simulation/build/custom \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRM_SIM_ARTIFACT_DIR=/home/jlcv/rm_simulation-artifacts
cmake --build /home/jlcv/rm_simulation/build/custom -j4
ctest --test-dir /home/jlcv/rm_simulation/build/custom --output-on-failure
```

启动脚本支持 `RM_SIM_BUILD_DIR`、`RM_SIM_ARTIFACT_DIR` 和 `RM_SIM_JOBS`。
相对输入路径仍以启动命令所在目录为准。GUI 和两个 CLI 都在同一 `bin/` 目录，
因此 Qt 通过同目录 CLI 做配置校验和离线仿真的流程继续有效。

## 生成文件与归档

手动运行时显式将输出路径放到外部目录：

```bash
mkdir -p ../rm_simulation-artifacts/runs ../rm_simulation-artifacts/previews
build/test/bin/rm_sim_loop configs/scenarios/camera/image.json \
  ../rm_simulation-artifacts/runs/camera.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/camera.json
```

CLI 的 PNG 目录仍位于轨迹旁，复制时应一起保留。Qt 临时仿真位于系统临时目录；
长期保存的轨迹、截图应选择外部产物目录。项目已有输入配置可以继续保存到 `configs/`。

整理前的 `build/`、`build-test/`、`build-core/` 和 `outputs/` 已整体移动到
`/home/jlcv/rm_simulation-artifacts/archive/layout-before-2026-11-06/`，没有删除旧实验数据。
该处另有整理前的源码快照。旧 CMake 缓存包含原绝对路径，作为归档保留；
当前程序由项目内统一的 `build/` 重新配置和构建。

实时运行使用 `modules/runner/src/live_runner.cpp`、`modules/adapters/src/live.cpp`、`apps/cli/live_output.hpp`；
UI 接收即时 BGR 帧和状态，行为见 [REALTIME.md](REALTIME.md)。
