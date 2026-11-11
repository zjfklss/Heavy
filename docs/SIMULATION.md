# 英雄仿真联调（`--simulation`）

`build/hero --simulation` 从 RM 仿真器（`rm_simulation`）接收 BGR 图像和带采样时间的云台反馈，跑与实车相同的检测（YOLOv5）、解算、跟踪和 `auto_outpost::Shooter`，再把瞄准/发射指令回给仿真器。不打开相机、串口或 ROS。实现与 `sp_jlcv_aimer` 的仿真入口相同，只是角度约定和弹速检查按英雄改了。

## 快速开始

仿真工程的源码快照已经放在本仓库的 `rm_simulation/`（来自 sp_jlcv_aimer，首次运行会自动构建，需要 Qt5、libpng、nlohmann-json）。默认优先用它，没有时退回同级目录 `../rm_simulation`，也可以用 `--sim-project=绝对路径` 指定：

注意：快照里的 `configs/scenarios/sensors/hero.json` 是本仓库新增的英雄预设，相机内参、手眼外参和弹速取自 `configs/hero.yaml`；改了 `hero.yaml` 里的标定后，要同步改这个预设。

```bash
./build/hero --simulation --show                         # 自动打开仿真 UI
./build/hero --simulation --sim-preset=infantry4 --show  # 借用步兵预设：它的弹速 23 m/s 会被英雄检查拒绝，需先在仿真 UI 里改成 10~20
```

程序会调用 `<仿真工程>/start_simulation.sh`，打开 `configs/scenarios/sensors/<预设>.json` 对应的仿真 UI。预设默认 `hero`；`--sim-preset` 可写预设名或 JSON 路径。程序在 `rm_simulation-artifacts/runs/hero/` 生成本次运行用的 JSON（不改原 JSON），并在退出时关闭本次启动的仿真 UI 和后端。`RM_SIM_ARTIFACT_DIR` 可改产物目录。

手动、无图形界面或仿真器在另一台机器时，只启动自瞄端：

```bash
./build/hero --simulation --sim-ui=false --sim-host=0.0.0.0 --sim-port=17001
# 仿真器一侧：algorithm.kind=external，adapter=generic，host/port 指向自瞄机器
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| 第一个位置参数 | `configs/simulation/hero.yaml` | 仿真用配置 |
| `--sim-ui` | `true` | 是否自动启动仿真 UI |
| `--sim-project` | `rm_simulation/`（没有则 `../rm_simulation`） | 仿真工程目录 |
| `--sim-preset` | `hero` | 仿真场景预设名或 JSON 路径 |
| `--sim-host` / `--sim-port` | `127.0.0.1` / `17001` | 自瞄端监听地址 |
| `--sim-once` | `false` | 一个回合后退出；默认断开后继续等下一回合 |
| `--no-fire` | `false` | 只瞄准，不请求发射 |
| `--show` | `false` | 调试窗，见 [SHOW_UI.md](SHOW_UI.md) |

`--simulation` 不能与 `--video`、`--smoke-test`、`--camera-test`、`--check-config`、`--no-serial`、`--frames` 同时使用。等待连接时 Ctrl+C 退出。

## 仿真器一侧必须满足

握手时不满足以下条件会直接报错，而不是“连上了但从不开火”：

- **标称弹速在 10～20 m/s 之间（42 mm 弹丸）。** 英雄射击决策只接受这个范围，步兵预设常见的 23 m/s 会被拒绝。借用步兵场景时先在仿真 UI 里改弹速。
- 关节偏移为零、枪口安装为单位位姿（英雄 Solver 假设 yaw/pitch 轴与枪口共点）。
- 角度约定为 `generic`：弧度，yaw 左转为正，**pitch 抬头为正**，与英雄串口一致，不做符号翻转。

## 配置如何生效

`configs/simulation/hero.yaml` 来自 `configs/hero.yaml`，检测、跟踪、射击参数保持原值，只归零实车零偏和额外延迟补偿：`yaw_offset`/`pitch_offset`（实车本来就是 0）、`shoot_delay`（实车 0.075）、`serial.timestamp_offset`（实车 0.006）。这与 `sp_jlcv_aimer` 的 `configs/jlcv/simulation/` 是同一约定；仿真 UI 里配置的传感、命令、发射延迟独立生效，弹丸飞行时间仍参与预测。

每个回合在系统临时目录生成一份会话 YAML，用握手数据覆盖：相机内参（无畸变）、相机安装外参、`R_gimbal2imubody` 置单位阵、图像尺寸、敌方颜色、标称弹速，并关闭 `use_roi`。原配置文件不会被修改。

图像按采样时间与已到达的云台反馈对齐；反馈还没到的图像先缓存，超时丢弃。握手 `realtime=true` 时采样时间映射到本机 steady_clock；批处理模式保留逻辑时钟。射击决策输出的是“在开火窗口内”的电平，桥接层按仿真器的名义发射间隔把它转成不重复的单发请求。

## 代码位置

| 文件 | 作用 |
| --- | --- |
| `io/simulation/protocol/` | 仿真协议 SDK（TCP 分帧、握手校验），与 `sp_jlcv_aimer` 相同，同步方法见其 README |
| `io/simulation/bridge.*` | 协议 ↔ 英雄：标定覆盖、时间戳、姿态插值、角度换算、单发请求 |
| `src/simulation/launcher.*` | 选配置、生成运行 JSON、启动/关闭仿真进程 |
| `src/simulation/runner.*` | 仿真回合内的自瞄循环 |

`-DHERO_WITH_SIMULATION=OFF` 可去掉仿真入口。`tests/hero/simulation_test.cpp` 用假仿真客户端检查握手、旋转底座下反馈与指令的往返、弹速拒绝和完整回合，不需要仿真器。
