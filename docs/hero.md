# 英雄入口：架构、配置、运行与 ROS2 分支对照

环境、编译和上车命令见[主 README](../readme.md) 第 1 节；标定见[标定教程](相机标定操作流程.md)。

## 1. 数据流

```text
相机线程  io::Camera ──► MultiThreadDetector（YOLOv5 + 分类网络，OpenVINO）
                                   │ 装甲板（像素）
主线程    串口反馈按时间戳插值出云台姿态 ─► Solver（PnP，转世界系）─► Tracker（EKF）
                                   │ 目标列表
决策线程  auto_outpost::Shooter（反前哨站 / 反陀螺 / 平动）─► hero::Serial 发 13 字节帧
```

- 三个线程：采集、检测与跟踪、决策与发送（`src/hero.cpp`）。
- 串口反馈 16 字节、0x5A 开头：敌方颜色、比赛状态、重置标志、yaw/pitch（弧度，抬头为正）、弹速，CRC16。发送 13 字节、0xA5 开头：pitch、yaw、控制位（0 不控制 / 1 瞄准 / 2 发射），CRC16（`io/hero/hero_serial.cpp`）。
- 云台姿态按图像时间戳在反馈历史里插值；反馈超过 `serial.timeout` 未更新时只发“不控制”帧。
- 弹速取电控反馈并做 EMA（`serial.bullet_speed_alpha`），只接受 10～20 m/s；没有反馈时用 `serial.bullet_speed`。
- 电控置位重置、敌方颜色变化或反馈中断超过 0.1 s 时，跟踪器和射击决策重建。

## 2. 目录对应

| 职责 | 当前项目 | `sp_jlcv_aimer` 对应位置 |
| --- | --- | --- |
| 入口 | `src/hero.cpp` | `src/jlcv.cpp` |
| 硬件专属适配 | `io/hero/` | `io/jlcv/` |
| 相机 | `io/camera.*`、`io/hikrobot/` | `io/` |
| 仿真桥接 | `io/simulation/`、`src/simulation/` | 同名目录 |
| 调试窗 | `tools/jlcv_show/`、`src/hero_show.hpp` | `tools/jlcv_show/`、`src/jlcv_show.hpp` |
| 算法 | `tasks/auto_aim/`、`tasks/auto_outpost/` | `tasks/` |
| 配置与模型 | `configs/hero.yaml`、`assets/` | `configs/`、`assets/` |
| 回归检查 | `tests/hero/`、`tests/calibration/` | `tests/jlcv/` |

`tools/jlcv_show/` 与 `io/simulation/protocol/` 和 `sp_jlcv_aimer` 是同一份代码，两边同步时直接整目录复制。英雄专属的部分只有 `io/simulation/bridge.*`、`src/simulation/*` 和 `src/hero_show.hpp`。

## 3. 工程来源

| 部分 | 历史来源 |
| --- | --- |
| 算法主体 | `sp_vision_25`，`auto_aim_hero`，`2626dc2` |
| 相机驱动 | `sp_jlcv_aimer`，`373b5c2` |
| 调试窗、仿真协议 | `sp_jlcv_aimer`，`f02e42d` |
| 英雄接口和参数参考 | `jlauto-aim`，`f4cd99e` 及当时工作区修改 |

## 4. 配置要点

`configs/hero.yaml` 中模型相对路径以配置文件所在目录解析，`../assets/` 指向根目录模型。

| 字段 | 当前值 | 说明 |
| --- | --- | --- |
| `camera_matrix` / `distort_coeffs` | fx 3546.6 … | 与 ROS2 分支相同 |
| `R_camera2gimbal` / `t_camera2gimbal` | 相机俯仰 0.282 rad | 见第 7.3 节第 1 条 |
| `exposure_ms` / `gain` / `fps` | 3 / 16 / 165 | 与 ROS2 分支相同 |
| `serial.device` / `baud_rate` | `/dev/ttyUSB0` / 921600 | 与 ROS2 分支相同 |
| `serial.bullet_speed` | 15.8 | 无反馈时的后备弹速；ROS2 为 15.9 |
| `shoot_delay` | 0.075 | 发射指令到弹丸出膛的延迟补偿 |
| `yaw_offset` / `pitch_offset` | 0 / 0 | 单位度 |
| `layer_height_threshold` | 0.04 | 前哨站两层高差小于此值时按当前 pitch 选层，见第 7.3 节第 3 条 |
| `enemy_color` | blue | 仅无串口反馈时的后备；离线回放红方录像要改成 red |

`configs/simulation/hero.yaml` 是仿真副本：`shoot_delay` 和 `serial.timestamp_offset` 归零，其余相同，见 [SIMULATION.md](SIMULATION.md)。

## 5. 运行方式

| 模式 | 命令 | 打开的外设 |
| --- | --- | --- |
| 配置检查 | `build/hero --check-config` | 无 |
| 冒烟测试 | `build/hero --smoke-test` | 无 |
| 相机帧率 | `build/hero --camera-test --frames=1000` | 相机 |
| 视频回放 | `build/hero --video=录像.avi [--yaw= --pitch=] [--show]` | 无 |
| 无串口实测 | `build/hero --no-serial --show` | 相机 |
| 不发射实测 | `build/hero --no-fire --show` | 相机、串口 |
| 正式运行 | `build/hero` | 相机、串口 |
| 标定采图 | `build/capture 输出目录 [--no-serial]` | 相机、串口（只读） |
| 仿真 | `build/hero --simulation [--show]` | 无，TCP 连接仿真器 |

相机测试中，单次约 100 ms 无帧会重试，连续约 1 s 无有效帧进入重连；采集 FPS 不等于完整处理 FPS。日志每秒打印一次 `[Vision]`（检测/采集 FPS、图像延迟、装甲板数、跟踪状态）和 `[Decision]`（请求/实际发送的控制位、收发包计数）。

## 6. 回归检查

| CTest 项 | 内容 |
| --- | --- |
| `hero_unit` | CRC、编解码、反馈插值、选层、弹道 |
| `hero_config` / `hero_simulation_config` | 两份配置的 `--check-config` |
| `hero_smoke` | 加载模型跑三张空图 |
| `hero_simulation` | 仿真角度约定、旋转底座下的反馈/指令往返、拒绝 17 mm 弹速、无硬件完整会话、启动器 |
| `jlcv_show` / `jlcv_show_worker_exit` | 调试窗离屏渲染与退出 |
| `calibrate_calibration` / `calibrate_calibration_chessboard_11x8` | 合成两种板的图片，调用 `calibrate_camera`，检查能否还原已知内参 |
| `calibrate_robotworld_handeye_*` / `calibrate_handeye_*` | 按 `hero.yaml` 外参和真实云台轴偏置渲染 20 组姿态，两个手眼求解器、两种板都要还原外参 |

都不需要连接实车。

## 7. 与 `feature/hero-ros2` 分支的差异

`feature/hero-ros2` 是车上 `~/jlauto-aim/hero/src` 的 ROS2 Humble 工作区快照（`429f7f9`），最新一次提交 `fdfbc56` 修了三处射击问题。

### 7.1 一致的部分

- 串口协议逐字节兼容：帧头、长度、字段顺序、CRC16（初值 0xFFFF，多项式 0x8408）、`/dev/ttyUSB0` @ 921600、`timestamp_offset` 0.006。
- 相机参数一致：序列号 DA1830770、1440×1080、曝光 3 ms、增益 16、165 FPS、bayer_rg8。
- 内参一致；外参平移一致；`shoot_delay` 0.075、空气阻力 0.0068、零偏 0。
- 敌方颜色都取串口反馈 bit0（0 红 1 蓝）。

### 7.2 不同的部分

| 项目 | tongji-hero（本分支） | hero-ros2 |
| --- | --- | --- |
| 结构 | 一个可执行文件 `build/hero` | 多个 ROS2 节点，`vision_bringup.launch.py` 拉起 |
| 自启动 | 无 | `start.sh`（gnome-terminal + 5 s 看门狗，`chmod 777 /dev/ttyUSB0`） |
| 检测 | YOLOv5（OpenVINO）+ 分类网络 | 传统灯条 + `mlp_f1.onnx` 分类 |
| 跟踪 | sp_vision `Target` EKF | `armor_processor` EKF + IMM |
| 射击决策 | `auto_outpost::Shooter` | `solve_angle`（`opti_solver.cpp`） |
| 后备弹速 | 15.8 m/s | 15.9 m/s |
| 调试 | `--show` Qt 窗、`--simulation` | Foxglove / rosbag |

### 7.3 需要注意的差异（未改，待实车确认）

1. **相机俯仰外参差 0.86°。** 本分支 `R_camera2gimbal` 对应俯仰 0.282 rad（与 `camera_rpy` 一致）；ROS2 的 `launch_params.yaml` 写的是 `rpy: 0.00 0.267 0.015487`，但同一文件注释写“0.287178 与 0.277178 取中值”，中值应为 0.282，ROS2 的数值更像笔误。哪个对需要实车确认。
2. **丢目标时云台下沉。** `fdfbc56` 让 ROS2 在丢目标或弹道无解后 0.5 s 内保持上一次角度、只关发射。本分支仍发 `{control=0, pitch=0, yaw=0}`（`tasks/auto_outpost/shooter.cpp:121` 等处，以及反馈超时、退出时）。电控在控制位为 0 时是否仍执行角度，决定了会不会下沉。
3. **前哨站选层。** `fdfbc56` 改为始终打最高层。本分支 `select_layer`（`tasks/auto_outpost/shooter.cpp:16`）在两层高差小于 `layer_height_threshold`（0.04 m）时选离当前 pitch 最近的层，`tests/hero/hero_test.cpp:102` 也按这个行为断言。
4. **反陀螺转向不对称。** ROS2 的 `v_yaw < 1` 少了绝对值，已修。本分支用 `std::abs(ekf_x[7]) > 1` 判旋转，开火窗口乘了转向符号，没有这个问题。
5. **本分支自身的疑点。**
   - `tasks/auto_outpost/shooter.cpp:98,100` 的 `get_top_front` 写成了 `center_yaw - armors[i][3] / w`，按上下文应为 `(center_yaw - armors[i][3]) / w`；影响转速低于 3 rad/s 时选哪块板。
   - 平动目标的开火判断 `is_fire = true` 被注释掉了（`tasks/auto_outpost/shooter.cpp:267`），所以不旋转的目标只瞄准不发射。

## 8. 历史状态

2026-11-05 的记录中，相机专用测试约 164.9 FPS，完整视觉处理约 100～106 FPS；有 USB 重连和跟踪收敛失败记录。2026-11-09 用车上 `~/records` 的比赛录像回放，加入调试窗和仿真入口后的输出与原程序逐帧一致。
