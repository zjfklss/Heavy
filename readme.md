# 重装吊射

本目录是同济 SuperPower 25 赛季 `sp_vision` 上的重装吊射工作副本。吊射单独编译、单独运行，不走步兵或哨兵自瞄，也不依赖 ROS。弹道只用低抛；高抛角会算出来并记在日志里，不会拿去瞄准。

两台机器上的目录：

| 位置 | 路径 |
| --- | --- |
| 开发机 | `~/jl_sp_vision` |
| 远程 `jlcv-pc`（`192.168.1.25`） | `/home/jlcv/Heavy` |

远程这份已在 2026-10-12 编译通过。离线测试通过，俯仰角 33.27°。`--once` 进入 `READY`，当时没有接串口和相机，发送走 `DrySender`。

## 编译

吊射入口只需要编译器和这几项依赖：

```bash
sudo apt install -y g++ cmake libopencv-dev libfmt-dev libspdlog-dev libyaml-cpp-dev nlohmann-json3-dev
```

```bash
cmake -B build_lob -DHEAVY_LOB_ONLY=ON
cmake --build build_lob -j$(nproc)
```

`HEAVY_LOB_ONLY` 不编译 OpenVINO、海康相机和 YOLO。不带这个选项的 `cmake -B build` 仍是原同济全量工程，需要 OpenVINO 和相机 SDK，那不是吊射的运行方式。

## 运行

在仓库根目录执行：

```bash
./build_lob/heavy_lob_offline_test
./build_lob/heavy_lob configs/heavy_lob.yaml --once
./build_lob/heavy_lob configs/heavy_lob.yaml --once --fire
./build_lob/heavy_lob configs/heavy_lob.yaml --once --undock
```

默认 `--dry-run=true`，只打印报文，不写串口。要开设备时：

```bash
./build_lob/heavy_lob configs/heavy_lob_robot.yaml --loop --dry-run=false --port=/dev/ttyUSB0
```

串口打不开会退回 `DrySender`。`--loop` 以 50 Hz 循环，直到收到退出信号。

| 参数 | 作用 |
| --- | --- |
| `--once` | 跑到终态后退出 |
| `--fire` | `READY` 时把开火位置上 |
| `--undock` | 模拟吊射模块未对接，应进入 `INHIBITED` 并清掉开火 |
| `--loop` | 持续循环 |
| `--dry-run` | 默认 true |
| `--port` | 串口，默认 `/dev/ttyUSB0` |

`--once` 的退出码：未对接且已抑制为 0；`READY` 且 `z=1` 为 0；`--fire` 且 `z=2` 为 0。其余非 0 表示没有到达对应终态。

## 配置

`configs/heavy_lob.yaml` 是开环验收：门闩打开，用来在没有实车信号时把解算跑通。

`configs/heavy_lob_robot.yaml` 是实车默认：`module_unlocked`、`module_docked` 都是 false，没有对接和放行信号时不会进入可发射状态。风车区域命中会抑制吊射。

`base_yaw` 是底盘相对云台的偏航零位，单位度。实车标定前保持配置里的值，不要把它当成已经标完的 imu 世界系航向。

## 目录

```
src/heavy_lob.cpp                         入口
tasks/lob_shot/manager.cpp                状态机、门闩、永远低抛
tasks/lob_shot/projectile_motion/         弹道
tasks/lob_shot/a5_packet.cpp              0xA5 组包，CRC 用 tools::get_crc16
tasks/lob_shot/gimbal_adapter.hpp         同济云台 SP 帧
tasks/lob_shot/lob_serial.cpp             DrySender / SerialSender
configs/heavy_lob.yaml                    开环验收
configs/heavy_lob_robot.yaml              实车门闩默认关闭
tests/heavy_lob_offline_test.cpp          离线测试
tools/                                    crc、日志、Plotter、退出
```

同济原有的 `tasks/auto_aim`、`tasks/auto_buff`、`io` 仍在树里，吊射入口不链接它们。

## 报文

瞄准指令带两路编码：

- 0xA5：给重装吊射模块，`z=0` 空闲，`z=1` 瞄准，`z=2` 开火
- 云台 SP：同济云台帧，yaw 为底盘相对角（`yaw_frame=chassis_rel`）

开火只在进入 `READY` 的边沿、并且调用方把 fire 置上时发出。未对接、取消或重新上闩会把 fire 清掉，避免停在自动开火。

## 还没有接上的部分

- 真实云台串口和 C 板
- 海康相机
- `imu_world` 航向的实车标定
- 链上 `io::Gimbal` 设备

所以上面的远程结果是本机和 `jlcv-pc` 上的离线/干跑，不是上车击发。
