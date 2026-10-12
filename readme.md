# 重装视觉（同济壳 · 英雄电控）

同济 `sp_vision` 无 ROS 壳上的重装栈：吊射永远低抛；自瞄默认**钉头**（小陀螺不追板）。电控是自家英雄 `0xA5/0x5A`，不是同济 CAN/SP。

**操作手请先看：** [docs/操作手速查.md](docs/操作手速查.md)

| 入口 | 用途 |
| --- | --- |
| `heavy_standard` | **上车主入口**（短焦自瞄/打符/吊射），全量编译 |
| `heavy_lob` | 只吊射；`HEAVY_LOB_ONLY` 可无 OpenVINO 干跑 |

本轮只做短焦。钉头在 `tasks/auto_aim/aimer.cpp`（`nail_mode`）；配置见 `configs/heavy_standard.yaml` / `heavy_nail.yaml`。

| 位置 | 路径 |
| --- | --- |
| 开发机 | `~/jl_sp_vision` |
| 远程 `jlcv-pc`（`192.168.1.25`） | `/home/jlcv/Heavy`（已接 GitHub） |
| GitHub | https://github.com/zjfklss/Heavy |
| 钉头参考 | 远程 `jlauto-aim/hero` · `processAssessment2` |
| 串口参考 | 远程 `jlauto-aim/hero` · `jlcv_serial_driver` |

`jlcv-pc`：离线测 pitch≈33.27°；`build_full/heavy_standard` 已编过。短焦内参仍是占位，上场前用圆点标定替换。

## 一、本项目依赖安装（统一用 Ubuntu 22.04）

`jlcv-pc` 是 Ubuntu 22.04.5。按这个版本装，不要按旧文档去装 Ubuntu 20.04 和 ROS Galactic。

#### 1. 编译依赖

吊射入口只需要这些：

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    git \
    cmake \
    libopencv-dev \
    libfmt-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    libeigen3-dev
```

`build-essential` 里包含 gcc、g++、make。

#### 2. yaml-cpp

用上面的 `libyaml-cpp-dev`（Ubuntu 22.04 上是 0.7）。不要再从 GitHub 下 yaml-cpp 源码改它的 CMakeLists。

参数写在 `configs/*.yaml`。改完保存即可，不用重新编译。

#### 3. 相机 SDK

吊射这条编译（`HEAVY_LOB_ONLY`）不链接相机。海康、迈德威视的头文件和库已经放在 `io/hikrobot`、`io/mindvision`，给同济全量工程用。

以后要开海康相机时，再装 Linux 版 MVS，并按厂商说明做两件事：

1. 跑 `set_usbfs_memory_size.sh`，加大 USB 缓存
2. 跑 `set_usb_priority.sh`，让非管理员也能打开 USB3 相机

注意区分 x86_64 和 arm64。装的时候对上机器架构。

#### 4. 不安装 ROS 2

吊射入口没有 ROS 节点，不用 FishROS，不用 Galactic，也不用 Humble。

树里的 `io/ros2`、`tasks/auto_aim`、`tasks/auto_buff` 是同济原工程。  
**上车主入口 `heavy_standard` 必须全量编译**（OpenVINO + 海康）。`HEAVY_LOB_ONLY` 只给吊射干跑/标定离线解算。

哨兵的 Cartographer、octomap、rosbag 也不在本入口里装。

#### 5. 安装 SSH

```bash
sudo apt-get install -y openssh-server
sudo systemctl enable --now ssh
sudo systemctl status ssh
```

小电脑网口设在 `192.168.1.0/24`，开发机插上同一根网线就能连。当前小电脑地址是 `192.168.1.25`。远程桌面用 Remmina，协议 RDP，端口 3389。远程桌面的用户名和密码在小电脑的「设置 → 共享 → 远程桌面」里，和 SSH 登录密码不是同一项。

串口要用时把用户加进 `dialout`，重新登录后生效：

```bash
sudo usermod -a -G dialout $USER
```

## 二、推荐软件安装

#### 1. VS Code

到官网下安装包。Snap 商店里的是阉割版。

#### 2. Remmina

本机远程看 `jlcv-pc` 桌面用它。连接项「英雄小电脑」，地址 `192.168.1.25`，RDP，端口 3389。

#### 3. PlotJuggler

`--loop` 时程序把 JSON 用 UDP 发到 `127.0.0.1:9870`。在小电脑上打开 PlotJuggler，UDP 监听这个端口，能看到 `status`、`yaw_deg`、`pitch_deg`、`z`。

#### 4. cutecom

```bash
sudo apt-get install -y cutecom
```

看串口原始数据时用。吊射程序自己占用串口时，先关掉 cutecom。

#### 5. terminator

```bash
sudo apt-get install -y terminator
```

如果 `Ctrl+Shift+E` 和输入法冲突，在 terminator 的快捷键里改掉分屏那一项。

#### 6. 输入法

```bash
sudo apt-get install -y ibus-libpinyin
```

#### 7. VLC

最小安装的系统没有播放器。录完调试视频用 VLC 看。

```bash
sudo apt-get install -y vlc
```

#### 8. qv4l2

调普通 USB 相机参数。工业相机不用这个。

```bash
sudo apt-get install -y libv4l-dev qv4l2
```

#### 9. 其他

```bash
sudo apt-get install -y unzip unrar git net-tools
```

## 三、代码使用

吊射 only：

```bash
cmake -B build_lob -DHEAVY_LOB_ONLY=ON
cmake --build build_lob -j$(nproc)
```

上车主入口（小电脑已装 OpenVINO / 海康）：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
cmake -B build_full -DHEAVY_LOB_ONLY=OFF
cmake --build build_full -j$(nproc) --target heavy_standard heavy_lob heavy_lob_offline_test
./build_full/heavy_standard configs/heavy_standard.yaml
```

主要程序：

1. `heavy_standard`：短焦自瞄（钉头）+ 打符 + 吊射（全量）
2. `heavy_lob` / `heavy_lob_offline_test`：吊射
3. `calibrate_*`：内参 + 两套手眼（`HEAVY_LOB_ONLY` 也可编离线解算）

#### 1. 目录

```
src/heavy_standard.cpp                    上车主循环（HeroBoard）
src/heavy_lob.cpp                         吊射入口
io/hero_board.*                           英雄串口，形状照 CBoard
tasks/lob_shot/                           状态机、0xA5/0x5A、永远低抛
tasks/auto_aim/aimer.cpp                  钉头 + 已修 ekf[7] 转速
calibration/                              小圆点板
configs/heavy_standard.yaml               上车短焦（内参先占位）
configs/heavy_lob.yaml / _robot.yaml      吊射干跑 / 实车门闩
docs/操作手速查.md                         最短上车说明
docs/同济上游意见草稿.md                   同济原码问题（#15）
```

根目录的 `autostart.sh` 是同济全量视觉的自启，里面写的是 `~/Desktop/sp_vision_25`。不要跑这个脚本。

#### 2. 先跑测试

```bash
./build_lob/heavy_lob_offline_test
./build_lob/heavy_lob configs/heavy_lob.yaml --once
./build_lob/heavy_lob configs/heavy_lob.yaml --once --fire
./build_lob/heavy_lob configs/heavy_lob.yaml --once --undock
```

正常时：

1. 离线测试打印 `PASS`，俯仰角约 33.27°
2. `--once` 打印 `status=READY`，`z=1`
3. `--once --fire` 为 `z=2`
4. `--once --undock` 为 `INHIBITED`，开火被清掉

#### 3. 参数文件

两个 yaml 改完保存就生效，不用重新编译。

1. `configs/heavy_lob.yaml`：开环验收。`module_unlocked`、`module_docked` 为 true，没有实车信号也能把解算跑通。
2. `configs/heavy_lob_robot.yaml`：实车默认。这两个门闩是 false。没有对接和放行信号时不会进入可发射状态。

`base_yaw` 是底盘在 map 下的朝向，单位**弧度**（代码里直接做 `cos` / `sin`）。实车标定前用文件里的值。

目标点、枪口偏移、弹速、风车圆柱都在这两个文件里。风车区域命中会抑制吊射。

#### 4. 命令行

| 参数 | 作用 |
| --- | --- |
| `--once` | 跑到终态后退出 |
| `--fire` | `READY` 时把开火位置上 |
| `--undock` | 模拟吊射模块未对接 |
| `--loop` | 50 Hz 循环，直到退出 |
| `--dry-run` | 默认 true，只打印报文 |
| `--port` | 串口，默认 `/dev/ttyUSB0`（英雄 `device_name`） |
| `--baud` | 波特率，默认 `921600` |
| `--deploy` | 0xA5 deploy 位置 1（英雄部署模式） |

实车循环（串口约定对齐英雄 `jlcv_serial_driver`：`/dev/ttyUSB0` @ 921600，下发 0xA5，回传 0x5A）：

```bash
./build_lob/heavy_lob configs/heavy_lob_robot.yaml --loop --dry-run=false --port=/dev/ttyUSB0
```

`--deploy=true` 或 yaml `deploy: true`。回传 `curr_pitch` / `curr_yaw` 进闭环；`bullet_speed` >1 时写进解算。串口打不开会退回 `DrySender`。

`--once` 的退出码：到达预期终态为 0。没到 `READY`、开火后不是 `z=2`、或未对接时没有被抑制，返回非 0。

#### 5. 报文

1. `0xA5` 给吊射模块。`z=0` 空闲，`z=1` 瞄准，`z=2` 开火。
2. 云台 SP 帧。yaw 是底盘相对角，日志里记 `yaw_frame=chassis_rel`。

开火只在进入 `READY` 的边沿、并且加了 `--fire`（或调用方把 fire 置上）时发出。未对接、取消、重新上闩会把 fire 清掉。

#### 6. 还没接上的硬件

协议与工具已齐：0xA5/0x5A、英雄标定配置与离线程序。实车仍缺：门闩运行时信号、海康相机接线、`imu_world` / 站位实测。上面 dry-run 不是上车击发。

## 四、调试

#### 1. 看日志

程序用 spdlog 打到终端。先看这几项：`sender=`、`status=`、`pitch_deg=`、`z=`、`fire=`。

`sender=DrySender (no hardware)` 表示没写串口。`sender=SerialSender` 才是打开了设备。

#### 2. 曲线

只在 `--loop` 时向本机 UDP `9870` 发 JSON。PlotJuggler 收 `status`、`yaw_deg`、`pitch_deg`、`z`。`--once` 只跑一轮，不推曲线。

#### 3. 用 gdb

```bash
cmake -B build_lob -DHEAVY_LOB_ONLY=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build_lob -j$(nproc)
gdb --args ./build_lob/heavy_lob configs/heavy_lob.yaml --once
```

#### 4. 录屏时长

```bash
gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 300
```

单位是秒。默认太短，调试录像会被截断。

## 五、吊射审核结论与调参

对照本机旧仓 `~/heavy`（ROS）与现役 `~/jl_sp_vision`（无 ROS、永远低抛）后的结论。调参前先读这一节。

#### 1. 与旧 Heavy（ROS）的差异

| 项 | 旧 Heavy（`~/heavy`） | 现役 |
| --- | --- | --- |
| 弹道 | 先解高抛，高抛无解则失败；瞄准用低抛 | 只用低抛；高抛只记日志 `high_unused` |
| 位姿 | TF：`map→base_link→muzzle` | yaml 先验 `base_*`、`base_yaw`、`muzzle_*` |
| yaw 基点 | 底盘原点 | 枪口点（`muzzle_dx/dy` 为 0 时两者一致） |
| 门闩 | 话题 `/lob_shot/module_*` | 只读 yaml 与 `--undock`，运行时无信号输入 |
| 反馈 | `/joint_states` | 解析英雄 `0x5A`（pitch/yaw/bullet_speed）；无串口时 DrySender |
| 出口 | 话题交给串口驱动打包 0xA5 | 本程序自打包 0xA5（含 deploy）；同济 SP 只打日志 |
| 空闲发帧 | IDLE 不发 | IDLE 不发；`--loop` 下 **INHIBITED 会持续发 z=0** |
| 标定 | ROS handeye 包 | 无 ROS：`calibration/` + `configs/calibration_hero.yaml` / `camera_info.yaml` |

已对齐：状态机 `WAITING → YAW → PITCH → READY`；门闩语义；风车；`pitch_max`；dry-run 33.27°；英雄串口与双套手眼工具。

#### 2. 已知未接线与风险

1. 直接用 `heavy_lob_robot.yaml` 会一直 `INHIBITED`：门闩默认 false，又没有运行时信号能打开。
2. 闭环要有真实 0x5A；无串口或反馈坏掉时仍会停在 `*_CONVERGING`（可先 `open_loop: true`）。
3. `--loop --fire` 一进入 READY 就持续发 `z=2`，没有运行时开火开关。
4. 实车 yaml 的 `friction_coeff` 默认 0.0，旧 Heavy `robot.yaml` 是 0.001（联调建议先改回 0.001）。
5. 标定采图 `capture` 需全量编译（相机 SDK）；`HEAVY_LOB_ONLY` 只带离线解算三件套。

**改 yaml 即可**：几何、弹速、摩擦、门闩初值、开环、风车、符号、公差、串口/波特率/deploy、标定板与云台偏置。  
**必须改代码 / 接线**：门闩运行时信号、INHIBITED 不发帧、运行时开火/取消、实车相机与站位标定数据落盘。

#### 3. 分场景调参

##### 本机 dry-run / 离线（`configs/heavy_lob.yaml`）

几何不要乱改。先跑通这四条，退出码都应为 0：

```bash
./build_lob/heavy_lob_offline_test
./build_lob/heavy_lob configs/heavy_lob.yaml --once
./build_lob/heavy_lob configs/heavy_lob.yaml --once --fire
./build_lob/heavy_lob configs/heavy_lob.yaml --once --undock
```

基线日志：`yaw_deg≈16.04`，`pitch_deg≈33.27`，`--once` 为 `z=1`，`--fire` 为 `z=2`，`--undock` 为 `INHIBITED`。

##### 上车前联调（仍 dry-run）

另存一份 robot 配置副本，临时设：

- `module_unlocked: true`、`module_docked: true`
- `open_loop: true`

再填实测 `base_*`、`muzzle_*`，用 `--once` 确认不是 `windmill block`、`pitch_deg` 不超过 `pitch_max_deg`。用 cutecom 核对 `a5=` 十六进制：以 `a5` 开头、共 13 字节。

##### 实车开环（有几何、无反馈）

必须 `open_loop: true`。开环下 `yaw_tol` / `pitch_tol` / `aim_timeout_s` 不起作用。门闩在接入信号前只能在 yaml 里人工打开，**操作手确认模块已对接后再放行**。

1. 先不加 `--fire`，看云台方向，调 `yaw_sign`、`pitch_sign`、`base_yaw`
2. 确认到位后再加 `--fire`（注意会持续开火）

##### 实车闭环（有 yaw / pitch 反馈）

`SerialSender` 已解析 0x5A。接上电控后 `open_loop: false`，再调：

| 参数 | 现象 | 步进 | 验收 |
| --- | --- | --- | --- |
| `yaw_tol` / `pitch_tol`（rad，默认 0.02） | 停在 `*_CONVERGING` | 0.005 | PlotJuggler 里 `status` 能到 READY |
| `aim_timeout_s` | `IDLE (... timeout)` | 1 s | 不再超时 |
| `pitch_sign` / `yaw_sign` | 越追越远 | 只有 ±1 | 误差单调减小 |

反馈符号必须与下发命令同一坐标约定。无反馈时先退回 `open_loop: true`。

##### 风车抑制区

| 参数 | 说明 | 步进 | 验收 |
| --- | --- | --- | --- |
| `windmill_x/y` | 按规则图纸 | — | `--once` 不是 `windmill block` |
| `windmill_radius` | 误抑制或漏抑制 | 0.1 m | 半径很大时必须被抑制 |
| `windmill_z_min/max` | 弹道从上方越过却被误判 | 0.1 m | 同上 |

实车先验 `(6.429, 11.690)` 到目标的连线，在 x=14.5 处离风车中心约 2.1 m，比半径 1.7 m 只多约 0.4 m。站位一变必须重跑 `--once`。

##### 弹道与几何（dry-run 基线 33.27° 上实测）

| 参数 | 解决什么 | 灵敏度（约） | 步进 | 验收 |
| --- | --- | --- | --- | --- |
| `bullet_speed` | 整体远近（第一杠杆） | 16→37.3°，17→30.4° | 0.1 m/s | `pitch_deg` / 实弹 |
| `friction_coeff` | 远距离偏近（下降段） | 0.001→33.39° | 0.0005 | 同上；旧车用 0.001 |
| `pitch_max_deg` | `low exceeds pitch_max` | — | 按机械限位 | 看状态 |
| `muzzle_dx/dy/dz` | 枪口系统偏差 | `dz`+0.1 → −0.39° | 0.01 m | `pitch_deg`/`yaw_deg` |
| `base_x/y/z` | 站位 | `base_x`+0.5 → pitch −0.98° | 实测 | 同上 |
| `base_yaw`（**rad**） | yaw 常数偏置 | 1:1 进 yaw | 0.005 rad | `yaw_deg` |
| `pitch_sign`/`yaw_sign` | 云台反向 | — | ±1 | 先不加 `--fire` |

推荐顺序：符号 → `base_yaw` → 几何 → `bullet_speed` → `friction`。弹速已可从 0x5A 实时写入（回传 >1 时）。

## 六、相机标定（上车感知前必做）

**主路径：英雄对称小圆点板 7×7 @ 25mm。** 无 ROS。逐步操作见：

**[docs/圆点标定操作流程.md](docs/圆点标定操作流程.md)**

吊射 `HEAVY_LOB_ONLY` 运行时不读相机，但已编三套离线解算；采图 `capture` 需全量编译（相机 SDK）。结果给 `solver` / `buff_solver`。

| 配置 | 用途 |
| --- | --- |
| `configs/calibration.yaml` | **主配置**（7×7@25 + 轴偏置 + blob） |
| `configs/camera_info.yaml` | 内参（ROS 格式） |
| `configs/calibration_hero.yaml` | 与主配置同内容的别名 |
| `configs/calibration_legacy_10x7.yaml` | 旧同济大板旁路 |

```bash
# 离线
cmake -B build_lob -DHEAVY_LOB_ONLY=ON && cmake --build build_lob -j$(nproc) \
  --target calibrate_camera calibrate_handeye calibrate_robotworld_handeye
# 采图（全量）
cmake -B build && cmake --build build -j$(nproc) --target capture

./build/capture --config-path=configs/calibration.yaml \
  --output-folder=assets/img_with_q --attitude=serial
./build_lob/calibrate_camera assets/img_with_q \
  --config-path=configs/calibration.yaml --camera-info=configs/camera_info.yaml --preview=auto
./build_lob/calibrate_robotworld_handeye assets/img_with_q \
  --config-path=configs/calibration.yaml --camera-info=configs/camera_info.yaml \
  --output=configs/calibration_result.yaml --preview=auto
```

手眼主路径为 `calibrate_robotworld_handeye`；`calibrate_handeye` 为辅。CLI 用 `--key=value`。性能：blob 复用、采图预览半分辨率检测、默认 `--preview=auto`。
