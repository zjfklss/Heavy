# RoboMaster 算法组视觉串口图传新人交接文档

> 面向后续接手的算法组同学。本文档说明 `linuxsender1` 和 `linuxideo` 两个目录的用途、数据流、运行方式、关键参数和常见问题。

## 1. 项目一句话说明

本项目用于把车端海康工业相机画面压缩成低码率 H264 视频流，通过 RoboMaster 自定义控制器/串口链路传输，再在接收端解码显示。

当前代码分成两部分：

| 目录 | 作用 | 主要语言/框架 |
| --- | --- | --- |
| `/home/aslianefy/linuxsender1` | ROS2 车端工作区：相机采集、图像预处理、H264 编码、串口收发、本地调试解码 | C++ / Python / ROS2 / GStreamer / OpenCV |
| `/home/aslianefy/linuxideo` | 非 ROS 的 MQTT 接收端：从裁判系统/中转设备 MQTT topic 接收 `CustomByteBlock` 并解码显示 | Python / MQTT / protobuf / PyAV / OpenCV |

## 2. 总体数据流

### 2.1 车端发送链路

```text
HikCameraNode
  ↓ /image_raw
VideoEncoderNode
  ↓ OpenCV crop/resize/static_simplify
GStreamer x264enc
  ↓ H264 byte-stream
frame_queue_ 缓存
  ↓ 每 20ms 取最多 298 字节
SerialTest::send0310Packet
  ↓ RoboMaster 0x0310 自定义数据
裁判系统/客户端/中转设备
```

### 2.2 接收解码链路

```text
MQTT broker 192.168.12.1:3333
  ↓ topic: CustomByteBlock
custom_byte_block_pb2.CustomByteBlock.data
  ↓ [2字节小端长度][H264 chunk][padding]
PyAV H264 decoder
  ↓ BGR frame
OpenCV imshow
```

### 2.3 本地 ROS 调试链路

`linuxsender1` 里还有一个本地 ROS 解码节点，用于不经过 MQTT、直接验证串口收到的数据是否能解码：

```text
SerialTest 接收 0x0311
  ↓ publish /from_custom_client
VideoDecoderNode
  ↓ PyAV decode
OpenCV imshow
```

注意：本地调试链路和实际 MQTT 接收链路不是完全同一个通信路径，但视频包格式基本一致。

## 3. 关键协议格式

### 3.1 视频 chunk 格式

发送端每次通过自定义数据发送固定 300 字节 payload：

```text
byte 0      : h264_len 低 8 位
byte 1      : h264_len 高 8 位
byte 2..N   : H264 chunk，有效长度为 h264_len
剩余字节    : padding，通常为 0
```

其中：

- `h264_len` 为 little-endian `uint16`。
- 单包 H264 有效数据最多 `298` 字节。
- 如果 `h264_len == 0`，接收端会忽略该包。
- 接收端会把连续 chunk 喂给 PyAV 的 H264 parser/decoder。

### 3.2 RoboMaster 串口命令

`serial_test` 封装了 RoboMaster 自定义数据帧：

| cmd_id | 方向 | 作用 | 单帧数据区 |
| --- | --- | --- | --- |
| `0x0310` | 车端发送给客户端 | 自定义机器人数据，当前用于发送视频 chunk | 300 字节 |
| `0x0311` | 客户端发送给车端 | 自定义客户端命令，当前用于接收控制/状态 | 30 字节 |

发送函数：

- `SerialTest::sendToCustomClient()`
- `SerialTest::send0310Packet()`
- `SerialTest::send0311Packet()`

### 3.3 MQTT topic

`linuxideo` 订阅：

| topic | protobuf | 用途 |
| --- | --- | --- |
| `CustomByteBlock` | `CustomByteBlock { bytes data = 1; }` | 承载视频 chunk 或测试字符串 |
| `DeployModeStatusSync` | `DeployModeStatusSync { optional uint32 status = 1; }` | 部署模式状态同步 |

`linuxideo` 发布：

| topic | protobuf | 用途 |
| --- | --- | --- |
| `CustomControl` | `CustomControl { optional bytes data = 1; }` | 回传控制数据给车端 |

## 4. `linuxsender1` 代码结构

```text
linuxsender1/
├── src/
│   ├── bringup/
│   │   └── launch/sniper.launch.py              # 一键启动相机、编码器、串口、解码器
│   ├── hik_camera/
│   │   └── src/hik_camera_node.cpp              # 海康相机 ROS2 component
│   ├── doorlock_sniper/
│   │   ├── src/video_encoder_node.cpp           # 图像预处理 + H264 编码 + 串口分包发送
│   │   ├── include/doorlock_sniper/video_encoder_node.hpp
│   │   └── msg/VideoPacket.msg                  # 旧/预留消息
│   ├── serial_test/
│   │   ├── src/serial_test.cpp                  # RoboMaster 串口协议封装
│   │   ├── src/crc.cpp                          # CRC8/CRC16
│   │   └── src/main.cpp                         # 独立串口测试节点
│   └── doorlock_decoder/
│       └── doorlock_decoder/video_decoder_node.py # ROS 本地 H264 解码显示
├── build/
├── install/
└── log/
```

## 5. `linuxideo` 代码结构

```text
linuxideo/
├── mqtt_video_receiver_v4.py       # 当前主要 MQTT 视频接收/解码脚本
├── mqtt_video_listening.py         # 简单订阅测速脚本，只统计 CustomByteBlock Hz/kbps
├── custom_byte_block.proto         # CustomByteBlock protobuf 定义
├── CustomControl.proto             # CustomControl protobuf 定义
├── DeployModeStatusSync.proto      # DeployModeStatusSync protobuf 定义
├── *_pb2.py                        # protoc 生成的 Python 文件
├── opencv_qt_fix.py                # OpenCV Qt 字体路径修复
└── receiver.log                    # 接收端运行日志
```

## 6. 运行前环境检查

### 6.1 硬件/系统依赖

车端通常需要：

- ROS2 环境。
- 海康 MVS SDK，默认代码查找 `/opt/MVS/include` 和 `/opt/MVS/lib/64`。
- 可用海康工业相机，且没有被 MVS GUI 或其他进程占用。
- 串口设备，默认 `/dev/ttyUSB0`。
- GStreamer、x264、OpenCV、cv_bridge。

接收端通常需要：

- Python 3。
- `paho-mqtt`。
- `protobuf`。
- `av` / PyAV。
- `opencv-python` 或系统 OpenCV Python 绑定。
- 能访问 MQTT broker：默认 `192.168.12.1:3333`。

### 6.2 串口权限

如果打不开 `/dev/ttyUSB0`，先检查：

```bash
ls -l /dev/ttyUSB0
groups
```

常见处理方式：

```bash
sudo usermod -aG dialout $USER
# 退出重新登录后生效
```

临时调试可用：

```bash
sudo chmod 666 /dev/ttyUSB0
```

## 7. 编译与启动

### 7.1 编译 `linuxsender1`

```bash
cd /home/aslianefy/linuxsender1
colcon build --symlink-install
source install/setup.bash
```

如果只改了某个包，可以指定包名：

```bash
colcon build --symlink-install --packages-select doorlock_sniper
source install/setup.bash
```

### 7.2 一键启动车端

```bash
cd /home/aslianefy/linuxsender1
source install/setup.bash
ros2 launch bringup sniper.launch.py
```

当前 launch 会启动：

- `hik_camera::HikCameraNode`
- `doorlock_sniper::VideoEncoderNode`
- `serial_test_node`
- `doorlock_decoder.decoder_node`

注意：`VideoEncoderNode` 内部已经创建了一个 `SerialTest` 实例；launch 里又启动了一个独立 `serial_test_node`。如果两者同时打开同一个 `/dev/ttyUSB0`，可能发生串口占用。实际比赛/稳定运行时建议只保留一个串口拥有者。

### 7.3 启动 MQTT 接收端

```bash
cd /home/aslianefy/linuxideo
python3 mqtt_video_receiver_v4.py
```

如果只想看 MQTT 是否有数据和频率：

```bash
cd /home/aslianefy/linuxideo
python3 mqtt_video_listening.py
```

## 8. 重要参数说明

主要参数在 `sniper.launch.py` 里配置。

### 8.1 图像与编码参数

| 参数 | 当前值 | 说明 | 调参建议 |
| --- | --- | --- | --- |
| `crop_size` | `800` | 从原图中心裁剪 ROI 的尺寸 | 目标太小可减小，视野不够可增大 |
| `output_size` | `300` | 编码前 resize 到 `300x300` | 越大越清晰，但码率压力越大 |
| `output_fps` | `60` | 输入给编码器的目标帧率 | 串口带宽紧张时可降到 30/40 |
| `target_bitrate` | `80` | x264 目标码率，单位 kbps | 串口 50Hz×300B 下不要盲目升高 |
| `x264_preset` | `veryslow` | x264 压缩预设 | CPU 不够时改 `veryfast`/`faster` |
| `enable_display` | `True` | 车端本地显示调试窗口 | 无桌面环境或比赛运行可关掉 |

### 8.2 静态简化参数

静态简化用于低码率下保留运动区域、弱化静态背景，减少码率浪费。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `static_simplify` | `True` | 是否启用静态背景简化 |
| `motion_threshold` | `14` | 当前帧与背景差异超过该值认为有运动 |
| `motion_erode_px` | `2` | 运动 mask 腐蚀大小，去噪 |
| `motion_dilate_px` | `6` | 运动 mask 膨胀大小，扩大保留区域 |
| `motion_trail_frames` | `90` | 运动轨迹保留帧数 |
| `trail_disable_motion_ratio` | `0.30` | 大面积运动时禁用 trail，避免整屏拖影 |
| `bg_update_alpha` | `0.01` | 背景更新速度 |
| `bg_blur_sigma` | `1.8` | 背景/差分平滑强度 |
| `center_clear_size` | `150` | 中心区域强制保留尺寸 |
| `force_monochrome` | `False` | 是否强制灰度化后再编码 |

### 8.3 串口参数

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `serial_device` | `/dev/ttyUSB0` | 串口设备路径 |
| `serial_baud_rate` | `921600` | 串口波特率 |
| `serial_send_hz` | `50` | 自定义数据发送频率，上限代码限制为 50 |

粗略带宽估算：

```text
300 bytes × 50 Hz = 15000 bytes/s ≈ 120 kbps payload
```

实际还要扣除串口帧头、CRC、链路抖动，所以 `target_bitrate=80kbps` 是比较合理的低码率设置。若画面卡顿，优先降低 `target_bitrate`、`output_fps` 或 `output_size`。

## 9. 日志怎么看

### 9.1 编码端日志

`VideoEncoderNode` 会打印：

- GStreamer 是否初始化成功。
- 当前编码模式。
- 帧大小统计。
- 串口发送包数、字节数、队列积压、丢帧数。

重点关注：

| 现象 | 可能含义 |
| --- | --- |
| `total_queued_bytes_` 持续增长 | 编码码率超过串口可承载带宽 |
| `drop_frames` 增加 | 队列积压后开始丢旧帧 |
| `GStreamer element creation failed` | 缺 GStreamer/x264 插件 |
| 相机 fatal | 相机未连接、被占用或 SDK 异常 |

### 9.2 接收端日志

`linuxideo/mqtt_video_receiver_v4.py` 会同时输出到终端和 `receiver.log`。

重点关注：

| 字段 | 含义 |
| --- | --- |
| `cbb` / `pkts` | 收到的 MQTT 视频包数 |
| `fps` | 解码出的帧数 |
| `drops` | 接收端队列满丢弃 chunk |
| `dec_err` | H264 解码错误 |
| `reset` | 解码器重置次数 |
| `kbps` | 接收端统计到的 payload 码率 |

## 10. 常见问题排查

### 10.1 相机打不开

检查顺序：

1. 海康 MVS SDK 是否安装在 `/opt/MVS`。
2. 相机是否被 MVS GUI 或其他程序占用。
3. USB/网口供电是否稳定。
4. `hik_camera_node.cpp` 打印的错误码。

### 10.2 串口打不开或无数据

检查顺序：

1. `/dev/ttyUSB0` 是否存在。
2. 用户是否在 `dialout` 组。
3. 是否有两个进程同时打开同一个串口。
4. 波特率是否和对端一致，当前默认 `921600`。
5. 物理连接是否接反或接触不良。

### 10.3 接收端 MQTT 连不上

检查顺序：

1. 当前电脑是否能 ping 到 `192.168.12.1`。
2. MQTT broker 端口是否为 `3333`。
3. 是否在正确网络/车载 Wi-Fi 下。
4. 防火墙是否阻止连接。

### 10.4 有包但没画面

常见原因：

- `CustomByteBlock.data` 不是视频数据，而是类似 `Robot ...` 的测试字符串。
- H264 chunk 丢包太多，decoder 等不到关键参数或 IDR。
- 发送端码率过高，串口队列积压并丢帧。
- 接收端 `width/height` 显示参数和实际编码尺寸不一致，导致显示/准星不符合预期。

建议：

1. 先跑 `mqtt_video_listening.py` 看 `CustomByteBlock` 频率和 kbps。
2. 再跑 `mqtt_video_receiver_v4.py` 看 `dec_err/reset/fps`。
3. 降低 `target_bitrate` 到 `60`，或把 `output_fps` 降到 `30` 测试。
4. 确认发送端 x264 日志没有持续积压。

### 10.5 画面延迟越来越大

这是典型的生产速度大于发送速度。

处理优先级：

1. 降 `target_bitrate`。
2. 降 `output_fps`。
3. 降 `output_size`。
4. 减小静态区域细节，让 `static_simplify` 更激进。
5. 检查串口实际频率是否真的到 `50Hz`。

### 10.6 OpenCV 窗口打不开

可能原因：

- 当前是无桌面 SSH 环境。
- 没有正确设置 `DISPLAY`。
- OpenCV Qt 插件路径冲突。

处理：

- 比赛运行可把 `enable_display` 或 `display` 关掉。
- `linuxideo` 已调用 `opencv_qt_fix.py` 修复部分 Qt 字体路径问题。

## 11. 推荐新人上手顺序

1. 先阅读本文件，理解数据流。
2. 看 `sniper.launch.py`，知道系统启动了哪些节点。
3. 看 `serial_test.cpp`，理解 0x0310/0x0311 和 CRC 封装。
4. 看 `video_encoder_node.cpp`，重点读：参数声明、GStreamer 初始化、`serial_timer_callback()`。
5. 跑 `mqtt_video_listening.py`，确认链路上是否有数据。
6. 跑 `mqtt_video_receiver_v4.py`，确认能否解码出画面。
7. 最后再调图像预处理和码率参数。

## 12. 修改代码时的注意事项

### 12.1 不要随便改包大小

当前很多地方默认 300 字节 payload、前 2 字节长度、最多 298 字节 H264。如果要改，需要同步修改：

- `VideoEncoderNode::serial_timer_callback()`。
- `SerialTest::send0310Packet()` 里的 300 字节限制。
- 对端解析逻辑。
- RoboMaster 自定义数据协议限制。

### 12.2 不要字节级丢弃 H264 流

H264 是连续码流，随便从中间丢字节会造成长时间解码失败。当前发送端使用“帧级队列 + 丢旧帧”的方式，尽量避免破坏码流结构。

### 12.3 参数调优先从带宽算起

低码率链路最重要的是带宽预算。先算链路最大 payload，再决定 `target_bitrate`，不要只看画质。

### 12.4 串口只让一个节点拥有

如果编码节点内部已经创建 `SerialTest`，就不要再启动另一个独立 `serial_test_node` 抢同一个设备。调试时可以保留，稳定运行时建议清理 launch。

## 13. 常用命令速查

```bash
# 编译全部
cd /home/aslianefy/linuxsender1
colcon build --symlink-install
source install/setup.bash

# 编译编码包
colcon build --symlink-install --packages-select doorlock_sniper
source install/setup.bash

# 启动车端
ros2 launch bringup sniper.launch.py

# 看 ROS topic
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /from_custom_client

# 启动 MQTT 解码
cd /home/aslianefy/linuxideo
python3 mqtt_video_receiver_v4.py

# MQTT 频率统计
python3 mqtt_video_listening.py

# 检查串口
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0
```

## 14. 后续建议 TODO

这些不是新人必须马上做的事，但建议后续维护时逐步改：

- 把 `sniper.launch.py` 中的独立 `serial_test_node` 和编码节点内置串口职责理清，避免串口抢占。
- 给 `mqtt_video_receiver_v4.py` 增加命令行参数，例如 `--host --port --width --height --scale`。
- 把 300 字节、298 字节、MQTT host 等 magic number 集中成配置。
- 增加一个纯文件回放工具：保存 H264 chunk 后离线解码，方便排查是链路问题还是编码问题。
- 给串口发送端增加更明确的队列积压报警，例如超过 1 秒带宽时直接打印红色 warning。
- 明确比赛部署模式：车端是否需要本地显示、是否需要 ROS 本地 decoder、是否只保留 MQTT 接收端。

---

维护建议：每次改动协议、包大小、topic 名称、launch 默认参数，都同步更新本文档。
