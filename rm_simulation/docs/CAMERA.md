# 相机图像输出

实时 UI 使用后台图像生成及直接 BGR 帧流，停止前持续运行；不再播放轨迹或等待 PNG 文件。见 [REALTIME.md](REALTIME.md)。
下文 PNG 归档说明适用于保留的 CLI 批处理导出。

更新：2026-11-06。相机提供独立图像输出；现在可通过 [协议适配包](ADAPTERS.md) 对接外部进程，自瞄本体仍由外部工程实现。
默认传感器输出已改为图像；原几何特征模式保留为显式兼容选项。
模块、统一编译目录和外部工作产物目录见 [PROJECT_LAYOUT.md](PROJECT_LAYOUT.md)。

## 启动和查看

```bash
./start_simulation.sh --config configs/scenarios/camera/image.json
```

左上为三维观察视角，左下为相机收到的画面。相机画面来自仿真相机内参和实际安装位姿，
不受三维观察视角的旋转、缩放或“选择目标”影响，持续显示最新已就绪画面；
上方标注采样和就绪时间，标注不写入图像像素。

“相机”页的“采样”可以切换图像/兼容特征，设置频率、延迟和种子；
“内参”调整分辨率和针孔参数；“成像”调整灯条颜色、尺寸、灯距和图像噪声。
界面灯条尺寸以 mm 输入，保存为 m。特征噪声、漏检、误检和特征模板仅在兼容特征模式生效。

“云台 → 运动”的“启动时朝向装甲板”默认开启。开始或应用重启时，相机对准场景第一个目标的可见装甲板，
前哨站使用实际装甲高度；机关对准靶组中心。初始化计算包含基座旋转、轴偏置、相机安装外参和云台限位。
初始化后由控制指令驱动，不用世界真值持续追踪。关闭该选项可手动设置初始 yaw/pitch。
JSON 字段为 `actuator.align_on_start`；实时 CLI 缺省开启，离线 CLI 和 C++ API 缺省关闭以保留既有实验初值。

无界面生成图像和轨迹：

```bash
mkdir -p ../rm_simulation-artifacts/runs
build/test/bin/rm_sim_loop configs/scenarios/camera/image.json ../rm_simulation-artifacts/runs/camera.json
build/test/bin/rm_sim_qt ../rm_simulation-artifacts/runs/camera.json
```

生成 `camera.json` 和旁边的 `camera.json.images/`，后者保存无损 PNG。
复制轨迹时请一起复制该目录并保持相对位置。相同像素的帧复用文件，旧图像不会自动清理。
PNG 是标准 RGB 文件；OpenCV `imread` 可直接读为 BGR8。

## 图像内容和边界

`modules/camera/src/image.cpp` 使用独立的 CPU 光栅化器，无 GUI 也能生成图像：

- 由 `fx/fy/cx/cy` 和采样时刻相机位姿计算透视投影，支持近裁剪及图像边缘裁剪。
- 仅生成浮空装甲壳、红/蓝灯条、2026 装甲标识和机关圆靶；车身、轮组、塔身及支架均已移除。
- 灯条与数字只显示在受击面正面；深度缓冲始终处理上述实体间的遮挡，与目标数组顺序无关。
- 2×超采样后缩小到配置分辨率，最终输出三个通道的 8 位 BGR 像素。
- 支持逐通道独立高斯灰度噪声，裁剪到 0–255；同程序、配置和种子可复现。

这是**合成图像相机**，尚无真实材质、光照标定、镜头畸变、曝光积分、运动模糊或真实背景。
相机实体是用于成像遮挡的简化形状，与 Qt 观察窗口的装饰网格分别实现，均不改变世界命中几何。
机关目前只表达亮灭与圆靶，不复现完整官方灯效和扇叶纹理；车辆支持英雄 1、工程 2、步兵 3/4、哨兵炮塔图案，前哨站自动使用塔形图案。
“目标 → 标识（2026）”选择车辆贴纸；JSON 为 `scene.targets[].armor_marking`，可用 `1/2/3/4/sentry` 字符串，旧文件默认 `3`。
同一目标各板相同，多目标可分别设置。图案来自官方附录，UI 与相机共享掩码，参见 [标识资源说明](../modules/camera/assets/README.md)。

灯条布局是可调整的合成值，不声称来自实机测量。`small_light_spacing/big_light_spacing`
独立于装甲碰撞宽度，当前按装甲外形宽度是否大于 0.18 m 选择大小布局。
真实视觉验证前仍需校准灯距、灯长、数字图案、相机内参和安装外参。

## JSON 配置

当前自瞄的三套传感器配置位于 [configs/scenarios/sensors](../configs/scenarios/sensors/README.md)：
infantry3、infantry4、sentry，分别保留来源相机的尺寸、165 Hz 采样率、内参、安装外参、灯条颜色及标称弹速。
完整 JSON 可由 UI 直接载入；默认不运行算法。原始畸变系数和曝光等未模拟参数记录在该目录说明中。

```json
{
  "sensor": {
    "output": "image",
    "frequency": 60,
    "latency": 0.012,
    "seed": 1,
    "calibration": {
      "id": "synthetic-pinhole-v1",
      "width": 1280, "height": 720,
      "fx": 1200, "fy": 1200, "cx": 640, "cy": 360
    },
    "image": {
      "color": "blue",
      "light_length": 0.056,
      "light_width": 0.010,
      "small_light_spacing": 0.135,
      "big_light_spacing": 0.230,
      "noise_sigma": 0
    }
  },
  "algorithm": {"kind": "none"}
}
```

此片段放进闭环 JSON，完整例子见 `configs/scenarios/camera/image.json`。
`output` 默认为 `image`；图像模式默认不使用算法，也可选择 `algorithm.kind=external` 对接独立算法进程。显式 `image + pixel_servo` 校验报错，
不将几何真值偷偷转换成检测结果。旧文件显式选择 `pixel_servo` 且未指定 output 时，
CLI/Qt 按 `features` 兼容；旧示例现已显式声明 `output: features`。

所有新字段同样拒绝未知键。图像最多 16 megapixels，内参绝对值最多 1e9，
灯条宽度小于长度，颜色为 `red/blue`；校验阶段不渲染、不写图像文件。

## 采样、接口和日志

C++ 的 `SensorPacket::image` 为 `shared_ptr<const Image>`；`Image` 只有宽、高、连续 BGR 字节。
图像模式的 `observations` 为空，不包含真值角点、目标 ID、类别或距离。
实际相机姿态仍只记录在评估 `TruthAnnotation`。

同一 tick 读取世界和执行器实际姿态后生成图像，延迟队列保存这张原始图像；
就绪时不会按新姿态重画。60/144 Hz 等采样时钟、延迟向上取整和终点不排空语义保持不变。
`AlgorithmInput.observations` 提供图像；外部进程可使用新增协议适配包接收同一通道，协议层不包含检测、跟踪或瞄准。

`Runner::image_sink` 在采样时接收帧，供 CLI 流式写 PNG。`RunLog.observations`
仅保留帧元数据，不把完整视频驻留在日志内存中；延迟队列和算法收件箱持有尚未消费的像素。
校验根据分辨率、频率、延迟、算法周期和回合时长估算待交付像素，超过 512 MiB 时拒绝运行；
这不包括光栅化器的工作缓冲或调用方自行保留的图像。
Qt 按就绪时间显示最新可用图像，采样后尚未就绪的末尾帧仍会保存，但不会提前显示。
实时 UI 显示最新已就绪的实际像素，不保存整个视频；仅 CLI 批处理将全部采样图像归档为 PNG。

输出的 `algorithm_channels.observations` 每帧带有：

```json
{
  "capture_tick": 0, "ready_tick": 12,
  "capture_time": 0, "ready_time": 0.012,
  "observations": [],
  "image": {
    "file": "camera.json.images/4ab170a5b2278623.png",
    "width": 1280, "height": 720,
    "encoding": "bgr8", "storage": "png"
  }
}
```

`encoding` 描述内存像素；`storage` 描述磁盘容器。`summary.image_frames` 计相机采样帧数，
不是去重后的 PNG 文件数。整个轨迹仍含评估真值，未来重放算法只能读取算法通道。

## 构建和验证

世界、相机渲染、执行器、Runner 和 C++ 测试仍只依赖 C++17 标准库。
CLI 新增 libpng 依赖，用于无损图像导出；不需要 OpenCV、自瞄工程或相机 SDK。

2026-11-06 验证：完整构建 `14/14` 通过；核心构建 `3/3` 通过。
新增检查包括解析投影位置、内参与姿态方向、背面/视野外过滤、前景遮挡及顺序无关性、
图像噪声种子、采样与延迟原帧保持、日志不持有视频、PNG CRC/解码/通道顺序、
文件可复现、配置校验、Qt 图像回放和成像参数保存/载入。
