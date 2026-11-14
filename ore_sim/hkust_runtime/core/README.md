# 装配数值核心

`arm_exchange_core` 放 Host 和 MuJoCo 应用共用的数值模型与算法。它不含 ROS 节点和消息转换：调用方传 NumPy 数组和普通配置映射，时间戳、坐标查找、话题语义和任务状态由运行时边界层负责。

该包以 `ament_python` 安装，构建后 Host 和仿真进程使用同一套实现。

## 包结构

```text
core/
├── arm_exchange_core/
│   ├── __init__.py              # 包内 YAML 配置加载
│   ├── arm_model.py             # 批量 FK、解析 IK、刚体动力学
│   ├── joint_space.py           # 关节限位、周期差、惩罚
│   ├── trajectory.py            # 固定时长五次多项式时间参数化
│   ├── transform.py             # 批量变换与四元数工具
│   ├── perception/
│   │   ├── detector.py          # OpenVINO YOLO + LiteHRNet 推理
│   │   └── pose.py              # 关键点 PnP
│   ├── planning/
│   │   ├── collision.py         # 胶囊-网格碰撞与距离下界
│   │   ├── type2.py             # OMPL BIT* 点到点规划
│   │   ├── type3.py             # 装配约束分层规划
│   │   └── viterbi.py           # 分层最短路动态规划
│   ├── assets/collision/        # 装配站碰撞网格
│   └── system_config.example.yaml
└── test/test_core_contracts.py  # 数值与配置契约测试
```

## 机器人模型

`ArmModel` 是六轴机器人模型。所有公开运动学、动力学方法都带显式批量维；单次查询形状为 `(1, ...)`，不另设标量接口。

| 操作 | 输入 | 输出 |
| --- | --- | --- |
| 正运动学 | 关节 `(B, 6)` | 7 个连杆变换 `(B, 7, 4, 4)` 和 TCP 变换 `(B, 4, 4)` |
| 解析逆运动学 | TCP 变换 `(B, 4, 4)` | 8 组分支 `(B, 8, 6)` 和有效掩码 `(B, 8)` |
| 逆动力学 | 位置、速度、加速度 `(B, 6)` | 关节力矩 `(B, 6)` |
| 外部 TCP 力旋量 | `[力, 力矩]` `(B, 6)` | 等效关节力矩 `(B, 6)` |

配置中的工具偏置用于区分 DH 第 6 轴和真实 TCP。`JointSpace` 统一关节限位和差值约定：无限位连续关节按 2π 取模比较，有界关节直接相减。规划器和控制器因此共享同一套连续性与限位语义。

逆动力学和外部力旋量接口供模拟下位机使用。后者把 TCP 力旋量沿串联链传播，得到虚功等效关节力矩；预留力前馈路径见[仿真说明](../sim/arm_exchange_sim/README.md)。

## 感知

`YoloHRNetBackend` 实现 top-down 感知路径：OpenVINO YOLO 先检测候选框，裁剪归一化后送 OpenVINO LiteHRNet，关键点再映射回原图。后端返回 `KeypointObservation`，包含有序图像点、置信度和检测元数据。

`PnPEstimator` 把图像点和配置的三维点表匹配。它用 RANSAC + SQPnP 选初值，剔除相机深度非法的解，再用 Levenberg-Marquardt 精修。输出为相机相对物体的齐次变换和平均重投影误差。相机标定和 TF 组合由 Host 感知节点负责。

## 规划

规划模块对应技术报告里的 II 型和 III 型：

- `plan_joint_path_bitstar()` 封装 OMPL BIT*，在六维关节空间内查询。状态有效性函数由调用方提供，碰撞检查不进入 OMPL 适配层。
- `Type3Planner` 采样装配路径和 roll 域，使用配置的单个解析 IK 分支，计算碰撞与关节限位代价，再用 `solve_viterbi()` 解分层图。
- 严格 III 型图找不到完整可行路径时，可选的 best-effort 流程给无效节点高代价，只用带边界的 Trust Region Reflective 最小二乘修复选中的坏点；修复后的路径必须重新做碰撞检查。
- 终点延续检查让 P 轴段在选择最终 roll 时考虑后续 Q 轴扫描。

`CollisionChecker` 用配置的胶囊体表示机械臂连杆，用包内三角网格表示装配站；碰撞查询用 hpp-fcl，保守距离下界用于软安全裕度。碰撞资产通过 Python 包资源解析，调用方不传仓库相对网格路径。

`FixedDurationParameterizer` 把稀疏路点转成采样后的五次多项式关节轨迹，输出位置、速度、加速度和共享时间戳；执行和 ROS 消息发布留在 Host 层。

## 配置

从模板生成本机配置（该文件被 Git 忽略）：

```bash
cp core/arm_exchange_core/system_config.example.yaml \
  core/arm_exchange_core/system_config.yaml
```

`load_config()` 默认读包内文件，也接受显式路径，便于测试或外部工具。主要段：

| 段 | 归属 |
| --- | --- |
| `arm` | DH 几何、工具偏置、关节限位、动力学参数 |
| `collision` | 机械臂胶囊与碰撞资产选择 |
| `perception` | OpenVINO 模型、设备、关键点 schema、PnP 参数 |
| `planning.exchange` | 接近、III 型图、恢复、轨迹参数 |
| `planning.type3` | 装配约束的物理几何 |

模型路径和 OpenVINO 设备是部署相关项。几何、关节约定和关键点顺序必须在感知、规划、仿真之间保持一致。

## 运行时边界

核心包不订阅、不发布。Host 在边界处做窄转换：

- `sensor_msgs/Image` 和 `CameraInfo` 转成图像数组和相机矩阵。
- `geometry_msgs/PoseStamped` 转成齐次变换供规划使用。
- `sensor_msgs/JointState` 转成 `(1, 6)` 关节批量。
- 规划出的 NumPy 轨迹转成任务层使用的 Host/控制器消息。

这样数值代码不依赖 ROS 图即可测试，也避免在算法内部维护第二套消息式数据模型。

## 验证

先构建工作区，再在仓库根目录跑完整运行时测试：

```bash
scripts/build.sh
scripts/test.sh
```

核心契约测试覆盖 PnP 重建、检测器/配置兼容、批量 FK/IK 与动力学、碰撞资产与批量行为、III 型坐标组合，以及 P→Q 终点延续约束。
