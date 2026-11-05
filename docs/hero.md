# 英雄入口：架构、配置与相机检查

本项目参照本机 `sp_jlcv_aimer` 的根目录分层组织。环境与完整教程见[主 README](../readme.md)，内参和两张板见[标定教程](圆点标定操作流程.md)。

## 1. 目录对应

| 职责 | 当前项目 | 参考仓库的对应位置 |
| --- | --- | --- |
| 入口 | `src/hero.cpp` | `src/jlcv.cpp` |
| 硬件专属适配 | `io/hero/` | `io/jlcv/` |
| 相机 | `io/camera.*`、`io/hikrobot/` | `io/` |
| 算法 | `tasks/auto_aim/`、`tasks/auto_outpost/` | `tasks/` |
| 公共工具 | `tools/` | `tools/` |
| 配置与模型 | `configs/hero.yaml`、`assets/` | `configs/`、`assets/` |
| 回归检查 | `tests/hero/` | `tests/jlcv/` |

原 `tongji_hero/` 嵌套目录已撤掉。旧 Heavy 的独立入口、脚本和专属配置已移除；当前使用同一套根目录构建。

## 2. 工程来源

| 部分 | 历史来源 |
| --- | --- |
| 当前算法主体 | `sp_vision_25`，`auto_aim_hero`，`2626dc2` |
| 相机驱动 | 本机 `sp_jlcv_aimer`，`373b5c2` |
| 英雄接口和参数参考 | `jlauto-aim`，`f4cd99e` 及当时工作区修改 |

目录采用参考仓库的分层，不表示把当前算法整体替换为参考仓库版本。独立仓库 `sp_jlcv_aimer` 和 `jlauto-aim` 不属于本次修改范围。

## 3. 配置与模型路径

入口默认配置为 `configs/hero.yaml`。模型相对路径以配置文件所在目录解析，因此配置中的 `../assets/` 指向根目录模型。

本次只调整文件路径和构建组织。相机分辨率、曝光、帧率、串口及算法参数保持迁移前数值。历史参考参数不能称为本次实测结果。

## 4. 仅检查相机采集

完成主 README 的构建后：

```bash
cd ~/Heavy
./build/hero configs/hero.yaml --camera-test --frames=1000
```

此模式只打开相机，不打开串口、不加载模型，不保存内参标定照片。采集 FPS 不等于完整处理 FPS。

运行前关闭占用同一相机的 MVS 或其他程序。启动日志记录实际 USB 链路速度：5000 Mbit/s 为 USB3，480 Mbit/s 为 USB2。全分辨率 165 FPS 目标依赖可用的 USB3 链路。

单次约 100 ms 无帧会重试，连续约 1 s 无有效帧或其他 SDK 错误进入重连。断连时检查线缆、接口和供电，软件重试不能消除物理故障。

## 5. 标定与回归检查

内参离线工具为 `calibration/calibrate_camera.cpp`。两张圆点板对应 `configs/calibration.yaml` 和 `configs/calibration_10x7.yaml`。详细流程与打印文件见[教程](圆点标定操作流程.md)。

已有测试迁至 `tests/hero/`，CTest 项为 `hero_unit`、`hero_config`、`hero_smoke`，均不要求连接实机。`hero_smoke` 加载已有模型进行空图检查。

## 6. 历史状态

2026-11-05 的既有记录中，相机专用测试约 164.9 FPS，完整视觉处理约 100～106 FPS；有 USB 重连和跟踪收敛失败记录，尚未完成全部实机验证。

本文不提供该发射平台的实机手眼接入、瞄准或射击操作。此次整理没有连接远程设备。
