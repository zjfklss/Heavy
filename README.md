# 重装吊射

军临 RM2027 重装的雷达吊射，独立仓库。定位用已有 NUDT，控制顺序抄 ITL，弹道取高抛，出口走本队 `0xA5` / `z=0/1/2`。

**不改** `/home/jlcv/jlauto-aim` 里的 `hero/`、`NUDT_navigation/`、`jlcv_serial_driver`、海康驱动。

## 先看哪份文档

| 文件 | 内容 |
| --- | --- |
| [`docs/开源对照.md`](docs/开源对照.md) | 为什么没有更好的完整开源、抄什么、不抄什么 |
| [`docs/修改说明.md`](docs/修改说明.md) | 相对 ITL / 本队半成品改了什么 |
| [`docs/运行方式.md`](docs/运行方式.md) | 路径写死的编译、离线 demo、静态 TF 干跑、实车挂载 |

## 最短验收

本机 bash 常已叠 NUDT / OpenVINO。只 `source /opt/ros/jazzy` **不够**，`ros2` 仍找不到本包。每次开终端都要再 `source /home/jlcv/heavy/install/setup.bash`。

```bash
python3 /home/jlcv/heavy/tools/demo01_offline_lob_shot.py
cd /home/jlcv/heavy
source /opt/ros/jazzy/setup.bash
PATH="/usr/bin:/opt/ros/jazzy/bin:$PATH" colcon build --symlink-install --packages-up-to heavy_lob_shot --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source /home/jlcv/heavy/install/setup.bash
ros2 launch heavy_lob_shot dry_run.launch.py
# 仿真（必须 DOMAIN=31，避开实车 Livox）：
# export ROS_DOMAIN_ID=31
# ros2 launch heavy_lob_shot rmuc_sim.launch.py
# ros2 launch heavy_lob_shot rmuc_nav.launch.py
```

干跑日志出现 `DRY_RUN PASS` 且状态到 `READY` 才算第一版 ROS 节点站住。发弹仍等电控联调。

自定义客户端在 `vendor/custom_client/`（`linuxsender1` + `linuxideo`，已排除约 7.2G 的 `sr/` venv）。说明见该目录 `README.md`。
