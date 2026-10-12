# CHANGELOG · 重装吊射迁同济

## 2026-10-08 夜 · 审查后 P1 补齐

### 新增/修复

- `encode_a5` 改用同济 `tools::get_crc16`；offline 交叉 `check_crc16`
- `gimbal_adapter.hpp`：组同济 `SP` 帧（mode/yaw/pitch/crc）；标明 `yaw_frame=chassis_rel`
- `SerialSender` 真实 open/write 路径保留；默认 dry-run
- 安全：边沿 trigger；门闩/取消清 fire（防自动开火）
- `base_yaw` + wrap；`heavy_lob_robot.yaml` 默认关门闩
- 同济 tools：Exiter / logger / Plotter / OpenCV CLI
- 四场景截图目录 `20261008_p1_four`
- 本地 `git init` 分支 `local-heavy-lob`，**禁止 push**

### 明确未完成

真实 Gimbal/CBoard 设备、海康、`.25` 部署。

### 勿再使用的表述

「全部迁移已完成」「接口已齐（含实机）」——不成立。
