# CHANGELOG · 重装吊射迁同济

## 2026-10-12 · heavy_standard 上车骨架

### 新增

- `io/hero_board`：照 `CBoard` 的 `imu_at` / `send`；英雄 `0xA5/0x5A`
- `src/heavy_standard.cpp`：同济 `mt_standard` 循环（自瞄 / 打符 / 吊射）
- `configs/heavy_standard.yaml`：短焦占位 + 钉头 + 吊射先验
- 自瞄/打符 pitch 翻号（同济抬头负 → 英雄抬头正）；吊射不翻、20 ms 节拍
- 全量 CMake 修 `heavy_lob` 链接（OpenCV + tools + spdlog）
- 英雄小电脑 `~/Heavy` 建 Git 仓并对齐 GitHub `zjfklss/Heavy`
- 同济上游意见草稿补 Issue 4/5；Issue 1 已开 #15

### 本队相对同济的修复（勿当上游已合）

- `aimer`：转速用 `ekf[7]` + `|w|`；钉头 `nail_mode`
- `buff_aimer`：`yaw/pitch` 先置 `last_*`，避免未初始化比较

### 明确未完成

短焦实车标定替换占位；长焦；串口动态切 buff/lob。

---

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
