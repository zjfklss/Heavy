# Heavy 重装自动对矿

本分支只做 RoboMaster 2027 重装机器人自动对矿。自瞄代码已移到其他分支，本分支不再包含 `src/hero.cpp`、`tasks/auto_aim/`、`tasks/auto_outpost/`、`io/hero/`。

## 现有内容

- 技术路线：`docs/重装自动对矿技术路线.md`
- 仿真运行时：`ore_sim/hkust_runtime/`（港科 RM2026 ROS 2 + MuJoCo，尚未接入）
- 保留复用：`tools/`、`calibration/`、`io/camera.*`、`io/hikrobot/`、标定配置与圆点板
- 自瞄备份：本地分支 `backup/heavy-ore-sim-with-autoaim-20261113`

## 构建

```bash
cmake -S . -B build -DBUILD_CALIBRATION=ON
cmake --build build -j4
```

产物：`build/calibrate_camera`、`build/calibrate_handeye`、`build/calibrate_robotworld_handeye`。

## 现状

- 对矿代码尚未实现。
- `ore_sim` 脚本仍按原仓库 `runtime/` 和 ROS 2 Humble 找路径，本机为 Lyrical；权重未拷贝。
- 本分支暂不推送。
