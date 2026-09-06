# ITL 控制三包（只读原文，不编译）

从本机 `/home/jlcv/jlauto-aim/ITL_Hero_Shoot` 拷出，对应 GitHub `qwq9966qwq/ITL_Hero_Shoot`。

提交号见同目录 `SOURCE.txt`。此目录有 `COLCON_IGNORE`，`colcon build` 不会编它们。

可编译副本在：

- `/home/jlcv/heavy/src/projectile_motion`
- `/home/jlcv/heavy/src/lob_shot_aiming`

相对原文只改了：`friction≈0` 不炸、`solve_high` 用真空高抛根作初值、`solvePitch` 输出物理仰角（本队 `pitch_sign` 再取符号）。`lob_shot_manager` 仍不整包编译（依赖 `utils` 日志 + `JointState` 云台口）。
