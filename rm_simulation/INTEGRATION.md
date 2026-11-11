# 在 SP 仓库中使用仿真

本目录包含 rm_simulation 的源码快照，仿真和自瞄仍作为独立进程运行。
从 SP 仓库根目录启动：

```bash
./rm_simulation/scripts/start_sp_vision.sh --robot=infantry4 --simulation
```

也可使用已构建的 `jlcv --simulation`，默认优先查找本目录；`--sim-project` 可以指定其他仿真工程。仿真构建文件位于 `rm_simulation/build/`，运行产物默认位于仓库根目录的 `rm_simulation-artifacts/`，均不纳入版本控制。仿真的构建、配置与协议说明见本目录的 README 和 docs。
