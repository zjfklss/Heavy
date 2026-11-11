#!/usr/bin/env bash
set -euo pipefail
sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sim_build="${RM_SIM_BUILD_DIR:-$sim_root/build/release}"
sim_artifacts="${RM_SIM_ARTIFACT_DIR:-$sim_root/../rm_simulation-artifacts}"
sim_jobs="${RM_SIM_JOBS:-4}"
if [[ ! "$sim_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo 'RM_SIM_JOBS 必须为正整数' >&2
  exit 2
fi
if [[ "${1:-}" == --help ]]; then
  echo '用法：./start_simulation.sh [轨迹.json] [--config 配置.json] [Qt 参数]'
  echo '自动构建并打开仿真配置界面；--build-only 只构建。'
  echo "编译目录：$sim_build"
  echo "生成文件目录：$sim_artifacts"
  echo '可通过 RM_SIM_BUILD_DIR、RM_SIM_ARTIFACT_DIR 和 RM_SIM_JOBS 覆盖默认值。'
  exit 0
fi
command -v cmake >/dev/null || { echo '缺少 cmake，请先安装。' >&2; exit 1; }
if ! cmake -S "$sim_root" -B "$sim_build" -DCMAKE_BUILD_TYPE=Release \
    -DRM_SIM_BUILD_QT=ON -DRM_SIM_BUILD_CLI=ON -DBUILD_TESTING=OFF \
    -DRM_SIM_ARTIFACT_DIR="$sim_artifacts"; then
  echo '配置失败：请检查 C++17 编译器、Qt 5/6 Widgets、nlohmann-json 及 libpng 开发包。' >&2
  exit 1
fi
cmake --build "$sim_build" --parallel "$sim_jobs"
if [[ "${1:-}" == --build-only ]]; then exit 0; fi
exec "$sim_build/bin/rm_sim_qt" "$@"
