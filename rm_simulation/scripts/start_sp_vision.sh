#!/usr/bin/env bash
set -euo pipefail
sim_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$sim_root/../src/jlcv.cpp" ]]; then
  default_vision_root="$sim_root/.."
else
  default_vision_root="$sim_root/../sp_vision_jlcv"
fi
vision_root="${RM_SIM_SP_VISION_DIR:-$default_vision_root}"
vision_build="$sim_root/build/sp-vision"
usage() {
  echo '用法：scripts/start_sp_vision.sh [--robot=infantry3|infantry4|sentry] [--simulation] [自瞄参数]'
  echo '构建并启动 jlcv；模式选择、YAML/JSON 选择和仿真启动均由 jlcv 程序处理。'
  echo '默认实机模式；--simulation 选择仿真 YAML 并自动打开对应仿真。'
  echo '例如：scripts/start_sp_vision.sh --robot=infantry4 --simulation --show=true'
  echo '--sim-ui=false 只运行 TCP 自瞄端，用于无界面或手动启动仿真。'
  echo 'RM_SIM_SP_VISION_DIR 可指定自瞄工程；--build-only 只构建 jlcv。'
}
vision_build_only=false
vision_arguments=()
while (( $# )); do
  case "$1" in
    --help) usage; exit 0 ;;
    --build-only) vision_build_only=true; shift ;;
    --robot)
      [[ $# -ge 2 ]] || { echo '--robot 需要指定兵种' >&2; exit 2; }
      vision_arguments+=("--robot=$2"); shift 2 ;;
    *) vision_arguments+=("$1"); shift ;;
  esac
done
vision_jobs="${RM_SIM_JOBS:-2}"
[[ "$vision_jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'RM_SIM_JOBS 必须为正整数' >&2; exit 2; }
cmake -S "$vision_root" -B "$vision_build" -DJLCV_WITH_SIMULATION=ON -DJLCV_WITH_SHOW_UI=ON -DBUILD_TESTING=OFF \
  -DJLCV_SIM_SDK_DIR="$sim_root/modules/adapters/sdk/cpp"
cmake --build "$vision_build" --target jlcv --parallel "$vision_jobs"
if [[ "$vision_build_only" == true ]]; then exit 0; fi
cd "$vision_root"
exec "$vision_build/jlcv" "${vision_arguments[@]}"
