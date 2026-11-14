#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PUBLIC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV="${VENV:-${PUBLIC_ROOT}/.venv}"

if [[ ! -x "${VENV}/bin/python" ]]; then
  echo "找不到 venv: ${VENV}" >&2
  echo "设置 VENV，或在 ${PUBLIC_ROOT}/.venv 创建指向可用环境的链接" >&2
  exit 1
fi
VENV="$(cd "${VENV}" && pwd)"

cd "${PUBLIC_ROOT}"

"${VENV}/bin/python" -m colcon build --symlink-install \
  --base-paths interfaces core sim host \
  --packages-select \
    arm_exchange_interfaces \
    arm_exchange_core \
    mujoco_engine \
    arm_exchange_sim \
    arm_exchange_host \
  --cmake-args -DPython3_EXECUTABLE="${VENV}/bin/python"
