#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PUBLIC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV="${VENV:-${PUBLIC_ROOT}/.venv}"

source "${SCRIPT_DIR}/setup_env.sh"
set -u
cd "${PUBLIC_ROOT}"
VENV="$(cd "${VENV}" && pwd)"

# 关闭无关的 user-site pytest 插件。
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 "${VENV}/bin/python" -m colcon test \
  --packages-select \
    arm_exchange_interfaces \
    arm_exchange_core \
    mujoco_engine \
    arm_exchange_sim \
    arm_exchange_host \
  --event-handlers console_direct+

colcon test-result --verbose
