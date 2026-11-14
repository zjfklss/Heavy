#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "setup_env.sh 需要 source："
  echo "  source scripts/setup_env.sh"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PUBLIC_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALL_SETUP="${PUBLIC_ROOT}/install/setup.bash"

for distro in "${ROS_DISTRO:-lyrical}" lyrical humble; do
  if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
    source "/opt/ros/${distro}/setup.bash"
    break
  fi
done

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "找不到 ROS 2 安装环境: ${INSTALL_SETUP}"
  echo "先运行 scripts/build.sh"
  return 1
fi

source "${INSTALL_SETUP}"
