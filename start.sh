#!/bin/bash
set -e

echo 'dji' | sudo -S chmod 777 /dev/ttyUSB0 || true
echo 'dji' | sudo -S ip link set lo multicast on || true

export ROS_HOME="/home/jlcv/jlauto-aim/hero"
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# 清掉可能混入的其他 workspace overlay
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset ROS_PACKAGE_PATH AMENT_CURRENT_PREFIX

source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true
source /opt/ros/humble/setup.bash
cd /home/jlcv/jlauto-aim/hero
source /home/jlcv/jlauto-aim/hero/install/setup.bash

echo "[start] AMENT_PREFIX_PATH head:"
echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | head -5

declare -a commands=("ros2 launch vision_bringup vision_bringup.launch.py")

start_commands() {
  for cmd in "${commands[@]}"
  do
    gnome-terminal -- bash -c "
      unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH;
      source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true;
      source /opt/ros/humble/setup.bash;
      echo 'dji' | sudo -S chmod 777 /dev/ttyUSB0 || true;
      cd /home/jlcv/jlauto-aim/hero;
      source /home/jlcv/jlauto-aim/hero/install/setup.bash;
      $cmd;
      exec bash;"
  done
}

watch_commands() {
  while true
  do
    for cmd in "${commands[@]}"
    do
      if ! pgrep -f "ros2 launch vision_bringup vision_bringup.launch.py" > /dev/null
      then
        echo "$cmd 未在运行, 重新启动"
        gnome-terminal -- bash -c "
          unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH;
          source /opt/intel/openvino_2024/setupvars.sh 2>/dev/null || true;
          source /opt/ros/humble/setup.bash;
          cd /home/jlcv/jlauto-aim/hero;
          source /home/jlcv/jlauto-aim/hero/install/setup.bash;
          $cmd;
          exec bash;"
      fi
    done
    sleep 5
  done
}

start_commands
watch_commands
