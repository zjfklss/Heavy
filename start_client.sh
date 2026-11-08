#!/bin/bash

# ======================== 固定配置区 ========================
SUDO_PWD="dji"

# 固定串口（已经绑定，永远不变）
SERIAL_PORT0="/dev/tty_serial0"
SERIAL_PORT1="/dev/tty_serial1"

# 工作空间1：hero（放 vision_bringup）
ROS_WS_HERO="/home/jlcv/hero_new_tracker"

# 工作空间2：这里改成你放 bringup 功能包的路径！！！
ROS_WS_BRINGUP="/home/jlcv/linuxsender126/linuxsender1"

# ROS 环境
ROS_DOMAIN_ID=24
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
# =============================================================


# 给串口权限
echo $SUDO_PWD | sudo -S chmod 666 $SERIAL_PORT0
echo $SUDO_PWD | sudo -S chmod 666 $SERIAL_PORT1


# ======================== 启动任务配置 ========================
# 格式："工作空间代号 | 启动命令"
# hero = 工作空间1
# bringup = 工作空间2

declare -a START_TASKS=(
    "hero | ros2 launch vision_bringup vision_bringup.launch.py"
    "bringup | ros2 launch bringup sniper.launch.py"
)
# =============================================================


# 启动单个任务
start_task() {
    local ws_name="$1"
    local cmd="$2"

    if [ "$ws_name" = "hero" ]; then
        WS_PATH="$ROS_WS_HERO"
    elif [ "$ws_name" = "bringup" ]; then
        WS_PATH="$ROS_WS_BRINGUP"
    else
        echo "未知工作空间"
        return
    fi

    gnome-terminal -- bash -c "
        export ROS_DOMAIN_ID=$ROS_DOMAIN_ID;
        export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION;
        export ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY;

        source /opt/ros/humble/setup.bash;
        source /opt/intel/openvino_2024/setupvars.sh;

        cd $WS_PATH;
        source install/setup.bash;

        echo '==================================================';
        echo '工作空间：'$WS_NAME;
        echo '启动命令：'$cmd;
        echo '==================================================';

        $cmd;
        exec bash;
    "
    sleep 1.2
}


# 启动所有
start_commands() {
    echo "=== 启动所有ROS节点 ==="
    for task in "${START_TASKS[@]}"; do
        IFS="|" read -r ws cmd <<< "$task"
        start_task "$(echo $ws | xargs)" "$(echo $cmd | xargs)"
    done
}


# 守护进程
watch_commands() {
    echo "=== 进程守护已启动，每5秒检查一次 ==="
    while true; do
        for task in "${START_TASKS[@]}"; do
            IFS="|" read -r ws cmd <<< "$task"
            cmd_trim=$(echo $cmd | xargs)
            
            if ! pgrep -f "$cmd_trim" > /dev/null; then
                echo "⚠️  重启：$cmd_trim"
                start_task "$(echo $ws | xargs)" "$cmd_trim"
            fi
        done
        sleep 5
    done
}


# 执行
start_commands
watch_commands
