import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ROS 参数文件（debug, flip 等），与 armor_processor 从同一路径读取
    config = os.path.join(
        get_package_share_directory('vision_bringup'),
        'config',
        'node_params.yaml'
    )

    # 注意：hero_yolo.yaml 是 Detector 内部通过 yaml-cpp 加载的，
    #       不是 ROS 参数文件，不要放在 parameters 里。

    return LaunchDescription([
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            parameters=[config]
        )
    ])
