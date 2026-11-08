import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('vision_bringup'),
        'config',
        'node_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='armor_processor',
            executable='armor_processor_node',
            name='armor_processor',
            output='screen',
            parameters=[config]
        )
    ])
