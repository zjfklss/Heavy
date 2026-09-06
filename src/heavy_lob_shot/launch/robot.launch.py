from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot')
    params_path = os.path.join(pkg, 'config', 'robot.yaml')
    with open(params_path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)
    muzzle = raw['heavy_lob_manager']['ros__parameters'].get('muzzle_xyz', [0.0, 0.0, 0.40])

    base_to_muzzle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_muzzle',
        arguments=[
            str(muzzle[0]), str(muzzle[1]), str(muzzle[2]),
            '0', '0', '0', '1', 'base_link', 'muzzle',
        ],
    )
    manager = Node(
        package='heavy_lob_shot',
        executable='heavy_lob_manager',
        name='heavy_lob_manager',
        parameters=[params_path],
        output='screen',
    )
    return LaunchDescription([base_to_muzzle, manager])
