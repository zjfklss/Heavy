from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot_manager')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    map_yaml = os.path.join(pkg, 'resource', 'maps', 'rmuc_2026.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
    )
    map_life = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_composition': 'False',
        }.items(),
    )
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '31'),
        map_server,
        map_life,
        nav,
    ])
