from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot')
    params = os.path.join(pkg, 'config', 'dry_run.yaml')

    map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['3', '1.5', '0', '0', '0', '0', '1', 'map', 'base_link'],
    )
    base_to_muzzle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_muzzle',
        arguments=['0', '0', '0.4', '0', '0', '0', '1', 'base_link', 'muzzle'],
    )
    manager = Node(
        package='heavy_lob_shot',
        executable='heavy_lob_manager',
        name='heavy_lob_manager',
        parameters=[params],
        output='screen',
    )
    watch = Node(
        package='heavy_lob_shot',
        executable='dry_run_watch',
        name='dry_run_watch',
        output='screen',
    )
    trigger = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/heavy/lob_trigger', 'std_msgs/msg/Bool', '{data: true}',
                ],
                output='screen',
            )
        ],
    )
    stop_after_watch = RegisterEventHandler(
        OnProcessExit(
            target_action=watch,
            on_exit=[EmitEvent(event=Shutdown(reason='dry_run watch finished'))],
        )
    )
    return LaunchDescription(
        [map_to_base, base_to_muzzle, manager, watch, trigger, stop_after_watch]
    )
