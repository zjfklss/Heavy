from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'resource', 'worlds', 'heavy_lob.sdf')
    params = os.path.join(pkg, 'config', 'gazebo.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 1 {world}',
            'gz_version': '8',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

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
    gimbal_bridge = Node(
        package='heavy_lob_shot',
        executable='gz_gimbal_bridge.py',
        name='gz_gimbal_bridge',
        output='screen',
    )
    watch = Node(
        package='heavy_lob_shot',
        executable='dry_run_watch',
        name='dry_run_watch',
        output='screen',
    )
    trigger = TimerAction(
        period=4.0,
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

    return LaunchDescription(
        [
            gazebo,
            bridge,
            map_to_base,
            base_to_muzzle,
            manager,
            gimbal_bridge,
            watch,
            trigger,
        ]
    )
