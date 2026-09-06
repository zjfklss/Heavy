from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'resource', 'worlds', 'rmuc_2026_sim.sdf')
    gui_config = os.path.join(pkg, 'resource', 'ign', 'gui.config')
    params = os.path.join(pkg, 'config', 'gazebo.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'rmuc_sim.rviz')
    models = os.path.join(pkg, 'resource', 'models')
    gz_res = models
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        gz_res = models + os.pathsep + existing

    # --gui-config 抄 ITL gazebo.launch.py，否则 Harmonic 只用 ~/.gz/sim/8/gui.config，世界里的 camera_pose 无效。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 1 --gui-config {gui_config} {world}',
            'gz_version': '8',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/heavy_hero/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/yaw_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/pitch_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/livox/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/camera', '/camera/image_gz'),
            ('/livox/lidar', '/scan_gz'),
            ('/livox/lidar/points', '/livox/lidar_gz'),
            ('/model/heavy_hero/odometry', '/odom_gz'),
            ('/odom', '/odom_gz'),
        ],
        parameters=[{
            'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
        }],
        output='screen',
    )

    gt_tf = Node(
        package='heavy_lob_shot',
        executable='gt_tf.py',
        name='gt_tf',
        output='screen',
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
    encoder = Node(
        package='heavy_lob_shot',
        executable='encoder_0310.py',
        name='encoder_0310',
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='true'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', '31'),
            SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
            SetEnvironmentVariable('GZ_PARTITION', 'heavy_rmuc'),
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res),
            SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_res),
            SetEnvironmentVariable('SDF_PATH', gz_res),
            gazebo,
            bridge,
            gt_tf,
            manager,
            gimbal_bridge,
            encoder,
            rviz,
        ]
    )
