# 抄自 ITL Real/src/bringup/launch/Simulation_shoot.launch.py
# https://github.com/qwq9966qwq/ITL_Hero_Shoot
# 不拉 Point-LIO / reloc / Gazebo：ITL 那三段依赖仿真雷达，本机吃性能。
# 吊射执行与 ITL 相同：静态 TF + manager + RViz 看可视化。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('heavy_lob_shot_manager')
    default_config = os.path.join(pkg, 'config', 'dry_run.yaml')
    map_yaml = os.path.join(pkg, 'resource', 'maps', 'rmuc_2026.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'itl_shoot.rviz')

    remappings = []

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='吊射配置（默认 dry_run.yaml：蓝方基地中间偏平大装甲）',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='是否启动 RViz',
    )

    stl = os.path.join(pkg, 'resource', 'models', 'rmuc_2026', 'meshes', 'rmuc_2026.stl')

    # 主吊射位：梯形高地附近平地，比基地地面高约 200 mm，不上梯形高地本体。
    # 高地顶 STL (4.583, 11.849, 1.028)=地面+400 mm（2026 §4.3.1）；车开不上去。
    # 站位 STL 高地东侧 z≈0.828 平台：地面 0.628 + 200 mm，距高心约 1.85 m。
    # 枪口仍相对底盘 +0.4 m。不是 ITL 出生 (2.6, 6.5, 0)，也不是停机坪 (1.881, 13.649, 1.340)。
    map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base',
        arguments=['6.429', '11.690', '0.828', '0', '0', '0', '1', 'map', 'base_link'],
    )
    base_to_muzzle = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_muzzle',
        arguments=['0', '0', '0.4', '0', '0', '0', '1', 'base_link', 'muzzle'],
    )
    # PolarBear simulation_robot：chassis→front_mid360 = 0.16 0 0.18，pitch=15°
    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar',
        arguments=[
            '0.16', '0', '0.18',
            '0', '0.13052619222', '0', '0.99144486137',
            'base_link', 'front_mid360',
        ],
    )

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

    field_cloud = Node(
        package='heavy_lob_shot_manager',
        executable='field_cloud.py',
        name='field_cloud',
        parameters=[{
            'stl_path': stl,
            'leaf_size': 0.05,
            'az_samples': 625,
            'el_samples': 32,
            'hz': 10.0,
            'range_noise': 0.02,
        }],
        output='screen',
    )
    reloc = Node(
        package='heavy_lob_shot_manager',
        executable='reloc_icp.py',
        name='reloc_icp',
        output='screen',
    )

    start_lob_shot = Node(
        package='heavy_lob_shot_manager',
        executable='heavy_lob_shot_manager',
        name='heavy_lob_shot_manager',
        parameters=[LaunchConfiguration('config_file')],
        remappings=remappings,
        output='screen',
    )

    start_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': False}],
        remappings=remappings,
    )

    # ITL 也是 trigger 才进 WAITING_TF；本队话题是 /lob_shot/trigger。
    trigger = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/lob_shot/trigger', 'std_msgs/msg/Bool', '{data: true}',
                ],
                output='screen',
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('ROS_DOMAIN_ID', '31'))
    ld.add_action(declare_config_file)
    ld.add_action(declare_rviz)
    ld.add_action(map_to_base)
    ld.add_action(base_to_muzzle)
    ld.add_action(base_to_lidar)
    ld.add_action(field_cloud)
    ld.add_action(reloc)
    ld.add_action(map_server)
    ld.add_action(map_life)
    ld.add_action(start_lob_shot)
    ld.add_action(start_rviz)
    ld.add_action(trigger)
    return ld
