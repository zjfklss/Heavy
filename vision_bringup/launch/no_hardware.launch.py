import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('vision_bringup'), 'launch'))

def generate_launch_description():
    
    from common import node_params, launch_params, declare_use_serial_cmd , declare_use_video_cmd, robot_state_publisher
    from common import armor_processor_node, joint_state_publisher, serial_driver_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    import yaml

    # load params for composable node
    with open(node_params, 'r') as f:
        camera_params = yaml.safe_load(f)['/camera_node']['ros__parameters']
    with open(node_params, 'r') as f:
        armor_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[camera_params],
    )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
            camera_node,
            ComposableNode(
                package='armor_detector',
                plugin='rm_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[armor_params],
            )
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
    )

    if (launch_params['camera'] == 'hik'):
        hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (launch_params['camera'] == 'mv'):
        mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')
        cam_detector = get_camera_detector_container(mv_camera_node)
    elif (launch_params['camera'] == 'galaxy'):
        galaxy_camera_node = get_camera_node('galaxy_camera', 'galaxy_camera::GxCamera')
        cam_detector = get_camera_detector_container(galaxy_camera_node)

    delay_serial_node = TimerAction(
        period = 1.0,
        actions=[serial_driver_node],
    )

    delay_processor_node = TimerAction(
        period = 1.5,
        actions=[armor_processor_node],
    )

    return LaunchDescription([
        declare_use_serial_cmd,
        declare_use_video_cmd,
        robot_state_publisher,
        joint_state_publisher,
        cam_detector,
        delay_serial_node,
        delay_processor_node,
    ])
    

    