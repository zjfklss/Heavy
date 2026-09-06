from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from pathlib import Path


def generate_launch_description():
    encode_size = 120
    display_scale = 3
    webp_quality = 3
    output_fps = 60
    serial_device = '/dev/ttyUSB0'
    serial_baud = 921600
    serial_hz = 50

    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[
                    {'exposure_time': 12000.0},
                    {'gain': 10.0}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[
                    {'input_topic': '/image_raw'},
                    {'encode_width': encode_size},
                    {'encode_height': encode_size},
                    {'webp_quality': webp_quality},
                    {'output_fps': output_fps},
                    {'output_size': 300},
                    {'enable_display': True},
                    {'crop_size': 800},
                    {'static_simplify': True},
                    {'motion_threshold': 14},
                    {'motion_erode_px': 2},
                    {'motion_dilate_px': 6},
                    {'bg_update_alpha': 0.01},
                    {'bg_blur_sigma': 1.8},
                    {'center_clear_size': 150},
                    {'force_monochrome': False},
                    {'serial_device': serial_device},
                    {'serial_baud_rate': serial_baud},
                    {'serial_send_hz': serial_hz},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    serial_node = Node(
        package='serial_test',
        executable='serial_test_node',
        name='serial_test',
        parameters=[
            {'device_name': serial_device},
            {'baud_rate': serial_baud},
            {'debug': False},
        ],
        output='screen',
    )

    decoder_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='video_decoder',
        parameters=[
            {'topic': '/from_custom_client'},
            {'display': True},
            {'encode_width': encode_size},
            {'encode_height': encode_size},
            {'display_scale': display_scale},
            {'crosshair_offset_x': 0},
            {'crosshair_offset_y': 0},
            {'crosshair_width': 1},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        encoder_container,
        serial_node,
        decoder_node
    ])
