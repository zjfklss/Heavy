from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_test',
            executable='serial_test_node',
            name='serial_test',
            parameters=[{
                'device_name': '/dev/ttyUSB0',  # 根据实际串口设备修改
                'baud_rate': 921600,
                'debug': True,
            }],
            output='screen'
        )
    ])