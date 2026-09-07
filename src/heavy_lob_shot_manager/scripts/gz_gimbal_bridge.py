#!/usr/bin/python3
"""把本队 /gimbal_command (Vector3) 转成 Gazebo JointPositionController 的 Double 话题。"""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class GzGimbalBridge(Node):
    def __init__(self):
        super().__init__('gz_gimbal_bridge')
        self.yaw_pub = self.create_publisher(Float64, '/yaw_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/pitch_cmd', 10)
        self.create_subscription(
            Vector3, '/gimbal_command', self.on_cmd, qos_profile_sensor_data
        )
        self.get_logger().info('gz_gimbal_bridge: /gimbal_command -> /yaw_cmd /pitch_cmd')

    def on_cmd(self, msg: Vector3) -> None:
        pitch = Float64()
        yaw = Float64()
        pitch.data = float(msg.x)
        yaw.data = float(msg.y)
        self.pitch_pub.publish(pitch)
        self.yaw_pub.publish(yaw)


def main() -> None:
    rclpy.init()
    node = GzGimbalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
