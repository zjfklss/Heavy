#!/usr/bin/python3
"""Gazebo 真值位姿 → 墙钟 TF / 话题。仿真时钟从 0 起，本机节点用墙钟。"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import rclpy


def _static(parent: str, child: str, x: float, y: float, z: float) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.w = 1.0
    return t


class GtTf(Node):
    def __init__(self) -> None:
        super().__init__('gt_tf')
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.known_frames = {
            'map', 'odom', 'base_link', 'base_footprint', 'chassis', 'muzzle', 'livox_frame',
            'camera_link', 'pitch_link', 'front_mid360', 'front_industrial_camera',
            'front_industrial_camera_optical_frame', 'front_rplidar_a2',
        }
        # 偏移按 PolarBear simulation_robot：base_footprint→chassis 0.063，
        # yaw 0.1376 + pitch 0.172，speed_monitor 0.07 + 枪口 0.15；Mid360 在 chassis (0.16,0,0.18)
        self.statics = [
            _static('map', 'odom', 0.0, 0.0, 0.0),
            _static('base_link', 'base_footprint', 0.0, 0.0, 0.0),
            _static('base_link', 'chassis', 0.0, 0.0, 0.063),
            _static('base_link', 'muzzle', 0.22, 0.0, 0.373),
            _static('base_link', 'livox_frame', 0.16, 0.0, 0.243),
            _static('base_link', 'pitch_link', 0.0, 0.0, 0.373),
            _static('pitch_link', 'camera_link', 0.10, 0.0, 0.045),
        ]
        now = self.get_clock().now().to_msg()
        for s in self.statics:
            s.header.stamp = now
        self.static_tf.sendTransform(self.statics)

        self.have_odom = False
        self.last_odom = TransformStamped()
        self.last_odom.header.frame_id = 'odom'
        self.last_odom.child_frame_id = 'base_link'
        self.last_odom.transform.translation.x = 2.6
        self.last_odom.transform.translation.y = 6.5
        self.last_odom.transform.translation.z = 0.75
        self.last_odom.transform.rotation.w = 1.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.cloud_pub = self.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_rel)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)

        self.create_subscription(Odometry, '/odom_gz', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/livox/lidar_gz', self.on_cloud, qos_rel)
        self.create_subscription(LaserScan, '/scan_gz', self.on_scan, qos_rel)
        self.create_subscription(Image, '/camera/image_gz', self.on_image, qos_rel)
        self.create_timer(0.02, self.tick)
        self.get_logger().info('gt_tf: wall-clock stamps; map==odom')

    def now_msg(self):
        return self.get_clock().now().to_msg()

    def adopt_frame(self, frame_id: str, parent: str, x: float, y: float, z: float) -> None:
        if not frame_id or frame_id in self.known_frames:
            return
        self.known_frames.add(frame_id)
        t = _static(parent, frame_id, x, y, z)
        t.header.stamp = self.now_msg()
        self.statics.append(t)
        self.static_tf.sendTransform(self.statics)
        self.get_logger().info(f'adopt sensor frame {parent} -> {frame_id}')

    def on_cloud(self, msg: PointCloud2) -> None:
        self.adopt_frame(msg.header.frame_id, 'base_link', 0.05, 0.0, 0.26)
        msg.header.stamp = self.now_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'livox_frame'
        self.cloud_pub.publish(msg)

    def on_scan(self, msg: LaserScan) -> None:
        self.adopt_frame(msg.header.frame_id, 'base_link', 0.05, 0.0, 0.26)
        msg.header.stamp = self.now_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'livox_frame'
        self.scan_pub.publish(msg)

    def on_image(self, msg: Image) -> None:
        self.adopt_frame(msg.header.frame_id, 'camera_link', 0.0, 0.0, 0.0)
        msg.header.stamp = self.now_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = 'camera_link'
        self.image_pub.publish(msg)

    def on_odom(self, msg: Odometry) -> None:
        stamp = self.now_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.last_odom = t
        self.have_odom = True
        self.tf.sendTransform(t)

    def tick(self) -> None:
        if self.have_odom:
            return
        self.last_odom.header.stamp = self.now_msg()
        self.tf.sendTransform(self.last_odom)


def main() -> None:
    rclpy.init()
    node = GtTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
