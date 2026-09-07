#!/usr/bin/python3
"""官方 RMUC STL → RViz 三维场地 + 仿真 Mid360 扫描。

先验地图发布抄 ITL relocalization.cpp：transient_local /prior_map。
扫描按 Livox Mid-360 官方点频：200000 点/s、典型 10 Hz → 约 20000 点/帧，
视场 360° × (−7°~+52°)，等价约 32 线 × 625 方位（不是 PolarBear 降采样的 180×16）。
测距噪声 2 cm（手册 1σ @10 m）。不跑 Gazebo。
https://www.livoxtech.com/mid-360/specs
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


def read_binary_stl(path: Path) -> np.ndarray:
    # 抄自 scripts/stl_to_pgm.py
    data = path.read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    verts = np.empty((n, 3, 3), dtype=np.float32)
    off = 84
    for i in range(n):
        verts[i] = np.frombuffer(data, dtype=np.float32, count=9, offset=off + 12).reshape(3, 3)
        off += 50
    return verts


def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    q = np.floor(pts / leaf).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[idx]


def sample_stl_surface(verts: np.ndarray, spacing: float, seed: int = 0) -> np.ndarray:
    a, b, c = verts[:, 0], verts[:, 1], verts[:, 2]
    ab = b - a
    ac = c - a
    area = 0.5 * np.linalg.norm(np.cross(ab, ac), axis=1)
    n_extra = np.ceil(area / (0.5 * spacing * spacing)).astype(np.int32)
    n_extra = np.clip(n_extra, 0, 64)
    parts = [a, b, c, (a + b + c) / 3.0]
    if int(n_extra.sum()) == 0:
        return np.concatenate(parts, axis=0)
    idx = np.repeat(np.arange(len(verts)), n_extra)
    rng = np.random.default_rng(seed)
    r = rng.random(idx.shape[0])
    s = rng.random(idx.shape[0])
    flip = (r + s) > 1.0
    r = np.where(flip, 1.0 - r, r)
    s = np.where(flip, 1.0 - s, s)
    parts.append((1.0 - r - s)[:, None] * a[idx] + r[:, None] * b[idx] + s[:, None] * c[idx])
    return np.concatenate(parts, axis=0)


def xyz_cloud(pts: np.ndarray, header: Header) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
    return msg


def _as_mat(tf: TransformStamped) -> tuple[np.ndarray, np.ndarray]:
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return r, np.array([t.x, t.y, t.z], dtype=np.float64)


class FieldCloud(Node):
    def __init__(self) -> None:
        super().__init__('field_cloud')
        self.declare_parameter('stl_path', '')
        self.declare_parameter('mesh_pose', [14.5, 8.0, 0.2])
        self.declare_parameter('leaf_size', 0.05)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_frame', 'front_mid360')
        self.declare_parameter('hz', 10.0)
        # Mid-360：200000 点/s ÷ 10 Hz ≈ 20000/帧 ≈ 32 线 × 625 方位
        self.declare_parameter('az_samples', 625)
        self.declare_parameter('el_samples', 32)
        self.declare_parameter('el_min', -0.12217304764)
        self.declare_parameter('el_max', 0.90757121104)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 40.0)
        self.declare_parameter('range_noise', 0.02)

        stl_s = str(self.get_parameter('stl_path').value)
        if not stl_s:
            raise RuntimeError('field_cloud: stl_path 必须由 launch 传入')
        stl = Path(stl_s)
        pose = [float(x) for x in self.get_parameter('mesh_pose').value]
        leaf = float(self.get_parameter('leaf_size').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.az_n = int(self.get_parameter('az_samples').value)
        self.el_n = int(self.get_parameter('el_samples').value)
        self.el_min = float(self.get_parameter('el_min').value)
        self.el_max = float(self.get_parameter('el_max').value)
        self.r_min = float(self.get_parameter('range_min').value)
        self.r_max = float(self.get_parameter('range_max').value)
        self.range_noise = float(self.get_parameter('range_noise').value)
        self.rng = np.random.default_rng(1)

        verts = read_binary_stl(stl)
        samples = sample_stl_surface(verts, leaf) + np.array(pose, dtype=np.float32)
        self.map_pts = voxel_downsample(samples.astype(np.float64), leaf)
        self.stl_uri = 'file://' + str(stl.resolve())
        self.mesh_pose = pose

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # ITL relocalization.cpp: prior_map transient_local，只发一次后来者也能收到
        self.prior_pub = self.create_publisher(PointCloud2, '/prior_map', map_qos)
        self.scan_pub = self.create_publisher(PointCloud2, '/registered_scan', sensor_qos)
        self.livox_pub = self.create_publisher(PointCloud2, '/livox/lidar', sensor_qos)
        self.mesh_pub = self.create_publisher(Marker, '/lob_shot/field_mesh', map_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._publish_prior()
        self._publish_mesh()
        hz = float(self.get_parameter('hz').value)
        self.create_timer(1.0 / max(hz, 1.0), self._on_tick)
        self.create_timer(2.0, self._publish_prior)
        self.get_logger().info(
            f'field_cloud prior={self.map_pts.shape[0]} leaf={leaf} stl={stl}'
        )

    def _header(self, frame: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame
        return h

    def _publish_prior(self) -> None:
        self.prior_pub.publish(xyz_cloud(self.map_pts, self._header(self.map_frame)))

    def _publish_mesh(self) -> None:
        m = Marker()
        m.header = self._header(self.map_frame)
        m.ns = 'field'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x = self.mesh_pose[0]
        m.pose.position.y = self.mesh_pose[1]
        m.pose.position.z = self.mesh_pose[2]
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color.r = 0.72
        m.color.g = 0.72
        m.color.b = 0.74
        m.color.a = 0.35
        m.mesh_resource = self.stl_uri
        m.mesh_use_embedded_materials = False
        m.frame_locked = True
        self.mesh_pub.publish(m)

    def _on_tick(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.lidar_frame, rclpy.time.Time()
            )
        except Exception:
            return
        r, t = _as_mat(tf)
        local = (self.map_pts - t) @ r
        rng = np.linalg.norm(local, axis=1)
        ok = (rng > self.r_min) & (rng < self.r_max)
        if not np.any(ok):
            return
        loc = local[ok]
        world = self.map_pts[ok]
        rng = rng[ok]
        az = np.arctan2(loc[:, 1], loc[:, 0])
        el = np.arcsin(np.clip(loc[:, 2] / rng, -1.0, 1.0))
        fov = (el >= self.el_min) & (el <= self.el_max)
        if not np.any(fov):
            return
        loc = loc[fov]
        world = world[fov]
        rng = rng[fov]
        az = az[fov]
        el = el[fov]
        az_i = np.floor((az + np.pi) / (2.0 * np.pi) * self.az_n).astype(np.int32)
        az_i = np.clip(az_i, 0, self.az_n - 1)
        el_i = np.floor((el - self.el_min) / (self.el_max - self.el_min) * self.el_n).astype(
            np.int32
        )
        el_i = np.clip(el_i, 0, self.el_n - 1)
        bins = el_i * self.az_n + az_i
        order = np.argsort(rng, kind='mergesort')
        _, first = np.unique(bins[order], return_index=True)
        pick = order[first]
        loc_h = loc[pick]
        world_h = world[pick]
        if self.range_noise > 0.0:
            nse = self.rng.normal(0.0, self.range_noise, size=len(pick))
            scale = 1.0 + nse / np.maximum(rng[pick], 1e-3)
            loc_h = loc_h * scale[:, None]
            world_h = t + loc_h @ r.T
        self.scan_pub.publish(xyz_cloud(world_h, self._header(self.map_frame)))
        self.livox_pub.publish(xyz_cloud(loc_h, self._header(self.lidar_frame)))


def main() -> None:
    rclpy.init()
    node = FieldCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
