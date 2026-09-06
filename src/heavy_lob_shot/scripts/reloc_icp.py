#!/usr/bin/python3
"""重定位实验：先验 /prior_map vs 扫描 /registered_scan。

循环抄 ITL relocalization.cpp::performRegistration
（voxel → align → 打 conv/err/xy/yaw）。
本机没有 ITL 的 small_gicp / utils，配准用点到点 ICP（Besl SVD + scipy KDTree）。
故意用偏差初值，看能否回到 Identity（扫描已在 map）。
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    x = raw[:, off['x'] : off['x'] + 4].view(np.float32).reshape(-1)
    y = raw[:, off['y'] : off['y'] + 4].view(np.float32).reshape(-1)
    z = raw[:, off['z'] : off['z'] + 4].view(np.float32).reshape(-1)
    pts = np.stack([x, y, z], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


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


def voxel(pts: np.ndarray, leaf: float) -> np.ndarray:
    if pts.size == 0:
        return pts
    q = np.floor(pts / leaf).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[idx]


def make_T(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    T[0, 3], T[1, 3] = x, y
    return T


def apply_T(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def icp_svd(
    source: np.ndarray,
    tree: cKDTree,
    target: np.ndarray,
    T: np.ndarray,
    max_dist: float,
    max_iters: int,
) -> tuple[np.ndarray, float, bool, int]:
    # ITL：rejector.max_dist + optimizer.max_iterations=10
    last_err = 1e9
    it = 0
    for it in range(1, max_iters + 1):
        src_t = apply_T(source, T)
        dist, idx = tree.query(src_t, workers=-1)
        keep = dist < max_dist
        if int(keep.sum()) < 50:
            return T, float(last_err), False, it
        a = src_t[keep]
        b = target[idx[keep]]
        ca = a.mean(axis=0)
        cb = b.mean(axis=0)
        aa = a - ca
        bb = b - cb
        u, _, vt = np.linalg.svd(aa.T @ bb)
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = vt.T @ u.T
        t = cb - r @ ca
        dT = np.eye(4)
        dT[:3, :3] = r
        dT[:3, 3] = t
        T = dT @ T
        err = float(dist[keep].mean())
        if abs(last_err - err) < 1e-4:
            return T, err, True, it
        last_err = err
    return T, float(last_err), True, it


class RelocIcp(Node):
    def __init__(self) -> None:
        super().__init__('reloc_icp')
        self.declare_parameter('leaf_size', 0.12)  # ITL Simulation.yaml registered_leaf_size
        self.declare_parameter('max_dist', 6.0)  # sqrt(ITL max_dist_sq=36)
        self.declare_parameter('max_iters', 10)
        self.declare_parameter('seed_xy', [1.2, 0.8])
        self.declare_parameter('seed_yaw_deg', 12.0)
        self.declare_parameter('map_frame', 'map')

        self.leaf = float(self.get_parameter('leaf_size').value)
        self.max_dist = float(self.get_parameter('max_dist').value)
        self.max_iters = int(self.get_parameter('max_iters').value)
        xy = [float(v) for v in self.get_parameter('seed_xy').value]
        yaw = float(self.get_parameter('seed_yaw_deg').value) * np.pi / 180.0
        self.T_seed = make_T(xy[0], xy[1], yaw)
        self.map_frame = str(self.get_parameter('map_frame').value)

        self.target: np.ndarray | None = None
        self.tree: cKDTree | None = None
        self.acc: list[np.ndarray] = []

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
        self.create_subscription(PointCloud2, '/prior_map', self._on_map, map_qos)
        self.create_subscription(PointCloud2, '/registered_scan', self._on_scan, sensor_qos)
        self.seeded_pub = self.create_publisher(PointCloud2, '/reloc/scan_seeded', 5)
        self.aligned_pub = self.create_publisher(PointCloud2, '/reloc/aligned', 5)
        self.pose_pub = self.create_publisher(PoseStamped, '/reloc/pose', 5)
        self.err_pub = self.create_publisher(Float32MultiArray, '/reloc/error', 5)

        # ITL reloc 定时器 2 Hz
        self.create_timer(0.5, self._align)
        self.get_logger().info(
            f'reloc_icp seed xy=({xy[0]:.2f},{xy[1]:.2f}) '
            f'yaw={float(self.get_parameter("seed_yaw_deg").value):.1f}deg '
            f'leaf={self.leaf} max_dist={self.max_dist}'
        )

    def _on_map(self, msg: PointCloud2) -> None:
        pts = voxel(cloud_xyz(msg), self.leaf)
        if pts.shape[0] < 100:
            return
        self.target = pts
        self.tree = cKDTree(pts)
        self.get_logger().info(f'reloc target {pts.shape[0]} pts')

    def _on_scan(self, msg: PointCloud2) -> None:
        pts = cloud_xyz(msg)
        if pts.shape[0] > 0:
            self.acc.append(pts)

    def _align(self) -> None:
        # 抄 ITL performRegistration：空累积则返回；voxel source；align；清空
        if self.tree is None or self.target is None or not self.acc:
            return
        src = voxel(np.concatenate(self.acc, axis=0), self.leaf)
        self.acc.clear()
        if src.shape[0] < 50:
            return
        T, err, conv, it = icp_svd(
            src, self.tree, self.target, self.T_seed.copy(), self.max_dist, self.max_iters
        )
        yaw = float(np.arctan2(T[1, 0], T[0, 0]))
        dx, dy = float(T[0, 3]), float(T[1, 3])
        seed_yaw = float(np.arctan2(self.T_seed[1, 0], self.T_seed[0, 0]))
        self.get_logger().info(
            f'reloc conv={int(conv)} iter={it} err={err:.4f} '
            f'out_xy=({dx:.3f},{dy:.3f}) out_yaw={yaw:.4f}rad '
            f'seed_xy=({self.T_seed[0,3]:.2f},{self.T_seed[1,3]:.2f}) seed_yaw={seed_yaw:.4f} '
            f'src={src.shape[0]}'
        )
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.map_frame
        self.seeded_pub.publish(xyz_cloud(apply_T(src, self.T_seed), hdr))
        self.aligned_pub.publish(xyz_cloud(apply_T(src, T), hdr))
        ps = PoseStamped()
        ps.header = hdr
        ps.pose.position.x = dx
        ps.pose.position.y = dy
        ps.pose.orientation.z = np.sin(yaw / 2.0)
        ps.pose.orientation.w = np.cos(yaw / 2.0)
        self.pose_pub.publish(ps)
        ev = Float32MultiArray()
        ev.data = [dx, dy, yaw, err, float(int(conv))]
        self.err_pub.publish(ev)


def main() -> None:
    rclpy.init()
    node = RelocIcp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
