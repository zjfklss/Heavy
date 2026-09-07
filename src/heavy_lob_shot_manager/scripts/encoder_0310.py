#!/usr/bin/python3
"""相机 Image → 0x0310 300B 分包 → MQTT CustomByteBlock。

仿真里帧内容用 JPEG（OpenCV 能解），外层仍是本队 300B/有效最多 298B 的 0x0310 分包。
"""

from __future__ import annotations

import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

SERIAL_PKT_SIZE = 300
MAX_CHUNK_DATA = 298


def pack_0310_chunk(payload: bytes) -> bytes:
    if len(payload) > MAX_CHUNK_DATA:
        payload = payload[:MAX_CHUNK_DATA]
    body = struct.pack('<H', len(payload)) + payload
    return body + b'\x00' * (SERIAL_PKT_SIZE - len(body))


def mqtt_client():
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, 'CallbackAPIVersion'):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    return mqtt.Client()


def image_to_bgr(msg: Image) -> np.ndarray | None:
    enc = (msg.encoding or '').lower()
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if arr.size < msg.height * max(msg.step, 1):
        return None
    row = arr[: msg.height * msg.step].reshape(msg.height, msg.step)
    if enc in ('rgb8', 'bgr8'):
        img = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img
    if enc in ('rgba8', 'bgra8'):
        img = row[:, : msg.width * 4].reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    return None


class Encoder0310(Node):
    def __init__(self) -> None:
        super().__init__('encoder_0310')
        self.broker = self.declare_parameter('mqtt_host', '127.0.0.1').get_parameter_value().string_value
        self.port = self.declare_parameter('mqtt_port', 1883).get_parameter_value().integer_value
        self.topic = self.declare_parameter('mqtt_topic', 'CustomByteBlock').get_parameter_value().string_value
        self.client = None
        try:
            self.client = mqtt_client()
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self.get_logger().info(f'0310 encoder MQTT {self.broker}:{self.port} {self.topic}')
        except Exception as exc:
            self.get_logger().error(f'MQTT connect failed: {exc}')
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)

    def publish_bytes(self, raw: bytes) -> None:
        if self.client is None:
            return
        offset = 0
        while offset < len(raw):
            piece = raw[offset:offset + MAX_CHUNK_DATA]
            offset += len(piece)
            self.client.publish(self.topic, pack_0310_chunk(piece), qos=0)

    def on_image(self, msg: Image) -> None:
        if self.client is None:
            return
        img = image_to_bgr(msg)
        if img is None:
            return
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
        if ok:
            self.publish_bytes(buf.tobytes())


def main() -> None:
    rclpy.init()
    node = Encoder0310()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
