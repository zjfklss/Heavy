#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import threading
import queue
import struct
import time

MAX_FRAME_SIZE = 65535
MAGIC_HEADER_SIZE = 4
MAX_CHUNK_DATA = 298
BUFFER_RESET_TIMEOUT = 2.0


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')

        self.declare_parameter('topic', '/from_custom_client')
        self.declare_parameter('display', True)
        self.declare_parameter('encode_width', 120)
        self.declare_parameter('encode_height', 120)
        self.declare_parameter('display_scale', 3)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)

        topic = self.get_parameter('topic').value
        self.display = self.get_parameter('display').value
        self.encode_width = int(self.get_parameter('encode_width').value)
        self.encode_height = int(self.get_parameter('encode_height').value)
        self.display_scale = max(1, int(self.get_parameter('display_scale').value))
        self.display_width = self.encode_width * self.display_scale
        self.display_height = self.encode_height * self.display_scale
        self.crosshair_offset_x = int(self.get_parameter('crosshair_offset_x').value)
        self.crosshair_offset_y = int(self.get_parameter('crosshair_offset_y').value)
        self.crosshair_width = max(1, int(self.get_parameter('crosshair_width').value))

        self.raw_queue = queue.Queue(maxsize=60)

        self._lock = threading.Lock()
        self._frame_count = 0
        self._packet_count = 0
        self._drop_count = 0
        self._decode_err_count = 0
        self._sync_loss_count = 0
        self._last_frame_time = time.monotonic()

        self._frame_buffer = bytearray()
        self._last_buffer_activity = time.monotonic()

        if self.display:
            self.frame_queue = queue.Queue(maxsize=3)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

        self.assembler_thread = threading.Thread(target=self._assembler_loop, daemon=True)
        self.assembler_thread.start()

        self.subscription = self.create_subscription(
            String,
            topic,
            self._packet_callback,
            10
        )

        self.create_timer(1.0, self._print_stats)
        self.get_logger().info(
            f'Decoder started: topic={topic} encode={self.encode_width}x{self.encode_height} '
            f'display={self.display_width}x{self.display_height} scale={self.display_scale}x'
        )

    def _packet_callback(self, msg):
        with self._lock:
            self._packet_count += 1
        raw = msg.data.encode('latin1')
        if len(raw) < 2:
            return
        chunk_len = raw[0] | (raw[1] << 8)
        if chunk_len == 0 or chunk_len > MAX_CHUNK_DATA or chunk_len + 2 > len(raw):
            return
        chunk = raw[2:2 + chunk_len]
        try:
            self.raw_queue.put_nowait(chunk)
        except queue.Full:
            with self._lock:
                self._drop_count += 1

    def _extract_frames_from_buffer(self):
        while True:
            if len(self._frame_buffer) < MAGIC_HEADER_SIZE:
                break

            total_size = struct.unpack_from("<I", self._frame_buffer, 0)[0]

            if total_size > MAX_FRAME_SIZE:
                self.get_logger().warn(f'Bogus frame size={total_size}, resetting buffer')
                self._frame_buffer.clear()
                with self._lock:
                    self._sync_loss_count += 1
                break

            needed = MAGIC_HEADER_SIZE + total_size
            if len(self._frame_buffer) < needed:
                break

            webp_data = bytes(self._frame_buffer[MAGIC_HEADER_SIZE:needed])
            del self._frame_buffer[:needed]
            self._last_buffer_activity = time.monotonic()

            try:
                img = cv2.imdecode(np.frombuffer(webp_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None and img.size > 0:
                    with self._lock:
                        self._frame_count += 1
                        self._last_frame_time = time.monotonic()
                    if self.display:
                        try:
                            self.frame_queue.put_nowait(img)
                        except queue.Full:
                            try:
                                self.frame_queue.get_nowait()
                            except queue.Empty:
                                pass
                            self.frame_queue.put_nowait(img)
            except Exception as e:
                with self._lock:
                    self._decode_err_count += 1
                self.get_logger().warn(f'Decode error: {e}')

    def _assembler_loop(self):
        while rclpy.ok():
            chunks = []
            try:
                chunk = self.raw_queue.get(timeout=0.02)
                chunks.append(chunk)
            except queue.Empty:
                now = time.monotonic()
                if (now - self._last_buffer_activity > BUFFER_RESET_TIMEOUT
                        and len(self._frame_buffer) > 0):
                    self.get_logger().warn(
                        f'Buffer timeout {now - self._last_buffer_activity:.1f}s, '
                        f'discarding {len(self._frame_buffer)} bytes'
                    )
                    self._frame_buffer.clear()
                    self._last_buffer_activity = now
                continue

            while True:
                try:
                    chunks.append(self.raw_queue.get_nowait())
                except queue.Empty:
                    break

            for chunk in chunks:
                self._frame_buffer.extend(chunk)
            self._last_buffer_activity = time.monotonic()

            self._extract_frames_from_buffer()

    def _print_stats(self):
        with self._lock:
            pkts = self._packet_count
            frames = self._frame_count
            drops = self._drop_count
            errs = self._decode_err_count
            sync_loss = self._sync_loss_count
            self._packet_count = 0
            self._frame_count = 0
            self._drop_count = 0
            self._decode_err_count = 0
            self._sync_loss_count = 0

        parts = [f'pkts={pkts}', f'fps={frames}', f'buf={len(self._frame_buffer)}B']
        if drops:
            parts.append(f'drops={drops}')
        if errs:
            parts.append(f'dec_err={errs}')
        if sync_loss:
            parts.append(f'sync_loss={sync_loss}')
        self.get_logger().info(f'Stats: {", ".join(parts)}')

    def _display_loop(self):
        cv2.namedWindow('Doorlock Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Doorlock Decoder', self.display_width, self.display_height)

        while rclpy.ok():
            latest = None
            try:
                latest = self.frame_queue.get(timeout=0.016)
            except queue.Empty:
                continue

            while True:
                try:
                    latest = self.frame_queue.get_nowait()
                except queue.Empty:
                    break

            if latest is None or latest.size == 0:
                continue

            img_disp = cv2.resize(
                latest,
                (self.display_width, self.display_height),
                interpolation=cv2.INTER_LANCZOS4
            )
            self._draw_overlay(img_disp)
            cv2.imshow('Doorlock Decoder', img_disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        h, w = img.shape[:2]
        cx = max(0, min(w - 1, w // 2 + self.crosshair_offset_x))
        cy = max(0, min(h - 1, h // 2 + self.crosshair_offset_y))
        cv2.line(img, (0, cy), (w - 1, cy), (230, 190, 235), self.crosshair_width, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), (230, 190, 235), self.crosshair_width, cv2.LINE_AA)
        cv2.circle(img, (w // 2, h // 2), 24, (170, 255, 170), 1, cv2.LINE_AA)

    def destroy_node(self):
        if self.display:
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self.display_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
