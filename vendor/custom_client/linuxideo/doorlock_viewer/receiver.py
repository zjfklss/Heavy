import threading
import queue
import struct
import time
import numpy as np
import cv2
import paho.mqtt.client as mqtt
from PySide6.QtCore import QObject, Signal

import sys
sys.path.insert(0, ".")
import CustomControl_pb2
import DeployModeStatusSync_pb2
import custom_byte_block_pb2

MAX_FRAME_SIZE = 65535
MAGIC_HEADER_SIZE = 4
MAX_CHUNK_DATA = 298
BUFFER_RESET_TIMEOUT = 2.0
SERIAL_PKT_SIZE = 300


class FrameReceiver(QObject):
    frame_ready = Signal(np.ndarray)
    stats_updated = Signal(dict)
    connected = Signal()
    disconnected = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._host = "192.168.12.1"
        self._port = 3333
        self._encode_width = 120
        self._encode_height = 120
        self._stop_event = threading.Event()

        self.raw_queue = queue.Queue(maxsize=60)
        self._frame_buffer = bytearray()
        self._last_buffer_activity = time.monotonic()

        self._lock = threading.Lock()
        self._cbb_count = 0
        self._cbb_short = 0
        self._cbb_parse_err = 0
        self._cbb_ident = 0
        self._cbb_payload_bytes = 0
        self._pkt_count = 0
        self._pkt_bytes = 0
        self._frame_count = 0
        self._drop_count = 0
        self._decode_err_count = 0
        self._sync_loss_count = 0
        self._last_stats_time = time.monotonic()
        self._last_frame_time = time.monotonic()

        self._init_mqtt()

    def _init_mqtt(self):
        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION1, client_id="doorlock_viewer"
        )
        self.mqtt_client.max_inflight_messages_set(100)
        self.mqtt_client.max_queued_messages_set(0)
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.mqtt_client.on_connect = self._on_connect #绑定连接回调函数
        self.mqtt_client.on_disconnect = self._on_disconnect #绑定断开连接时的回调函数
        self.mqtt_client.on_message = self._on_message #绑定接收信息时的回调函数

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            return
        self.connected.emit()
        client.subscribe("CustomByteBlock", qos=1)
        client.subscribe("DeployModeStatusSync", qos=0)

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.disconnected.emit()

    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic == "DeployModeStatusSync":
                self._handle_deploy_status(msg.payload)
                return

            if msg.topic != "CustomByteBlock":
                return

            with self._lock:
                self._cbb_count += 1

            try:
                custom_data = custom_byte_block_pb2.CustomByteBlock()
                custom_data.ParseFromString(msg.payload)
            except Exception:
                with self._lock:
                    self._cbb_parse_err += 1
                return

            raw = custom_data.data
            payload_len = len(raw) if raw else 0
            with self._lock:
                self._cbb_payload_bytes += payload_len

            if not raw or payload_len < 2:
                with self._lock:
                    self._cbb_short += 1
                return

            if raw.startswith(b"Robot "):
                with self._lock:
                    self._cbb_ident += 1
                return

            chunk_len = raw[0] | (raw[1] << 8)
            if chunk_len == 0 or chunk_len > MAX_CHUNK_DATA or chunk_len + 2 > payload_len:
                return
            chunk = bytes(raw[2:2 + chunk_len])

            with self._lock:
                self._pkt_count += 1
                self._pkt_bytes += chunk_len

            try:
                self.raw_queue.put_nowait(chunk)
            except queue.Full:
                with self._lock:
                    self._drop_count += 1
        except Exception:
            pass

    def _handle_deploy_status(self, payload):
        status_pb = DeployModeStatusSync_pb2.DeployModeStatusSync()
        status_pb.ParseFromString(payload)
        if status_pb.status in (0, 1):
            value = 1024 if status_pb.status == 1 else 0
            ctrl = CustomControl_pb2.CustomControl()
            ctrl.data = value.to_bytes(2, "big")
            self.mqtt_client.publish("CustomControl", ctrl.SerializeToString(), qos=1)

    def _extract_frames_from_buffer(self):
        while True:
            if len(self._frame_buffer) < MAGIC_HEADER_SIZE:
                break

            total_size = struct.unpack_from("<I", self._frame_buffer, 0)[0]

            if total_size > MAX_FRAME_SIZE:
                self._frame_buffer.clear()
                with self._lock:
                    self._sync_loss_count += 1
                break

            needed = MAGIC_HEADER_SIZE + total_size
            if len(self._frame_buffer) < needed:
                break

            img_data = bytes(self._frame_buffer[MAGIC_HEADER_SIZE:needed])
            del self._frame_buffer[:needed]
            self._last_buffer_activity = time.monotonic()

            try:
                img = cv2.imdecode(np.frombuffer(img_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None and img.size > 0:
                    with self._lock:
                        self._frame_count += 1
                        self._last_frame_time = time.monotonic()
                    self.frame_ready.emit(img)
            except Exception:
                with self._lock:
                    self._decode_err_count += 1

    def _assembler_loop(self):
        while not self._stop_event.is_set():
            chunks = []
            try:
                chunk = self.raw_queue.get(timeout=0.02)
                chunks.append(chunk)
            except queue.Empty:
                now = time.monotonic()
                if (now - self._last_buffer_activity > BUFFER_RESET_TIMEOUT
                        and len(self._frame_buffer) > 0):
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

    def _stats_loop(self):
        while not self._stop_event.is_set():
            time.sleep(1.0)
            now = time.monotonic()
            with self._lock:
                elapsed = now - self._last_stats_time
                if elapsed < 0.5:
                    continue
                stats = {
                    "fps": self._frame_count / elapsed,
                    "pkts": self._pkt_count,
                    "pkts_hz": self._pkt_count / elapsed,
                    "kbps": (self._pkt_bytes * 8) / elapsed / 1000,
                    "raw_q": self.raw_queue.qsize(),
                    "buf_bytes": len(self._frame_buffer),
                    "drops": self._drop_count,
                    "dec_err": self._decode_err_count,
                    "sync_loss": self._sync_loss_count,
                    "connected": True,
                    "host": self._host,
                    "port": self._port,
                    "encode_w": self._encode_width,
                    "encode_h": self._encode_height,
                }
                self._cbb_count = 0
                self._cbb_short = 0
                self._cbb_parse_err = 0
                self._cbb_ident = 0
                self._cbb_payload_bytes = 0
                self._pkt_count = 0
                self._pkt_bytes = 0
                self._frame_count = 0
                self._drop_count = 0
                self._decode_err_count = 0
                self._sync_loss_count = 0
                self._last_stats_time = now
            self.stats_updated.emit(stats)

    def apply_params(self, host, port, encode_width, encode_height):
        self._host = host
        self._port = port
        self._encode_width = encode_width
        self._encode_height = encode_height

    def start(self):
        self.mqtt_client.connect_async(self._host, self._port, keepalive=60)
        self.mqtt_client.loop_start()

        self._assembler_thread = threading.Thread(
            target=self._assembler_loop, name="assembler", daemon=True
        )
        self._assembler_thread.start()

        self._stats_thread = threading.Thread(
            target=self._stats_loop, name="stats", daemon=True
        )
        self._stats_thread.start()

    def stop(self):
        self._stop_event.set()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
