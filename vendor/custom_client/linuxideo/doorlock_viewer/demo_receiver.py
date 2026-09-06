import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import cv2
from PySide6.QtCore import QObject, Signal

MAX_CHUNK_DATA = 298


class DemoReceiver(QObject):
    """Simulates the full sender→link→receiver pipeline using a local video file.

    Reads frames from a recorded video, runs them through the actual WebP
    encode + serial-bandwidth-limited transmission model, then decodes and
    emits frames just like the real FrameReceiver. This faithfully reproduces
    the compression artifacts and frame-rate ceiling of the real link.

    Exposes the same Qt signal/method interface as FrameReceiver so MainWindow
    can use either interchangeably.
    """

    frame_ready = Signal(np.ndarray)
    stats_updated = Signal(dict)
    connected = Signal()
    disconnected = Signal()
    error = Signal(str)

    def __init__(self, video_path, encode_width=120, encode_height=120,
                 webp_quality=3, serial_hz=50, output_fps=60, parent=None):
        super().__init__(parent)
        self._video_path = str(video_path)
        self._encode_width = encode_width
        self._encode_height = encode_height
        self._webp_quality = webp_quality
        self._serial_hz = serial_hz
        self._output_fps = output_fps
        self._max_payload = MAX_CHUNK_DATA

        self._stop_event = threading.Event()
        self._queue = deque()
        self._queue_bytes = 0
        self._queue_lock = threading.Lock()

        self._lock = threading.Lock()
        self._pkt_count = 0
        self._pkt_bytes = 0
        self._frame_count = 0
        self._drop_count = 0
        self._enc_frames = 0
        self._enc_bytes = 0
        self._last_stats_time = time.monotonic()

    # --- compatibility shim with FrameReceiver ---
    def apply_params(self, host, port, encode_width, encode_height):
        self._encode_width = int(encode_width)
        self._encode_height = int(encode_height)

    def set_encode_params(self, webp_quality=None, serial_hz=None, output_fps=None):
        if webp_quality is not None:
            self._webp_quality = int(webp_quality)
        if serial_hz is not None:
            self._serial_hz = int(serial_hz)
        if output_fps is not None:
            self._output_fps = int(output_fps)

    def start(self):
        cap = cv2.VideoCapture(self._video_path)
        if not cap.isOpened():
            self.error.emit(f"Cannot open video: {self._video_path}")
            self.disconnected.emit()
            return
        cap.release()

        self.connected.emit()
        self._producer_thread = threading.Thread(
            target=self._producer_loop, name="demo_producer", daemon=True)
        self._transmit_thread = threading.Thread(
            target=self._transmit_loop, name="demo_transmit", daemon=True)
        self._stats_thread = threading.Thread(
            target=self._stats_loop, name="demo_stats", daemon=True)
        self._producer_thread.start()
        self._transmit_thread.start()
        self._stats_thread.start()

    def stop(self):
        self._stop_event.set()

    # --- sender side: read video, encode WebP, enqueue (drop oldest) ---
    def _preprocess(self, frame):
        h, w = frame.shape[:2]
        side = min(h, w)
        y0 = (h - side) // 2
        x0 = (w - side) // 2
        cropped = frame[y0:y0 + side, x0:x0 + side]
        return cv2.resize(
            cropped, (self._encode_width, self._encode_height),
            interpolation=cv2.INTER_AREA)

    def _producer_loop(self):
        cap = cv2.VideoCapture(self._video_path)
        src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        if src_fps <= 0:
            src_fps = 30.0
        frame_interval = 1.0 / src_fps
        encode_interval = 1.0 / max(self._output_fps, 1)
        last_encode = 0.0

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            now = time.monotonic()
            if now - last_encode >= encode_interval:
                last_encode = now
                small = self._preprocess(frame)
                ok, buf = cv2.imencode(
                    ".webp", small,
                    [cv2.IMWRITE_WEBP_QUALITY, self._webp_quality])
                if ok:
                    webp = buf.tobytes()
                    with self._lock:
                        self._enc_frames += 1
                        self._enc_bytes += len(webp)
                    with self._queue_lock:
                        self._queue.append(webp)
                        self._queue_bytes += len(webp) + 4
                        limit = self._max_payload * self._serial_hz
                        while self._queue_bytes > limit and len(self._queue) > 1:
                            old = self._queue.popleft()
                            self._queue_bytes -= len(old) + 4
                            with self._lock:
                                self._drop_count += 1

            elapsed = time.monotonic() - loop_start
            sleep_t = frame_interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        cap.release()

    # --- link side: serial-bandwidth-limited transmission model ---
    def _transmit_loop(self):
        interval = 1.0 / max(self._serial_hz, 1)
        sending = None
        sending_total = 0
        sending_offset = 0

        while not self._stop_event.is_set():
            tick_start = time.monotonic()

            if sending is None:
                with self._queue_lock:
                    if self._queue:
                        sending = self._queue.popleft()
                        self._queue_bytes -= len(sending) + 4
                        sending_total = len(sending) + 4  # 4-byte length header
                        sending_offset = 0

            if sending is not None:
                chunk = min(self._max_payload, sending_total - sending_offset)
                sending_offset += chunk
                with self._lock:
                    self._pkt_count += 1
                    self._pkt_bytes += chunk

                if sending_offset >= sending_total:
                    img = cv2.imdecode(
                        np.frombuffer(sending, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None and img.size > 0:
                        with self._lock:
                            self._frame_count += 1
                        self.frame_ready.emit(img)
                    sending = None

            elapsed = time.monotonic() - tick_start
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _stats_loop(self):
        while not self._stop_event.is_set():
            time.sleep(1.0)
            now = time.monotonic()
            with self._lock:
                elapsed = now - self._last_stats_time
                if elapsed < 0.5:
                    continue
                avg_frame = (self._enc_bytes / self._enc_frames) if self._enc_frames else 0
                stats = {
                    "fps": self._frame_count / elapsed,
                    "pkts": self._pkt_count,
                    "pkts_hz": self._pkt_count / elapsed,
                    "kbps": (self._pkt_bytes * 8) / elapsed / 1000,
                    "raw_q": len(self._queue),
                    "buf_bytes": self._queue_bytes,
                    "drops": self._drop_count,
                    "dec_err": 0,
                    "sync_loss": 0,
                    "connected": True,
                    "host": f"DEMO: {Path(self._video_path).name}",
                    "port": f"q={self._webp_quality}",
                    "encode_w": self._encode_width,
                    "encode_h": self._encode_height,
                    "avg_frame_bytes": avg_frame,
                }
                self._pkt_count = 0
                self._pkt_bytes = 0
                self._frame_count = 0
                self._drop_count = 0
                self._enc_frames = 0
                self._enc_bytes = 0
                self._last_stats_time = now
            self.stats_updated.emit(stats)
