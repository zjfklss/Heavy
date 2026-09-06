import queue
import threading
import time
import logging
import struct
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import CustomControl_pb2
import DeployModeStatusSync_pb2
import custom_byte_block_pb2
from opencv_qt_fix import ensure_cv2_qt_fontdir

ensure_cv2_qt_fontdir(cv2)

LOG_FILE = "receiver.log"
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.FileHandler(LOG_FILE, mode="w"),
        logging.StreamHandler(),
    ],
)
log = logging.getLogger("receiver")

MAGIC_HEADER_SIZE = 4
SERIAL_PKT_SIZE = 300
MAX_CHUNK_DATA = 298
MAX_FRAME_SIZE = 65535
BUFFER_RESET_TIMEOUT = 2.0


class VideoDecoder:
    def __init__(self, host="192.168.12.1", port=3333, encode_width=120, encode_height=120,
                 display_scale=3, crosshair=True):
        self.host = host
        self.port = port
        self.encode_width = encode_width
        self.encode_height = encode_height
        self.display_scale = display_scale
        self.display_width = encode_width * display_scale
        self.display_height = encode_height * display_scale
        self.crosshair = crosshair
        self.window_name = "Doorlock Decoder"

        self.raw_queue = queue.Queue(maxsize=60)
        self.frame_queue = queue.Queue(maxsize=3)
        self._stop_event = threading.Event()

        self._lock = threading.Lock()
        self._cbb_count = 0
        self._cbb_short = 0
        self._cbb_zero_len = 0
        self._cbb_oversize = 0
        self._cbb_ident = 0
        self._cbb_parse_err = 0
        self._cbb_payload_bytes = 0
        self._pkt_count = 0
        self._pkt_bytes = 0
        self._frame_count = 0
        self._drop_count = 0
        self._decode_err_count = 0
        self._sync_loss_count = 0
        self._last_stats_time = time.monotonic()
        self._last_frame_time = time.monotonic()
        self._last_deploy_status = None

        self._frame_buffer = bytearray()
        self._last_buffer_activity = time.monotonic()

        self._init_window()
        self._init_mqtt()

    def _init_window(self):
        dw = self.display_width
        dh = self.display_height
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.window_name, 100, 100)
        cv2.resizeWindow(self.window_name, dw, dh)
        cv2.waitKey(1)

    def _init_mqtt(self):
        self.mqtt_client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION1, client_id="1"
        )
        self.mqtt_client.max_inflight_messages_set(100)
        self.mqtt_client.max_queued_messages_set(0)
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.warning(f"[MQTT] connection failed: rc={rc}")
            return
        log.info("[MQTT] connected")
        client.subscribe("CustomByteBlock", qos=1)
        client.subscribe("DeployModeStatusSync", qos=0)

    def _on_disconnect(self, client, userdata, rc):
        if rc != 0:
            log.warning(f"[MQTT] unexpected disconnect (rc={rc}), auto-reconnecting ...")

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
            except Exception as e:
                with self._lock:
                    self._cbb_parse_err += 1
                log.error(f"[MQTT] parse error: {e}")
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
            if chunk_len == 0:
                with self._lock:
                    self._cbb_zero_len += 1
                return
            if chunk_len > MAX_CHUNK_DATA or chunk_len + 2 > payload_len:
                with self._lock:
                    self._cbb_oversize += 1
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
        except Exception as e:
            log.error(f"[MQTT] message error ({msg.topic}): {e}")

    def _handle_deploy_status(self, payload):
        status_pb = DeployModeStatusSync_pb2.DeployModeStatusSync()
        status_pb.ParseFromString(payload)
        if status_pb.status in (0, 1):
            self._last_deploy_status = status_pb.status
            value = 1024 if status_pb.status == 1 else 0
            ctrl = CustomControl_pb2.CustomControl()
            ctrl.data = value.to_bytes(2, "big")
            info = self.mqtt_client.publish(
                "CustomControl", ctrl.SerializeToString(), qos=1
            )
            log.info(
                "[DeployModeStatusSync] status=%d, publish CustomControl=%s, mid=%s",
                status_pb.status,
                ctrl.data.hex(),
                info.mid,
            )

    def _extract_frames_from_buffer(self):
        while True:
            if len(self._frame_buffer) < MAGIC_HEADER_SIZE:
                break

            total_size = struct.unpack_from("<I", self._frame_buffer, 0)[0]

            if total_size > MAX_FRAME_SIZE:
                log.warning(f"[Frame] bogus size={total_size}, resetting buffer")
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
                img = cv2.imdecode(
                    np.frombuffer(webp_data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if img is not None and img.size > 0:
                    with self._lock:
                        self._frame_count += 1
                        self._last_frame_time = time.monotonic()
                    self._push_frame(img)
            except Exception as e:
                with self._lock:
                    self._decode_err_count += 1
                log.warning(f"[Frame] decode error: {e}")

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
                    log.warning(f"[Buffer] timeout {now - self._last_buffer_activity:.1f}s, "
                                f"discarding {len(self._frame_buffer)} bytes")
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

    def _push_frame(self, img):
        try:
            self.frame_queue.put_nowait(img)
        except queue.Full:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait(img)

    def _display_tick(self):
        latest = None
        while True:
            try:
                latest = self.frame_queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            return

        img = cv2.resize(
            latest,
            (self.display_width, self.display_height),
            interpolation=cv2.INTER_LANCZOS4,
        )
        if self.crosshair:
            self._draw_overlay(img)
        cv2.imshow(self.window_name, img)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._stop_event.set()

    def _draw_overlay(self, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(img, (0, cy), (w - 1, cy), (230, 190, 235), 2, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), (230, 190, 235), 2, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 24, (170, 255, 170), 1, cv2.LINE_AA)

    def _print_stats(self):
        now = time.monotonic()
        with self._lock:
            elapsed = now - self._last_stats_time
            if elapsed < 1.0:
                return
            cbb = self._cbb_count
            cbb_short = self._cbb_short
            cbb_zero = self._cbb_zero_len
            cbb_over = self._cbb_oversize
            cbb_ident = self._cbb_ident
            cbb_parse = self._cbb_parse_err
            cbb_payload = self._cbb_payload_bytes
            pkts = self._pkt_count
            pkt_bytes = self._pkt_bytes
            frames = self._frame_count
            drops = self._drop_count
            errs = self._decode_err_count
            sync_loss = self._sync_loss_count
            last_frame_age = now - self._last_frame_time
            self._cbb_count = 0
            self._cbb_short = 0
            self._cbb_zero_len = 0
            self._cbb_oversize = 0
            self._cbb_ident = 0
            self._cbb_parse_err = 0
            self._cbb_payload_bytes = 0
            self._pkt_count = 0
            self._pkt_bytes = 0
            self._frame_count = 0
            self._drop_count = 0
            self._decode_err_count = 0
            self._sync_loss_count = 0
            self._last_stats_time = now

        fps = frames / elapsed
        hz = pkts / elapsed
        kbps = (pkt_bytes * 8) / elapsed / 1000
        raw_q = self.raw_queue.qsize()
        frame_q = self.frame_queue.qsize()
        avg_payload = (cbb_payload / cbb) if cbb else 0
        parts = [
            f"fps={fps:.1f}",
            f"pkts_hz={hz:.1f}",
            f"kbps={kbps:.1f}",
            f"buf={len(self._frame_buffer)}B",
            f"raw_q={raw_q}",
            f"frame_q={frame_q}",
            f"no_frame={last_frame_age:.2f}s",
        ]
        if cbb_short:
            parts.append(f"short={cbb_short}")
        if cbb_zero:
            parts.append(f"zero_len={cbb_zero}")
        if cbb_over:
            parts.append(f"oversize={cbb_over}")
        if cbb_ident:
            parts.append(f"ident={cbb_ident}")
        if cbb_parse:
            parts.append(f"parse_err={cbb_parse}")
        if drops:
            parts.append(f"drops={drops}")
        if errs:
            parts.append(f"dec_err={errs}")
        if sync_loss:
            parts.append(f"sync_loss={sync_loss}")
        log.info(f"[Stats] {', '.join(parts)}")

    def run(self):
        self.mqtt_client.connect(self.host, self.port, keepalive=60)
        self.mqtt_client.loop_start()

        assembler_thread = threading.Thread(
            target=self._assembler_loop, name="assembler", daemon=True
        )
        assembler_thread.start()

        try:
            while not self._stop_event.is_set():
                self._display_tick()
                self._print_stats()
                time.sleep(0.008)
        finally:
            self._stop_event.set()
            assembler_thread.join(timeout=2)
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            cv2.destroyAllWindows()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="MQTT JPEG Video Receiver")
    parser.add_argument("--host", default="192.168.12.1")
    parser.add_argument("--port", type=int, default=3333)
    parser.add_argument("--encode-width", type=int, default=120)
    parser.add_argument("--encode-height", type=int, default=120)
    parser.add_argument("--display-scale", type=int, default=3,
                        help="Scale factor for display (e.g. 3 means 120->360)")
    parser.add_argument("--no-crosshair", action="store_true",
                        help="Disable crosshair overlay")
    args = parser.parse_args()

    decoder = VideoDecoder(
        host=args.host,
        port=args.port,
        encode_width=args.encode_width,
        encode_height=args.encode_height,
        display_scale=args.display_scale,
        crosshair=not args.no_crosshair,
    )
    try:
        decoder.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
