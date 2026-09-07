#!/usr/bin/python3
"""本机 MQTT 收 0x0310 分包，拼 JPEG 显示。有 FSRCNN pb 则超分。"""

from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

MAX_CHUNK_DATA = 298


def unpack_chunk(pkt: bytes) -> bytes:
    if len(pkt) < 2:
        return b''
    n = struct.unpack_from('<H', pkt, 0)[0]
    n = min(n, MAX_CHUNK_DATA, max(0, len(pkt) - 2))
    return pkt[2:2 + n]


def mqtt_client():
    import paho.mqtt.client as mqtt

    if hasattr(mqtt, 'CallbackAPIVersion'):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    return mqtt.Client()


class Viewer:
    def __init__(self, model_path: str | None) -> None:
        self.net = None
        if model_path and Path(model_path).is_file():
            try:
                self.net = cv2.dnn.readNetFromTensorflow(model_path)
                print(f'FSRCNN loaded {model_path}')
            except Exception as exc:
                print(f'FSRCNN skip: {exc}')
        self.buf = bytearray()

    def apply_sr(self, img: np.ndarray) -> np.ndarray:
        if self.net is None:
            return img
        try:
            blob = cv2.dnn.blobFromImage(img)
            self.net.setInput(blob)
            out = self.net.forward()
            ch = out.shape[1] if out.ndim == 4 else 1
            if ch == 1:
                gray = np.clip(out.reshape(out.shape[-2], out.shape[-1]), 0, 255).astype(np.uint8)
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except Exception:
            return img
        return img

    def on_payload(self, payload: bytes) -> None:
        self.buf.extend(payload)
        while True:
            start = self.buf.find(b'\xff\xd8')
            if start < 0:
                if len(self.buf) > 50000:
                    del self.buf[:-2]
                return
            if start > 0:
                del self.buf[:start]
            end = self.buf.find(b'\xff\xd9', 2)
            if end < 0:
                if len(self.buf) > 500000:
                    del self.buf[:1000]
                return
            jpeg = bytes(self.buf[: end + 2])
            del self.buf[: end + 2]
            img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                cv2.imshow('heavy_0310', self.apply_sr(img))
                cv2.waitKey(1)


def main() -> None:
    host = '127.0.0.1'
    port = 1883
    topic = 'CustomByteBlock'
    model = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        import paho.mqtt.client as mqtt  # noqa: F401
    except ImportError:
        print('need python3-paho-mqtt')
        sys.exit(1)

    viewer = Viewer(model)

    def on_message(_c, _u, msg) -> None:
        viewer.on_payload(unpack_chunk(msg.payload))

    client = mqtt_client()
    client.on_message = on_message
    client.connect(host, port, 60)
    client.subscribe(topic)
    print(f'viewer MQTT {host}:{port} {topic}')
    client.loop_start()
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    client.loop_stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
