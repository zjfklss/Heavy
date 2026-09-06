#!/usr/bin/env python3

import time
import paho.mqtt.client as mqtt
import custom_byte_block_pb2

cbb_count = 0
cbb_bytes = 0
last_hz_time = time.monotonic()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected!")
        client.subscribe("#")
    else:
        print(f"Connection failed: {rc}")

def on_message(client, userdata, msg):
    global cbb_count, cbb_bytes, last_hz_time

    if msg.topic == "CustomByteBlock":
        cbb_count += 1
        cbb_bytes += len(msg.payload)

        now = time.monotonic()
        elapsed = now - last_hz_time
        if elapsed >= 1.0:
            hz = cbb_count / elapsed
            kbps = (cbb_bytes * 8) / elapsed / 1000
            print(f"[CustomByteBlock] {hz:.1f} Hz, {kbps:.1f} kbps ({cbb_count} msgs in {elapsed:.1f}s)")
            cbb_count = 0
            cbb_bytes = 0
            last_hz_time = now
        return

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id="1")
client.on_connect = on_connect
client.on_message = on_message

client.connect("192.168.12.1", 3333, 60)
client.loop_forever()
