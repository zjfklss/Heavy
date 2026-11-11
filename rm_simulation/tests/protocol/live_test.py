"""Live world pacing, bounded display stream, continuous sessions and slow external I/O."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

binary, qt, directory = sys.argv[1:]
root = Path(directory)
root.mkdir(parents=True, exist_ok=True)
base = {
    'closed_loop_version': 1,
    'scene': {'schema_version': 1, 'name': '实时仿真', 'duration': .1,
              'targets': [{'id': 'target', 'kind': 'car', 'position': [3, .2, .3],
                           'translation': [{'kind': 'velocity', 'velocity': .2}, {}, {}]}]},
    'sensor': {'output': 'image', 'frequency': 60,
               'calibration': {'width': 320, 'height': 240, 'fx': 400, 'fy': 400, 'cx': 160, 'cy': 120}},
    'algorithm': {'kind': 'none'},
}


def exact(stream, size):
    result = bytearray()
    while len(result) < size:
        chunk = stream(size-len(result))
        if not chunk:
            raise EOFError()
        result.extend(chunk)
    return bytes(result)


def receive(peer):
    magic, h, p = struct.unpack('!4sII', exact(peer.recv, 12))
    assert magic == b'RMS1' and h <= 1024*1024 and p <= 64*1024*1024
    return json.loads(exact(peer.recv, h)), exact(peer.recv, p)


def send(peer, value):
    value = {'protocol': 'rm-sim/1', **value}
    text = json.dumps(value).encode()
    peer.sendall(struct.pack('!4sII', b'RMS1', len(text), 0)+text)


class SlowPeer:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.steps, self.errors = [], []
        self.finished = False
        self.thread = threading.Thread(target=self.work, daemon=True)
        self.thread.start()

    def work(self):
        try:
            with self.listener.accept()[0] as peer:
                peer.settimeout(5)
                hello, _ = receive(peer)
                assert hello['configuration']['realtime'] is True
                assert hello['configuration']['duration'] is None
                send(peer, {'type': 'hello_ack', 'session_id': hello['session_id'],
                            'adapter': 'generic', 'peer_version': 'slow-live-test/1'})
                while True:
                    step, pixels = receive(peer)
                    if step['type'] == 'finish':
                        send(peer, {'type': 'finished', 'session_id': hello['session_id'], 'tick': step['tick']})
                        self.finished = True
                        return
                    assert not {'targets', 'truth', 'camera_pose'}.intersection(step)
                    assert len(step['images']) <= 1 and len(step['feedback']) <= 1000
                    if step['images']:
                        assert len(pixels) == 320*240*3
                    self.steps.append(step)
                    time.sleep(.12)  # Several camera periods; must never stall the world or UI.
                    send(peer, {'type': 'commands', 'session_id': hello['session_id'],
                                'sequence': step['sequence'], 'tick': step['tick'],
                                'commands': [{'control': True, 'yaw': .12, 'pitch': .05, 'fire': False}],
                                'telemetry': {'state': 'running'}})
        except Exception as error:
            self.errors.append(repr(error))
        finally:
            self.listener.close()


def session(name, config, peer=None):
    source = root / f'{name}.config.json'
    source.write_text(json.dumps(config, ensure_ascii=False))
    proc = subprocess.Popen([binary, '--live', str(source)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    frames, errors = [], []
    started = time.monotonic()

    def consume():
        try:
            while True:
                magic, h, p = struct.unpack('!4sII', exact(proc.stdout.read, 12))
                assert magic == b'RML1' and h <= 1024*1024 and p <= 64*1024*1024
                value = json.loads(exact(proc.stdout.read, h))
                pixels = exact(proc.stdout.read, p)
                assert value['protocol'] == 'rm-live/1'
                if 'camera' in value:
                    assert len(pixels) == 320*240*3 and value['camera']['encoding'] == 'bgr8'
                    assert value['camera']['capture_time'] <= value['camera']['ready_time'] <= value['frame']['time']
                frames.append((time.monotonic()-started, value))
        except EOFError:
            pass
        except Exception as error:
            errors.append(repr(error))

    reader = threading.Thread(target=consume, daemon=True)
    reader.start()
    try:
        deadline = started+5
        while not frames and time.monotonic() < deadline:
            assert proc.poll() is None, proc.stderr.read().decode()
            time.sleep(.01)
        assert frames and frames[0][0] < 2, 'First frame arrived only after completion'
        actuator = frames[0][1]['actuator']
        if config.get('actuator', {}).get('align_on_start', True):
            pose = actuator['camera_pose']
            projection_errors = []
            for plate in frames[0][1]['frame']['targets'][0]['surfaces']:
                delta = [a-b for a, b in zip(plate['center'], pose['translation'])]
                local = [sum(a*b for a, b in zip(axis, delta)) for axis in pose['rotation']]
                if local[2] > 0:
                    projection_errors.append(math.hypot(*local[:2])/local[2])
            assert projection_errors and min(projection_errors) < 1e-7, 'Live startup did not point at actual armor'
        else:
            assert abs(actuator['yaw']-config['actuator']['initial_yaw']) < 1e-12
            assert abs(actuator['pitch']-config['actuator']['initial_pitch']) < 1e-12
        time.sleep(1.2)
        assert proc.poll() is None, 'Live session ended at scenario.duration'
        assert len(frames) >= 25 and frames[-1][1]['frame']['time'] >= 1.05
        assert abs(frames[-1][1]['frame']['time']-frames[-1][0]) < .2, 'World clock fell behind wall time'
        assert any('camera' in f for _, f in frames)
        target = frames[-1][1]['frame']['targets'][0]
        if target['kind'] == 'car':
            assert target['position'][0] > 3.2, 'Target did not move while streaming'
        if peer:
            assert len(peer.steps) >= 5
            assert frames[-1][1]['adapter']['dropped_input_images'] > 0
            assert abs(frames[-1][1]['actuator']['yaw']-.12) < 1e-8
            assert max(b[0]-a[0] for a, b in zip(frames, frames[1:])) < .2
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=5)
        reader.join(timeout=2)
        stderr = proc.stderr.read().decode()
        assert proc.returncode == 0 and not errors, (stderr, errors)
        if peer:
            peer.thread.join(timeout=5)
            assert peer.finished and not peer.errors, peer.errors
        (root / f'{name}.result.json').write_text(json.dumps({
            'display_frames': len(frames), 'last_frame': frames[-1][1],
            'external_exchanges': len(peer.steps) if peer else 0}, ensure_ascii=False, indent=2))
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


session('continuous', base)
outpost = copy.deepcopy(base)
outpost['scene']['targets'] = [{'id': 'outpost', 'kind': 'outpost', 'position': [4, .2, 0]}]
session('outpost-aligned', outpost)
manual = copy.deepcopy(base)
manual['actuator'] = {'align_on_start': False, 'initial_yaw': .15, 'initial_pitch': -.05}
session('manual-start', manual)
slow = SlowPeer()
external = copy.deepcopy(base)
external['algorithm'] = {'kind': 'external', 'adapter': 'generic', 'host': '127.0.0.1',
                          'port': slow.port, 'connect_timeout': 3, 'step_timeout': 1}
session('slow-peer', external, slow)
source = root / 'continuous.config.json'
result = subprocess.run([qt, '--config', str(source), '--self-test-live', '--self-test-live-restart', '--screenshot', str(root / 'live_ui.png')],
                        capture_output=True, text=True, env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), timeout=15)
assert result.returncode == 0, result.stderr
assert not list(root.glob('*.images')), 'Live display created a PNG replay archive'
print('Live wall clock, frames before completion, continuous sessions, slow peer, commands, graceful stop and UI passed')
