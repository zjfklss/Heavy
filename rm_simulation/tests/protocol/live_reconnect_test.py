"""Keep the live world running while an independent algorithm process is rebuilt/restarted."""
import collections
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time


def exact(stream, size):
    result = bytearray()
    while len(result) < size:
        chunk = stream(size-len(result))
        if not chunk:
            raise EOFError()
        result.extend(chunk)
    return bytes(result)


def receive(peer):
    magic, header, pixels = struct.unpack('!4sII', exact(peer.recv, 12))
    assert magic == b'RMS1'
    value = json.loads(exact(peer.recv, header))
    exact(peer.recv, pixels)
    return value


def send(peer, value):
    data = json.dumps({'protocol': 'rm-sim/1', **value}).encode()
    peer.sendall(struct.pack('!4sII', b'RMS1', len(data), 0)+data)


def run_peer(port, yaw, mode):
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', port))
        listener.listen(1)
        print('listening', flush=True)
        with listener.accept()[0] as peer:
            peer.settimeout(5)
            hello = receive(peer)
            session = hello['session_id']
            print(session, flush=True)
            send(peer, {'type': 'hello_ack', 'session_id': session,
                        'adapter': 'generic', 'peer_version': 'restart-test/1'})
            sequence = 0
            while True:
                step = receive(peer)
                if step['type'] == 'finish':
                    if mode != 'close-on-finish':
                        send(peer, {'type': 'finished', 'session_id': session, 'tick': step['tick']})
                    return
                assert step['session_id'] == session and step['sequence'] == sequence
                assert not {'targets', 'truth', 'camera_pose'}.intersection(step)
                send(peer, {'type': 'commands', 'session_id': session, 'sequence': sequence,
                            'tick': step['tick']+1 if mode == 'bad-tick' else step['tick'],
                            'commands': [{'control': True, 'yaw': yaw, 'pitch': .05,
                                          'fire': True, 'request_id': 'restart-shot'}]})
                sequence += 1


if len(sys.argv) > 1 and sys.argv[1] == '--peer':
    run_peer(int(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
    sys.exit(0)


binary, qt, directory = sys.argv[1:]
root = Path(directory)
root.mkdir(parents=True, exist_ok=True)
with socket.socket() as reservation:
    reservation.bind(('127.0.0.1', 0))
    port = reservation.getsockname()[1]
config = {
    'closed_loop_version': 1,
    'scene': {'schema_version': 1, 'duration': .1,
              'targets': [{'id': 'target', 'kind': 'car', 'position': [3, .2, .3],
                           'translation': [{'kind': 'velocity', 'velocity': .2}, {}, {}]}]},
    'sensor': {'output': 'image', 'frequency': 60, 'image': {'renderer': 'cpu'},
               'calibration': {'width': 320, 'height': 240, 'fx': 400, 'fy': 400, 'cx': 160, 'cy': 120}},
    'algorithm': {'kind': 'external', 'adapter': 'generic', 'host': '127.0.0.1',
                  'port': port, 'connect_timeout': .2, 'step_timeout': .2},
}
source = root / 'restart.config.json'
source.write_text(json.dumps(config))


class Simulation:
    def __init__(self):
        self.frames = collections.deque(maxlen=256)
        self.errors = []
        self.proc = subprocess.Popen([binary, '--live', str(source)],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.reader = threading.Thread(target=self.consume, daemon=True)
        self.reader.start()

    def consume(self):
        try:
            while True:
                magic, header, pixels = struct.unpack('!4sII', exact(self.proc.stdout.read, 12))
                assert magic == b'RML1'
                frame = json.loads(exact(self.proc.stdout.read, header))
                exact(self.proc.stdout.read, pixels)
                self.frames.append(frame)
        except EOFError:
            pass
        except Exception as error:
            self.errors.append(repr(error))

    def until(self, predicate):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            assert self.proc.poll() is None, self.proc.stderr.read().decode()
            assert not self.errors, self.errors
            if self.frames and predicate(self.frames[-1]):
                return self.frames[-1]
            time.sleep(.01)
        raise AssertionError(('Live state did not arrive', self.frames[-1] if self.frames else None))

    def stop(self):
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=3)
        self.reader.join(timeout=2)
        assert self.proc.returncode == 0 and not self.errors, (self.proc.stderr.read().decode(), self.errors)

    def close(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()


def start_peer(yaw, mode='normal'):
    proc = subprocess.Popen([sys.executable, __file__, '--peer', str(port), str(yaw), mode],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.stdout.readline().strip() == 'listening', proc.stderr.read()
    return proc


peers = []
sim = Simulation()
try:
    waiting = sim.until(lambda f: f['adapter']['state'] == 'waiting' and f['frame']['time'] >= .15 and 'camera' in f)
    first = start_peer(.12)
    peers.append(first)
    connected = sim.until(lambda f: f['adapter']['state'] == 'connected' and
                          f['statistics']['shots'] == 1 and f['statistics']['commands'] >= 3)
    first_session = first.stdout.readline().strip()
    assert abs(connected['actuator']['yaw']-.12) < 1e-8
    first.terminate()  # The same action as closing the real auto-aim process for editing.
    first.wait(timeout=2)
    lost = sim.until(lambda f: f['adapter']['state'] == 'waiting' and
                     f['frame']['time'] >= connected['frame']['time']+.2)
    assert not lost['adapter']['telemetry'], 'Disconnected UI retained stale control telemetry'
    assert lost['statistics']['shots'] == 1, 'Repeated shot IDs fired more than once'
    assert lost['frame']['targets'][0]['position'][0] > waiting['frame']['targets'][0]['position'][0]
    second = start_peer(-.22, 'close-on-finish')
    peers.append(second)
    reconnected = sim.until(lambda f: f['adapter']['state'] == 'connected' and
                            f['adapter']['connections'] == 2 and f['statistics']['shots'] == 2 and
                            abs(f['actuator']['yaw']+.22) < 1e-8)
    assert second.stdout.readline().strip() != first_session, 'Restart reused an old session'
    assert reconnected['frame']['time'] > lost['frame']['time'], 'Reconnect reset world time'
    sim.stop()  # A peer disappearing during finish must also be a normal stop.
    second.wait(timeout=2)
    assert second.returncode == 0, second.stderr.read()
finally:
    sim.close()
    for peer in peers:
        if peer.poll() is None:
            peer.kill()
            peer.wait()

# No peer at all: both CLI and actual Qt must start, stream, and stop normally.
sim = Simulation()
try:
    sim.until(lambda f: f['adapter']['state'] == 'waiting' and f['frame']['time'] >= .1)
    sim.stop()
finally:
    sim.close()
result = subprocess.run([qt, '--config', str(source), '--self-test-live', '--screenshot', str(root / 'waiting_ui.png')],
                        capture_output=True, text=True, env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), timeout=10)
assert result.returncode == 0, result.stderr

# Invalid protocol data remains an error; only transport loss triggers automatic retry.
peer = start_peer(.12, 'bad-tick')
sim = Simulation()
try:
    sim.proc.wait(timeout=5)
    assert sim.proc.returncode != 0
    assert 'response/session/sequence mismatch' in sim.proc.stderr.read().decode()
finally:
    sim.close()
    if peer.poll() is None:
        peer.kill()
    peer.wait()
print('Absent peer, process termination/restart, fresh sessions, shot deduplication, stop and Qt waiting state passed')
