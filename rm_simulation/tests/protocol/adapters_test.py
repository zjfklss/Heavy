"""Real separate-process TCP exchanges; peers are protocol fixtures, not aiming algorithms."""
import copy
import json
import os
import pathlib
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

binary, peer_binary = sys.argv[1:3]
qt_binary = sys.argv[3] if len(sys.argv) > 3 else None
screenshot = sys.argv[4] if len(sys.argv) > 4 else None
VERSION = 'rm-sim/1'


def exact(peer, size):
    chunks = bytearray()
    while len(chunks) < size:
        chunk = peer.recv(size-len(chunks))
        if not chunk:
            raise EOFError('peer closed')
        chunks.extend(chunk)
    return bytes(chunks)


def receive(peer):
    magic, header, body = struct.unpack('!4sII', exact(peer, 12))
    assert magic == b'RMS1' and 0 < header <= 1024*1024 and body <= 64*1024*1024
    value = json.loads(exact(peer, header))
    assert value['protocol'] == VERSION
    return value, exact(peer, body)


def send(peer, header, body=b'', fragmented=False):
    header = {'protocol': VERSION, **header}
    text = json.dumps(header, separators=(',', ':')).encode()
    data = struct.pack('!4sII', b'RMS1', len(text), len(body)) + text + body
    if fragmented:
        for chunk in [data[:3], data[3:11], data[11:19], data[19:]]:
            peer.sendall(chunk)
    else:
        peer.sendall(data)


def assert_public(value):
    forbidden = {'targets', 'target_id', 'plate_id', 'hits', 'truth', 'projectiles', 'camera_pose', 'ideal_pixels'}
    if isinstance(value, dict):
        assert not forbidden.intersection(value), value
        for item in value.values():
            assert_public(item)
    elif isinstance(value, list):
        for item in value:
            assert_public(item)


class Fixture:
    def __init__(self, profile='generic', fault=None, commands=True, telemetry=None):
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(1)
        self.port = self.listener.getsockname()[1]
        self.profile, self.fault, self.commands = profile, fault, commands
        self.telemetry = telemetry
        self.errors, self.steps, self.frames = [], [], []
        self.hello = None
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        try:
            self.listener.settimeout(10)
            with self.listener.accept()[0] as peer:
                peer.settimeout(5)
                peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                h, body = receive(peer)
                assert h['type'] == 'hello' and not body and h['adapter'] == self.profile
                assert_public(h)
                self.hello = h
                assert h['units']['pitch_up_sign'] == (-1 if self.profile == 'sp_vision' else 1)
                ack = {'type': 'hello_ack', 'session_id': h['session_id'], 'adapter': self.profile,
                       'peer_version': 'python-fixture/1'}
                if self.fault == 'hello_profile':
                    ack['adapter'] = 'wrong'
                if self.fault == 'hello_version':
                    ack['protocol'] = 'wrong/1'
                send(peer, ack, fragmented=True)
                if self.fault in {'hello_profile', 'hello_version'}:
                    return
                while True:
                    step, payload = receive(peer)
                    assert_public(step)
                    assert step['session_id'] == h['session_id']
                    if step['type'] == 'finish':
                        assert not payload
                        send(peer, {'type': 'finished', 'session_id': h['session_id'], 'tick': step['tick']})
                        return
                    assert step['type'] == 'step' and step['sequence'] == len(self.steps)
                    assert abs(step['time'] - step['tick']*step['timestep']) < 1e-9
                    offset = 0
                    for frame in step['images']:
                        assert frame['encoding'] == 'bgr8' and frame['stride'] == frame['width']*3
                        assert frame['offset'] == offset and frame['bytes'] == frame['height']*frame['stride']
                        assert frame['capture_tick'] <= frame['ready_tick'] <= step['tick']
                        pixels = payload[offset:offset+frame['bytes']]
                        assert len(pixels) == frame['bytes'] and len(set(pixels)) > 1
                        self.frames.append(frame)
                        offset += frame['bytes']
                    assert offset == len(payload)
                    for feedback in step['feedback']:
                        assert feedback['capture_tick'] <= feedback['ready_tick'] <= step['tick']
                    self.steps.append(step)
                    commands = [{'control': True, 'yaw': .1, 'pitch': -.12 if self.profile == 'sp_vision' else .12,
                                 'fire': True, 'request_id': 'same-single-shot'}] if self.commands else []
                    reply = {'type': 'commands', 'session_id': h['session_id'], 'sequence': step['sequence'],
                             'tick': step['tick'], 'commands': commands}
                    if self.telemetry is not None:
                        reply['telemetry'] = self.telemetry
                    if self.fault == 'telemetry':
                        reply['telemetry'] = {'nested': {'invalid': True}}
                    if self.fault == 'timeout':
                        time.sleep(.2)
                        return
                    if self.fault == 'disconnect':
                        return
                    if self.fault == 'oversized_header':
                        peer.sendall(struct.pack('!4sII', b'RMS1', 1024*1024+1, 0))
                        return
                    if self.fault == 'bad_magic':
                        peer.sendall(struct.pack('!4sII', b'BAD!', 1, 0))
                        return
                    if self.fault == 'sequence':
                        reply['sequence'] += 1
                    if self.fault == 'float_sequence':
                        reply['sequence'] = float(reply['sequence'])
                    if self.fault == 'session':
                        reply['session_id'] = 'old-session'
                    if self.fault == 'tick':
                        reply['tick'] += 1
                    if self.fault == 'unknown':
                        reply['unknown'] = 1
                    if self.fault == 'command_unknown':
                        commands[0]['future_truth'] = 0
                    if self.fault == 'missing_id':
                        del commands[0]['request_id']
                    if self.fault == 'nan':
                        commands[0]['yaw'] = float('nan')
                    if self.fault == 'disabled_fire':
                        commands[0]['control'] = False
                    if self.fault == 'missing_angle':
                        del commands[0]['pitch']
                    if self.fault == 'too_many':
                        reply['commands'] *= 65
                    if self.fault == 'late_replay' and len(self.steps) == 2:
                        reply['sequence'] -= 1
                    send(peer, reply, body=b'bad' if self.fault == 'payload' else b'', fragmented=True)
                    if self.fault and self.fault != 'late_replay':
                        return
        except (EOFError, ConnectionResetError, BrokenPipeError):
            if self.fault is None:
                self.errors.append('Unexpected peer disconnect')
        except Exception as error:
            self.errors.append(repr(error))
        finally:
            self.listener.close()

    def join(self):
        self.thread.join(10)
        assert not self.thread.is_alive(), 'peer fixture failed to stop'
        assert not self.errors, self.errors


base = {
    'closed_loop_version': 1,
    'scene': {'schema_version': 1, 'duration': .06, 'targets': [{'id': 'car', 'kind': 'car', 'position': [2, 0, .3]}]},
    'sensor': {'output': 'image', 'frequency': 60, 'latency': .012,
               'calibration': {'width': 96, 'height': 64, 'fx': 90, 'fy': 90, 'cx': 48, 'cy': 32}},
    'actuator': {'initial_pitch': .08},
    'algorithm': {'kind': 'external', 'adapter': 'generic', 'host': '127.0.0.1', 'port': 17001,
                  'connect_timeout': .5, 'step_timeout': .5},
}
with tempfile.TemporaryDirectory() as temporary:
    root = pathlib.Path(temporary)
    source = root/'external.json'
    source.write_text(json.dumps(base))
    started = time.monotonic()
    result = subprocess.run([binary, '--validate', str(source)], capture_output=True, timeout=2)
    assert result.returncode == 0, result.stderr
    assert time.monotonic()-started < 1 and list(root.iterdir()) == [source], 'Validation connected or wrote outputs'
    for profile in ['generic', 'sp_vision']:
        fixture = Fixture(profile, telemetry={'frames_processed': 3, 'control': True, 'state': 'running'})
        config = copy.deepcopy(base)
        config['algorithm'].update(adapter=profile, port=fixture.port)
        source.write_text(json.dumps(config))
        trace = root/(profile+'.json')
        result = subprocess.run([binary, str(source), str(trace)], capture_output=True, timeout=5)
        assert result.returncode == 0, result.stderr
        fixture.join()
        data = json.loads(trace.read_text())
        assert data['adapter']['state'] == 'finished' and data['adapter']['completed_steps'] == 12
        assert data['adapter']['telemetry'] == fixture.telemetry
        assert b'[adapter] telemetry ' not in result.stdout
        assert data['adapter']['images_sent'] == 3 and data['adapter']['feedback_sent'] == 12
        assert data['summary']['shots'] == 1, 'Repeated single-shot requests were fired more than once'
        assert data['summary']['pending_observations'] == 1
        assert len(data['commands']) == 12
        assert all(c['due_tick'] >= c['issued_tick']+1 and abs(c['pitch']-.12) < 1e-9 for c in data['commands'])
        sign = -1 if profile == 'sp_vision' else 1
        assert abs(fixture.steps[0]['feedback'][0]['pitch']-.08*sign) < 1e-9
        assert data['resolved']['algorithm']['adapter'] == profile
    for fault in ['hello_profile', 'hello_version', 'timeout', 'disconnect', 'oversized_header', 'bad_magic',
                  'sequence', 'float_sequence', 'session', 'tick', 'unknown', 'command_unknown', 'missing_id',
                  'nan', 'disabled_fire', 'missing_angle', 'too_many', 'payload', 'late_replay', 'telemetry']:
        fixture = Fixture(fault=fault)
        config = copy.deepcopy(base)
        config['algorithm'].update(port=fixture.port, step_timeout=.05 if fault == 'timeout' else .5)
        source.write_text(json.dumps(config))
        trace = root/'failed.json'
        trace.write_text('keep-existing-trace')
        result = subprocess.run([binary, str(source), str(trace)], capture_output=True, timeout=5)
        fixture.join()
        assert result.returncode != 0, fault
        assert trace.read_text() == 'keep-existing-trace', fault
    for key, value in [('port', -1), ('port', 1.5), ('port', 65536), ('port', 2**40+17001),
                       ('host', 'localhost'), ('adapter', 'missing'), ('connect_timeout', 0),
                       ('step_timeout', 61), ('gain', .6)]:
        config = copy.deepcopy(base)
        config['algorithm'][key] = value
        source.write_text(json.dumps(config))
        assert subprocess.run([binary, '--validate', str(source)], capture_output=True).returncode != 0, key
    for section, key, value in [('sensor', 'output', 'features'), ('sensor', 'enabled', False),
                                ('actuator', 'feedback_enabled', False)]:
        config = copy.deepcopy(base)
        config[section][key] = value
        source.write_text(json.dumps(config))
        assert subprocess.run([binary, '--validate', str(source)], capture_output=True).returncode != 0, key
    # Real C++ SDK peer, independently launched and linked without vision code.
    for profile in ['generic', 'sp_vision']:
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        peer = subprocess.Popen([peer_binary, profile, str(port)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            assert 'Listening' in peer.stdout.readline()
            config = copy.deepcopy(base)
            config['algorithm'].update(adapter=profile, port=port)
            source.write_text(json.dumps(config))
            result = subprocess.run([binary, str(source), str(root/'cpp.json')], capture_output=True, timeout=5)
            stdout, stderr = peer.communicate(timeout=5)
            assert result.returncode == 0 and peer.returncode == 0, (result.stderr, stderr)
            assert 'steps=12 images=3' in stdout
        finally:
            if peer.poll() is None:
                peer.kill()
                peer.wait()
    if qt_binary:
        fixture = Fixture('sp_vision', commands=False)
        config = copy.deepcopy(base)
        config['scene']['duration'] = .3
        config['algorithm'].update(adapter='sp_vision', port=fixture.port)
        source.write_text(json.dumps(config))
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
        result = subprocess.run([qt_binary, '--config', str(source), '--config-section', 'scheduler', '--self-test',
                                 '--screenshot', screenshot], env=env, capture_output=True, timeout=15)
        assert result.returncode == 0, result.stderr
        fixture.join()
        assert fixture.hello['adapter'] == 'sp_vision' and len(fixture.steps) > 1
        assert fixture.hello['configuration']['realtime'] is True
print('External processes: both profiles, BGR/feedback, UI, SDK, timeout/disconnect, validation and replay protection passed')
