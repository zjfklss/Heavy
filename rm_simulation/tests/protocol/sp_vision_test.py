"""Optional end-to-end test with the user's real SP executable, not a protocol fixture.

Usage: python3 sp_vision_test.py SIM_CLI SP_EXECUTABLE SP_CONFIG QT_EXECUTABLE ARTIFACT_DIR
"""
import copy
import json
import os
from pathlib import Path
import queue
import re
import signal
import socket
import subprocess
import sys
import threading

sim, vision, profile, qt, directory = map(lambda p: str(Path(p).resolve()), sys.argv[1:])
root = Path(directory)
root.mkdir(parents=True, exist_ok=True)
with socket.socket() as reservation:
    reservation.bind(('127.0.0.1', 0))
    port = reservation.getsockname()[1]

lines, events = [], queue.Queue()
peer = subprocess.Popen([vision, profile, '--simulation=true', '--sim-ui=false', f'--sim-port={port}'],
                        cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def drain():
    for line in peer.stdout:
        lines.append(line)
        events.put(line)


thread = threading.Thread(target=drain, daemon=True)
thread.start()


def listening():
    while True:
        line = events.get(timeout=15)
        if '[JLCV simulation] listening' in line:
            return


base = {
    'closed_loop_version': 1,
    'scene': {'schema_version': 1, 'name': 'Real external SP communication test', 'duration': .3,
              'targets': [{'id': 'robot', 'kind': 'car', 'position': [3, .2, .3]}]},
    'sensor': {'output': 'image', 'frequency': 60, 'latency': 0,
               'calibration': {'width': 1440, 'height': 1080, 'fx': 2400, 'fy': 2400, 'cx': 720, 'cy': 540}},
    'actuator': {'feedback_latency': 0},
    'algorithm': {'kind': 'external', 'adapter': 'sp_vision', 'host': '127.0.0.1',
                  'port': port, 'connect_timeout': 10, 'step_timeout': 10},
}
results = []
try:
    listening()
    for color, latency in [('blue', 0), ('red', .025)]:
        config = copy.deepcopy(base)
        config['sensor']['image'] = {'color': color}
        config['actuator']['feedback_latency'] = latency
        source, trace = root / f'{color}.config.json', root / f'{color}.trace.json'
        source.write_text(json.dumps(config))
        result = subprocess.run([sim, str(source), str(trace)], cwd=root, capture_output=True,
                                text=True, timeout=30)
        assert result.returncode == 0, (result.stderr, ''.join(lines))
        (root / f'{color}.stdout.log').write_text(result.stdout)
        data = json.loads(trace.read_text())
        adapter, info = data['adapter'], data['adapter']['telemetry']
        assert adapter['peer_version'] == 'jlcv-sp-vision/simulation-1'
        assert adapter['state'] == 'finished' and adapter['completed_steps'] == 60
        assert adapter['images_sent'] == info['images_received'] == 18
        assert adapter['feedback_sent'] == info['feedback_received'] > 0
        assert 12 <= info['frames_processed'] <= 18 and info['dropped_images'] == 0, info
        assert isinstance(info['control'], bool) and 'yaw' in info and 'pitch' in info
        assert adapter['commands_received'] > 0 and data['commands'], 'Real SP returned no controls'
        assert any(abs(c['yaw']) > .01 for c in data['commands']), 'Off-centre target did not produce yaw control'
        assert '[adapter] telemetry ' not in result.stdout
        results.append(adapter)
        listening()  # Same independently running self-aim process accepts the next UI run.

    # An unsupported SP mounting model must fail explicitly, then accept another run.
    invalid = copy.deepcopy(base)
    invalid['actuator']['yaw_offset'] = [.1, 0, 0]
    bad = root / 'unsupported.config.json'
    bad.write_text(json.dumps(invalid))
    result = subprocess.run([sim, str(bad), str(root / 'unsupported.trace.json')], cwd=root,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode != 0 and not (root / 'unsupported.trace.json').exists()
    listening()

    source = root / 'qt.config.json'
    source.write_text(json.dumps(base))
    result = subprocess.run([qt, '--config', str(source), '--config-section', 'scheduler', '--self-test-live',
                             '--self-test-live-restart',
                             '--screenshot', str(root / 'real_sp_ui.png')], cwd=root,
                            env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), capture_output=True,
                            text=True, timeout=40)
    assert result.returncode == 0, (result.stderr, ''.join(lines))
    live_commands=re.search(r'Live UI:.*commands=(\d+).*observed_while_running=1',result.stderr)
    assert live_commands and int(live_commands.group(1))>0,result.stderr
    (root / 'qt.stdout.log').write_text(result.stdout+result.stderr)
    listening()
    # Camera/UART are intentionally absent on this path; original config validation still works.
    vision_source = next(parent for parent in Path(profile).parents
                         if (parent / 'assets/auto_aim_traditional').is_dir())
    result = subprocess.run([vision, profile, '--check-config=true'], cwd=vision_source,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    (root / 'result.json').write_text(json.dumps({'real_sp_sessions': results, 'qt': 'passed',
                                               'reconnect_after_error': 'passed', 'hardware_config': 'passed'}, indent=2))
finally:
    peer.send_signal(signal.SIGINT)
    try:
        peer.wait(timeout=12)
    except subprocess.TimeoutExpired:
        peer.kill()
        peer.wait()
    thread.join(timeout=2)
    (root / 'sp.stdout.log').write_text(''.join(lines))
print('Real SP process: BGR/feedback, commands/telemetry, delayed feedback, repeat UI runs and error recovery passed')
