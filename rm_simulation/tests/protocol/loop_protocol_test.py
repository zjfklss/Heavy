"""Validate reproducibility, public/truth separation and strict closed-loop configuration."""
import copy
import json
import pathlib
import subprocess
import sys
import tempfile

binary, folder = sys.argv[1], pathlib.Path(sys.argv[2])
with tempfile.TemporaryDirectory() as directory:
    directory = pathlib.Path(directory)
    for source in sorted(folder.glob('*.json')):
        validated = subprocess.run([binary, '--validate', str(source)], capture_output=True)
        assert validated.returncode == 0, validated.stderr
        outputs = []
        for i in range(2):
            out = directory / f'{i}.json'
            subprocess.run([binary, str(source), str(out)], check=True)
            outputs.append(out.read_bytes())
        assert outputs[0] == outputs[1], 'fixed-seed loop must reproduce'
        data = json.loads(outputs[0])
        assert data['frames'][0]['time'] == 0 and data['frames'][-1]['time'] == 3
        assert data['versions']['algorithm'] == 'pixel-servo/1'
        packets = data['algorithm_channels']['observations']
        for packet in packets:
            assert packet['ready_tick'] >= packet['capture_tick']
            for observation in packet['observations']:
                assert not {'target_id', 'plate_id', 'velocity', 'active'} & observation.keys()
        for command in data['commands']:
            assert command['due_tick'] >= command['issued_tick'] + 1
        ids = [launch['shot_id'] for launch in data['launches']]
        assert len(ids) == len(set(ids))
        if source.name == 'ideal.json':
            assert data['summary']['effective_hits'] >= 8
            assert data['summary']['contacts'] == data['summary']['effective_hits']
            assert len(packets) == 180
        else:
            assert len(packets) == 432
            assert all(p['ready_tick'] - p['capture_tick'] == 12 for p in packets)
    original = json.loads((folder / 'ideal.json').read_text())
    for name in ['unknown', 'bad_frequency', 'bad_pose', 'bad_latency', 'bad_feature', 'bad_event', 'bad_seed', 'tiny_timestep', 'bad_time']:
        config = copy.deepcopy(original)
        if name == 'unknown': config['sensor']['pixel_sigam'] = 1
        elif name == 'bad_frequency': config['sensor']['frequency'] = 1001
        elif name == 'bad_pose': config['actuator']['camera_mount'] = {'rotation': [[2, 0, 0], [0, 1, 0], [0, 0, 1]]}
        elif name == 'bad_latency': config['sensor']['latency'] = -1
        elif name == 'bad_feature': config['sensor']['appearances'] = {'missing': {'model_id': 'x', 'measured_class': 'armor', 'points': [[0, 0, 0]]}}
        elif name == 'bad_event': config['scene']['events'] = [{'time': 0, 'type': 'stop_outpost', 'target_id': 'target'}]
        elif name == 'bad_seed': config['actuator']['seed'] = -1
        elif name == 'tiny_timestep': config['scene']['world']['timestep'] = 1e-100
        else: config['commands'] = [{'time': 0.0005, 'yaw': 0, 'pitch': 0}]
        source, out = directory / 'invalid.json', directory / 'invalid-output.json'
        source.write_text(json.dumps(config))
        result = subprocess.run([binary, str(source), str(out)], capture_output=True)
        assert result.returncode != 0 and not out.exists(), name
        validation = subprocess.run([binary, '--validate', str(source)], capture_output=True)
        assert validation.returncode != 0, name
print('Closed-loop protocol checks passed')
