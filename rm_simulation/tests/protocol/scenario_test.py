"""Exercise the public scenario protocol and replay, not a second physics implementation."""
import json
import pathlib
import subprocess
import sys
import tempfile

binary, scenarios = sys.argv[1], pathlib.Path(sys.argv[2])
with tempfile.TemporaryDirectory() as work:
    work = pathlib.Path(work)
    for scenario in sorted(scenarios.glob('*.json')):
        a, b = work / 'a.json', work / 'b.json'
        subprocess.run([binary, str(scenario), str(a)], check=True)
        subprocess.run([binary, str(scenario), str(b)], check=True)
        assert a.read_bytes() == b.read_bytes(), 'replay must be deterministic'
        data = json.loads(a.read_text())
        assert data['frames'][0]['time'] == 0
        assert data['frames'][-1]['time'] == data['scenario']['duration']
        assert all(t['surfaces'] for f in data['frames'] for t in f['targets'])
        if scenario.name == 'world_demo.json':
            assert data['summary'] == {'shots': 3, 'contacts': 2, 'effective_hits': 2}
            outpost = next(t for t in data['frames'][-1]['targets'] if t['id'] == 'outpost')
            assert outpost['rotation'][1] == 0
    original = json.loads((scenarios / 'world_demo.json').read_text())
    for change in ['unknown', 'unaligned', 'negative_seed', 'bad_rune', 'bad_marking', 'duplicate']:
        data = json.loads(json.dumps(original))
        if change == 'unknown': data['targets'][0]['raduis'] = 0.3
        elif change == 'unaligned': data['shots'][0]['time'] = 0.2005
        elif change == 'negative_seed': data['targets'][0]['seed'] = -1
        elif change == 'bad_rune': data['targets'][-1]['rune_a'] = 10
        elif change == 'bad_marking': data['targets'][0]['armor_marking'] = 'G'
        else: data['targets'].append(data['targets'][0])
        source, output = work / 'invalid.json', work / 'invalid-output.json'
        source.write_text(json.dumps(data))
        p = subprocess.run([binary, str(source), str(output)], capture_output=True)
        assert p.returncode != 0, change
        assert not output.exists(), 'invalid run created partial output'
print('Scenario replay, known hits, stop event and invalid input tests passed')
