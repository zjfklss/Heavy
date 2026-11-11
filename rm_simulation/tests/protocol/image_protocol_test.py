"""Exercise camera defaults, PNG pixels, timestamps and offline trace portability."""
import copy
import json
import pathlib
import struct
import subprocess
import sys
import tempfile
import zlib

binary = sys.argv[1]


def read_png(path):
    data = path.read_bytes()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    offset, compressed = 8, b''
    while offset < len(data):
        size = struct.unpack('>I', data[offset:offset+4])[0]
        kind = data[offset+4:offset+8]
        chunk = data[offset+8:offset+8+size]
        crc = struct.unpack('>I', data[offset+8+size:offset+12+size])[0]
        assert zlib.crc32(kind + chunk) & 0xffffffff == crc
        if kind == b'IHDR':
            width, height, bits, color, compression, filtering, interlace = struct.unpack('>IIBBBBB', chunk)
            assert (bits, color, compression, filtering, interlace) == (8, 2, 0, 0, 0)
        if kind == b'IDAT':
            compressed += chunk
        offset += 12 + size
    raw = zlib.decompress(compressed)
    stride = width * 3
    assert len(raw) == (stride + 1) * height
    previous, pixels = bytearray(stride), bytearray()
    for y in range(height):
        mode = raw[y*(stride+1)]
        row = bytearray(raw[y*(stride+1)+1:(y+1)*(stride+1)])
        assert mode in range(5)
        for i in range(stride):
            a = row[i-3] if i >= 3 else 0
            b = previous[i]
            c = previous[i-3] if i >= 3 else 0
            if mode == 1:
                prediction = a
            elif mode == 2:
                prediction = b
            elif mode == 3:
                prediction = (a+b)//2
            elif mode == 4:
                p = a+b-c
                distances = [abs(p-a), abs(p-b), abs(p-c)]
                prediction = [a, b, c][distances.index(min(distances))]
            else:
                prediction = 0
            row[i] = (row[i] + prediction) & 255
        pixels.extend(row)
        previous = row
    return width, height, pixels


with tempfile.TemporaryDirectory() as temporary:
    root = pathlib.Path(temporary)
    config = {
        'closed_loop_version': 1,
        'scene': {'schema_version': 1, 'duration': .05, 'targets': [
            {'id': 'car', 'kind': 'car', 'position': [2, 0, .3]}]},
        'sensor': {'frequency': 60, 'latency': .0121, 'image': {'noise_sigma': 1.2},
                   'calibration': {'width': 320, 'height': 240, 'fx': 400, 'fy': 400, 'cx': 160, 'cy': 120}},
    }
    source = root/'input.json'
    source.write_text(json.dumps(config))
    subprocess.run([binary, '--validate', str(source)], check=True)
    assert list(root.iterdir()) == [source], 'Validation wrote image artifacts'
    normalized = []
    for name in ['a', 'b']:
        trace = root/f'{name}.json'
        subprocess.run([binary, str(source), str(trace)], check=True)
        result = json.loads(trace.read_text())
        assert result['resolved']['sensor']['output'] == 'image'
        assert result['versions']['algorithm'] == 'none/1'
        assert result['commands'] == [] and result['launches'] == []
        packets = result['algorithm_channels']['observations']
        assert [p['capture_tick'] for p in packets] == [0, 17, 34]
        assert [p['ready_tick'] for p in packets] == [13, 30, 47]
        assert result['summary']['image_frames'] == 3
        assert all(not p['observations'] for p in packets)
        for packet in packets:
            assert not {'target_id', 'plate_id', 'camera_pose'} & packet.keys()
            image = packet['image']
            path = trace.parent/image['file']
            width, height, rgb = read_png(path)
            assert (width, height) == (320, 240)
            assert image['encoding'] == 'bgr8' and image['storage'] == 'png'
            # Verify the PNG contains exactly the BGR frame identified by its content hash.
            bgr = bytearray(rgb)
            bgr[0::3], bgr[2::3] = rgb[2::3], rgb[0::3]
            checksum = 14695981039346656037
            for byte in struct.pack('<II', width, height) + bgr:
                checksum = ((checksum ^ byte) * 1099511628211) & 0xffffffffffffffff
            assert path.stem == f'{checksum:016x}', 'PNG channel conversion changed pixels'
            packet['image']['file'] = path.name
        normalized.append(result)
    assert normalized[0] == normalized[1], 'Camera logs/pixels must reproduce for a fixed seed'
    for key, value in [('output', 'invalid'), ('image', {'light_width': .2}),
                       ('image', {'noise_sigma': -1}), ('image', {'unknown': 1}),
                       ('calibration', {'width': 65536, 'height': 65536})]:
        bad = copy.deepcopy(config)
        bad['sensor'][key] = value
        source.write_text(json.dumps(bad))
        assert subprocess.run([binary, '--validate', str(source)], capture_output=True).returncode != 0
    bad = copy.deepcopy(config)
    bad['sensor']['output'] = 'image'
    bad['algorithm'] = {'kind': 'pixel_servo'}
    source.write_text(json.dumps(bad))
    assert subprocess.run([binary, '--validate', str(source)], capture_output=True).returncode != 0
    config['sensor']['enabled'] = False
    source.write_text(json.dumps(config))
    trace = root/'disabled.json'
    subprocess.run([binary, str(source), str(trace)], check=True)
    assert json.loads(trace.read_text())['summary']['image_frames'] == 0
    assert not (root/'disabled.json.images').exists()
print('Camera PNG, image-only messages, timing, reproducibility and validation passed')
