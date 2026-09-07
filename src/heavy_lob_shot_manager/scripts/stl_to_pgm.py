#!/usr/bin/python3
"""把 RMUC 2026 STL 俯视栅格化成 Nav2 用地图。"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    verts = np.empty((n, 3, 3), dtype=np.float32)
    off = 84
    for i in range(n):
        verts[i] = np.frombuffer(data, dtype=np.float32, count=9, offset=off + 12).reshape(3, 3)
        off += 50
    return verts


def rasterize(
    verts: np.ndarray,
    pose_xyz: tuple[float, float, float],
    res: float,
    z_occ: float,
) -> tuple[np.ndarray, float, float]:
    pts = verts.reshape(-1, 3) + np.array(pose_xyz, dtype=np.float32)
    xmin, ymin = 0.0, 0.0
    xmax = float(np.ceil(pts[:, 0].max() / res) * res)
    ymax = float(np.ceil(pts[:, 1].max() / res) * res)
    w = int(round((xmax - xmin) / res))
    h = int(round((ymax - ymin) / res))
    occ = np.zeros((h, w), dtype=bool)
    xs = ((pts[:, 0] - xmin) / res).astype(np.int32)
    ys = ((pts[:, 1] - ymin) / res).astype(np.int32)
    inb = (xs >= 0) & (ys >= 0) & (xs < w) & (ys < h)
    high = pts[:, 2] >= z_occ
    occ[ys[inb & high], xs[inb & high]] = True
    img = np.full((h, w), 205, dtype=np.uint8)
    img[ys[inb], xs[inb]] = 254
    img[occ] = 0
    return img, xmin, ymin


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--stl', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--pose', nargs=3, type=float, default=[14.5, 8.0, -0.23])
    parser.add_argument('--res', type=float, default=0.05)
    parser.add_argument('--z-occ', type=float, default=0.35)
    args = parser.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    verts = read_binary_stl(Path(args.stl))
    img, ox, oy = rasterize(verts, tuple(args.pose), args.res, args.z_occ)
    pgm = out / 'rmuc_2026.pgm'
    yaml = out / 'rmuc_2026.yaml'
    header = (
        f'P5\n# RMUC 2026 from STL\n{img.shape[1]} {img.shape[0]}\n255\n'
    ).encode()
    pgm.write_bytes(header + np.flipud(img).tobytes())
    yaml.write_text(
        f'image: rmuc_2026.pgm\nmode: trinary\nresolution: {args.res}\n'
        f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
        encoding='utf-8',
    )
    print(f'wrote {pgm} {img.shape[1]}x{img.shape[0]} origin=({ox},{oy})')


if __name__ == '__main__':
    main()
