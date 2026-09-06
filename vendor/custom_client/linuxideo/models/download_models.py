#!/usr/bin/env python3
"""Download FSRCNN super-resolution models for the "D: Neural" SR mode.

The models are the small (~40-60KB) FSRCNN .pb files published by the OpenCV
contrib `dnn_superres` module. They are fetched into this folder with the exact
filenames the viewer expects: FSRCNN_x2.pb / FSRCNN_x3.pb / FSRCNN_x4.pb

Usage:
    python3 download_models.py            # download x2, x3, x4
    python3 download_models.py --scale 3  # only x3
"""

import argparse
import sys
import urllib.request
from pathlib import Path

MODELS_DIR = Path(__file__).resolve().parent

# Official OpenCV dnn_superres sample models (Saafke/FSRCNN_Tensorflow, the
# repo linked from the OpenCV docs). Use the raw.githubusercontent.com direct
# link to avoid the github.com -> codeload redirect. Mirrors are tried in order.
RAW_BASE = (
    "https://raw.githubusercontent.com/Saafke/FSRCNN_Tensorflow/master/models"
)
MIRRORS = [
    "https://raw.githubusercontent.com/Saafke/FSRCNN_Tensorflow/master/models",
    "https://cdn.jsdelivr.net/gh/Saafke/FSRCNN_Tensorflow@master/models",
    "https://github.com/Saafke/FSRCNN_Tensorflow/raw/master/models",
]

MODELS = {
    2: "FSRCNN_x2.pb",
    3: "FSRCNN_x3.pb",
    4: "FSRCNN_x4.pb",
}


def download(filename: str, dest: Path):
    last_err = None
    for base in MIRRORS:
        url = f"{base}/{filename}"
        try:
            print(f"  fetching {url}")
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req, timeout=30) as resp:
                data = resp.read()
            if len(data) < 1024:
                raise RuntimeError(f"file too small ({len(data)} bytes), likely an error page")
            dest.write_bytes(data)
            print(f"  saved {dest.name} ({len(data)} bytes)")
            return
        except Exception as e:
            print(f"  mirror failed: {e}")
            last_err = e
    raise RuntimeError(f"all mirrors failed (last: {last_err})")


def main():
    parser = argparse.ArgumentParser(description="Download FSRCNN SR models")
    parser.add_argument("--scale", type=int, choices=[2, 3, 4],
                        help="Download only this scale (default: all)")
    args = parser.parse_args()

    scales = [args.scale] if args.scale else [2, 3, 4]
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    failures = []
    for scale in scales:
        filename = MODELS[scale]
        dest = MODELS_DIR / filename
        if dest.exists() and dest.stat().st_size > 1024:
            print(f"x{scale}: already present ({dest.name}), skipping")
            continue
        print(f"x{scale}:")
        try:
            download(filename, dest)
        except Exception as e:
            print(f"  FAILED: {e}", file=sys.stderr)
            failures.append((scale, str(e)))

    print()
    if failures:
        print("Some downloads failed:")
        for scale, err in failures:
            print(f"  x{scale}: {err}")
        print()
        print("Manual fallback — download these files into this folder:")
        for scale, _err in failures:
            filename = MODELS[scale]
            print(f"  {RAW_BASE}/{filename}\n    -> {MODELS_DIR / filename}")
        sys.exit(1)

    print(f"Done. Models are in {MODELS_DIR}")
    print("Select 'D: Neural (FSRCNN)' in the viewer's View > Super Resolution menu.")


if __name__ == "__main__":
    main()
