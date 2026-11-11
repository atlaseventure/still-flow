"""Cache generated video fixtures by their inputs and verify their contents."""

import contextlib
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import uuid
from pathlib import Path

import cv2
import numpy as np


def cached_fixture(source, arguments):
    generator = source / "scripts/generate_video.py"
    ffmpeg = subprocess.run(["ffmpeg", "-version"], check=True, capture_output=True).stdout
    fingerprint = hashlib.sha256(
        generator.read_bytes()
        + (source / "uv.lock").read_bytes()
        + json.dumps(arguments).encode()
        + ffmpeg
        + cv2.__version__.encode()
        + np.__version__.encode()
    ).hexdigest()
    root = source / ".local/test-cache/video"
    root.mkdir(parents=True, exist_ok=True)
    target = root / fingerprint
    names = ("input.mp4", "reference.mp4", "truth.json")

    def valid():
        try:
            manifest = json.loads((target / "manifest.json").read_text())
            return all(manifest[name] == hashlib.sha256((target / name).read_bytes()).hexdigest() for name in names)
        except OSError, KeyError, ValueError:
            return False

    if valid():
        print(f"Synthetic fixture cache hit: {fingerprint}", flush=True)
        return target
    if target.exists():
        with contextlib.suppress(FileNotFoundError):
            target.rename(root / (fingerprint + ".invalid-" + uuid.uuid4().hex))
    with tempfile.TemporaryDirectory(prefix=".generate-", dir=root) as directory:
        staging = Path(directory) / "fixture"
        subprocess.run([sys.executable, str(generator), *arguments, "--output", str(staging)], check=True)
        manifest = {name: hashlib.sha256((staging / name).read_bytes()).hexdigest() for name in names}
        (staging / "manifest.json").write_text(json.dumps(manifest))
        try:
            os.rename(staging, target)
        except OSError:
            if not valid():
                raise
    print(f"Synthetic fixture cached: {fingerprint}", flush=True)
    return target
