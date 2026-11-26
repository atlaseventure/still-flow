"""Shared media inspection for local video tools."""

import hashlib
import json
import math
import subprocess
from fractions import Fraction
from itertools import pairwise
from pathlib import Path


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def probe(path):
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_streams",
            "-show_frames",
            "-show_entries",
            "stream:frame=best_effort_timestamp_time",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        timeout=300,
    )
    data = json.loads(result.stdout)
    if len(data.get("streams", [])) != 1:
        raise ValueError(f"Expected a video stream: {path}")
    stream = data["streams"][0]
    timestamps = [float(frame["best_effort_timestamp_time"]) for frame in data.get("frames", [])]
    if not timestamps or not all(math.isfinite(t) for t in timestamps):
        raise ValueError(f"Missing or invalid frame timestamps: {path}")
    if any(b <= a for a, b in pairwise(timestamps)):
        raise ValueError(f"Frame timestamps must increase: {path}")
    return {"stream": stream, "timestamps": timestamps}


def matching_timeline(left, right, tolerance=1e-5, check_dimensions=True):
    a, b = left["timestamps"], right["timestamps"]
    if len(a) != len(b):
        raise ValueError(f"Frame count mismatch: {len(a)} versus {len(b)}")
    for key in ("width", "height"):
        if check_dimensions and left["stream"][key] != right["stream"][key]:
            raise ValueError(f"Video {key} mismatch")
    difference = max(abs((x - a[0]) - (y - b[0])) for x, y in zip(a, b, strict=True))
    if difference > tolerance:
        raise ValueError(f"Presentation timestamp difference {difference:.9f}s exceeds {tolerance}s")
    return difference


def android_input(media):
    stream = media["stream"]
    width, height = stream["width"], stream["height"]
    if any(x < 32 or x > 4096 or x % 2 for x in (width, height)):
        raise ValueError("Input dimensions must be even and between 32 and 4096")
    if stream.get("codec_name") != "h264" or stream.get("pix_fmt") not in {"yuv420p", "yuvj420p"}:
        raise ValueError("Use an 8-bit H.264 YUV420 input")
    if stream.get("field_order", "progressive") not in {"progressive", "unknown"}:
        raise ValueError("Use progressive video")
    if any(float(item.get("rotation", 0)) % 360 != 0 for item in stream.get("side_data_list", [])):
        raise ValueError("Bake display rotation into the input pixels before processing")
    if float(stream.get("tags", {}).get("rotate", 0)) % 360 != 0:
        raise ValueError("Bake display rotation into the input pixels before processing")
    if stream.get("sample_aspect_ratio", "1:1") not in {"1:1", "N/A"}:
        raise ValueError("Use square pixels")
    if stream.get("color_transfer") in {"smpte2084", "arib-std-b67"}:
        raise ValueError("Convert HDR to SDR before processing")
    matrix = stream.get("color_space")
    if matrix not in {"bt709", "smpte170m", "bt470bg"}:
        raise ValueError("Input must declare a BT.709 or BT.601 color matrix")
    fps = float(Fraction(stream["avg_frame_rate"]))
    if not math.isfinite(fps) or fps <= 0 or media["timestamps"][0] < 0:
        raise ValueError("Invalid frame rate or negative input timestamp")
    return {
        "width": width,
        "height": height,
        "fps": fps,
        "bt709": matrix == "bt709",
        "fullRange": stream.get("color_range") == "pc",
    }
