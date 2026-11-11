"""Prepare a deterministic CLI regression and measure motion against its reference."""

import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
from video_fixture import cached_fixture


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cli", required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fixture = cached_fixture(
        args.source, ["--width", "320", "--height", "180", "--frames", "60", "--no-rolling-shutter"]
    )
    args.output.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            args.cli,
            str(fixture / "input.mp4"),
            str(args.output / "output.mp4"),
            "--backend",
            "cpu",
            "--decoder",
            "cpu",
            "--crop-mode",
            "fixed",
            "--crop",
            "0.08",
            "--rs",
            "off",
            "--threads",
            "2",
            "--overwrite",
        ],
        check=True,
    )

    def read(name):
        result = subprocess.run(
            [
                "ffmpeg",
                "-v",
                "error",
                "-i",
                str((args.output if name == "output.mp4" else fixture) / name),
                "-f",
                "rawvideo",
                "-pix_fmt",
                "gray",
                "-",
            ],
            check=True,
            capture_output=True,
        )
        return np.frombuffer(result.stdout, np.uint8).reshape(-1, 180, 320).astype(np.float32)

    source, reference, output = (read(name) for name in ("input.mp4", "reference.mp4", "output.mp4"))
    if len(source) != 60 or len(reference) != 60 or len(output) != 60:
        raise RuntimeError("Frame count changed")
    yy, xx = np.mgrid[:180, :320].astype(np.float32)
    mx, my = 0.08 * 319 + xx * 0.84, 0.08 * 179 + yy * 0.84
    source = np.array([cv2.remap(f, mx, my, cv2.INTER_LINEAR) for f in source])
    reference = np.array([cv2.remap(f, mx, my, cv2.INTER_LINEAR) for f in reference])

    def jitter(frames):
        shifts = np.array(
            [
                cv2.phaseCorrelate(ref[16:-16, 16:-16].copy(), f[16:-16, 16:-16].copy())[0]
                for ref, f in zip(reference, frames, strict=True)
            ]
        )
        return float(np.sqrt(np.mean(np.diff(shifts, axis=0) ** 2)))

    before, after = jitter(source), jitter(output)
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-show_streams", "-of", "json", str(args.output / "output.mp4")],
        check=True,
        capture_output=True,
        text=True,
    )
    streams = json.loads(probe.stdout)["streams"]
    audio = sum(s["codec_type"] == "audio" for s in streams)
    report = json.loads((args.output / "output.mp4.profile.json").read_text())
    lgpl = int(report["build"]["ffmpeg_license"].startswith("LGPL"))
    encoder = report["codecs"]["encoder"]["implementation"]
    (args.output / "metrics.txt").write_text(
        f"{len(output)} {audio} {before} {after} {report['video']['frames']} {lgpl} {encoder}\n"
    )
    print(f"Reference-relative jitter: {before:.6f} -> {after:.6f}")


if __name__ == "__main__":
    main()
