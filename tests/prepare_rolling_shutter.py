"""Measure startup deformation in offline and realtime outputs of one cached fixture."""

import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
from video_fixture import cached_fixture


def read_video(path):
    capture = cv2.VideoCapture(str(path))
    frames = []
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape[:2] != (720, 1280):
                raise RuntimeError(f"Unexpected video dimensions: {path}")
            frames.append(cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (640, 360)))
    finally:
        capture.release()
    if len(frames) != 90:
        raise RuntimeError(f"Expected 90 decoded frames, got {len(frames)}: {path}")
    return frames


def deformation(reference, frames):
    """Remove a global similarity; measure remaining 2D error in output pixels."""
    residuals = []
    mask = np.zeros((360, 640), np.uint8)
    mask[24:-24, 24:-24] = 255
    for index, (ref, frame) in enumerate(zip(reference, frames, strict=True)):
        points = cv2.goodFeaturesToTrack(ref, 800, 0.01, 8, mask=mask)
        if points is None or len(points) < 100:
            raise RuntimeError(f"Insufficient reference features at frame {index}")
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(ref, frame, points, None, winSize=(31, 31), maxLevel=3)
        back, backward, _ = cv2.calcOpticalFlowPyrLK(
            frame, ref, tracked, points.copy(), winSize=(31, 31), maxLevel=3, flags=cv2.OPTFLOW_USE_INITIAL_FLOW
        )
        valid = (
            (status.ravel() > 0) & (backward.ravel() > 0) & (np.linalg.norm(back[:, 0] - points[:, 0], axis=1) < 0.8)
        )
        a, b = points[valid, 0], tracked[valid, 0]
        if len(a) < 100 or np.ptp(a[:, 1]) < 250:
            raise RuntimeError(f"Insufficient matching coverage at frame {index}")
        # Least squares uses every consistent match: a robust fit could discard
        # the distorted rows whose residual this regression needs to measure.
        design = np.zeros((2 * len(a), 4))
        design[::2, :2] = a
        design[1::2, 0], design[1::2, 1] = a[:, 1], -a[:, 0]
        design[::2, 2], design[1::2, 3] = 1, 1
        model = np.linalg.lstsq(design, b.ravel(), rcond=None)[0]
        error = (b.ravel() - design @ model).reshape(-1, 2) * 2
        rms = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
        if not np.isfinite(rms):
            raise RuntimeError(f"Nonfinite deformation at frame {index}")
        residuals.append(rms)
    return residuals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    fixture = cached_fixture(args.source, ["--width", "1280", "--height", "720", "--frames", "90", "--rolling-shutter"])
    args.output.mkdir(parents=True, exist_ok=True)
    yy, xx = np.mgrid[:360, :640].astype(np.float32)

    def crop(frames):
        return [cv2.remap(frame, 0.08 * 639 + xx * 0.84, 0.08 * 359 + yy * 0.84, cv2.INTER_LINEAR) for frame in frames]

    reference = crop(read_video(fixture / "reference.mp4"))
    before = deformation(reference, crop(read_video(fixture / "input.mp4")))
    for mode in ("offline", "realtime"):
        output = args.output / f"{mode}.mp4"
        subprocess.run(
            [
                args.cli,
                str(fixture / "input.mp4"),
                str(output),
                "--mode",
                mode,
                "--backend",
                "cpu",
                "--decoder",
                "cpu",
                "--encoder",
                "openh264",
                "--crop-mode",
                "fixed",
                "--crop",
                "0.08",
                "--rs",
                "on",
                "--threads",
                "2",
                "--overwrite",
            ],
            check=True,
        )
        after = deformation(reference, read_video(output))
        metrics = [
            len(after),
            np.mean(before[:10]),
            after[0],
            np.mean(after[:10]),
            np.mean(after[:30]),
            np.mean(after[30:60]),
        ]
        (args.output / f"{mode}.txt").write_text(" ".join(str(value) for value in metrics) + "\n")
        (args.output / f"{mode}.json").write_text(
            json.dumps({"input_rms_px": before, "output_rms_px": after}, indent=2) + "\n"
        )
        print(f"{mode} startup deformation: {metrics}", flush=True)


if __name__ == "__main__":
    main()
