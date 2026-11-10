#!/usr/bin/env python3
"""Generate a reproducible shaky clip with optional rolling shutter and its stable reference."""

import argparse
import json
import shutil
import subprocess
from contextlib import ExitStack, contextmanager, suppress
from pathlib import Path
from tempfile import TemporaryDirectory


@contextmanager
def encoding(process):
    try:
        yield process
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError("Fixture encoding failed")
    finally:
        if process.poll() is None:
            process.terminate()
        if not process.stdin.closed:
            with suppress(BrokenPipeError):
                process.stdin.close()
        process.wait()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--frames", type=int, default=180)
    p.add_argument(
        "--rolling-shutter",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Simulate rolling shutter (default: enabled)",
    )
    p.add_argument("--output", type=Path, default=Path(".local/validation/720p"))
    p.add_argument("--overwrite", action="store_true")
    args = p.parse_args()
    if args.width < 32 or args.height < 32 or args.width % 2 or args.height % 2:
        p.error("width and height must be even integers of at least 32")
    if args.frames < 1:
        p.error("frames must be positive")
    if shutil.which("ffmpeg") is None:
        p.error("ffmpeg must be available on PATH")
    try:
        import cv2
        import numpy as np
    except ImportError:
        p.error("run with: uv run --locked --group video python scripts/generate_video.py [options]")
    names = ("input.mp4", "reference.mp4", "truth.json")
    for name in names:
        target = args.output / name
        if target.exists() and (not args.overwrite or not target.is_file()):
            p.error(f"output exists: {target}; use --overwrite to replace files")
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    w, h, fps = args.width, args.height, 30
    readout_seconds = 0.018 if args.rolling_shutter else 0.0
    rng = np.random.default_rng(42)
    canvas = np.full((h + 512, w + 512, 3), 65, np.uint8)
    for y in range(0, canvas.shape[0], 32):
        for x in range(0, canvas.shape[1], 40):
            color = tuple(int(c) for c in rng.integers(50, 200, 3))
            cv2.rectangle(canvas, (x + 2, y + 2), (x + 37, y + 29), color, -1)
    for _ in range(1800):
        x, y = rng.integers(0, canvas.shape[1]), rng.integers(0, canvas.shape[0])
        cv2.circle(canvas, (int(x), int(y)), int(rng.integers(1, 5)), (225, 215, 200), -1)
    for x in range(100, canvas.shape[1], 180):
        cv2.line(canvas, (x, 0), (x, canvas.shape[0] - 1), (235, 235, 235), 3)
    for y in range(100, canvas.shape[0], 180):
        cv2.line(canvas, (0, y), (canvas.shape[1] - 1, y), (25, 25, 25), 4)
    for y in range(150, canvas.shape[0], 160):
        cv2.putText(
            canvas, "STILL FLOW  012345  C++20", (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240, 240, 240), 2, cv2.LINE_AA
        )
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    scale = w / 1280

    def encoder(path):
        return subprocess.Popen(
            [
                "ffmpeg",
                "-v",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pixel_format",
                "bgr24",
                "-video_size",
                f"{w}x{h}",
                "-framerate",
                str(fps),
                "-i",
                "-",
                "-f",
                "lavfi",
                "-i",
                f"sine=frequency=440:sample_rate=48000:duration={args.frames / fps}",
                "-map",
                "0:v",
                "-map",
                "1:a",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "fast",
                "-crf",
                "15",
                "-pix_fmt",
                "yuv420p",
                "-colorspace",
                "bt709",
                "-color_primaries",
                "bt709",
                "-color_trc",
                "bt709",
                "-c:a",
                "aac",
                "-shortest",
                str(path),
            ],
            stdin=subprocess.PIPE,
        )

    with TemporaryDirectory(prefix=".generate-", dir=args.output) as directory:
        temporary = Path(directory)
        with ExitStack() as stack:
            source = stack.enter_context(encoding(encoder(temporary / "input.mp4")))
            reference = stack.enter_context(encoding(encoder(temporary / "reference.mp4")))
            for i in range(args.frames):
                t = i / fps
                # Continuous camera pose sampled at the exposure time of each scanline.
                row_time = t + (yy / (h - 1) - 0.5) * readout_seconds
                pan = 9 * t * scale
                dx = scale * (
                    7 * np.sin(2 * np.pi * 6 * row_time)
                    + 5 * np.sin(2 * np.pi * 9.2 * row_time)
                    + 7 * np.sin(2 * np.pi * 42 * row_time)
                )
                dy = scale * 5 * np.sin(2 * np.pi * 5.3 * row_time)
                angle = 0.003 * np.sin(2 * np.pi * 4.1 * row_time)
                mx = np.cos(angle) * (xx - w / 2) - np.sin(angle) * (yy - h / 2) + w / 2 + 256 + pan + dx
                my = np.sin(angle) * (xx - w / 2) + np.cos(angle) * (yy - h / 2) + h / 2 + 256 + dy
                shaken = cv2.remap(canvas, mx.astype(np.float32), my.astype(np.float32), cv2.INTER_LINEAR)
                stable = cv2.remap(canvas, xx + 256 + pan, yy + 256, cv2.INTER_LINEAR)
                source.stdin.write(shaken.tobytes())
                reference.stdin.write(stable.tobytes())
        (temporary / "truth.json").write_text(
            json.dumps(
                {
                    "width": w,
                    "height": h,
                    "frames": args.frames,
                    "fps": fps,
                    "readout_seconds": readout_seconds,
                    "pan_pixels_per_second": 9 * scale,
                    "camera_jitter_hz": [6, 9.2, 42],
                    "seed": 42,
                    "description": "Static textured scene; continuous camera pose sampled per row; reference retains deliberate pan.",
                },
                indent=2,
            )
        )
        for name in names:
            (temporary / name).replace(args.output / name)
    print(args.output)


if __name__ == "__main__":
    main()
