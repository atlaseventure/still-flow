#!/usr/bin/env python3
"""Compare frame timing and image geometry; export a reference/candidate preview."""

import argparse
import json
import math
import subprocess
from pathlib import Path

from video_tools import digest, matching_timeline, probe


def compare(reference, candidate, output, max_displacement=None, preview=True, jobs=12):
    import cv2
    import numpy as np

    cv2.setNumThreads(min(jobs, 2))
    cv2.setRNGSeed(0)
    left, right = probe(reference), probe(candidate)
    pts_error = matching_timeline(left, right)
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "reference": {"path": str(reference), "sha256": digest(reference)},
        "candidate": {"path": str(candidate), "sha256": digest(candidate)},
        "frames": len(left["timestamps"]),
        "width": left["stream"]["width"],
        "height": left["stream"]["height"],
        "max_pts_difference_seconds": pts_error,
        "max_displacement_pixels": max_displacement,
        "measurements": [],
    }
    caps = [cv2.VideoCapture(str(path)) for path in (reference, candidate)]
    try:
        for index in range(report["frames"]):
            values = [cap.read() for cap in caps]
            if not all(ok for ok, _ in values):
                raise RuntimeError(f"Decode failed at frame {index}")
            a, b = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for _, frame in values]
            p = cv2.goodFeaturesToTrack(a, 800, 0.01, 10)
            record = {"frame": index, "status": "insufficient_tracks"}
            if p is not None and len(p) >= 16:
                q, status, _ = cv2.calcOpticalFlowPyrLK(a, b, p, None, winSize=(31, 31), maxLevel=3)
                back, backward, _ = cv2.calcOpticalFlowPyrLK(b, a, q, None, winSize=(31, 31), maxLevel=3)
                valid = (
                    (status.ravel() > 0) & (backward.ravel() > 0) & (np.linalg.norm(back[:, 0] - p[:, 0], axis=1) < 0.5)
                )
                if valid.sum() >= 16:
                    matrix, inliers = cv2.estimateAffinePartial2D(
                        p[valid],
                        q[valid],
                        method=cv2.RANSAC,
                        ransacReprojThreshold=0.5,
                    )
                    if matrix is not None and inliers.sum() >= 16:
                        displacement = np.linalg.norm(q[valid, 0] - p[valid, 0], axis=1)
                        center = np.array([a.shape[1] / 2, a.shape[0] / 2])
                        shift = matrix[:, :2] @ center + matrix[:, 2] - center
                        record.update(
                            status="measured",
                            tracks=int(valid.sum()),
                            p95_displacement_pixels=float(np.percentile(displacement, 95)),
                            center_shift_pixels=float(np.linalg.norm(shift)),
                            rotation_degrees=float(np.degrees(np.arctan2(matrix[1, 0], matrix[0, 0]))),
                            scale=float(np.linalg.norm(matrix[:, 0])),
                        )
            report["measurements"].append(record)
        if any(cap.read()[0] for cap in caps):
            raise RuntimeError("Decoder produced unexpected extra frames")
    finally:
        for cap in caps:
            cap.release()
    measured = [r for r in report["measurements"] if r["status"] == "measured"]
    worst = max((r["p95_displacement_pixels"] for r in measured), default=None)
    report.update(
        measured_frames=len(measured),
        worst_p95_displacement_pixels=worst,
        status="measured" if measured else "insufficient_tracks",
    )
    if max_displacement is not None:
        report["status"] = "passed" if len(measured) == report["frames"] and worst <= max_displacement else "failed"
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    if preview:
        width = min(report["width"], 960)
        width -= width % 2
        filters = (
            f"[0:v]setpts=PTS-STARTPTS,scale={width}:-2,pad=iw:ih+40:0:40:black,"
            "drawtext=font=monospace:text=Reference:fontcolor=white:fontsize=24:x=20:y=7[a];"
            f"[1:v]setpts=PTS-STARTPTS,scale={width}:-2,pad=iw:ih+40:0:40:black,"
            "drawtext=font=monospace:text=Candidate:fontcolor=white:fontsize=24:x=20:y=7[b];"
            "[a][b]hstack=inputs=2:shortest=1[v]"
        )
        with (output / "preview.log").open("wb") as log:
            subprocess.run(
                [
                    "ffmpeg",
                    "-v",
                    "error",
                    "-n",
                    "-i",
                    str(reference),
                    "-i",
                    str(candidate),
                    "-filter_complex",
                    filters,
                    "-map",
                    "[v]",
                    "-an",
                    "-c:v",
                    "libx264",
                    "-threads",
                    str(jobs),
                    "-preset",
                    "fast",
                    "-crf",
                    "19",
                    "-pix_fmt",
                    "yuv420p",
                    "-fps_mode",
                    "passthrough",
                    "-movflags",
                    "+faststart",
                    str(output / "comparison.mp4"),
                ],
                check=True,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=1800,
            )
    print(json.dumps({k: v for k, v in report.items() if k != "measurements"}), flush=True)
    if report["status"] == "failed":
        raise RuntimeError(f"Geometry comparison failed; see {output / 'metrics.json'}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New artifact directory")
    parser.add_argument(
        "--max-displacement", type=float, help="Require every frame's p95 displacement below this pixel limit"
    )
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument("--jobs", type=int, default=12)
    args = parser.parse_args()
    if args.jobs < 1 or (
        args.max_displacement is not None and (not math.isfinite(args.max_displacement) or args.max_displacement <= 0)
    ):
        parser.error("jobs and max-displacement must be positive")
    try:
        compare(
            args.reference.resolve(),
            args.candidate.resolve(),
            args.output.resolve(),
            args.max_displacement,
            not args.no_preview,
            args.jobs,
        )
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "failed", "error": str(error)}), flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
