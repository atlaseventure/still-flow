#!/usr/bin/env python3
"""Build the AAR consumer, process a video on Android and export reproducible artifacts."""

import argparse
import json
import math
import os
import shlex
import shutil
import subprocess
import uuid
from pathlib import Path

from video_tools import android_input, digest, matching_timeline, probe

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = "io.stillflow.runner"


def event(stage, **values):
    print(json.dumps({"stage": stage, **values}), flush=True)


class Device:
    def __init__(self, executable, serial):
        self.command = [executable]
        version = self.run("version").decode(errors="replace")
        self.windows_adb = os.name != "nt" and ".exe" in version.lower()
        devices = self.run("devices").decode().splitlines()[1:]
        ready = [line.split()[0] for line in devices if len(line.split()) == 2 and line.split()[1] == "device"]
        if serial is None:
            if len(ready) != 1:
                raise RuntimeError("Connect exactly one authorized device or select one with --serial")
            serial = ready[0]
        if serial not in ready:
            raise RuntimeError(f"Device is not online and authorized: {serial}")
        self.serial = serial
        self.command += ["-s", serial]
        api = int(self.shell("getprop", "ro.build.version.sdk").decode().strip())
        abi = self.shell("getprop", "ro.product.cpu.abilist").decode().strip().split(",")
        if api < 29 or "arm64-v8a" not in abi:
            raise RuntimeError("The runner requires Android 10+ and arm64-v8a")

    def run(self, *arguments, timeout=120, stdout=None):
        result = subprocess.run(
            self.command + list(arguments),
            check=True,
            stdout=stdout or subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
        return result.stdout

    def shell(self, *arguments):
        return self.run("shell", shlex.join(arguments))

    def local_path(self, path):
        if self.windows_adb:
            return subprocess.check_output(["wslpath", "-w", str(path)], text=True).strip()
        return str(path)

    def push(self, source, destination):
        self.run("push", self.local_path(source), destination)

    def export(self, source, destination):
        # exec-out and a binary file preserve bytes across the WSL/Windows adb boundary.
        with destination.open("xb") as stream:
            self.run("exec-out", "run-as", PACKAGE, "cat", source, stdout=stream)

    def clean(self, directory, app=False):
        prefix = ["run-as", PACKAGE] if app else []
        names = self.shell(*prefix, "ls", directory).decode().split()
        allowed = {"input.mp4", "config.json", "output.mp4", "result.json"}
        if not set(names) <= allowed:
            raise RuntimeError(f"Unexpected files in owned run directory: {directory}")
        if names:
            self.shell(*prefix, "rm", *(f"{directory}/{name}" for name in names))
        self.shell(*prefix, "rmdir", directory)


def logged(command, path, timeout):
    event("command", log=str(path))
    with path.open("wb") as stream:
        subprocess.run(command, cwd=ROOT, check=True, stdout=stream, stderr=subprocess.STDOUT, timeout=timeout)


def run(args):
    source = args.input.resolve(strict=True)
    media = probe(source)
    config = android_input(media)
    config.update(
        realtime=args.realtime,
        bitrate=args.bitrate,
        horizontalScan=args.horizontal_scan,
        settings={"backend": args.backend.upper()},
    )
    keys = {
        "motion_estimator": "motionEstimator",
        "analysis_width": "analysisWidth",
        "features": "features",
        "bands": "bands",
        "crop_mode": "cropMode",
        "rolling_shutter": "rollingShutter",
        "smooth_seconds": "smoothSeconds",
        "rs_seconds": "rsSeconds",
        "crop": "crop",
        "rs_strength": "rsStrength",
    }
    for name, field in keys.items():
        value = getattr(args, name)
        if value is not None:
            config["settings"][field] = value.upper() if isinstance(value, str) else value
    device = Device(args.adb, args.serial)
    run_id = uuid.uuid4().hex
    output = args.output.resolve() if args.output else ROOT / ".local/android-runs" / run_id
    output.mkdir(parents=True, exist_ok=False)
    manifest = {
        "status": "running",
        "run": run_id,
        "serial": device.serial,
        "input": {"path": str(source), "sha256": digest(source)},
        "config": config,
        "revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT)),
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    temporary = f"/data/local/tmp/stillflow-{run_id}"
    app_directory = f"files/{run_id}"
    remote_created = app_created = installed = False
    try:
        wrapper = str(ROOT / "api/android/gradlew")
        aar = (
            args.aar.resolve(strict=True) if args.aar else ROOT / "api/android/build/outputs/aar/stillflow-release.aar"
        )
        if args.aar is None:
            logged(
                [
                    wrapper,
                    "-p",
                    "api/android",
                    "assembleRelease",
                    "lintRelease",
                    "--console",
                    "plain",
                    f"-Pstillflow.jobs={args.jobs}",
                    f"--max-workers={args.jobs}",
                ],
                output / "aar-build.log",
                args.timeout,
            )
        logged(
            [
                wrapper,
                "-p",
                "tools/android",
                "assembleRelease",
                "lintRelease",
                "--console",
                "plain",
                f"-Pstillflow.aar={aar}",
                f"--max-workers={args.jobs}",
            ],
            output / "app-build.log",
            args.timeout,
        )
        apk_dir = ROOT / "tools/android/build/outputs/apk/release"
        metadata = json.loads((apk_dir / "output-metadata.json").read_text())
        if len(metadata["elements"]) != 1:
            raise RuntimeError("Expected one runner APK")
        apk = apk_dir / metadata["elements"][0]["outputFile"]
        manifest.update(aar_sha256=digest(aar), apk_sha256=digest(apk))
        event("install", serial=device.serial)
        (output / "install.log").write_bytes(device.run("install", "-r", device.local_path(apk), timeout=args.timeout))
        installed = True
        device.shell("mkdir", temporary)
        remote_created = True
        device.shell("run-as", PACKAGE, "mkdir", "-p", app_directory)
        app_created = True
        config_path = output / "config.json"
        config_path.write_text(json.dumps(config, indent=2) + "\n")
        for path, name in ((source, "input.mp4"), (config_path, "config.json")):
            device.push(path, f"{temporary}/{name}")
            device.shell("run-as", PACKAGE, "cp", f"{temporary}/{name}", f"{app_directory}/{name}")
        remote_hash = device.shell("run-as", PACKAGE, "sha256sum", f"{app_directory}/input.mp4").decode().split()[0]
        if remote_hash != manifest["input"]["sha256"]:
            raise RuntimeError("Input transfer checksum mismatch")
        event("process", serial=device.serial, run=run_id)
        with (output / "device.log").open("wb") as stream:
            device.run(
                "shell",
                "am",
                "instrument",
                "-w",
                "-r",
                "-e",
                "run",
                run_id,
                f"{PACKAGE}/{PACKAGE}.ExportRunner",
                timeout=args.timeout,
                stdout=stream,
            )
        log = (output / "device.log").read_text().replace("\r", "")
        if f"STILLFLOW_COMPLETE {run_id}" not in log or "INSTRUMENTATION_CODE: -1" not in log:
            raise RuntimeError(f"Instrumentation failed; see {output / 'device.log'}")
        device.export(f"{app_directory}/result.json", output / "device.json")
        result = json.loads((output / "device.json").read_text())
        if result.get("status") != "passed" or result.get("run") != run_id:
            raise RuntimeError("Missing successful result for this run")
        device.export(f"{app_directory}/output.mp4", output / "output.mp4")
        expected = device.shell("run-as", PACKAGE, "sha256sum", f"{app_directory}/output.mp4").decode().split()[0]
        if digest(output / "output.mp4") != expected:
            raise RuntimeError("Output transfer checksum mismatch")
        pts_error = matching_timeline(media, probe(output / "output.mp4"))
        if result["frames"] != len(media["timestamps"]):
            raise RuntimeError("Reported frame count mismatch")
        if args.backend == "gles" and (not result["gpu"] or result["backend"] != "gles"):
            raise RuntimeError("Requested GLES execution was not active")
        manifest.update(device=result, output_sha256=expected, max_pts_difference_seconds=pts_error)
        if args.reference:
            from compare_video import compare

            compare(
                args.reference.resolve(strict=True),
                output / "output.mp4",
                output / "comparison",
                args.max_displacement,
                not args.no_preview,
                args.jobs,
            )
        manifest["status"] = "passed"
    except BaseException as error:
        manifest.update(status="failed", error=str(error))
        raise
    finally:
        if installed:
            try:
                device.shell("am", "force-stop", PACKAGE)
            except (OSError, subprocess.SubprocessError) as error:
                manifest["stop_error"] = str(error)
        for created, directory, app in ((app_created, app_directory, True), (remote_created, temporary, False)):
            if created:
                try:
                    device.clean(directory, app)
                except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
                    manifest.setdefault("cleanup_errors", []).append(str(error))
                    event("cleanup", status="failed", directory=directory, error=str(error))
        cleanup_failed = manifest["status"] == "passed" and ("cleanup_errors" in manifest or "stop_error" in manifest)
        if cleanup_failed:
            manifest.update(status="failed", error="Device cleanup failed; see manifest.json")
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
        if cleanup_failed:
            raise RuntimeError(manifest["error"])
    event("complete", status="passed", output=str(output), frames=manifest["device"]["frames"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, help="New artifact directory; defaults to .local/android-runs/<run>")
    parser.add_argument("--serial", help="Required when multiple devices are connected")
    parser.add_argument("--adb", default="adb")
    parser.add_argument("--aar", type=Path, help="Use an existing AAR instead of building the SDK")
    parser.add_argument("--jobs", type=int, default=12)
    parser.add_argument("--timeout", type=int, default=1800, help="Per-stage timeout in seconds")
    parser.add_argument("--bitrate", type=int, default=12_000_000)
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--backend", choices=("auto", "cpu", "gles"), default="gles")
    parser.add_argument("--motion-estimator", choices=("opencv", "translation"))
    parser.add_argument("--analysis-width", type=int)
    parser.add_argument("--features", type=int)
    parser.add_argument("--bands", type=int)
    parser.add_argument("--crop-mode", choices=("auto", "fixed", "adaptive"))
    parser.add_argument("--rolling-shutter", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--horizontal-scan", action="store_true")
    parser.add_argument("--smooth-seconds", type=float)
    parser.add_argument("--rs-seconds", type=float)
    parser.add_argument("--crop", type=float)
    parser.add_argument("--rs-strength", type=float)
    parser.add_argument("--reference", type=Path, help="Optional host output to compare with the exported video")
    parser.add_argument(
        "--max-displacement", type=float, help="Require matching geometry against --reference, in pixels"
    )
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()
    if min(args.jobs, args.timeout, args.bitrate) <= 0:
        parser.error("jobs, timeout and bitrate must be positive")
    if args.max_displacement is not None and (
        not args.reference or not math.isfinite(args.max_displacement) or args.max_displacement <= 0
    ):
        parser.error("max-displacement requires a reference and a positive finite value")
    for name in ("smooth_seconds", "rs_seconds", "crop", "rs_strength"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0):
            parser.error(f"{name} must be finite and nonnegative")
    if not shutil.which(args.adb):
        parser.error("adb was not found")
    try:
        run(args)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        event("run", status="failed", error=str(error))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
