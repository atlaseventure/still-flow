"""Resolve package/project calibration and camera-clock IMU data for the Android runner."""

import csv
import json
import math
import struct
from bisect import bisect_left
from itertools import pairwise

RATE_COLUMNS = ["t_video_s", "wx", "wy", "wz"]
ACCEL_COLUMNS = ["ax", "ay", "az"]


def read_imu(path):
    with path.open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader, [])
        columns = RATE_COLUMNS + (ACCEL_COLUMNS if set(header).intersection(ACCEL_COLUMNS) else [])
        if not set(columns) <= set(header):
            raise ValueError(f"Missing IMU columns in {path}")
        indices = [header.index(column) for column in columns]
        rows = []
        for line in reader:
            if not line:
                continue
            if len(line) != len(header):
                raise ValueError(f"Invalid IMU row at {path}:{reader.line_num}")
            sample = [float(line[i]) for i in indices]
            if not all(map(math.isfinite, sample)) or (rows and sample[0] < rows[-1][0]):
                raise ValueError(f"Invalid IMU measurement at {path}:{reader.line_num}")
            rows.append(sample)
    if not rows:
        raise ValueError(f"Empty IMU track in {path}")
    return columns, rows


def records(data):
    offset = 0
    while offset < len(data):
        key, kind, stride, count = struct.unpack_from(">4scBH", data, offset)
        start = offset + 8
        size = stride * count
        offset = start + ((size + 3) & ~3)
        if offset > len(data):
            raise ValueError("Truncated GPMF record")
        yield key, kind, stride, count, data[start : start + size]


def gyro_packets(stream):
    while header := stream.read(8):
        key, kind, stride, count = struct.unpack(">4scBH", header)
        if key != b"DEVC" or kind != b"\0":
            raise ValueError("Invalid GPMF device block")
        size = stride * count
        padded = (size + 3) & ~3
        block = stream.read(padded)
        if len(block) != padded:
            raise ValueError("Truncated GPMF device block")
        for key, _, _, _, payload in records(memoryview(block)[:size]):
            if key != b"STRM":
                continue
            tick = total = count = 0
            microsecond_clock = False
            for field, kind, stride, n, value in records(payload):
                if kind == b"L" and len(value) == 4:
                    if field == b"TICK":
                        tick = int.from_bytes(value, "big")
                    elif field == b"TSMP":
                        total = int.from_bytes(value, "big")
                if field == b"STMP":
                    microsecond_clock = True
                if field == b"GYRO" and kind == b"s" and stride == 6:
                    count = n
            if count:
                if not tick or microsecond_clock:
                    raise ValueError("Expected a GPMF millisecond clock")
                yield tick * 0.001, total, count


def sample_period(path, samples):
    anchors = []
    seen = 0
    try:
        with path.open("rb") as stream:
            for time, total, count in gyro_packets(stream):
                if total != seen + count or total > len(samples):
                    return None
                if anchors and time <= anchors[-1][1]:
                    return None
                # Only retime a CSV whose samples use this packet interpolation.
                end = min(total, len(samples) - 1)
                if end > seen + 1:
                    dt = (samples[end][0] - samples[seen][0]) / (end - seen)
                    if any(
                        abs(samples[i][0] - (samples[seen][0] + (i - seen) * dt)) > 2e-6 for i in range(seen + 1, end)
                    ):
                        return None
                anchors.append((seen, time))
                seen = total
    except OSError, ValueError, struct.error:
        return None
    if seen != len(samples) or len(anchors) < 3:
        return None
    x = sum(i for i, _ in anchors) / len(anchors)
    y = sum(t for _, t in anchors) / len(anchors)
    period = sum((i - x) * (t - y) for i, t in anchors) / sum((i - x) ** 2 for i, _ in anchors)
    return period if all(abs(t - y - (i - x) * period) <= period for i, t in anchors) else None


def align_packet_clock(config, directory, samples):
    raw = config.get("other_files", {}).get("raw_gpmf")
    ref = config.get("gyroflow_reference", {})
    offset = config["imu"].get("time_offset_s")
    if not raw or ref.get("integration_method") != 2 or offset is None or abs(samples[0][0] + offset) > 1e-6:
        return False
    sync = {}
    for key, value in ref.get("sync_offsets_ms", {}).items():
        if not key.endswith("s") or type(value) not in (int, float):
            return False
        try:
            time = float(key[:-1])
        except ValueError:
            return False
        if not math.isfinite(time) or not math.isfinite(value):
            return False
        sync[time] = value / 1000
    times = sorted(sync)
    if not times or any(a + sync[a] >= b + sync[b] for a, b in pairwise(times)):
        return False
    period = sample_period(directory / raw, samples)
    if period is None:
        return False
    # The reference integration timestamps intervals at the last rate sample.
    for i, sample in enumerate(samples):
        t = i * period
        hi = bisect_left(times, t)
        if 0 < hi < len(times):
            a, b = times[hi - 1], times[hi]
            correction = sync[a] + (sync[b] - sync[a]) * ((t - a) / (b - a))
        else:
            correction = sync[times[0] if hi == 0 else times[-1]]
        sample[0] = t + correction - period * 0.5
    return True


def packaged_calibration(config, directory):
    lens = config["lens"]
    project = config.get("source", {}).get("gyroflow_project")
    if project and "asymmetrical" not in lens and "profile" in lens:
        try:
            companion = json.loads((directory / project).read_text()).get("calibration_data", {})
        except OSError, ValueError:
            companion = {}
        params = companion.get("fisheye_params", {})
        if (
            companion.get("identifier") == lens["profile"]
            and companion.get("calib_dimension") == lens["calib_dimension"]
            and all(params.get(k) == lens[k] for k in ("camera_matrix", "distortion_coeffs"))
        ):
            return companion, params
    return lens, lens


def read_lens(calibration, intrinsics, model):
    if model.startswith("opencv_fisheye"):
        model = "OPENCV_FISHEYE"
    elif model == "none":
        model = "NONE"
    else:
        raise ValueError(f"Unsupported lens model {model}")
    dims = calibration["calib_dimension"]
    matrix = intrinsics["camera_matrix"]
    symmetric = not calibration.get("asymmetrical", True)
    lens = {
        "model": model,
        "calibWidth": dims["w"],
        "calibHeight": dims["h"],
        "fx": matrix[0][0],
        "fy": matrix[1][1],
        "cx": dims["w"] / 2 if symmetric else matrix[0][2],
        "cy": dims["h"] / 2 if symmetric else matrix[1][2],
        "k": intrinsics["distortion_coeffs"],
    }
    if len(lens["k"]) != 4:
        raise ValueError("Lens requires four distortion coefficients")
    return lens


def load_gyro(args, destination):
    path = args.gyro_config
    directory = path.parent
    config = json.loads(path.read_text())
    if "calibration_data" in config and "stabilization" in config:
        calibration = config["calibration_data"]
        intrinsics = calibration["fisheye_params"]
        model = "opencv_fisheye"
        ref = config["stabilization"]
        integration = config.get("gyro_source", {}).get("integration_method")
        readout = ref.get("frame_readout_time", calibration.get("frame_readout_time", 0))
        direction = ref.get("frame_readout_direction", "TopToBottom")
        imu = args.imu or path.with_suffix(".imu_camera.csv")
        output = calibration.get("output_dimension")
    else:
        calibration, intrinsics = packaged_calibration(config, directory)
        model = config["lens"]["model"]
        ref = config.get("gyroflow_reference", {})
        integration = ref.get("integration_method")
        rs = config.get("rolling_shutter", {})
        readout = rs.get("readout_ms", 0)
        direction = rs.get("direction", "TopToBottom")
        imu = args.imu or directory / config["imu"]["file"]
        output = None
    if direction not in ("TopToBottom", "BottomToTop"):
        raise ValueError(f"Unsupported readout direction {direction}")
    lens = read_lens(calibration, intrinsics, model)
    if "lens_correction_amount" in ref:
        lens["lensCorrectionAmount"] = ref["lens_correction_amount"]
    params = ref.get("smoothing_params", {})
    if isinstance(params, list):
        params = {p["name"]: p["value"] for p in params}
    settings = {"readoutMs": readout, "readoutTopToBottom": direction == "TopToBottom"}
    for key, field in (
        ("smoothness", "smoothness"),
        ("smoothness_pitch", "smoothnessPitch"),
        ("smoothness_roll", "smoothnessRoll"),
        ("smoothness_yaw", "smoothnessYaw"),
        ("max_smoothness", "maxSmoothness"),
        ("alpha_0_1s", "alpha01s"),
    ):
        if key in params:
            settings[field] = params[key]
    if "per_axis" in params:
        settings["perAxis"] = bool(params["per_axis"])
    if "stabilization" in config or "gyroflow_reference" in config:
        settings["zoomEnvelope"] = ref.get("adaptive_zoom_method", 1) == 1
    if "max_zoom_iterations" in ref:
        settings["maxZoomIterations"] = ref["max_zoom_iterations"]
    zoom = args.max_zoom if args.max_zoom is not None else ref.get("max_zoom")
    if zoom is not None:
        settings["maxZoom"] = zoom / 100 if zoom > 10 else zoom
    window = (
        args.zoom_window
        if args.zoom_window is not None
        else ref.get("adaptive_zoom_window_s", ref.get("adaptive_zoom_window"))
    )
    if window is not None:
        settings.update(adaptiveZoomWindowS=window, adaptiveZoom=window > 0)
    if "fov" in ref:
        settings["fov"] = ref["fov"]
    output = ref.get("output_dimension", output)
    if output:
        settings.update(outputWidth=output["w"], outputHeight=output["h"])
    if args.output_size:
        settings["outputWidth"], settings["outputHeight"] = args.output_size
    if integration in (1, 2) and ref.get("horizon_lock_amount", 0) > 0:
        settings["integrationMethod"] = "COMPLEMENTARY"
    if args.imu_integration in ("gyro", "complementary"):
        settings["integrationMethod"] = args.imu_integration.upper()
    for key, value in (("smoothness", args.gyro_smooth), ("fov", args.fov), ("readoutMs", args.readout_ms)):
        if value is not None:
            settings[key] = value
    if args.lens_correction is not None:
        lens["lensCorrectionAmount"] = args.lens_correction
    if args.gyro_offset is not None:
        settings["timeOffsetS"] = args.gyro_offset / 1000
    columns, samples = read_imu(imu)
    aligned = args.imu is None and "imu" in config and align_packet_clock(config, directory, samples)
    with destination.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(samples)
    return {
        "lens": lens,
        "settings": settings,
        "imu_clock": "gpmf_tick_with_project_sync" if aligned else "csv",
        "imu_samples": len(samples),
    }
