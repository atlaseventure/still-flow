# Android video runner

Headless Kotlin app consuming the packaged AAR, with hardware H.264 decoding/encoding and selectable CPU/GLES rendering. Requires Linux or WSL, the Android SDK, an authorized Android 10+ arm64 device, `adb`, FFmpeg and `uv`.

From the repository root:

```sh
uv run --locked --group video python scripts/android.py input.mp4 --gpu-surface --mobile --vision-threads 2
uv run --locked --group video python scripts/android.py input.mp4 --serial DEVICE --reference host.mp4 --max-displacement 0.5
uv run --locked --group video python scripts/compare_video.py host.mp4 android.mp4 --output .local/comparison
```

Inputs must be progressive 8-bit H.264 with even dimensions, square pixels, SDR BT.601/BT.709 color metadata and rotation baked into the pixels. Output contains video only and preserves presentation timing. Unsupported media fails before deployment.

The runner builds and installs a minified, debuggable consumer of the release AAR. `--aar` selects an existing AAR. Rendering defaults to explicit GLES; other stabilization parameters use SDK defaults unless overridden. Build parallelism defaults to 12 and encoding bitrate to 12 Mbps. `--help` lists overrides.

Each run writes videos, logs, effective settings, device details and checksums under a new `.local/android-runs/` directory. `--output` selects a new directory. Binary transfers are verified, including when WSL uses Windows adb. The application remains installed; only the run's device files are removed.

Comparison aligns presentation timestamps and measures feature displacement at native resolution. The preview shows reference on the left and candidate on the right. `--max-displacement` requires every frame to have sufficient tracks and a 95th-percentile displacement within the selected pixel limit. Without it, the tool reports measurements without asserting geometric equivalence. These comparisons assess consistency; use a known-motion reference when assessing stabilization quality.
