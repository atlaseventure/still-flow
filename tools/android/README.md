# Android video runner

English | [简体中文](README.zh-CN.md)

Process videos on an Android device using the Still Flow AAR and compare results with a reference video.

## Requirements

Requires Linux or WSL, a JDK, Android SDK/NDK, CMake, `adb`, FFmpeg and `uv`. Set `ANDROID_HOME` to the SDK directory and connect an authorized Android 10+ arm64 device with `GL_EXT_YUV_target` support.

Inputs must be progressive 8-bit H.264 with even dimensions, square pixels and SDR BT.601/BT.709 color metadata. Apply rotation to the pixels before processing. Output contains video only and preserves frame timestamps.

## Usage

Run from the repository root:

```sh
uv run --locked --group video python scripts/android.py input.mp4
uv run --locked --group video python scripts/android.py input.mp4 --realtime --mobile
uv run --locked --group video python scripts/android.py input.mp4 --serial DEVICE --reference host.mp4 --max-displacement 0.5
uv run --locked --group video python scripts/android.py --help
```

Processing defaults to offline. `--realtime` enables fixed-lookahead streaming; `--mobile` selects lighter analysis settings. Encoding bitrate defaults to 12 Mbps. Use `--serial` when multiple devices are connected, or `--aar` to supply an existing AAR.

Each run saves its video and reports under `.local/android-runs/`. Use `--output` to select a new directory. The runner builds and installs the app; after processing, it removes temporary device files and leaves the app installed.

## Compare results

`--reference` compares frame timing and image geometry. The preview shows the reference on the left and device output on the right. `--max-displacement` requires every frame to have enough tracked features and a 95th-percentile displacement within the specified pixel limit; failure returns a nonzero exit status. Omit the threshold to report measurements only.
