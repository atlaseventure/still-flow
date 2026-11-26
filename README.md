# Still Flow

English | [简体中文](README.zh-CN.md)

Video stabilization and rolling shutter correction with offline and fixed-lookahead streaming modes. Provides a desktop CLI, C/C++ SDK and Kotlin/Java Android AAR. Supports Linux amd64 and Android arm64-v8a.

## Build and run

Requires Linux amd64, a C++20 compiler, CMake, Ninja, Make, pkg-config, Python and Perl. Dependencies are downloaded automatically.

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --compare comparison.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --mode realtime --lookahead 1.35
./build/linux-gcc/stillflow --help
```

Input must be progressive 8-bit SDR with square pixels and even dimensions.

- Processing defaults to offline; realtime mode uses 1.35 seconds of lookahead by default.
- `--backend auto` prefers CUDA with CPU fallback. Use `--backend cpu --decoder cpu --encoder openh264` for software processing.
- The crop budget defaults to 8% per edge. `--crop-mode auto` selects a fixed window per shot; `fixed` applies an exact centered crop, and `adaptive` allows slow viewport motion.
- Rolling shutter correction is enabled by default; `--rs off` disables it.
- Output preserves frame timestamps. `--audio auto` copies compatible tracks and converts others to AAC; `--audio none` omits audio.

### Gyroscope stabilization

Offline and fixed-lookahead streaming gyro stabilization both run on desktop CUDA and CPU, and on Android with GLES Surface rendering, GLES compute or CPU.

Supply a stabilization JSON or a Gyroflow project with its companion `<stem>.imu_camera.csv`. On desktop, `--backend auto` prefers CUDA with CPU fallback.

```sh
./build/linux-gcc/stillflow input.mp4 auto.mp4 --gyro-config camera.gyroflow --backend auto
./build/linux-gcc/stillflow input.mp4 cpu.mp4 --gyro-config camera.gyroflow --backend cpu --decoder cpu --encoder openh264
./build/linux-gcc/stillflow input.mp4 stream.mp4 --mode realtime --lookahead 1.35 --gyro-config camera.gyroflow
```

- In streaming mode, `--lookahead` sets the lookahead in seconds and `--buffer-frames` caps the number of retained frames.
- Override the camera-frame CSV with `--imu PATH`, shift IMU time with `--gyro-offset MS`, and select `--imu-integration auto|gyro|complementary`.
- Set framing with `--output-size WxH`, `--fov FACTOR`, `--max-zoom RATIO` and `--zoom-window SECONDS` (0 for static zoom).
- Adjust `--gyro-smooth` and `--lens-correction` from 0 to 1. Set rolling shutter readout with `--readout-ms MS`.

In the Core C API, use `sf_gyro_plan_create` for offline plans or `sf_gyro_stream_create` for streaming. Feed IMU samples and frame timestamps with `sf_gyro_stream_push_imu` and `sf_gyro_stream_push_frame`. Drain ready plans with `sf_gyro_stream_take_ready`. On `SF_AGAIN` from a frame push, drain and retry that timestamp. After `sf_gyro_stream_finish`, drain to `SF_END`. Both modes expose each frame's map through `sf_frame_plan_get_gyro_map`. The Android AAR provides `StillFlow.planGyro` and `StillFlow.GyroStream`.

## Core SDK

Build and install the standalone SDK, or run the C memory-frame example:

```sh
cmake --preset core
cmake --build --preset core
./build/core-sdk/examples/memory/stillflow_memory
cmake --install build/core-sdk --prefix /path/to/sdk
```

Use `find_package(StillFlow CONFIG REQUIRED)` to integrate the SDK. See the [C example](examples/memory) and [API headers](api/c/include/stillflow) for usage.

## Android

Requires Android API 26 or later. To build the AAR, install a JDK, Android SDK/NDK and CMake, then set `ANDROID_HOME` to the SDK directory:

```sh
./api/android/gradlew -p api/android assembleRelease
```

For a native SDK build, set `ANDROID_NDK_HOME` to the NDK directory:

```sh
cmake --preset android-arm64
cmake --build --preset android-arm64
```

See [Android integration](api/android/README.md) for app usage and [Android video runner](tools/android/README.md) for device processing.

## Tests and quality

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) for quality checks and synthetic video generation. Video regression also requires FFmpeg. Run tests after building the corresponding preset.

```sh
ctest --preset linux-gcc
uv run --locked --group quality python scripts/quality.py check --build-dir build/linux-gcc/app
uv run --locked --group video python scripts/generate_video.py --output .local/synthetic --frames 60
```

## License

[MIT](LICENSE) © 2026 hiro.

Binary distributions must retain the included dependency notices and corresponding sources. Linked dependencies, including FFmpeg and NVIDIA runtime components, have their own license terms.
