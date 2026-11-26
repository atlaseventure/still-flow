# Still Flow

English | [简体中文](README.zh-CN.md)

Still Flow stabilizes video and corrects rolling shutter using the footage alone. Both offline and streaming modes also support gyroscope data.

## Linux CLI

Build on Linux amd64 with GCC (C++20), CMake, Ninja, Make, pkg-config, Python and Perl. The first build downloads dependencies.

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4
```

- **Media.** Input must be progressive 8-bit SDR with square pixels and even dimensions. Output is H.264 in MP4 or MKV. Frame timestamps are preserved. Compatible audio is copied and other tracks are converted to AAC.
- **Visual stabilization.** Offline processing and rolling shutter correction are enabled by default. The crop window stays fixed within each shot, with a budget of 8% per edge. Tighter budgets reduce correction strength.
- **Streaming.** Add `--mode realtime` for single-pass processing. The default lookahead is 1.35 seconds.
- **Gyroscope input.** Pass `--gyro-config stabilization.json` or a Gyroflow project. For `camera.gyroflow`, place `camera.imu_camera.csv` alongside it or supply the camera-frame IMU CSV with `--imu PATH`.

Rendering prefers CUDA with CPU fallback. For fully software-based processing, use `--backend cpu --decoder cpu --encoder openh264`.

## SDKs

Build and install the standalone C/C++ SDK, then import it with `find_package(StillFlow CONFIG REQUIRED)`. [C example](examples/memory)

```sh
cmake --preset core
cmake --build --preset core
cmake --install build/core-sdk --prefix /path/to/sdk
```

The Kotlin/Java AAR supports Android arm64-v8a, API 26 and later. [Android integration](api/android/README.md) · [Device video runner](tools/android/README.md)

## License

[MIT](LICENSE) © 2026 hiro. Binary distributions must retain the bundled dependency notices and corresponding sources.
