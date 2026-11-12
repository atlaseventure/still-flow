# Still Flow

English | [简体中文](README.zh-CN.md)

Offline and fixed-lag streaming video stabilization and rolling shutter correction, with a desktop CLI, a frame-based C++ engine, a C API and an Android AAR for Kotlin/Java. Targets are Linux amd64 and Android arm64-v8a. Desktop GPU processing uses CUDA and NVDEC/NVENC; Android provides OpenGL ES compute rendering and hardware MediaCodec integration.

## Build and run

The desktop source build requires Linux amd64, a C++20 compiler, CMake, Ninja, Make, pkg-config, Python and Perl. Dependencies are downloaded and verified automatically. Builds and desktop processing default to 12 workers.

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4 --compare comparison.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --mode realtime --lookahead 1.35
./build/linux-gcc/stillflow --help
```

`--backend auto` prefers CUDA and falls back to CPU. Hardware codecs are selected when GPU rendering is used; `--backend cpu --decoder cpu --encoder openh264` forces software processing. Input must be progressive 8-bit SDR with square pixels and even dimensions. Output preserves presentation timestamps; missing reordered AVI timestamps use the fixed-rate stream clock. `--audio auto` copies container-compatible tracks and converts others to AAC. Use `--audio aac` to encode all audio as AAC, `copy` to require stream copying, or `none` to omit audio.

Offline CLI processing displays staged progress. File API consumers receive structured progress through `sf_file_callbacks.progress`; completion is reported after output and reports are published.

The default crop budget is 8% per edge. `--crop-mode fixed` applies the exact centered crop; `auto` selects a fixed window per shot, and `adaptive` permits slow viewport motion. `--rs off` disables rolling shutter correction. `--crf` controls OpenH264 QP or NVENC CQ; values across encoders are not quality-equivalent. `--preset` applies to NVENC.

Realtime mode decodes once, with 1.35 seconds of lookahead and a 120-frame buffer by default. `auto` and `adaptive` anticipate crop demand across the lookahead window; `fixed` keeps a centered crop. Capacity pressure, cuts and final draining shorten lookahead. Tracking recovery needs about 0.2 seconds plus two frames of context. Original PTS and audio synchronization are preserved. File processing keeps every frame; reports include throughput and per-frame timings. Live hosts use the streaming API's capacity feedback to drop incoming frames.

## Core SDK

The core and standalone C SDK require no media or system APIs. Eigen is fetched when unavailable. Build the SDK and run the plain C memory-frame example:

```sh
cmake --preset core
cmake --build --preset core
./build/core-sdk/examples/memory/stillflow_memory
cmake --install build/core-sdk --prefix /path/to/sdk
```

Consumers use `find_package(StillFlow CONFIG REQUIRED)` and link `StillFlow::c` for the C API or `StillFlow::application` for C++. Push frames into an analyzer, replay requested recovery frames, finish the plan, then extract frame plans by index and render them. The portable estimator handles translation; the optional OpenCV estimator provides full tracking. The CPU renderer accepts gray8, BGR8 and RGBA8; analysis also accepts YUV luma planes. Use C headers and binaries from the same build. Default functions return initialized settings; ownership and callback contracts are documented in the installed headers.

For a static C SDK, configure with `-DSTILLFLOW_C_API_SHARED=OFF`. For Android native consumers, set `ANDROID_NDK_HOME` to an installed NDK directory:

```sh
cmake --preset android-arm64
cmake --build --preset android-arm64
```

Use C++ `StreamAnalyzer` or C `sf_stream_*` for streaming: push, drain ready plans, finish, then drain again. Hosts retain full-resolution frames until their index is ready. `SF_AGAIN` means wait or relieve capacity; `SF_END` means draining is complete. Owned `sf_frame_plan` handles carry frame indices and timestamps and are passed directly to the renderer.

## Android

Build the Kotlin/Java AAR with an installed JDK, Android SDK/NDK and CMake. Set `ANDROID_HOME` to the SDK directory:

```sh
./api/android/gradlew -p api/android assembleRelease
```

Android requires API 26 or later. Rendering defaults to GPU with CPU fallback; hardware codec helpers select hardware implementations. Media input, output and scheduling belong to the host app. See [Android integration](api/android/README.md) for usage. Native build concurrency defaults to 12; override with `-Pstillflow.jobs=N`.

## Tests and quality

Install FFmpeg and [uv](https://docs.astral.sh/uv/getting-started/installation/) for synthetic video regression. Test presets use Catch2; generated inputs are cached under `.local` and refreshed when generation inputs change.

```sh
ctest --preset core
ctest --preset linux-gcc
uv run --locked --group quality python scripts/quality.py check --build-dir build/linux-gcc/app
uv run --locked --group video python scripts/generate_video.py --output .local/synthetic --frames 60
```

Quality commands emit JSON Lines and exit nonzero on failure. `format` applies formatting; `format --check` checks it. Use `--jobs N` to change analysis concurrency. Synthetic generation supports `--type parallax`, `--no-rolling-shutter` and `--overwrite`.

## License

[MIT](LICENSE) © 2026 hiro.

The optional desktop module dynamically links LGPL FFmpeg and uses BSD OpenH264 by default. Installation includes replaceable FFmpeg shared libraries, license notices and corresponding media sources with build configuration. Version output and reports identify the linked FFmpeg license. Distributing desktop binaries requires compliance with their linked dependencies; the standalone core SDK excludes FFmpeg.

Keep the installed notices and corresponding sources with binary distributions. Installation verifies the dependency package's checksums and requires complete distribution materials, including for externally supplied dependency prefixes. CUDA builds include the matching NVIDIA license and distribution notice; NVIDIA runtime components follow those terms separately from the project's MIT license.
