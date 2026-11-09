# Still Flow

English | [简体中文](README.zh-CN.md)

Offline video stabilization and rolling shutter wobble suppression without training. Includes a command-line interface, comparison videos, and performance reports by processing stage. NVIDIA GPUs can accelerate coordinate mapping, boundary checks, image resampling, and video decoding and encoding (NVDEC/NVENC).

## Build

On Linux, provide a C++20 compiler, CMake, Ninja, Make, pkg-config, Python, and Perl. The first build downloads verified, pinned sources and builds media and numerical dependencies locally.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Dependencies use 12 compilation jobs each; adjust with `-DSTILLFLOW_BUILD_JOBS=N`. Incremental builds reuse downloaded sources and compiled libraries.

The build detects the CUDA Toolkit automatically; GPU execution requires a compatible NVIDIA driver. Set `CUDACXX` to `nvcc` for a custom toolkit location. Use `-DSTILLFLOW_WITH_CUDA=OFF` for a CPU-only build.

FFmpeg is built with NVDEC/NVENC support from header-only NVIDIA codec interfaces. No CUDA Toolkit is required, and the driver libraries load at runtime, so binaries also run without an NVIDIA driver. Hardware codecs require driver 550 or newer. Use `-DSTILLFLOW_WITH_NVCODEC=OFF` to build without them.

## Code quality

Use Python 3.14 for the quality tools:

```sh
python3.14 -m venv .local/quality-venv
.local/quality-venv/bin/python -m pip install -r scripts/requirements-quality.txt
.local/quality-venv/bin/python scripts/quality.py format
.local/quality-venv/bin/python scripts/quality.py check --build-dir build
```

`format --check` checks formatting without writing; `lint` runs static analysis. `check` runs both and requires a completed CLI build. Commands emit JSON Lines and exit nonzero on failure. Git-ignored files are excluded.

Static analysis defaults to 12 concurrent processes and reuses successful results when compilation inputs are unchanged. Use `--jobs N` to adjust concurrency or `--no-cache` to run a full analysis.

CUDA analysis uses the toolkit headers; toolkits without cuRAND get an empty stand-in for the cuRAND header that Clang always includes. Use `--require-cuda` to require CUDA coverage; CPU builds report CUDA sources as skipped. Split toolkit installations can add header paths with `--extra-arg=-isystem/path/to/include`.

## Library integration

The core follows Clean Architecture: domain algorithms and application use cases depend on ports for media, images, rendering, outputs and runtime services. The CLI assembles desktop adapters; Android and embedded hosts can supply their own adapters.

Build and install the core with an installed Eigen package:

```sh
cmake -S . -B build-lib -G Ninja -DSTILLFLOW_SUPERBUILD=OFF \
  -DSTILLFLOW_BUILD_CLI=OFF -DSTILLFLOW_BUILD_ADAPTERS=OFF
cmake --build build-lib
cmake --install build-lib --prefix /path/to/sdk
```

For cross-compilation, pass the target platform's `CMAKE_TOOLCHAIN_FILE`. Consumers import `find_package(StillFlow CONFIG REQUIRED)`, link `StillFlow::application`, implement the application ports and call `stillflow::stabilize`. The core requires a C++20 standard library with filesystem and exception support.

To add a rendering backend, implement `RenderBackend`, register it with `make_renderer_factory` and inject `Services::renderers`. Registration order determines automatic selection and fallback; explicit selection fails on backend errors. Images are exchanged through read mappings or native handles with lifetime ownership.

## Synthetic footage

Install FFmpeg and the Python dependencies in `scripts/requirements.txt`. Generate a shaky rolling-shutter clip, a stable reference and parameter metadata:

```sh
python3 scripts/generate_video.py --output .local/synthetic --frames 60
```

Use `--overwrite` to replace existing footage.

## Usage

Process a video and generate a side-by-side comparison:

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

CUDA is selected automatically, with CPU fallback when unavailable or if execution fails. `--backend cpu` forces CPU rendering; `--backend cuda` requires CUDA and exits on failure. Reports record each backend's device, capabilities, frame counts and failure reasons; rendering timings include transfers and synchronization. Motion estimation and trajectory optimization run on the CPU.

With CUDA rendering, `--decoder auto` and `--encoder auto` use NVDEC and NVENC, and fall back to CPU codecs per stream when hardware is unavailable or rejects a stream (for example H.264 4:4:4, or a comparison wider than 4096 pixels). `--decoder nvdec` and `--encoder nvenc` require hardware and exit on failure; `cpu` and `x264` force software. NVDEC frames stay in GPU memory: analysis downloads only a reduced luma plane, CUDA rendering samples the NV12 planes directly, and NVENC reads the rendered frames in place (comparison videos are also composed on the GPU). Frames are converted on the host only when a CPU stage needs them, such as CPU rendering or x264 encoding. GPU colour conversion rounds differently from swscale and analysis uses luma instead of BGR grey, so outputs are close to the CPU path but not byte-identical. CPU rendering with CPU decoding and x264 is unchanged. NVENC uses constant-quality VBR: `--crf` sets the CQ level, and x264 `--preset` names map to NVENC `p1`..`p7` (or pass `p1`..`p7` directly). Bitrate at the same value differs from x264. Reports list each stream's codec, hardware frame count and fallback reasons.

Compare scanline correction with the same crop. `--rs off` keeps the same motion estimation and disables scanline compensation. Omit the comparison video when benchmarking:

```sh
./build/stillflow input.mp4 global.mp4 --rs off --crop-mode fixed --report global.json
./build/stillflow input.mp4 corrected.mp4 --rs on --crop-mode fixed --report corrected.json
./build/stillflow --help
```

Input must be progressive 8-bit SDR video with square pixels and even dimensions. Output preserves variable frame rate timestamps and copies compatible audio tracks. Use MKV or `--no-audio` if the audio codec is incompatible with the output container.

By default, each shot gets a fixed, aspect-preserving crop computed from the motion geometry. `--crop` sets the maximum equivalent per-edge crop (default 8%, retaining at least 70.56% of the area); the window can be off-center. `--crop-mode fixed` uses the exact centered crop, while `--crop-mode adaptive` allows slow viewport motion and zoom for long shots. Output dimensions stay unchanged. Offline planning smooths correction strength when the crop budget is insufficient. Reports separate crop, mesh-geometry and final boundary limits and record the actual retained area. The scan axis follows rotation metadata; use `--scan-axis` for footage whose pixels were rotated and whose rotation metadata was removed.

Trajectory processing follows the [LightStab demo](https://github.com/liutao23/LightStab): accumulated mesh displacements, causal cascaded smoothing, and compensation from the trajectory difference. Shot starts use reliable subsequent motion to initialize scanline smoothing. Gaussian kernels adapt to motion and confidence without training. `--smooth` and `--rs-smooth` control the temporal kernel width; zero disables the corresponding correction.

Scanline correction is a geometric approximation suited to small motions between frames. Inspect results for strong parallax, fast foreground motion, blur, and high-frequency wobble. Fit residuals in performance reports are diagnostic measurements rather than image quality scores.

## License

[MIT](LICENSE) © 2026 hiro.
