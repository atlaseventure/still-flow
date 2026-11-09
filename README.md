# Still Flow

English | [简体中文](README.zh-CN.md)

Offline video stabilization and rolling shutter wobble suppression without training. Includes a command-line interface, comparison videos, and performance reports by processing stage. NVIDIA GPUs can accelerate coordinate mapping, boundary checks, and image resampling.

## Build

On Linux, provide a C++20 compiler, CMake, Ninja, Make, pkg-config, Python, and Perl. The first build downloads verified, pinned sources and builds media and numerical dependencies locally.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Dependencies use 12 compilation jobs each; adjust with `-DSTILLFLOW_BUILD_JOBS=N`. Incremental builds reuse downloaded sources and compiled libraries.

The build detects the CUDA Toolkit automatically; GPU execution requires a compatible NVIDIA driver. Set `CUDACXX` to `nvcc` for a custom toolkit location. Use `-DSTILLFLOW_WITH_CUDA=OFF` for a CPU-only build.

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

CUDA analysis requires toolkit headers including cuRAND. Use `--require-cuda` to require CUDA coverage; CPU builds report CUDA sources as skipped. Split toolkit installations can add header paths with `--extra-arg=-isystem/path/to/include`.

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

CUDA is selected automatically, with CPU fallback when unavailable or if execution fails. `--backend cpu` forces CPU rendering; `--backend cuda` requires CUDA and exits on failure. Reports record each backend's device, capabilities, frame counts and failure reasons; rendering timings include transfers and synchronization. Motion estimation, trajectory optimization, and video codecs run on the CPU.

Compare scanline correction with the same crop. `--rs off` keeps the same motion estimation and disables scanline compensation. Omit the comparison video when benchmarking:

```sh
./build/stillflow input.mp4 global.mp4 --rs off --report global.json
./build/stillflow input.mp4 corrected.mp4 --rs on --report corrected.json
./build/stillflow --help
```

Input must be progressive 8-bit SDR video with square pixels and even dimensions. Output preserves variable frame rate timestamps and copies compatible audio tracks. Use MKV or `--no-audio` if the audio codec is incompatible with the output container.

The default crop removes 8% from each edge while retaining the output dimensions. Correction strength decreases automatically when the crop margin is insufficient. The scan axis follows rotation metadata; use `--scan-axis` for footage whose pixels were rotated and whose rotation metadata was removed.

Scanline correction is a geometric approximation suited to small motions between frames. Inspect results for strong parallax, fast foreground motion, blur, and high-frequency wobble. Fit residuals in performance reports are diagnostic measurements rather than image quality scores.

## License

[MIT](LICENSE) © 2026 hiro.
