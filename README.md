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

## Usage

Process a video and generate a side-by-side comparison:

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

CUDA is selected automatically, with CPU fallback when unavailable or if execution fails. `--gpu off` forces CPU rendering; `--gpu cuda` requires CUDA and exits on failure. Reports record the actual backend, device, frame counts, and fallback reason; GPU timings include transfers and synchronization. Motion estimation, trajectory optimization, and video codecs run on the CPU.

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
