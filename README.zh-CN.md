# Still Flow

[English](README.md) | 简体中文

Still Flow 仅凭视频画面即可完成防抖和滚动快门校正。离线和流式处理也都支持陀螺仪数据。

## Linux 命令行

在 Linux amd64 上使用支持 C++20 的 GCC 构建，需要 CMake、Ninja、Make、pkg-config、Python 和 Perl。首次构建会下载依赖。

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4
```

- **输入输出。** 输入须为逐行扫描、方形像素、宽高均为偶数的 8 位 SDR 视频。输出为 H.264 编码的 MP4 或 MKV，保留帧时间戳。兼容音轨直接复制，其余转为 AAC。
- **视觉防抖。** 默认离线处理并启用滚动快门校正。每个镜头内的裁剪窗口固定，默认每边最多裁剪 8%。收紧裁剪预算会降低校正强度。
- **流式处理。** 添加 `--mode realtime` 即可单遍处理，默认前视 1.35 秒。
- **陀螺仪输入。** 通过 `--gyro-config` 指定 `stabilization.json` 或 Gyroflow 项目。使用 `camera.gyroflow` 时，将 `camera.imu_camera.csv` 放在同一目录，或用 `--imu PATH` 指定相机坐标系下的 IMU 数据文件。

渲染优先使用 CUDA，无法使用时回退到 CPU。全程使用软件处理需指定 `--backend cpu --decoder cpu --encoder openh264`。

## SDK

构建并安装独立的 C/C++ SDK 后，通过 `find_package(StillFlow CONFIG REQUIRED)` 引入。[C 示例](examples/memory)

```sh
cmake --preset core
cmake --build --preset core
cmake --install build/core-sdk --prefix /path/to/sdk
```

Kotlin/Java AAR 支持 Android arm64-v8a，要求 API 26 及以上。[Android 集成](api/android/README.zh-CN.md) · [设备视频运行器](tools/android/README.zh-CN.md)

## 许可证

[MIT](LICENSE) © 2026 hiro。分发二进制时须保留随附的依赖许可声明和对应源码。
