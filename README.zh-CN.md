# Still Flow

[English](README.md) | 简体中文

视频防抖与滚动快门校正，支持离线和固定前视流式处理，提供桌面 CLI、C/C++ SDK 和 Kotlin/Java Android AAR。支持 Linux amd64 与 Android arm64-v8a。

## 构建与使用

需要 Linux amd64、C++20 编译器、CMake、Ninja、Make、pkg-config、Python 和 Perl。依赖自动下载。

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --compare comparison.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --mode realtime --lookahead 1.35
./build/linux-gcc/stillflow --help
```

输入须为逐行扫描、方形像素、偶数尺寸的 8 位 SDR 视频。

- 默认离线处理；实时模式默认前视 1.35 秒。
- `--backend auto` 优先使用 CUDA，失败时回退 CPU。使用 `--backend cpu --decoder cpu --encoder openh264` 启用软件处理。
- 默认裁剪预算为每边 8%。`--crop-mode auto` 为每个镜头选择固定窗口，`fixed` 使用精确居中裁剪，`adaptive` 允许窗口缓慢移动。
- 默认启用滚动快门校正；`--rs off` 关闭校正。
- 输出保留帧时间戳。`--audio auto` 复制兼容音轨，其余转为 AAC；`--audio none` 移除音频。

### 陀螺仪防抖

离线陀螺仪防抖在桌面端支持 CUDA 和 CPU，在 Android 上支持 GLES Surface 渲染、GLES 计算和 CPU。实时流式陀螺仪防抖请求返回 `SF_UNSUPPORTED`。

输入防抖配置 JSON，或提供 Gyroflow 项目及其配套的 `<stem>.imu_camera.csv`。桌面端的 `--backend auto` 优先使用 CUDA，无法使用时回退到 CPU。

```sh
./build/linux-gcc/stillflow input.mp4 auto.mp4 --gyro-config camera.gyroflow --backend auto
./build/linux-gcc/stillflow input.mp4 cpu.mp4 --gyro-config camera.gyroflow --backend cpu --decoder cpu --encoder openh264
```

- 使用 `--imu PATH` 指定相机坐标系 IMU 数据，`--gyro-offset MS` 调整时间偏移，`--imu-integration auto|gyro|complementary` 选择积分方式。
- 使用 `--output-size WxH`、`--fov FACTOR` 和 `--max-zoom RATIO` 调整构图，`--zoom-window SECONDS` 设置变焦窗口（0 为固定缩放）。
- `--gyro-smooth` 和 `--lens-correction` 分别调整平滑强度和镜头校正量，取值均为 0 到 1。`--readout-ms MS` 设置滚动快门读出时间。

核心 C API 通过 `sf_gyro_plan_create` 创建离线计划，通过 `sf_frame_plan_get_gyro_map` 读取逐帧映射。Android AAR 提供 `StillFlow.planGyro` 和 `GpuFrames.render`。

## 核心 SDK

构建、安装独立 SDK，或运行 C 内存帧示例：

```sh
cmake --preset core
cmake --build --preset core
./build/core-sdk/examples/memory/stillflow_memory
cmake --install build/core-sdk --prefix /path/to/sdk
```

通过 `find_package(StillFlow CONFIG REQUIRED)` 引入 SDK。用法见 [C 示例](examples/memory)和 [API 头文件](api/c/include/stillflow)。

## Android

要求 Android API 26 及以上。构建 AAR 需要 JDK、Android SDK/NDK 和 CMake，并将 `ANDROID_HOME` 指向 SDK 目录：

```sh
./api/android/gradlew -p api/android assembleRelease
```

构建原生 SDK 时，将 `ANDROID_NDK_HOME` 指向 NDK 目录：

```sh
cmake --preset android-arm64
cmake --build --preset android-arm64
```

应用接入见 [Android 集成](api/android/README.zh-CN.md)，设备视频处理见 [Android 视频运行器](tools/android/README.zh-CN.md)。

## 测试与质量

质量检查和合成视频生成需要 [uv](https://docs.astral.sh/uv/getting-started/installation/)，视频回归还需要 FFmpeg。测试前先构建对应预设。

```sh
ctest --preset linux-gcc
uv run --locked --group quality python scripts/quality.py check --build-dir build/linux-gcc/app
uv run --locked --group video python scripts/generate_video.py --output .local/synthetic --frames 60
```

## 许可证

[MIT](LICENSE) © 2026 hiro。

分发二进制时保留附带的依赖许可声明和对应源码。FFmpeg、NVIDIA 运行库等依赖各自遵循其许可条款。
