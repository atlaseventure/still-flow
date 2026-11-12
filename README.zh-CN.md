# Still Flow

[English](README.md) | 简体中文

离线与固定前视流式视频防抖、滚动快门校正，提供桌面 CLI、按帧调用的 C++ 引擎、C API 和 Kotlin/Java Android AAR。支持 Linux amd64 与 Android arm64-v8a。桌面 GPU 处理使用 CUDA、NVDEC/NVENC；Android 提供 OpenGL ES 计算渲染与硬件 MediaCodec 集成。

## 构建与使用

桌面源码构建需要 Linux amd64、C++20 编译器、CMake、Ninja、Make、pkg-config、Python 和 Perl。依赖自动下载并校验。构建和桌面处理默认使用 12 个工作线程。

```sh
cmake --preset linux-gcc
cmake --build --preset linux-gcc
./build/linux-gcc/stillflow input.mp4 output.mp4 --compare comparison.mp4
./build/linux-gcc/stillflow input.mp4 output.mp4 --mode realtime --lookahead 1.5
./build/linux-gcc/stillflow --help
```

`--backend auto` 优先使用 CUDA，失败时回退 CPU。GPU 渲染默认优先使用硬件编解码；`--backend cpu --decoder cpu --encoder openh264` 强制软件处理。输入须为逐行扫描、方形像素、偶数尺寸的 8 位 SDR 视频。输出保留显示时间戳；AVI 重排帧缺失时间戳时使用固定帧率时基补齐。`--audio auto` 复制容器兼容音轨，其余转为 AAC；`--audio aac` 将全部音轨转为 AAC，`copy` 要求直接复制，`none` 移除音频。

离线 CLI 显示分阶段进度。文件 API 通过 `sf_progress.event` 回调结构化进度，输出和报告发布成功后报告完成。

默认裁剪预算为每边 8%。`--crop-mode fixed` 使用精确居中裁剪，`auto` 为每个镜头选择固定窗口，`adaptive` 允许窗口缓慢移动。`--rs off` 关闭滚动快门校正。`--crf` 分别控制 OpenH264 QP 或 NVENC CQ，相同数值的画质并不等价。`--preset` 用于 NVENC。

实时模式单遍解码，默认前视 1.5 秒、最多缓存 120 帧。`auto` 与 `adaptive` 使用缓慢调整的裁剪窗口，`fixed` 保持居中固定裁剪。容量耗尽、镜头切换及结束排空时会缩短前视；跟踪恢复至少需要约 0.2 秒及两帧上下文。输出保留原始 PTS 和音画同步。文件处理保留全部帧，实际吞吐与逐帧耗时见报告；直播宿主通过流式 API 的容量反馈决定丢帧。

## 核心 SDK

核心与独立 C SDK 不依赖媒体库或系统 API，缺少 Eigen 时自动下载。构建 SDK 并运行纯 C 内存帧示例：

```sh
cmake --preset core
cmake --build --preset core
./build/core-sdk/examples/memory/stillflow_memory
cmake --install build/core-sdk --prefix /path/to/sdk
```

通过 `find_package(StillFlow CONFIG REQUIRED)` 引入，C API 链接 `StillFlow::c`，C++ 接口链接 `StillFlow::application`。调用方推入帧、按请求补送恢复帧、生成规划，再按索引渲染。可移植估计器处理平移，可选 OpenCV 估计器提供完整跟踪。CPU 渲染支持 gray8、BGR8、RGBA8，分析还支持 YUV 亮度平面。结构体、所有权和回调约定见安装后的头文件。

流式调用使用 C++ `StreamAnalyzer` 或 C `sf_stream_*`：推帧后取出就绪规划，结束后排空。宿主持有全尺寸帧直到对应索引就绪；C 返回的单帧规划使用索引 0 渲染并独立销毁。

静态 C SDK 使用 `-DSTILLFLOW_C_API_SHARED=OFF`。Android 原生开发将 `ANDROID_NDK_HOME` 指向已安装的 NDK 目录：

```sh
cmake --preset android-arm64
cmake --build --preset android-arm64
```

## Android

构建 Kotlin/Java AAR 需要已安装的 JDK、Android SDK/NDK 和 CMake。将 `ANDROID_HOME` 指向 SDK 目录：

```sh
./api/android/gradlew -p api/android assembleRelease
```

Android 要求 API 26 及以上。渲染默认优先使用 GPU，失败时回退 CPU；编解码辅助接口选择硬件实现。媒体输入、输出与任务调度由宿主应用负责。用法见 [Android 集成](api/android/README.md)。原生构建默认并行数为 12，通过 `-Pstillflow.jobs=N` 修改。

## 测试与质量

合成视频回归需要 FFmpeg 和 [uv](https://docs.astral.sh/uv/getting-started/installation/)。测试使用 Catch2，合成输入缓存在 `.local`，生成条件变化时自动刷新。

```sh
ctest --preset core
ctest --preset linux-gcc
uv run --locked --group quality python scripts/quality.py check --build-dir build/linux-gcc/app
uv run --locked --group video python scripts/generate_video.py --output .local/synthetic --frames 60
```

质量命令输出 JSON Lines，失败时返回非零状态。`format` 修改格式，`format --check` 仅检查；`--jobs N` 调整分析并行数。视频生成支持 `--type parallax`、`--no-rolling-shutter` 和 `--overwrite`。

## 许可证

[MIT](LICENSE) © 2026 hiro。

可选桌面模块默认动态链接 LGPL FFmpeg，使用 BSD OpenH264 编码。安装包含可替换的 FFmpeg 共享库、许可证声明、对应媒体源码及构建配置。版本输出与报告记录实际链接的 FFmpeg 许可证。分发桌面二进制时须遵守所链接依赖的许可证；独立核心 SDK 不包含 FFmpeg。

分发二进制时保留安装附带的声明和对应源码。安装会核验依赖包的校验和并要求分发材料完整，外部提供的依赖前缀同样适用。CUDA 构建附带对应的 NVIDIA 许可及分发说明；NVIDIA 运行库组件单独遵循这些条款，项目自有代码采用 MIT 许可。
