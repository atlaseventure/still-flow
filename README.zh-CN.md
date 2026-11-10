# Still Flow

[English](README.md) | 简体中文

无需训练的离线视频防抖与卷帘快门晃动抑制，提供命令行处理、前后对比视频和分阶段性能报告。支持使用 NVIDIA GPU 加速坐标映射、边界检查、图像重采样以及视频解码和编码（NVDEC/NVENC）。

## 构建

Linux 环境需具备 C++20 编译器、CMake、Ninja、Make、pkg-config、Python 和 Perl。首次构建下载固定版本的源码、校验完整性，并在本地构建媒体与数值计算依赖。

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

每个依赖默认使用 12 个编译任务，可通过 `-DSTILLFLOW_BUILD_JOBS=N` 调整。增量构建复用已下载的源码和已编译的依赖。

构建时自动检测 CUDA Toolkit；GPU 运行需要兼容的 NVIDIA 驱动。自定义工具包位置可通过 `CUDACXX` 指定 `nvcc`。使用 `-DSTILLFLOW_WITH_CUDA=OFF` 构建纯 CPU 版本。

FFmpeg 通过只含头文件的 NVIDIA 编解码接口启用 NVDEC/NVENC，不需要 CUDA Toolkit；驱动库在运行时加载，因此没有 NVIDIA 驱动时程序同样可以运行。硬件编解码需要 550 或更新版本的驱动。使用 `-DSTILLFLOW_WITH_NVCODEC=OFF` 构建不含硬件编解码的版本。

## 代码质量

安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)，由其管理 Python 和锁定版本的脚本依赖：

```sh
uv run --locked --group quality python scripts/quality.py format
uv run --locked --group quality python scripts/quality.py check --build-dir build
```

`format --check` 只检查格式；`lint` 执行静态分析。`check` 执行两者，需要先完成 CLI 构建。命令输出 JSON Lines，失败时返回非零退出码，排除 Git 忽略的文件。

静态分析默认 12 并发，编译输入未变化时复用已通过的结果。使用 `--jobs N` 调整并发数，或用 `--no-cache` 执行完整分析。

CUDA 分析使用工具包头文件；工具包缺少 cuRAND 时，会为 Clang 始终包含的 cuRAND 头文件提供空的替代文件。使用 `--require-cuda` 强制检查 CUDA；CPU 构建会报告跳过的 CUDA 源文件。分开安装的工具包可通过 `--extra-arg=-isystem/path/to/include` 补充头文件路径。

## 库集成

核心库按 Clean Architecture 分为领域算法与应用用例，通过接口接入媒体、图像、渲染、输出和运行时服务。CLI 负责组装桌面适配器；Android 和嵌入式宿主可提供自己的适配器。

使用已安装的 Eigen 单独构建、安装核心库：

```sh
cmake -S . -B build-lib -G Ninja -DSTILLFLOW_SUPERBUILD=OFF \
  -DSTILLFLOW_BUILD_CLI=OFF -DSTILLFLOW_BUILD_ADAPTERS=OFF
cmake --build build-lib
cmake --install build-lib --prefix /path/to/sdk
```

交叉编译时传入目标平台的 `CMAKE_TOOLCHAIN_FILE`。使用方通过 `find_package(StillFlow CONFIG REQUIRED)` 导入并链接 `StillFlow::application`，实现应用层端口后调用 `stillflow::stabilize`。核心库需要支持 C++20、文件系统和异常的标准库。

新增渲染后端时实现 `RenderBackend`，通过 `make_renderer_factory` 注册并注入 `Services::renderers`。注册顺序决定自动选择与回退顺序；显式选择后端时失败即退出。图像通过持有生命周期的只读映射或原生句柄交换。

## 合成素材

需安装 FFmpeg 和 uv。生成带抖动与卷帘快门效应的视频、稳定参考视频和参数记录：

```sh
uv run --locked --group video python scripts/generate_video.py --output .local/synthetic --frames 60
```

使用 `--overwrite` 替换已有素材。

## 使用

处理视频并生成前后对比：

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

默认自动使用 CUDA，不可用或执行失败时回退到 CPU。`--backend cpu` 强制使用 CPU，`--backend cuda` 要求 CUDA 可用，失败时退出。性能报告按后端记录设备、能力、处理帧数和失败原因；渲染阶段耗时包含数据传输与同步。运动估计和轨迹优化使用 CPU。

使用 CUDA 渲染时，`--decoder auto` 和 `--encoder auto` 采用 NVDEC 和 NVENC；硬件不可用或不支持某路视频流时（例如 H.264 4:4:4，或宽度超过 4096 像素的对比视频），该路流单独回退到 CPU 编解码。`--decoder nvdec` 和 `--encoder nvenc` 要求使用硬件，失败时退出；`cpu` 和 `x264` 强制使用软件编解码。NVDEC 解码的帧保留在显存中：分析阶段只下载缩小后的亮度平面，CUDA 渲染直接采样 NV12 平面，NVENC 原地读取渲染结果（对比视频也在 GPU 上合成）。只有 CPU 渲染或 x264 编码等需要主机像素的阶段才会在主机上转换。GPU 颜色转换的舍入方式与 swscale 不同，分析使用亮度而非 BGR 灰度，因此输出与 CPU 路径非常接近但不逐字节一致。CPU 渲染配合 CPU 解码和 x264 的输出保持不变。NVENC 使用恒定质量 VBR：`--crf` 设置 CQ 级别，x264 的 `--preset` 名称映射到 NVENC 的 `p1`..`p7`，也可直接指定 `p1`..`p7`。相同数值下码率与 x264 不同。性能报告列出每路流的编解码器、硬件处理帧数和回退原因。

使用相同裁剪量对照扫描行修正效果；`--rs off` 保留相同运动估计，仅关闭扫描行补偿。性能比较时省略对比视频：

```sh
./build/stillflow input.mp4 global.mp4 --rs off --crop-mode fixed --report global.json
./build/stillflow input.mp4 corrected.mp4 --rs on --crop-mode fixed --report corrected.json
./build/stillflow --help
```

输入使用逐行、方形像素、偶数宽高的 8-bit SDR 视频。输出保留可变帧率时间戳，复制兼容的音轨；音轨与容器不兼容时使用 MKV 或 `--no-audio`。

默认根据运动几何为每个镜头选择固定裁切窗口，保持宽高比和输出尺寸。`--crop` 设置等效每侧裁切预算，默认 8%，至少保留 70.56% 的面积；窗口可以偏心。`--crop-mode fixed` 使用指定的居中固定裁切量；`--crop-mode adaptive` 允许长镜头中缓慢移动窗口和缩放。预算不足时利用离线前视平滑调整补偿强度。报告分别记录裁切、网格形变和最终边界限幅，以及实际保留面积。扫描方向根据旋转元数据调整；已经旋转像素且移除元数据的素材可用 `--scan-axis` 指定。

轨迹处理沿用 [LightStab demo](https://github.com/liutao23/LightStab) 的网格位移累积、因果级联平滑与轨迹差补偿，在镜头起始处利用可靠的后续运动初始化扫描行平滑，使用无需训练的运动与置信度自适应高斯核。`--smooth` 和 `--rs-smooth` 调整时间核宽度；设为零时关闭对应补偿。

扫描行修正采用几何近似，适合小幅帧间运动；严重视差、快速前景运动、模糊和高频果冻效应需要单独检查。性能报告中的拟合残差用于诊断，不作为画质评分。

## 许可证

[MIT](LICENSE) © 2026 hiro。
