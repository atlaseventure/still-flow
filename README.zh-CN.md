# Still Flow

[English](README.md) | 简体中文

无需训练的离线视频防抖与卷帘快门晃动抑制，提供命令行处理、前后对比视频和分阶段性能报告。支持使用 NVIDIA GPU 加速坐标映射、边界检查和图像重采样。

## 构建

Linux 环境需具备 C++20 编译器、CMake、Ninja、Make、pkg-config、Python 和 Perl。首次构建下载固定版本的源码、校验完整性，并在本地构建媒体与数值计算依赖。

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

每个依赖默认使用 12 个编译任务，可通过 `-DSTILLFLOW_BUILD_JOBS=N` 调整。增量构建复用已下载的源码和已编译的依赖。

构建时自动检测 CUDA Toolkit；GPU 运行需要兼容的 NVIDIA 驱动。自定义工具包位置可通过 `CUDACXX` 指定 `nvcc`。使用 `-DSTILLFLOW_WITH_CUDA=OFF` 构建纯 CPU 版本。

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

需安装 FFmpeg，以及 `scripts/requirements.txt` 中的 Python 依赖。生成带抖动与卷帘快门效应的视频、稳定参考视频和参数记录：

```sh
python3 scripts/generate_video.py --output .local/synthetic --frames 60
```

使用 `--overwrite` 替换已有素材。

## 使用

处理视频并生成前后对比：

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

默认自动使用 CUDA，不可用或执行失败时回退到 CPU。`--backend cpu` 强制使用 CPU，`--backend cuda` 要求 CUDA 可用，失败时退出。性能报告按后端记录设备、能力、处理帧数和失败原因；渲染阶段耗时包含数据传输与同步。运动估计、轨迹优化和视频编解码使用 CPU。

使用相同裁剪量对照扫描行修正效果；`--rs off` 保留相同运动估计，仅关闭扫描行补偿。性能比较时省略对比视频：

```sh
./build/stillflow input.mp4 global.mp4 --rs off --report global.json
./build/stillflow input.mp4 corrected.mp4 --rs on --report corrected.json
./build/stillflow --help
```

输入使用逐行、方形像素、偶数宽高的 8-bit SDR 视频。输出保留可变帧率时间戳，复制兼容的音轨；音轨与容器不兼容时使用 MKV 或 `--no-audio`。

默认每侧裁剪 8%，保持输出尺寸。裁剪空间不足时自动降低修正强度。扫描方向根据旋转元数据调整；已经旋转像素且移除元数据的素材可用 `--scan-axis` 指定。

扫描行修正采用几何近似，适合小幅帧间运动；严重视差、快速前景运动、模糊和高频果冻效应需要单独检查。性能报告中的拟合残差用于诊断，不作为画质评分。

## 许可证

[MIT](LICENSE) © 2026 hiro。
