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

## 使用

处理视频并生成前后对比：

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

默认自动使用 CUDA，不可用或执行失败时回退到 CPU。`--gpu off` 强制使用 CPU，`--gpu cuda` 要求 CUDA 可用，失败时退出。性能报告记录实际后端、设备、处理帧数和回退原因；GPU 阶段耗时包含数据传输与同步。运动估计、轨迹优化和视频编解码使用 CPU。

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
