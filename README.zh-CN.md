# Still Flow

[English](README.md) | 简体中文

无需训练的离线视频防抖与卷帘快门晃动抑制，提供命令行处理、前后对比视频和分阶段性能报告。主机版本使用 CPU，适合验证效果和定位计算瓶颈。

## 构建

Ubuntu 安装依赖并构建 Release 版本：

```sh
sudo apt install build-essential cmake ninja-build pkg-config libopencv-dev libeigen3-dev nlohmann-json3-dev libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## 使用

处理视频并生成前后对比：

```sh
./build/stillflow input.mp4 output.mp4 --compare comparison.mp4
```

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
