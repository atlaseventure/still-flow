# Android SDK

[English](README.md) | 简体中文

Still Flow 为 Kotlin 和 Java 应用中的解码帧提供防抖，支持 Android arm64-v8a（API 26 及以上）。应用负责解码、输出编码和音频处理。

## 集成 AAR

安装 JDK、Android SDK/NDK 和 CMake，将 `ANDROID_HOME` 指向 SDK 目录，在仓库根目录构建。

```sh
./api/android/gradlew -p api/android assembleRelease
```

将 `stillflow-release.aar` 复制到应用的 `libs/` 目录，在应用的 Gradle 构建脚本中添加以下依赖。纯 Java 应用还需引入 Kotlin 标准库。

```kotlin
dependencies {
    implementation(files("libs/stillflow-release.aar"))
}
```

## 离线处理

按显示顺序送帧，时间戳须递增，帧时间戳和帧时长均以微秒计。视觉防抖使用 `StillFlow.Analyzer`。调用 `finish()` 前，对 `nextRecoveryFrame()` 请求的每一帧，用原始像素和时间戳调用 `recover()` 补送。处理视频文件时可按需重新解码。

陀螺仪防抖使用 `StillFlow.planGyro`，根据帧时间戳和时长、镜头配置及相机坐标系下的 IMU 样本生成计划。IMU 时间戳以**秒**计。渲染紧凑排列的像素帧时，按帧计划的输出宽高和输入像素格式为 `StillFlow.Renderer` 分配直接缓冲区，读取完输出后再复用。

## 流式处理

`StillFlow.Stream` 和 `StillFlow.GyroStream` 默认前视 1.35 秒。每个已接收帧的全分辨率图像都要保留到对应的帧计划就绪。

- `tryPush()` 或 `tryPushFrame()` 返回 `false` 表示该帧尚未被接收。先调用 `poll()` 取出就绪计划，再重试同一帧。
- 使用 `GyroStream` 时，通过 `pushImu()` 送入新增样本，在调用 `finish()` 前送完剩余样本。
- 调用 `finish()` 后，继续取出结果并渲染剩余帧，直到收到 `End`。每个 `Ready` 结果持有独立的帧计划，渲染或提取映射后须关闭该计划。

在 API 29 及以上版本，可在同一工作线程上使用 `GpuFrames` 渲染到编码器 Surface。陀螺仪帧计划使用 `gyroMap()`，编码器尺寸和 `GpuFrames` 输出尺寸均须与计划的输出宽高一致，可与解码尺寸不同。[MediaCodec 集成示例](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt)
