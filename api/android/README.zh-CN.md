# Android SDK

[English](README.md) | 简体中文

为 Kotlin 和 Java 应用提供视频防抖，支持 Android arm64，要求 API 26 及以上。

## 构建与集成

安装 JDK、Android SDK/NDK 和 CMake，将 `ANDROID_HOME` 指向 SDK 目录。在仓库根目录运行：

```sh
./api/android/gradlew -p api/android assembleRelease
```

将生成的 AAR 复制到应用的 `libs` 目录，并添加依赖：

```kotlin
dependencies {
    implementation(files("libs/stillflow-release.aar"))
}
```

应用需要 Kotlin 标准库，启用 Kotlin 的 Android 项目已包含该依赖。

## 帧防抖

在工作线程执行处理。应用负责提供解码后的帧，以及输出编码和音频处理。渲染默认优先使用 GPU，失败时回退 CPU。

以下离线示例中，`frames` 是全分辨率 RGBA8 格式的 `StillFlow.Frame` 列表，使用直接缓冲区并显式指定行跨度。显示时间戳须递增，时间戳和帧时长均以微秒为单位。`consume` 须在缓冲区复用前完成输出读取。

```kotlin
import io.stillflow.StillFlow
import java.nio.ByteBuffer

val video = StillFlow.Video(width, height, fps)
val plan = StillFlow.Analyzer(video).use { analyzer ->
    frames.forEachIndexed { index, frame ->
        analyzer.push(frame, timestampsUs[index], durationsUs[index])
    }
    var request = analyzer.nextRecoveryFrame()
    while (request != null) {
        val index = request.index.toInt()
        analyzer.recover(frames[index], timestampsUs[index])
        request = analyzer.nextRecoveryFrame()
    }
    analyzer.finish()
}
plan.use {
    StillFlow.Renderer().use { renderer ->
        val output = ByteBuffer.allocateDirect(width * height * 4)
        frames.forEachIndexed { index, frame ->
            plan.frame(index.toLong()).use { framePlan ->
                consume(renderer.render(framePlan, frame, output))
            }
        }
    }
}
```

调用 `finish()` 前，使用原始像素和时间戳补送全部请求的恢复帧。处理视频文件时可按需重新解码，避免将整个视频保存在内存中。

## 流式处理

使用独立的 `StreamSettings` 创建 `StillFlow.Stream(video, settings, streaming)`，默认前视 1.35 秒。通过 `tryPush()` 推帧，按索引保留全分辨率帧，再调用 `poll()`。`Ready` 持有独立的 `FramePlan`，渲染或提取不可变 `WarpMap` 后关闭；`Pending` 表示需要更多输入。结束时调用 `finish()`，持续取出结果直到 `End`。被拒绝的输入未被消费，可先处理就绪结果再重试，或丢弃新到达的直播帧。

计划和单帧计划在父对象关闭后仍有效。`FramePlan.map(width, height)` 用于 `GpuFrames` Surface 渲染。`StreamPlanner` 提供有界工作线程，返回带不可变映射的计划。使用 `configureThreads()` 时，在启动工作线程前设置进程级 OpenCV 线程数。

硬件视频输入输出用法见[视频运行器示例](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt)。
