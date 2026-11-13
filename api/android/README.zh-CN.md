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

StillFlow(width, height, fps).use { session ->
    frames.forEachIndexed { index, frame ->
        session.analyze(frame, timestampsUs[index], durationsUs[index])
    }
    var request = session.nextRecoveryFrame()
    while (request != null) {
        val index = request.index.toInt()
        session.recover(frames[index], timestampsUs[index])
        request = session.nextRecoveryFrame()
    }
    session.finish()
    val output = ByteBuffer.allocateDirect(width * height * 4)
    frames.forEachIndexed { index, frame ->
        consume(session.render(index, frame, output))
    }
}
```

调用 `finish()` 前，使用原始像素和时间戳补送全部请求的恢复帧。处理视频文件时可按需重新解码，避免将整个视频保存在内存中。

## 流式处理

创建会话时设置 `settings.realtime = true`，默认前视 1.35 秒。通过 `tryPush()` 推帧，按索引保留全分辨率帧，并取出 `takeReady()` 返回的就绪结果。每次使用对应帧调用 `renderReady()` 后，再取下一项。推帧被拒绝时，先处理就绪结果再重试，或丢弃新到达的直播帧。结束时调用 `finishStream()`，再处理剩余结果。

硬件视频输入输出用法见[视频运行器示例](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt)。
