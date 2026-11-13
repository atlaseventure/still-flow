# Android SDK

English | [简体中文](README.zh-CN.md)

Video stabilization for Kotlin and Java apps on Android arm64, API 26 or later.

## Build and integrate

Install a JDK, Android SDK/NDK and CMake, and set `ANDROID_HOME` to the SDK directory. Run from the repository root:

```sh
./api/android/gradlew -p api/android assembleRelease
```

Copy the generated AAR into your app's `libs` directory and add:

```kotlin
dependencies {
    implementation(files("libs/stillflow-release.aar"))
}
```

The app needs the Kotlin standard library, included by Kotlin-enabled Android projects.

## Stabilize frames

Run processing on a worker thread. The app supplies decoded frames and handles output encoding and audio. Rendering defaults to GPU with CPU fallback.

This offline example uses `frames`, a list of full-resolution RGBA8 `StillFlow.Frame` objects backed by direct buffers with an explicit row stride. Supply increasing presentation timestamps and frame durations in microseconds. `consume` must finish reading each output before the buffer is reused.

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

Replay all requested recovery frames with their original pixels and timestamps before calling `finish()`. For video files, decode frames again as needed instead of keeping the entire video in memory.

## Streaming

Create `StillFlow.Stream(video, settings, streaming)` with separate `StreamSettings`; lookahead defaults to 1.35 seconds. Push with `tryPush()`, retain full frames by index, and call `poll()`. A `Ready` result owns a `FramePlan`; close it after rendering or extracting an immutable `WarpMap`. `Pending` means more input is needed. End with `finish()` and drain until `End`. Rejected pushes consume nothing: drain and retry, or drop the incoming live frame.

Plans and frame plans remain valid after their parent closes. Use `FramePlan.map(width, height)` for `GpuFrames` surface rendering. `StreamPlanner` supplies a bounded worker and returns plans with immutable maps. Configure process-wide OpenCV thread count with `configureThreads()` before starting workers.

For hardware video input and output, see the [video runner example](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt).
