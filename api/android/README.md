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

Replay all requested recovery frames with their original pixels and timestamps before calling `finish()`. For video files, decode frames again as needed instead of keeping the entire video in memory.

## Streaming

Set `settings.realtime = true` when creating the session; lookahead defaults to 1.35 seconds. Push with `tryPush()`, retain full frames by index, and drain `takeReady()`. Call `renderReady()` with the corresponding frame before taking the next result. If a push is rejected, drain and retry, or drop the incoming live frame. End with `finishStream()` and drain the remaining results.

For hardware video input and output, see the [video runner example](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt).
