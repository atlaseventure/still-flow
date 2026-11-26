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

## Gyroscope stabilization

`StillFlow.planGyro` takes input dimensions, frame timestamps and durations in microseconds, camera-frame IMU samples, a lens profile and gyro settings. It returns a standard offline `StillFlow.Plan`.

On API 29+, use `FramePlan.gyroMap()` with `GpuFrames` for zero-copy rendering to an encoder Surface. Set the encoder and `GpuFrames` to the plan's output dimensions, independently of the decoder size. Here `nextDecodedImage()` supplies the next decoder image in presentation order, and `bt709` and `fullRange` describe the input color format.

```kotlin
val settings = StillFlow.GyroSettings().apply {
    outputWidth = 2704
    outputHeight = 1520
}
StillFlow.planGyro(width, height, timestampsUs, durationsUs, imuSamples, lens, settings).use { plan ->
    GpuFrames(width, height, 2, encoderSurface,
        outputWidth = settings.outputWidth, outputHeight = settings.outputHeight).use { gpu ->
        timestampsUs.forEachIndexed { index, ptsUs ->
            plan.frame(index.toLong()).use { frame ->
                nextDecodedImage().use { image ->
                    gpu.render(image, frame.gyroMap(), ptsUs - timestampsUs[0], bt709, fullRange)
                }
            }
        }
    }
}
```

For packed frames, `StillFlow.Renderer` accepts gyro frame plans with `StillFlow.Backend.GLES` (compute) or `StillFlow.Backend.CPU`. Size the direct destination buffer for the frame plan's `outputWidth × outputHeight` and the input pixel format. The returned `StillFlow.Frame` has those output dimensions.

For fixed-lookahead streaming on API 29+, configure the encoder Surface for the `StillFlow.GyroStream` output dimensions. On one worker thread, the app's `decodeFrames` supplies each image in presentation order with its zero-based timestamp and duration in microseconds and its new IMU samples. It closes the image when the callback returns.

```kotlin
StillFlow.GyroStream(width, height, lens, settings, streaming).use { stream ->
    GpuFrames(width, height, streaming.maxBufferedFrames, encoderSurface,
        outputWidth = stream.outputWidth, outputHeight = stream.outputHeight).use { gpu ->
        val pending = ArrayDeque<GpuFrames.Frame>()
        fun drain(): StillFlow.StreamResult {
            while (true) {
                val result = stream.poll()
                if (result !is StillFlow.StreamResult.Ready) return result
                result.plan.use { gpu.render(pending.removeFirst(), it.gyroMap(), it.info.ptsUs) }
            }
        }

        decodeFrames { image, ptsUs, durationUs, imuBatch ->
            stream.pushImu(imuBatch)
            while (!stream.tryPushFrame(ptsUs, durationUs)) drain()
            pending.addLast(gpu.captureTexture(image, bt709, fullRange))
            drain()
        }
        stream.pushImu(remainingImu)
        stream.finish()
        check(drain() == StillFlow.StreamResult.End)
    }
}
```

`captureTexture` keeps pending images on the GPU without pixel readback. Rendering in plan order returns textures to the pool, bounded by `maxBufferedFrames`. Call `profile()` before closing the stream to read cumulative stage timings.

## Streaming

Create `StillFlow.Stream(video, settings, streaming)` with separate `StreamSettings`; lookahead defaults to 1.35 seconds. Push with `tryPush()`, retain full frames by index, and call `poll()`. A `Ready` result owns a `FramePlan`; close it after rendering or extracting an immutable `WarpMap`. `Pending` means more input is needed. End with `finish()` and drain until `End`. Rejected pushes consume nothing: drain and retry, or drop the incoming live frame.

Plans and frame plans remain valid after their parent closes. Use `FramePlan.map(width, height)` for `GpuFrames` surface rendering. `StreamPlanner` supplies a bounded worker and returns plans with immutable maps. Configure process-wide OpenCV thread count with `configureThreads()` before starting workers.

For hardware video input and output, see the [video runner example](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt).
