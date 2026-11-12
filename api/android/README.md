# Android SDK

Kotlin implementation with Java-callable bindings for frame stabilization, GPU rendering and Android hardware codecs. Build from the repository root with `./api/android/gradlew -p api/android assembleRelease`, then add the generated AAR to your app:

```kotlin
dependencies {
    implementation(files("libs/stillflow-release.aar"))
}
```

The consuming app needs the Kotlin standard library, supplied by Kotlin-enabled Android projects.

Run processing on a worker thread. In offline mode, analyze decoded frames in presentation order, replay requested recovery frames, finish the plan, then decode the original frames again and render by index. Timestamps and durations use microseconds.

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
    frames.forEachIndexed { index, rgbaFrame ->
        val stabilized = session.render(index, rgbaFrame, output)
        consume(stabilized) // Consume before reusing output.
    }
}
```

For offline recovery, decode again only when requested and feed the requested indices in order. `finish()` requires recovery to be complete. Use the same source luma and timestamps for analysis and replay.

For offline GPU Surface output on Android 10+, finish analysis, then decode again into an `ImageReader` Surface. Pass each decoded `Image`, `session.frameMap(index)` and its relative PTS to `GpuFrames.render(image, map, ptsUs, bt709, fullRange)`. Use the validated source color matrix/range and an encoder input Surface; drain the encoder around rendering. The call completes source reads before returning, so close the Image immediately afterward. This path requires `GL_EXT_YUV_target` and keeps full-resolution pixels on the GPU.

`Frame` accepts direct buffers containing gray8, BGR8 or RGBA8, with an explicit row stride. It captures the buffer's remaining range. Analysis copies input as needed; rendering writes to caller-owned storage. Keep input unchanged during each call and close sessions with Kotlin `use` or Java try-with-resources.

`Settings.backend` defaults to `AUTO`, preferring OpenGL ES 3.1 compute and falling back to CPU on backend failures. `GLES` requires GPU execution; `CPU` selects software rendering. `rendererInfo()` reports the active device and fallback reason. GPU rendering includes upload and readback for the buffer API.

Motion estimation defaults to `OPENCV`, using feature tracking for translation, rotation, scale and rolling-shutter motion. Analysis and trajectory planning run on CPU. `Settings.motionEstimator = TRANSLATION` selects the lightweight translation-only estimator for constrained workloads. The AAR includes OpenCV and its license notices.

`HardwareCodecs.decoder(format, surface)` and `HardwareCodecs.encoder(format)` return configured hardware `MediaCodec` instances. The host starts, feeds, drains and releases them. Pass a null decoder surface for `Image` output and use `AndroidFrames.luma(image)` for analysis before releasing the decoded buffer. For encoding, configure Surface input or flexible YUV input and supply the corresponding pixels. Packed render frames require conversion to the encoder's input format. On API 26–28, codec selection uses vendor names because the hardware capability flag is unavailable.

Set `settings.realtime = true` for streaming. Call `tryPush()` and drain `takeReady()`; retain full frames by index and call `renderReady()` before taking another result. Finish with `finishStream()` and drain again. A rejected push means the queue is full: drain and retry or drop the incoming frame. Copy MediaCodec output into host-owned buffers before releasing it. Delay audio by the same lookahead while preserving original PTS.

Use `StillFlow.mobileSettings()` for 640-pixel analysis with 600 features. OpenCV defaults to four threads; call `StillFlow.configureThreads()` before starting analysis workers to override this process-wide setting. Streaming adapts feature quotas within `settings.features`; set `settings.adaptiveTracking = false` for a fixed budget.

For GPU-resident streaming on Android 10+ with `GL_EXT_YUV_target`, decode into an `ImageReader` Surface and call `GpuFrames.captureAsync()` before closing each Image. Poll `readLuma()` on the GPU thread while consuming ready plans; drain pending luma before submitting more than three captures. Pass `wait = true` when flushing, or use `capture()` for a synchronous capture. Retain each frame by index, then use `takeReadyMap()` and `GpuFrames.render()` with the encoder input Surface. The GPU cache owns the delayed frames; only the reduced source-Y analysis image is read back. Keep all `GpuFrames` calls on its creating thread and drain the encoder while processing.

`StreamPlanner` runs analysis and planning on a worker with bounded input/output queues. Retry rejected submissions while consuming `takeReady()` results, then call `tryFinish()` and drain until `finished`. Plans own independent maps and can be consumed by the GPU thread while analysis continues. Close the planner to cancel pending work and join its worker.

`profile()` returns cumulative per-stage wall-time aggregates and P50/P95/P99 estimates with bounded native storage. `StreamPlanner.cpuProfile()` reports analysis and planning CPU time on its worker after finishing; OpenCV helper threads are excluded.
