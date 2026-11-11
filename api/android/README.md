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

`Frame` accepts direct buffers containing gray8, BGR8 or RGBA8, with an explicit row stride. It captures the buffer's remaining range. Analysis copies input as needed; rendering writes to caller-owned storage. Keep input unchanged during each call and close sessions with Kotlin `use` or Java try-with-resources.

`Settings.backend` defaults to `AUTO`, preferring OpenGL ES 3.1 compute and falling back to CPU on backend failures. `GLES` requires GPU execution; `CPU` selects software rendering. `rendererInfo()` reports the active device and fallback reason. GPU rendering includes upload and readback for the buffer API.

Motion estimation defaults to `OPENCV`, using feature tracking for translation, rotation, scale and rolling-shutter motion. Analysis and trajectory planning run on CPU. `Settings.motionEstimator = TRANSLATION` selects the lightweight translation-only estimator for constrained workloads. The AAR includes OpenCV and its license notices.

`HardwareCodecs.decoder(format, surface)` and `HardwareCodecs.encoder(format)` return configured hardware `MediaCodec` instances. The host starts, feeds, drains and releases them. Pass a null decoder surface for `Image` output and use `AndroidFrames.luma(image)` for analysis before releasing the decoded buffer. For encoding, configure Surface input or flexible YUV input and supply the corresponding pixels. Packed render frames require conversion to the encoder's input format. On API 26–28, codec selection uses vendor names because the hardware capability flag is unavailable.

Set `settings.realtime = true` for streaming. Call `tryPush()` and drain `takeReady()`; retain full frames by index and call `renderReady()` before taking another result. Finish with `finishStream()` and drain again. A rejected push means the queue is full: drain and retry or drop the incoming frame. Copy MediaCodec output into host-owned buffers before releasing it. Delay audio by the same lookahead while preserving original PTS.
