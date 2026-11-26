# Android SDK

English | [简体中文](README.zh-CN.md)

Still Flow stabilizes decoded frames in Kotlin and Java apps on Android arm64-v8a (API 26+). The app handles decoding, output encoding and audio.

## Integrate the AAR

Install a JDK, Android SDK/NDK and CMake. Set `ANDROID_HOME` to the SDK directory and build from the repository root.

```sh
./api/android/gradlew -p api/android assembleRelease
```

Copy `stillflow-release.aar` to the app's `libs/` directory and add the following to its Gradle build. Java-only apps also need the Kotlin standard library.

```kotlin
dependencies {
    implementation(files("libs/stillflow-release.aar"))
}
```

## Offline processing

Pass frames in presentation order with increasing timestamps and frame durations in microseconds. For visual stabilization, push frames to `StillFlow.Analyzer`. Before calling `finish()`, replay every frame requested by `nextRecoveryFrame()` through `recover()` with its original pixels and timestamp. File inputs can be decoded again on demand.

For gyroscope stabilization, `StillFlow.planGyro` builds the plan from frame timing, a lens profile and camera-frame IMU samples. IMU timestamps are in **seconds**. For packed frames, give `StillFlow.Renderer` a direct output buffer sized for the frame plan's output dimensions and the input pixel format. Finish reading the output before reusing that buffer.

## Streaming

`StillFlow.Stream` and `StillFlow.GyroStream` default to 1.35 seconds of lookahead. Retain each accepted full-resolution frame until its plan is ready.

- If `tryPush()` or `tryPushFrame()` returns `false`, the frame was not consumed. Poll for ready plans before retrying the same frame.
- With `GyroStream`, supply new samples through `pushImu()` and send any remaining samples before calling `finish()`.
- After `finish()`, keep polling until `End` to render the final frames. Each `Ready` result owns a frame plan. Close it after rendering or extracting its map.

On API 29+, use `GpuFrames` on a single worker thread to render to an encoder Surface. For gyroscope plans, use `gyroMap()` and set both the encoder size and the `GpuFrames` output size to the planned dimensions, which can differ from the decoder size. [MediaCodec integration example](../../tools/android/src/main/kotlin/io/stillflow/runner/ExportRunner.kt)
