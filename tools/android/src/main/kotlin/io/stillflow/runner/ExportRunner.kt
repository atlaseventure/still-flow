package io.stillflow.runner

import android.app.Instrumentation
import android.media.Image
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaExtractor
import android.media.MediaFormat
import android.media.MediaMuxer
import android.os.Build
import android.os.Bundle
import io.stillflow.AndroidFrames
import io.stillflow.StreamPlanner
import io.stillflow.GpuFrames
import android.media.ImageReader
import android.graphics.ImageFormat
import android.hardware.HardwareBuffer
import io.stillflow.HardwareCodecs
import io.stillflow.StillFlow
import java.io.File
import java.nio.ByteBuffer
import kotlin.math.abs
import kotlin.math.sin
import org.json.JSONObject

/** Headless consumer of the packaged AAR, invoked by scripts/android.py. */
class ExportRunner : Instrumentation() {
    private var runId = ""
    private val report = JSONObject()

    override fun onCreate(arguments: Bundle?) {
        super.onCreate(arguments)
        runId = arguments?.getString("run") ?: ""
        start()
    }

    override fun onStart() {
        try {
            require(runId.matches(Regex("[a-f0-9]{32}"))) { "Invalid run ID" }
            val directory = File(targetContext.filesDir, runId)
            val config = JSONObject(File(directory, "config.json").readText())
            val realtime = config.optBoolean("realtime")
            if (config.optBoolean("verifyBackends")) report.put("backend_verification", verifyGyroBackends())
            when {
                realtime && config.has("gyro") -> exportGyroStream(directory, config)
                config.has("gyro") -> exportGyro(directory, config)
                realtime -> exportStream(directory, config)
                else -> exportOffline(directory, config)
            }
            report.put("status", "passed").put("run", runId)
                .put("sdk", StillFlow.buildDescription()).put("model", Build.MODEL)
                .put("manufacturer", Build.MANUFACTURER).put("api", Build.VERSION.SDK_INT)
            File(directory, "result.json").writeText(report.toString(2))
            finish(-1, Bundle().apply { putString("stream", "STILLFLOW_COMPLETE $runId\n") })
        } catch (error: Throwable) {
            finish(1, Bundle().apply { putString("stream", "STILLFLOW_FAILED $runId\n${error.stackTraceToString()}") })
        }
    }

    private fun timingJson(stages: Map<String, TimingStatistics>) = JSONObject().apply {
        stages.forEach { (name, stats) ->
            put(name, JSONObject().put("calls", stats.count).put("total_ms", stats.total)
                .put("mean_ms", if (stats.count == 0L) 0.0 else stats.total / stats.count)
                .put("p50_ms", stats.percentile(0.50)).put("p95_ms", stats.percentile(0.95))
                .put("p99_ms", stats.percentile(0.99)).put("max_ms", stats.maximum))
        }
    }

    private fun profilingJson() = JSONObject()
        .put("percentile_method", "logarithmic_histogram_midpoint")
        .put("histogram_min_ms", TimingStatistics.HISTOGRAM_MIN_MS)
        .put("histogram_bins_per_octave", TimingStatistics.HISTOGRAM_BINS_PER_OCTAVE)
        .put("histogram_bins", TimingStatistics.HISTOGRAM_BINS)

    private inline fun <T> timed(stages: MutableMap<String, TimingStatistics>, stage: String, work: () -> T): T {
        val start = System.nanoTime()
        try { return work() } finally {
            stages.getOrPut(stage) { TimingStatistics() }.add((System.nanoTime() - start) / 1e6)
        }
    }

    private fun progress(stage: String, frames: Int) {
        val event = JSONObject().put("stage", stage).put("frames", frames)
        sendStatus(0, Bundle().apply { putString("stream", "STILLFLOW_EVENT $event\n") })
    }

    private fun extractor(input: File): MediaExtractor {
        val ex = MediaExtractor()
        try {
            ex.setDataSource(input.path)
            for (index in 0 until ex.trackCount) {
                if (ex.getTrackFormat(index).getString(MediaFormat.KEY_MIME)?.startsWith("video/") == true) {
                    ex.selectTrack(index)
                    return ex
                }
            }
            error("No video track")
        } catch (error: Throwable) {
            ex.release()
            throw error
        }
    }

    private fun decode(input: File, needed: () -> Boolean = { true }, images: ImageReader? = null, consume: (Image, Long, MediaFormat) -> Unit): Int {
        val ex = extractor(input)
        var decoder: MediaCodec? = null
        try {
            val format = ex.getTrackFormat(ex.sampleTrackIndex)
            if (images == null) format.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            val codec = HardwareCodecs.decoder(format, images?.surface)
            decoder = codec
            check(codec.codecInfo.isHardwareAccelerated) { "Software decoder" }
            report.put("decoder", codec.name)
            codec.start()
            var inputEnded = false
            var done = false
            var count = 0
            var lastProgress = System.nanoTime()
            val info = MediaCodec.BufferInfo()
            var outputFormat = format
            while (!done && needed()) {
                check(System.nanoTime() - lastProgress < STALL_NS) { "Decoder stalled" }
                if (!inputEnded) {
                    val index = codec.dequeueInputBuffer(1000)
                    if (index >= 0) {
                        val bytes = ex.readSampleData(checkNotNull(codec.getInputBuffer(index)), 0)
                        if (bytes < 0) {
                            codec.queueInputBuffer(index, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                            inputEnded = true
                        } else {
                            codec.queueInputBuffer(index, 0, bytes, ex.sampleTime, 0)
                            ex.advance()
                        }
                    }
                }
                val index = codec.dequeueOutputBuffer(info, 1000)
                if (index == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) outputFormat = codec.outputFormat
                if (index >= 0) {
                    var released = false
                    try {
                        if (info.size > 0) {
                            if (images == null) {
                                checkNotNull(codec.getOutputImage(index)) { "Missing decoded Image" }.use {
                                    consume(it, info.presentationTimeUs, outputFormat)
                                }
                            } else {
                                codec.releaseOutputBuffer(index, true)
                                released = true
                                val deadline = System.nanoTime() + STALL_NS
                                var image = images.acquireNextImage()
                                while (image == null) {
                                    check(System.nanoTime() < deadline) { "Decoder Surface stalled" }
                                    Thread.sleep(1)
                                    image = images.acquireNextImage()
                                }
                                image.use { consume(it, info.presentationTimeUs, outputFormat) }
                            }
                            count++
                        }
                        done = info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                        lastProgress = System.nanoTime()
                    } finally {
                        if (!released) codec.releaseOutputBuffer(index, false)
                    }
                }
            }
            return count
        } finally {
            decoder?.release()
            ex.release()
        }
    }

    private fun verifyGyroBackends(): JSONObject {
        val width = 160
        val height = 120
        val times = LongArray(7) { it * 33333L }
        val durations = LongArray(times.size) { 33333L }
        var maxDelta = 0
        val verification = JSONObject()
        StillFlow.Renderer(StillFlow.Backend.CPU).use { cpu ->
            StillFlow.Renderer(StillFlow.Backend.GLES).use { gles ->
                fun compareFrame(step: StillFlow.FramePlan, moving: Boolean, settings: StillFlow.GyroSettings): Int {
                    var maximum = 0
                    val map = step.gyroMap()
                    if (moving) check(map.sensorRows && map.rowCount > 1) else {
                        check(map.outFx == map.inFx && map.outFy == map.inFy && map.fovScale == 1.0)
                        check(map.rowRotation(0).contentEquals(doubleArrayOf(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)))
                    }
                    for (format in StillFlow.PixelFormat.entries) {
                        val channels = format.bytesPerPixel
                        val pixels = ByteBuffer.allocateDirect(width * height * channels)
                        for (y in 0 until height) for (x in 0 until width) for (c in 0 until channels) {
                            pixels.put(((x * 7 + y * 13 + c * 37) % 256).toByte())
                        }
                        pixels.flip()
                        val frame = StillFlow.Frame(width, height, format, width * channels, pixels)
                        fun render(renderer: StillFlow.Renderer): ByteBuffer {
                            val destination = ByteBuffer.allocateDirect(settings.outputWidth * settings.outputHeight * channels)
                            val output = renderer.render(step, frame, destination)
                            check(output.width == settings.outputWidth && output.height == settings.outputHeight && output.format == format)
                            return output.pixels
                        }
                        val a = render(cpu)
                        val b = render(gles)
                        for (i in 0 until a.remaining()) {
                            if (!moving) {
                                val stride = settings.outputWidth * channels
                                val expected = pixels[((i / stride + 2) * width + 2) * channels + i % stride]
                                check(a[i] == expected && b[i] == expected) { "Identity mismatch at $format byte $i" }
                            }
                            val delta = abs((a[i].toInt() and 255) - (b[i].toInt() and 255))
                            check(delta <= 1) { "CPU/GLES mismatch at $format byte $i: $delta" }
                            maximum = maxOf(maximum, delta)
                        }
                    }
                    return maximum
                }

                for (moving in listOf(false, true)) {
                    val lens = StillFlow.LensProfile().apply {
                        fx = 70.2
                        fy = 70.2
                        cx = (width - 1) / 2.0
                        cy = (height - 1) / 2.0
                        if (moving) {
                            model = StillFlow.LensModel.OPENCV_FISHEYE
                            k = doubleArrayOf(0.04440465777694087, 0.01946789951179939, -0.004476697539343917, -0.002042912877740792)
                        }
                    }
                    val settings = StillFlow.GyroSettings().apply {
                        // An integer crop retains the planner's sensor margin with unit pixel scale.
                        outputWidth = if (moving) width else width - 4
                        outputHeight = if (moving) 90 else height - 4
                        fov = outputWidth.toDouble() / width
                        readoutMs = if (moving) 11.0 else 0.0
                    }
                    val imu = (0..60).map { i ->
                        val t = i * 0.005
                        if (moving) StillFlow.ImuSample(t, 0.22 * sin(80 * t), 0.4, 0.11 * sin(120 * t))
                        else StillFlow.ImuSample(t, 0.0, 0.0, 0.0)
                    }
                    StillFlow.planGyro(width, height, times, durations, imu, lens, settings).use { plan ->
                        plan.frame(3).use { step ->
                            maxDelta = maxOf(maxDelta, compareFrame(step, moving, settings))
                        }
                    }
                    if (moving) {
                        val streaming = StillFlow.StreamSettings().apply { maxBufferedFrames = 2 }
                        StillFlow.GyroStream(width, height, lens, settings, streaming).use { stream ->
                            check(stream.outputWidth == settings.outputWidth && stream.outputHeight == settings.outputHeight)
                            check(stream.poll() === StillFlow.StreamResult.Pending)
                            stream.pushImu(emptyList())
                            var cursor = 0
                            var emitted = 0
                            var retries = 0
                            var streamDelta = 0
                            fun consumeReady() {
                                val result = stream.poll()
                                check(result is StillFlow.StreamResult.Ready)
                                result.plan.use { step ->
                                    val info = step.info
                                    check(info.index == emitted.toLong() && info.ptsUs == times[emitted] && info.durationUs == durations[emitted])
                                    streamDelta = maxOf(streamDelta, compareFrame(step, true, settings))
                                }
                                emitted++
                            }
                            for (i in times.indices) {
                                val cutoff = times[i] / 1e6 + settings.readoutMs / 2000
                                while (cursor < imu.size && imu[cursor].timeSeconds <= cutoff) stream.pushImu(imu[cursor++])
                                if (!stream.tryPushFrame(times[i], durations[i])) {
                                    consumeReady()
                                    check(stream.tryPushFrame(times[i], durations[i]))
                                    retries++
                                }
                            }
                            check(retries > 0)
                            stream.pushImu(imu.subList(cursor, imu.size))
                            stream.finish()
                            stream.finish()
                            while (emitted < times.size) consumeReady()
                            check(stream.poll() === StillFlow.StreamResult.End)
                            verification.put("stream_max_channel_delta", streamDelta)
                        }
                    }
                }
                check(cpu.info.backend == StillFlow.Backend.CPU && gles.info.backend == StillFlow.Backend.GLES && gles.info.gpu)
                verification.put("gles_device", gles.info.device)
            }
        }
        return verification.put("status", "passed").put("max_channel_delta", maxDelta).put("identity_max_channel_delta", 0)
            .put("formats", org.json.JSONArray(StillFlow.PixelFormat.entries.map { it.name }))
    }

    private data class GyroInput(val lens: StillFlow.LensProfile, val settings: StillFlow.GyroSettings, val samples: List<StillFlow.ImuSample>)

    private fun gyroInput(directory: File, gyro: JSONObject): GyroInput {
        val calibration = gyro.getJSONObject("lens")
        val lens = StillFlow.LensProfile().apply {
            model = StillFlow.LensModel.valueOf(calibration.getString("model"))
            calibWidth = calibration.getInt("calibWidth")
            calibHeight = calibration.getInt("calibHeight")
            fx = calibration.getDouble("fx")
            fy = calibration.getDouble("fy")
            cx = calibration.getDouble("cx")
            cy = calibration.getDouble("cy")
            k = calibration.getJSONArray("k").let { values -> DoubleArray(values.length()) { values.getDouble(it) } }
            lensCorrectionAmount = calibration.optDouble("lensCorrectionAmount", lensCorrectionAmount)
        }
        val options = gyro.getJSONObject("settings")
        val settings = StillFlow.GyroSettings().apply {
            smoothness = options.optDouble("smoothness", smoothness)
            smoothnessPitch = options.optDouble("smoothnessPitch", smoothnessPitch)
            smoothnessYaw = options.optDouble("smoothnessYaw", smoothnessYaw)
            smoothnessRoll = options.optDouble("smoothnessRoll", smoothnessRoll)
            maxSmoothness = options.optDouble("maxSmoothness", maxSmoothness)
            alpha01s = options.optDouble("alpha01s", alpha01s)
            readoutMs = options.optDouble("readoutMs", readoutMs)
            fov = options.optDouble("fov", fov)
            maxZoom = options.optDouble("maxZoom", maxZoom)
            adaptiveZoomWindowS = options.optDouble("adaptiveZoomWindowS", adaptiveZoomWindowS)
            timeOffsetS = options.optDouble("timeOffsetS", timeOffsetS)
            perAxis = options.optBoolean("perAxis", perAxis)
            readoutTopToBottom = options.optBoolean("readoutTopToBottom", readoutTopToBottom)
            adaptiveZoom = options.optBoolean("adaptiveZoom", adaptiveZoom)
            zoomEnvelope = options.optBoolean("zoomEnvelope", zoomEnvelope)
            outputWidth = options.optInt("outputWidth", outputWidth)
            outputHeight = options.optInt("outputHeight", outputHeight)
            maxZoomIterations = options.optInt("maxZoomIterations", maxZoomIterations)
            integrationMethod = StillFlow.ImuIntegration.valueOf(options.optString("integrationMethod", integrationMethod.name))
        }
        val samples = File(directory, "imu.csv").bufferedReader().use { reader ->
            val header = reader.readLine()
            val accel = header == "t_video_s,wx,wy,wz,ax,ay,az"
            require(accel || header == "t_video_s,wx,wy,wz") { "Invalid IMU header" }
            reader.lineSequence().map { line ->
                val v = line.split(',').map { it.toDouble() }
                require(v.size == if (accel) 7 else 4)
                if (accel) StillFlow.ImuSample(v[0], v[1], v[2], v[3], v[4], v[5], v[6], true)
                else StillFlow.ImuSample(v[0], v[1], v[2], v[3])
            }.toList()
        }
        return GyroInput(lens, settings, samples)
    }

    private fun exportGyro(directory: File, config: JSONObject) {
        val began = System.nanoTime()
        val stages = linkedMapOf<String, TimingStatistics>()
        val gyro = config.getJSONObject("gyro")
        val (lens, settings, samples) = gyroInput(directory, gyro)
        val input = File(directory, "input.mp4")
        val timestamps = timed(stages, "analysis.timestamps") {
            val ex = extractor(input)
            try {
                buildList {
                    while (ex.sampleTime >= 0) {
                        add(ex.sampleTime)
                        ex.advance()
                    }
                }.sorted().toLongArray()
            } finally { ex.release() }
        }
        val durations = LongArray(timestamps.size) { i ->
            if (i == timestamps.lastIndex) Math.round(1_000_000 / config.getDouble("fps")) else timestamps[i + 1] - timestamps[i]
        }
        val width = config.getInt("width")
        val height = config.getInt("height")
        timed(stages, "plan.gyro") { StillFlow.planGyro(width, height, timestamps, durations, samples, lens, settings) }.use { plan ->
            val (outWidth, outHeight) = plan.frame(0).use { it.outputWidth to it.outputHeight }
            var minZoom = Double.POSITIVE_INFINITY
            var maxZoom = 0.0
            var sumZoom = 0.0
            renderOfflineSurface(input, File(directory, "output.mp4"), config, plan, timestamps.asList(), stages,
                outWidth, outHeight, { frame -> frame.gyroMap().also {
                    minZoom = minOf(minZoom, it.zoomRatio)
                    maxZoom = maxOf(maxZoom, it.zoomRatio)
                    sumZoom += it.zoomRatio
                } }) { image, map, pts, bt709, fullRange -> render(image, map, pts, bt709, fullRange) }
            gyro.put("zoom_ratio", JSONObject().put("min", minZoom).put("mean", sumZoom / timestamps.size).put("max", maxZoom))
            report.put("gyro", gyro).put("output_width", outWidth).put("output_height", outHeight)
        }
        report.put("mode", "offline_gyro").put("decode_passes", 1).put("stages", timingJson(stages))
            .put("profiling", profilingJson()).put("total_seconds", (System.nanoTime() - began) / 1e9)
    }

    private fun exportOffline(directory: File, config: JSONObject) {
        val input = File(directory, "input.mp4")
        val output = File(directory, "output.mp4")
        val width = config.getInt("width")
        val height = config.getInt("height")
        val fps = config.getDouble("fps")
        require(width in 32..4096 && height in 32..4096 && width % 2 == 0 && height % 2 == 0 && fps.isFinite() && fps > 0)
        val options = config.getJSONObject("settings")
        StillFlow.configureThreads(options.optInt("visionThreads", 4))
        val settings = StillFlow.Settings().apply {
            motionEstimator = StillFlow.MotionEstimator.valueOf(options.optString("motionEstimator", motionEstimator.name))
            analysisWidth = options.optInt("analysisWidth", analysisWidth)
            features = options.optInt("features", features)
            bands = options.optInt("bands", bands)
            cropMode = StillFlow.CropMode.valueOf(options.optString("cropMode", cropMode.name))
            rollingShutter = options.optBoolean("rollingShutter", rollingShutter)
            smoothSeconds = options.optDouble("smoothSeconds", smoothSeconds)
            rsSeconds = options.optDouble("rsSeconds", rsSeconds)
            crop = options.optDouble("crop", crop)
            rsStrength = options.optDouble("rsStrength", rsStrength)
        }
        report.put("settings", JSONObject().apply {
            put("backend", StillFlow.Backend.GLES.name)
            put("motionEstimator", settings.motionEstimator.name)
            put("analysisWidth", settings.analysisWidth)
            put("features", settings.features)
            put("bands", settings.bands)
            put("cropMode", settings.cropMode.name)
            put("rollingShutter", settings.rollingShutter)
            put("smoothSeconds", settings.smoothSeconds)
            put("rsSeconds", settings.rsSeconds)
            put("crop", settings.crop)
            put("rsStrength", settings.rsStrength)
        })
        val timestamps = mutableListOf<Long>()
        val durationUs = Math.round(1_000_000 / fps)
        val began = System.nanoTime()
        val stages = linkedMapOf<String, TimingStatistics>()
        StillFlow.Analyzer(StillFlow.Video(width, height, fps, config.getBoolean("horizontalScan")), settings).use { session ->
            val analysisStart = System.nanoTime()
            val frames = decode(input) { image, pts, _ ->
                timed(stages, "analysis.analyze") { session.push(AndroidFrames.luma(image), pts, durationUs) }
                timestamps.add(pts)
                if (timestamps.size % 60 == 0) progress("analyze", timestamps.size)
            }
            stages.getOrPut("analysis.decode_pass") { TimingStatistics() }.add((System.nanoTime() - analysisStart) / 1e6)
            var request = session.nextRecoveryFrame()
            var recoveryFrames = 0
            if (request != null) {
                var index = 0L
                decode(input, { request != null }) { image, pts, _ ->
                    if (index == checkNotNull(request).index) {
                        session.recover(AndroidFrames.luma(image), pts)
                        recoveryFrames++
                        request = session.nextRecoveryFrame()
                    }
                    index++
                }
                check(request == null) { "Input ended during tracking recovery" }
            }
            report.put("recovery_frames", recoveryFrames)
            timed(stages, "analysis.plan") { session.finish() }.use { plan ->
                session.close()
                check(frames > 0 && plan.info.frameCount == frames.toLong()) { "Analysis frame count mismatch" }
                report.put("analysis_seconds", (System.nanoTime() - began) / 1e9)
                report.put("mode", "offline").put("decode_passes", if (recoveryFrames > 0) 3 else 2)
                renderOfflineSurface(input, output, config, plan, timestamps, stages, width, height, { it.map(width, height) }) {
                    image, map, pts, bt709, fullRange -> render(image, map, pts, bt709, fullRange)
                }
            }
            report.put("stages", timingJson(stages)).put("profiling", profilingJson())
                .put("total_seconds", (System.nanoTime() - began) / 1e9)
        }
    }

    private fun <M> renderOfflineSurface(
        input: File, output: File, config: JSONObject, plan: StillFlow.Plan,
        timestamps: List<Long>, stages: MutableMap<String, TimingStatistics>, outWidth: Int, outHeight: Int,
        frameMap: (StillFlow.FramePlan) -> M,
        renderMap: GpuFrames.(Image, M, Long, Boolean, Boolean) -> Unit,
    ) {
        val width = config.getInt("width")
        val height = config.getInt("height")
        val format = MediaFormat.createVideoFormat("video/avc", outWidth, outHeight).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, config.getInt("bitrate"))
            setFloat(MediaFormat.KEY_FRAME_RATE, config.getDouble("fps").toFloat())
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
            setInteger(MediaFormat.KEY_COLOR_STANDARD, MediaFormat.COLOR_STANDARD_BT709)
            setInteger(MediaFormat.KEY_COLOR_RANGE, MediaFormat.COLOR_RANGE_LIMITED)
            setInteger(MediaFormat.KEY_COLOR_TRANSFER, MediaFormat.COLOR_TRANSFER_SDR_VIDEO)
        }
        val encoder = HardwareCodecs.encoder(format)
        try {
            val surface = encoder.createInputSurface()
            try {
                val writer = MediaMuxer(output.path, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
                try {
                    val sink = EncoderSink(encoder, writer)
                    encoder.start()
                    GpuFrames(width, height, 2, surface, outputWidth = outWidth, outputHeight = outHeight).use { gpu ->
                        ImageReader.newInstance(width, height, ImageFormat.PRIVATE, 2, HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE).use { images ->
                            var index = 0
                            val renderStart = System.nanoTime()
                            val rendered = decode(input, images = images) { image, pts, _ ->
                                check(index < timestamps.size && pts == timestamps[index]) { "Decode timestamp mismatch" }
                                val map = timed(stages, "render.map") { plan.frame(index.toLong()).use(frameMap) }
                                timed(stages, "render.surface") {
                                    sink.drain(0)
                                    gpu.renderMap(image, map, pts - timestamps[0], config.getBoolean("bt709"), config.getBoolean("fullRange"))
                                    sink.drain(0)
                                }
                                index++
                                if (index % 60 == 0) progress("render", index)
                            }
                            timed(stages, "render.flush") { sink.finish() }
                            stages.getOrPut("render.decode_pass") { TimingStatistics() }.add((System.nanoTime() - renderStart) / 1e6)
                            check(rendered == timestamps.size && index == rendered && sink.frames == rendered) { "GPU offline export lost frames" }
                            report.put("frames", rendered).put("width", width).put("height", height)
                                .put("backend", "gles").put("gpu", true).put("device", gpu.device)
                                .put("encoder", encoder.name).put("gpu_surface", true).put("fallback_reason", "")
                        }
                    }
                } finally { writer.release() }
            } finally { surface.release() }
        } finally { encoder.release() }
    }

    private fun exportStream(directory: File, config: JSONObject) {
        val input = File(directory, "input.mp4")
        val width = config.getInt("width")
        val height = config.getInt("height")
        val fps = config.getDouble("fps")
        require(width in 32..4096 && height in 32..4096 && width % 2 == 0 && height % 2 == 0 && fps.isFinite() && fps > 0)
        val options = config.getJSONObject("settings")
        StillFlow.configureThreads(options.optInt("visionThreads", 4))
        val settings = StillFlow.Settings().apply {
            motionEstimator = StillFlow.MotionEstimator.valueOf(options.optString("motionEstimator", motionEstimator.name))
            analysisWidth = options.optInt("analysisWidth", analysisWidth)
            features = options.optInt("features", features)
            bands = options.optInt("bands", bands)
            cropMode = StillFlow.CropMode.valueOf(options.optString("cropMode", cropMode.name))
            rollingShutter = options.optBoolean("rollingShutter", rollingShutter)
            smoothSeconds = options.optDouble("smoothSeconds", smoothSeconds)
            rsSeconds = options.optDouble("rsSeconds", rsSeconds)
            crop = options.optDouble("crop", crop)
            rsStrength = options.optDouble("rsStrength", rsStrength)
        }
        report.put("settings", JSONObject().apply {
            put("backend", StillFlow.Backend.GLES.name)
            put("motionEstimator", settings.motionEstimator.name)
            put("analysisWidth", settings.analysisWidth)
            put("features", settings.features)
            put("bands", settings.bands)
            put("cropMode", settings.cropMode.name)
            put("rollingShutter", settings.rollingShutter)
            put("smoothSeconds", settings.smoothSeconds)
            put("rsSeconds", settings.rsSeconds)
            put("crop", settings.crop)
            put("rsStrength", settings.rsStrength)
        })
        val streaming = streamSettings(options)
        report.getJSONObject("settings").put("adaptiveTracking", streaming.adaptiveTracking)
        val scale = minOf(1.0, settings.analysisWidth.toDouble() / width)
        val aw = maxOf(32, Math.round(width * scale).toInt())
        val ah = maxOf(32, Math.round(height * scale).toInt())
        val duration = Math.round(1_000_000 / fps)
        val began = System.nanoTime()
        withStreamSurface(directory, config, width, height, streaming.maxBufferedFrames, width, height,
            config.optBoolean("computeReadback", true), began) { gpu, images, sink, encoded, encoder ->
            StreamPlanner(StillFlow.Video(width, height, fps, config.getBoolean("horizontalScan")), settings, streaming).use { planner ->
                val pending = java.util.ArrayDeque<Pending>()
                val awaitingAnalysis = java.util.ArrayDeque<Pending>()
                val stages = linkedMapOf<String, TimingStatistics>()
                fun mark(name: String, start: Long) { stages.getOrPut(name) { TimingStatistics() }.add((System.nanoTime() - start) / 1e6) }
                var received = 0
                var rendered = 0
                var peak = 0
                var firstOutputSeconds = 0.0
                var firstOutputAfterInputs = 0
                var origin = 0L
                var latest = 0L
                val events = org.json.JSONArray()
                fun drain(flushing: Boolean) {
                    while (true) {
                        val ready = planner.takeReady() ?: break
                        val item = pending.removeFirst()
                        check(ready.index == item.index && ready.index == rendered.toLong() && ready.ptsUs == item.pts)
                        val start = System.nanoTime()
                        sink.drain(0)
                        gpu.render(item.frame, ready, origin)
                        val renderCompleted = System.nanoTime()
                        stages.getOrPut("render.cpu_submit") { TimingStatistics() }.add(gpu.renderSubmitMs)
                        stages.getOrPut("render.surface_submit_wait") { TimingStatistics() }.add(gpu.surfaceSubmitMs)
                        sink.drain(0)
                        mark("render.surface", start)
                        events.put(JSONObject().put("frame", ready.index).put("pts_us", ready.ptsUs)
                            .put("duration_us", ready.durationUs).put("flushing", flushing)
                            .put("input_ns", item.receivedNs - began)
                            .put("analysis_start_ns", ready.analysisStartedNs - began)
                            .put("analysis_complete_ns", ready.analysisCompletedNs - began)
                            .put("plan_complete_ns", ready.planCompletedNs - began)
                            .put("render_start_ns", start - began).put("render_complete_ns", renderCompleted - began))
                        stages.getOrPut("stream.residence") { TimingStatistics() }.add((System.nanoTime()-item.receivedNs)/1e6)
                        if (rendered == 0) {
                            firstOutputSeconds = (System.nanoTime()-began)/1e9
                            firstOutputAfterInputs = received
                        }
                        rendered++
                    }
                }
                fun submitAnalysis(wait: Boolean) {
                    while (awaitingAnalysis.isNotEmpty()) {
                        val item = awaitingAnalysis.first()
                        val luma = gpu.readLuma(item.frame, wait) ?: break
                        stages.getOrPut("decode.proxy_readback_wait") { TimingStatistics() }.add(gpu.proxyReadbackMs)
                        stages.getOrPut("decode.proxy_transfer") { TimingStatistics() }.add(gpu.proxyTransferMs)
                        while (!planner.trySubmit(luma, item.pts, duration)) {
                            drain(false)
                            Thread.sleep(1)
                        }
                        awaitingAnalysis.removeFirst()
                    }
                }
                val decoded = decode(input, images = images) { image, pts, _ ->
                    if (received == 0) origin = pts else check(pts > latest)
                    latest = pts
                    if (config.optBoolean("paced")) {
                        val target = began + (pts - origin) * 1000
                        while (System.nanoTime() < target) {
                            submitAnalysis(false)
                            drain(false)
                            Thread.sleep(1)
                        }
                    }
                    if (awaitingAnalysis.size >= 2 || pending.size == streaming.maxBufferedFrames) submitAnalysis(true)
                    while (pending.size == streaming.maxBufferedFrames) {
                        drain(false)
                        if (pending.size == streaming.maxBufferedFrames) Thread.sleep(1)
                    }
                    val start = System.nanoTime()
                    val synchronous = config.optBoolean("synchronousCapture", false)
                    val frame = if (synchronous) gpu.capture(image, aw, ah, config.getBoolean("bt709"), config.getBoolean("fullRange"))
                        else gpu.captureAsync(image, aw, ah, config.getBoolean("bt709"), config.getBoolean("fullRange"))
                    stages.getOrPut("decode.capture_proxy") { TimingStatistics() }.add(
                        (System.nanoTime() - start) / 1e6 - if (synchronous) gpu.proxyReadbackMs else 0.0)
                    stages.getOrPut("decode.capture_cpu_submit") { TimingStatistics() }.add(gpu.captureSubmitMs)
                    stages.getOrPut("decode.source_release_wait") { TimingStatistics() }.add(gpu.sourceReleaseWaitMs)
                    val item = Pending(received.toLong(), pts, frame, start)
                    pending.addLast(item)
                    awaitingAnalysis.addLast(item)
                    received++
                    peak = maxOf(peak, pending.size)
                    submitAnalysis(false)
                    drain(false)
                    if (received % 60 == 0) progress("gpu_stream", received)
                }
                submitAnalysis(true)
                val beforeFlush = rendered
                while (!planner.tryFinish()) { drain(false); Thread.sleep(1) }
                while (!planner.finished) { drain(true); Thread.sleep(1) }
                timed(stages, "encode.drain_eos") { sink.finish() }
                check(decoded == received && rendered == received && sink.frames == received && pending.isEmpty()) { "GPU streaming lost frames" }
                report.put("peak_rss_mib", peakRssMiB())
                report.put("retained_texture_bytes", gpu.retainedTextureBytes)
                report.put("readback_backend", gpu.readbackBackend)
                val cpu = planner.cpuProfile()
                report.put("worker_cpu", JSONObject().put("analysis_ms", cpu.analysisMs).put("planning_ms", cpu.planningMs))
                report.put("frames", received).put("width", width).put("height", height)
                    .put("backend", "gles").put("gpu", true).put("device", gpu.device)
                    .put("encoder", encoder.name).put("total_seconds", (System.nanoTime() - began) / 1e9)
                    .put("mode", "realtime").put("gpu_surface", true)
                    .put("native_profile", org.json.JSONArray(planner.profile())).put("stages", timingJson(stages)).put("profiling", profilingJson()).put("frame_events", events).put("encoded_events", encoded)
                    .put("streaming", JSONObject().put("peak_buffered_frames", peak).put("lookahead_seconds", streaming.lookaheadSeconds)
                        .put("first_output_seconds", firstOutputSeconds).put("first_output_after_inputs", firstOutputAfterInputs)
                        .put("frames_before_flush", beforeFlush).put("frames_during_flush", rendered-beforeFlush).put("decode_passes", 1))
            }
        }
    }

    private fun exportGyroStream(directory: File, config: JSONObject) {
        val began = System.nanoTime()
        val width = config.getInt("width")
        val height = config.getInt("height")
        val duration = Math.round(1_000_000 / config.getDouble("fps"))
        val gyro = config.getJSONObject("gyro")
        val (lens, settings, samples) = gyroInput(directory, gyro)
        val streaming = streamSettings(config.getJSONObject("settings"))
        val stages = linkedMapOf<String, TimingStatistics>()
        fun record(name: String, ms: Double) { stages.getOrPut(name) { TimingStatistics() }.add(ms) }
        val readoutHalfS = abs(settings.readoutMs) / 2000
        val bt709 = config.getBoolean("bt709")
        val fullRange = config.getBoolean("fullRange")
        val paced = config.optBoolean("paced")
        StillFlow.GyroStream(width, height, lens, settings, streaming).use { stream ->
            withStreamSurface(directory, config, width, height, streaming.maxBufferedFrames, stream.outputWidth,
                stream.outputHeight, false, began) { gpu, images, sink, encoded, encoder ->
                val pending = java.util.ArrayDeque<Pending>()
                val events = org.json.JSONArray()
                var imuCursor = 0
                var origin = 0L
                var received = 0
                var rendered = 0
                var peak = 0
                var firstOutputSeconds = 0.0
                var firstOutputAfterInputs = 0
                var minZoom = Double.POSITIVE_INFINITY
                var sumZoom = 0.0
                var maxZoom = 0.0

                fun drain(flushing: Boolean) {
                    while (true) {
                        val result = stream.poll()
                        when (result) {
                            StillFlow.StreamResult.Pending -> { check(!flushing); return }
                            StillFlow.StreamResult.End -> { check(flushing); return }
                            is StillFlow.StreamResult.Ready -> {
                                val (info, map) = result.plan.use { it.info to it.gyroMap() }
                                val item = pending.removeFirst()
                                check(info.index == item.index && info.ptsUs == item.pts && info.index == rendered.toLong())
                                minZoom = minOf(minZoom, map.zoomRatio)
                                sumZoom += map.zoomRatio
                                maxZoom = maxOf(maxZoom, map.zoomRatio)
                                val start = System.nanoTime()
                                sink.drain(0)
                                gpu.render(item.frame, map, info.ptsUs - origin)
                                val completed = System.nanoTime()
                                sink.drain(0)
                                record("render.surface", (System.nanoTime() - start) / 1e6)
                                record("render.cpu_submit", gpu.renderSubmitMs)
                                record("render.surface_submit_wait", gpu.surfaceSubmitMs)
                                record("stream.residence", (completed - item.receivedNs) / 1e6)
                                events.put(JSONObject().put("frame", info.index).put("pts_us", info.ptsUs)
                                    .put("duration_us", info.durationUs).put("flushing", flushing)
                                    .put("input_ns", item.receivedNs - began).put("plan_complete_ns", start - began)
                                    .put("render_start_ns", start - began).put("render_complete_ns", completed - began))
                                if (rendered == 0) {
                                    firstOutputSeconds = (completed - began) / 1e9
                                    firstOutputAfterInputs = received
                                }
                                rendered++
                            }
                        }
                    }
                }

                val decoded = decode(File(directory, "input.mp4"), images = images) { image, pts, _ ->
                    if (received == 0) origin = pts
                    val frameT = (pts - origin) / 1e6
                    if (paced) {
                        val target = began + (pts - origin) * 1000
                        while (System.nanoTime() < target) {
                            drain(false)
                            Thread.sleep(1)
                        }
                    }
                    val start = System.nanoTime()
                    val first = imuCursor
                    while (imuCursor < samples.size && samples[imuCursor].timeSeconds + settings.timeOffsetS <= frameT + readoutHalfS) imuCursor++
                    if (first < imuCursor) stream.pushImu(samples.subList(first, imuCursor))
                    if (pending.size >= streaming.maxBufferedFrames) drain(false)
                    check(pending.size < streaming.maxBufferedFrames) { "Gyro stream did not release a texture at capacity" }
                    val frame = gpu.captureTexture(image, bt709, fullRange)
                    record("decode.capture_cpu_submit", gpu.captureSubmitMs)
                    record("decode.source_release_wait", gpu.sourceReleaseWaitMs)
                    while (!stream.tryPushFrame(pts, duration)) {
                        val previous = pending.size
                        drain(false)
                        check(pending.size < previous) { "Gyro stream made no progress under backpressure" }
                    }
                    pending.addLast(Pending(received.toLong(), pts, frame, start))
                    received++
                    peak = maxOf(peak, pending.size)
                    drain(false)
                    if (received % 60 == 0) progress("gyro_stream", received)
                }
                val beforeFlush = rendered
                if (imuCursor < samples.size) stream.pushImu(samples.subList(imuCursor, samples.size))
                stream.finish()
                drain(true)
                timed(stages, "encode.drain_eos") { sink.finish() }
                check(decoded > 0 && decoded == received && rendered == received && sink.frames == received && pending.isEmpty()) {
                    "GPU gyro stream lost frames"
                }
                gyro.put("zoom_ratio", JSONObject().put("min", minZoom).put("mean", sumZoom / rendered).put("max", maxZoom))
                report.put("mode", "realtime_gyro").put("decode_passes", 1).put("frames", rendered)
                    .put("width", width).put("height", height).put("output_width", stream.outputWidth).put("output_height", stream.outputHeight)
                    .put("backend", "gles").put("gpu", true).put("gpu_surface", true).put("device", gpu.device)
                    .put("encoder", encoder.name).put("peak_rss_mib", peakRssMiB()).put("retained_texture_bytes", gpu.retainedTextureBytes)
                    .put("gyro", gyro).put("frame_events", events).put("encoded_events", encoded)
                    .put("native_profile", org.json.JSONArray(stream.profile())).put("stages", timingJson(stages)).put("profiling", profilingJson())
                    .put("streaming", JSONObject().put("peak_buffered_frames", peak).put("lookahead_seconds", streaming.lookaheadSeconds)
                        .put("max_buffered_frames", streaming.maxBufferedFrames).put("first_output_seconds", firstOutputSeconds)
                        .put("first_output_after_inputs", firstOutputAfterInputs).put("frames_before_flush", beforeFlush)
                        .put("frames_during_flush", rendered - beforeFlush).put("decode_passes", 1))
                    .put("total_seconds", (System.nanoTime() - began) / 1e9)
            }
        }
    }

    private fun peakRssMiB() = File("/proc/self/status").readLines().firstOrNull { it.startsWith("VmHWM:") }
        ?.trim()?.split(Regex("\\s+"))?.get(1)?.toDouble()?.div(1024) ?: JSONObject.NULL

    private fun streamSettings(options: JSONObject) = StillFlow.StreamSettings().apply {
        lookaheadSeconds = options.optDouble("lookaheadSeconds", lookaheadSeconds)
        maxBufferedFrames = options.optInt("maxBufferedFrames", maxBufferedFrames)
        adaptiveTracking = options.optBoolean("adaptiveTracking", adaptiveTracking)
    }

    private data class Pending(val index: Long, val pts: Long, val frame: GpuFrames.Frame, val receivedNs: Long)

    private fun withStreamSurface(
        directory: File, config: JSONObject, width: Int, height: Int, capacity: Int,
        outputWidth: Int, outputHeight: Int, computeReadback: Boolean, began: Long,
        process: (GpuFrames, ImageReader, EncoderSink, org.json.JSONArray, MediaCodec) -> Unit,
    ) {
        val format = MediaFormat.createVideoFormat("video/avc", outputWidth, outputHeight).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, config.getInt("bitrate"))
            setFloat(MediaFormat.KEY_FRAME_RATE, config.getDouble("fps").toFloat())
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 2)
            setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
            setInteger(MediaFormat.KEY_COLOR_STANDARD, MediaFormat.COLOR_STANDARD_BT709)
            setInteger(MediaFormat.KEY_COLOR_RANGE, MediaFormat.COLOR_RANGE_LIMITED)
            setInteger(MediaFormat.KEY_COLOR_TRANSFER, MediaFormat.COLOR_TRANSFER_SDR_VIDEO)
        }
        val encoder = HardwareCodecs.encoder(format)
        try {
            val surface = encoder.createInputSurface()
            try {
                val writer = MediaMuxer(File(directory, "output.mp4").path, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
                try {
                    val encoded = org.json.JSONArray()
                    val sink = EncoderSink(encoder, writer) { pts, ns ->
                        encoded.put(JSONObject().put("pts_us", pts).put("output_ns", ns - began))
                    }
                    encoder.start()
                    GpuFrames(width, height, capacity, surface, computeReadback, outputWidth, outputHeight).use { gpu ->
                        ImageReader.newInstance(width, height, ImageFormat.PRIVATE, 2, HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE).use { images ->
                            process(gpu, images, sink, encoded, encoder)
                        }
                    }
                } finally { writer.release() }
            } finally { surface.release() }
        } finally { encoder.release() }
    }

    private class EncoderSink(val codec: MediaCodec, val muxer: MediaMuxer, val onOutput: (Long, Long) -> Unit = { _, _ -> }) {
        private val info = MediaCodec.BufferInfo()
        private var track = -1
        var frames = 0
            private set
        private var eos = false

        fun drain(timeout: Long) {
            var index = codec.dequeueOutputBuffer(info, timeout)
            while (index != MediaCodec.INFO_TRY_AGAIN_LATER) {
                if (index == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    check(track < 0) { "Second encoder format" }
                    track = muxer.addTrack(codec.outputFormat)
                    muxer.start()
                } else if (index >= 0) {
                    try {
                        if (info.size > 0 && info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == 0) {
                            check(track >= 0) { "Missing muxer track" }
                            muxer.writeSampleData(track, checkNotNull(codec.getOutputBuffer(index)), info)
                            onOutput(info.presentationTimeUs, System.nanoTime())
                            frames++
                        }
                        eos = eos || info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                    } finally {
                        codec.releaseOutputBuffer(index, false)
                    }
                }
                index = codec.dequeueOutputBuffer(info, 0)
            }
        }

        fun finish() {
            codec.signalEndOfInputStream()
            finishOutput()
        }

        private fun finishOutput() {
            val deadline = System.nanoTime() + STALL_NS
            while (!eos) {
                drain(1000)
                check(System.nanoTime() < deadline) { "Encoder EOS timeout" }
            }
            muxer.stop()
        }
    }

    companion object {
        private const val STALL_NS = 30_000_000_000L
    }
}
