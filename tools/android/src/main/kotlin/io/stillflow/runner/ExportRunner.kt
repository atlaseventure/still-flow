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
            if (config.optBoolean("realtime")) exportStream(directory, config) else exportOffline(directory, config)
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

    private fun exportOffline(directory: File, config: JSONObject) {
        val input = File(directory, "input.mp4")
        val output = File(directory, "output.mp4")
        val width = config.getInt("width")
        val height = config.getInt("height")
        val fps = config.getDouble("fps")
        require(width in 32..4096 && height in 32..4096 && width % 2 == 0 && height % 2 == 0 && fps.isFinite() && fps > 0)
        val options = config.getJSONObject("settings")
        StillFlow.configureThreads(options.optInt("visionThreads", 2))
        val settings = StillFlow.Settings().apply {
            backend = StillFlow.Backend.GLES
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
            put("backend", settings.backend.name)
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
        StillFlow(width, height, fps, config.getBoolean("horizontalScan"), settings).use { session ->
            val analysisStart = System.nanoTime()
            val frames = decode(input) { image, pts, _ ->
                timed(stages, "analysis.analyze") { session.analyze(AndroidFrames.luma(image), pts, durationUs) }
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
            check(frames > 0 && timed(stages, "analysis.plan") { session.finish() }.frameCount == frames.toLong()) { "Analysis frame count mismatch" }
            report.put("analysis_seconds", (System.nanoTime() - began) / 1e9)
            report.put("mode", "offline").put("decode_passes", if (recoveryFrames > 0) 3 else 2)
            renderOfflineSurface(input, output, config, session, timestamps, stages)
            report.put("stages", timingJson(stages)).put("profiling", profilingJson())
                .put("total_seconds", (System.nanoTime() - began) / 1e9)
        }
    }

    private fun renderOfflineSurface(
        input: File, output: File, config: JSONObject, session: StillFlow,
        timestamps: List<Long>, stages: MutableMap<String, TimingStatistics>,
    ) {
        val width = config.getInt("width")
        val height = config.getInt("height")
        val format = MediaFormat.createVideoFormat("video/avc", width, height).apply {
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
                    GpuFrames(width, height, 2, surface).use { gpu ->
                        ImageReader.newInstance(width, height, ImageFormat.PRIVATE, 2, HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE).use { images ->
                            var index = 0
                            val renderStart = System.nanoTime()
                            val rendered = decode(input, images = images) { image, pts, _ ->
                                check(index < timestamps.size && pts == timestamps[index]) { "Decode timestamp mismatch" }
                                val map = timed(stages, "render.map") { session.frameMap(index) }
                                timed(stages, "render.surface") {
                                    sink.drain(0)
                                    gpu.render(image, map, pts - timestamps[0], config.getBoolean("bt709"), config.getBoolean("fullRange"))
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
        val output = File(directory, "output.mp4")
        val width = config.getInt("width")
        val height = config.getInt("height")
        val fps = config.getDouble("fps")
        require(width in 32..4096 && height in 32..4096 && width % 2 == 0 && height % 2 == 0 && fps.isFinite() && fps > 0)
        val options = config.getJSONObject("settings")
        StillFlow.configureThreads(options.optInt("visionThreads", 2))
        val settings = StillFlow.Settings().apply {
            lookaheadSeconds = options.optDouble("lookaheadSeconds", lookaheadSeconds)
            backend = StillFlow.Backend.GLES
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
            put("backend", settings.backend.name)
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
        settings.realtime = true
        val scale = minOf(1.0, settings.analysisWidth.toDouble() / width)
        val aw = maxOf(32, Math.round(width * scale).toInt())
        val ah = maxOf(32, Math.round(height * scale).toInt())
        val duration = Math.round(1_000_000 / fps)
        val format = MediaFormat.createVideoFormat("video/avc", width, height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, config.getInt("bitrate"))
            setFloat(MediaFormat.KEY_FRAME_RATE, fps.toFloat())
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 2)
            setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
            setInteger(MediaFormat.KEY_COLOR_STANDARD, MediaFormat.COLOR_STANDARD_BT709)
            setInteger(MediaFormat.KEY_COLOR_RANGE, MediaFormat.COLOR_RANGE_LIMITED)
            setInteger(MediaFormat.KEY_COLOR_TRANSFER, MediaFormat.COLOR_TRANSFER_SDR_VIDEO)
        }
        val began = System.nanoTime()
        val encoder = HardwareCodecs.encoder(format)
        try {
            val surface = encoder.createInputSurface()
            try {
                val writer = MediaMuxer(File(directory, "output.mp4").path, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
                try {
                    val sink = EncoderSink(encoder, writer)
                    encoder.start()
                    GpuFrames(width, height, settings.maxBufferedFrames, surface).use { gpu ->
                        ImageReader.newInstance(width, height, ImageFormat.PRIVATE, 2, HardwareBuffer.USAGE_GPU_SAMPLED_IMAGE).use { images ->
                            StreamPlanner(width, height, fps, config.getBoolean("horizontalScan"), settings).use { planner ->
                                data class Pending(val index: Long, val pts: Long, val frame: GpuFrames.Frame, val receivedNs: Long)
                                val pending = java.util.ArrayDeque<Pending>()
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
                                        sink.drain(0)
                                        mark("render.surface", start)
                                        events.put(JSONObject().put("frame", ready.index).put("pts_us", ready.ptsUs)
                                            .put("duration_us", ready.durationUs).put("flushing", flushing))
                                        stages.getOrPut("stream.residence") { TimingStatistics() }.add((System.nanoTime()-item.receivedNs)/1e6)
                                        if (rendered == 0) {
                                            firstOutputSeconds = (System.nanoTime()-began)/1e9
                                            firstOutputAfterInputs = received
                                        }
                                        rendered++
                                    }
                                }
                                val decoded = decode(input, images = images) { image, pts, _ ->
                                    if (received == 0) origin = pts else check(pts > latest)
                                    latest = pts
                                    val start = System.nanoTime()
                                    val frame = gpu.capture(image, aw, ah, config.getBoolean("bt709"), config.getBoolean("fullRange"))
                                    mark("decode.capture_proxy", start)
                                    pending.addLast(Pending(received.toLong(), pts, frame, start))
                                    while (!planner.trySubmit(frame.luma, pts, duration)) {
                                        drain(false)
                                        Thread.sleep(1)
                                    }
                                    received++
                                    peak = maxOf(peak, pending.size)
                                    drain(false)
                                    if (received % 60 == 0) progress("gpu_stream", received)
                                }
                                val beforeFlush = rendered
                                while (!planner.tryFinish()) { drain(false); Thread.sleep(1) }
                                while (!planner.finished) { drain(true); Thread.sleep(1) }
                                sink.finish()
                                check(decoded == received && rendered == received && sink.frames == received && pending.isEmpty()) { "GPU streaming lost frames" }
                                val peakRss = File("/proc/self/status").readLines().firstOrNull { it.startsWith("VmHWM:") }
                                    ?.trim()?.split(Regex("\\s+"))?.get(1)?.toDouble()?.div(1024)
                                report.put("peak_rss_mib", peakRss ?: JSONObject.NULL)
                                report.put("frames", received).put("width", width).put("height", height)
                                    .put("backend", "gles").put("gpu", true).put("device", gpu.device)
                                    .put("encoder", encoder.name).put("total_seconds", (System.nanoTime() - began) / 1e9)
                                    .put("mode", "realtime").put("gpu_surface", true)
                                    .put("native_profile", org.json.JSONArray(planner.profile())).put("stages", timingJson(stages)).put("profiling", profilingJson()).put("frame_events", events)
                                    .put("streaming", JSONObject().put("peak_buffered_frames", peak).put("lookahead_seconds", settings.lookaheadSeconds)
                                        .put("first_output_seconds", firstOutputSeconds).put("first_output_after_inputs", firstOutputAfterInputs)
                                        .put("frames_before_flush", beforeFlush).put("frames_during_flush", rendered-beforeFlush).put("decode_passes", 1))
                            }
                        }
                    }
                } finally { writer.release() }
            } finally { surface.release() }
        } finally { encoder.release() }
    }

    private class EncoderSink(val codec: MediaCodec, val muxer: MediaMuxer) {
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
