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
import io.stillflow.HardwareCodecs
import io.stillflow.StillFlow
import java.io.File
import java.nio.ByteBuffer
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
            export(directory, config)
            report.put("status", "passed").put("run", runId)
                .put("sdk", StillFlow.buildDescription()).put("model", Build.MODEL)
                .put("manufacturer", Build.MANUFACTURER).put("api", Build.VERSION.SDK_INT)
            File(directory, "result.json").writeText(report.toString(2))
            finish(-1, Bundle().apply { putString("stream", "STILLFLOW_COMPLETE $runId\n") })
        } catch (error: Throwable) {
            finish(1, Bundle().apply { putString("stream", "STILLFLOW_FAILED $runId\n${error.stackTraceToString()}") })
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

    private fun decode(input: File, needed: () -> Boolean = { true }, consume: (Image, Long, MediaFormat) -> Unit): Int {
        val ex = extractor(input)
        var decoder: MediaCodec? = null
        try {
            val format = ex.getTrackFormat(ex.sampleTrackIndex)
            format.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            val codec = HardwareCodecs.decoder(format, null)
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
                    try {
                        if (info.size > 0) {
                            checkNotNull(codec.getOutputImage(index)) { "Missing decoded Image" }.use {
                                consume(it, info.presentationTimeUs, outputFormat)
                            }
                            count++
                        }
                        done = info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                        lastProgress = System.nanoTime()
                    } finally {
                        codec.releaseOutputBuffer(index, false)
                    }
                }
            }
            return count
        } finally {
            decoder?.release()
            ex.release()
        }
    }

    private fun export(directory: File, config: JSONObject) {
        val input = File(directory, "input.mp4")
        val output = File(directory, "output.mp4")
        val width = config.getInt("width")
        val height = config.getInt("height")
        val fps = config.getDouble("fps")
        require(width in 32..4096 && height in 32..4096 && width % 2 == 0 && height % 2 == 0 && fps.isFinite() && fps > 0)
        val options = config.getJSONObject("settings")
        val settings = StillFlow.Settings().apply {
            backend = StillFlow.Backend.valueOf(options.optString("backend", backend.name))
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
        StillFlow(width, height, fps, config.getBoolean("horizontalScan"), settings).use { session ->
            val frames = decode(input) { image, pts, _ ->
                session.analyze(AndroidFrames.luma(image), pts, durationUs)
                timestamps.add(pts)
                if (timestamps.size % 60 == 0) progress("analyze", timestamps.size)
            }
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
            check(frames > 0 && session.finish().frameCount == frames.toLong()) { "Analysis frame count mismatch" }
            report.put("analysis_seconds", (System.nanoTime() - began) / 1e9)
            val fmt = MediaFormat.createVideoFormat("video/avc", width, height).apply {
                setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
                setInteger(MediaFormat.KEY_BIT_RATE, config.getInt("bitrate"))
                setFloat(MediaFormat.KEY_FRAME_RATE, fps.toFloat())
                setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
                setInteger(MediaFormat.KEY_COLOR_STANDARD, MediaFormat.COLOR_STANDARD_BT709)
                setInteger(MediaFormat.KEY_COLOR_RANGE, MediaFormat.COLOR_RANGE_LIMITED)
                setInteger(MediaFormat.KEY_COLOR_TRANSFER, MediaFormat.COLOR_TRANSFER_SDR_VIDEO)
            }
            val encoder = HardwareCodecs.encoder(fmt)
            var muxer: MediaMuxer? = null
            try {
                check(encoder.codecInfo.isHardwareAccelerated) { "Software encoder" }
                report.put("encoder", encoder.name)
                val writer = MediaMuxer(output.path, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
                muxer = writer
                val sink = EncoderSink(encoder, writer)
                encoder.start()
                val inputPixels = ByteBuffer.allocateDirect(width * height * 4)
                val outputPixels = ByteBuffer.allocateDirect(width * height * 4)
                val frame = StillFlow.Frame(width, height, StillFlow.PixelFormat.RGBA8, width * 4, inputPixels)
                var index = 0
                val rendered = decode(input) { image, pts, _ ->
                    check(index < timestamps.size && pts == timestamps[index]) { "Decode timestamp mismatch" }
                    // The host validates the source matrix and range before deployment.
                    val bt709 = config.getBoolean("bt709")
                    val full = config.getBoolean("fullRange")
                    pixels(true, image, inputPixels, width, height, bt709, full)
                    session.render(index, frame, outputPixels)
                    sink.submit(outputPixels, width, height, pts - timestamps[0])
                    index++
                    if (index % 60 == 0) progress("render", index)
                }
                check(rendered == frames) { "Render frame count mismatch" }
                sink.finish(timestamps.last() - timestamps[0] + durationUs)
                check(sink.frames == frames) { "Encode frame count mismatch" }
                val renderer = session.rendererInfo()
                report.put("frames", frames).put("width", width).put("height", height)
                    .put("backend", renderer.backend).put("gpu", renderer.gpu).put("device", renderer.device)
                    .put("fallback_reason", renderer.fallbackReason)
                    .put("total_seconds", (System.nanoTime() - began) / 1e9)
            } finally {
                encoder.release()
                muxer?.release()
            }
        }
    }

    private class EncoderSink(val codec: MediaCodec, val muxer: MediaMuxer) {
        private val info = MediaCodec.BufferInfo()
        private var track = -1
        var frames = 0
            private set
        private var eos = false

        private fun drain(timeout: Long) {
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

        private fun nextInput(): Int {
            val deadline = System.nanoTime() + STALL_NS
            while (true) {
                drain(0)
                val index = codec.dequeueInputBuffer(1000)
                if (index >= 0) return index
                check(System.nanoTime() < deadline) { "Encoder stalled" }
            }
        }

        fun submit(rgba: ByteBuffer, width: Int, height: Int, pts: Long) {
            val index = nextInput()
            checkNotNull(codec.getInputImage(index)) { "Missing encoder Image" }.use {
                pixels(false, it, rgba, width, height, true, false)
            }
            codec.queueInputBuffer(index, 0, width * height * 3 / 2, pts, 0)
            drain(0)
        }

        fun finish(pts: Long) {
            codec.queueInputBuffer(nextInput(), 0, 0, pts, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
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
        init { System.loadLibrary("runner_pixels") }

        private fun pixels(decode: Boolean, image: Image, rgba: ByteBuffer, width: Int, height: Int, bt709: Boolean, full: Boolean) {
            val crop = image.cropRect
            require(crop.width() == width && crop.height() == height && crop.left % 2 == 0 && crop.top % 2 == 0) { "Image crop" }
            val p = image.planes
            val b = Array(3) { index ->
                val div = if (index == 0) 1 else 2
                p[index].buffer.duplicate().apply {
                    position(position() + crop.top / div * p[index].rowStride + crop.left / div * p[index].pixelStride)
                }.slice()
            }
            convert(decode, width, height, rgba, b[0], p[0].rowStride, p[0].pixelStride,
                b[1], p[1].rowStride, p[1].pixelStride, b[2], p[2].rowStride, p[2].pixelStride, bt709, full)
        }

        @JvmStatic private external fun convert(
            toRgba: Boolean, width: Int, height: Int, rgba: ByteBuffer,
            y: ByteBuffer, yr: Int, yp: Int, u: ByteBuffer, ur: Int, up: Int, v: ByteBuffer, vr: Int, vp: Int,
            bt709: Boolean, full: Boolean,
        )
    }
}
