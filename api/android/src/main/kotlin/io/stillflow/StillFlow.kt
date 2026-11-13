package io.stillflow

import java.nio.ByteBuffer

/** Frame stabilization. Processing calls are synchronous; use a worker thread. */
object StillFlow {
    init { System.loadLibrary("stillflow_jni") }

    class Video @JvmOverloads constructor(
        @JvmField val width: Int,
        @JvmField val height: Int,
        @JvmField val fps: Double,
        @JvmField val horizontalScan: Boolean = false,
    ) {
        init { require(width > 0 && height > 0 && fps.isFinite() && fps > 0) }
    }

    enum class PixelFormat(internal val nativeValue: Int, @JvmField val bytesPerPixel: Int) {
        GRAY8(0, 1), BGR8(1, 3), RGBA8(2, 4)
    }
    enum class CropMode { AUTO, FIXED, ADAPTIVE }
    enum class Backend { AUTO, CPU, GLES }
    enum class MotionEstimator { TRANSLATION, OPENCV }

    /** Settings are copied when an analyzer or stream is created. Public fields also support Java callers. */
    class Settings {
        @JvmField var motionEstimator = MotionEstimator.OPENCV
        @JvmField var analysisWidth: Int
        @JvmField var features: Int
        @JvmField var bands: Int
        @JvmField var cropMode: CropMode
        @JvmField var rollingShutter: Boolean
        @JvmField var smoothSeconds: Double
        @JvmField var rsSeconds: Double
        @JvmField var crop: Double
        @JvmField var rsStrength: Double

        init {
            val values = nativeDefaults()
            analysisWidth = values[0].toInt()
            features = values[1].toInt()
            bands = values[2].toInt()
            cropMode = CropMode.entries[values[3].toInt()]
            rollingShutter = values[4] != 0.0
            smoothSeconds = values[5]
            rsSeconds = values[6]
            crop = values[7]
            rsStrength = values[8]
        }
    }

    /** Captures the remaining direct-buffer range. Keep its storage unchanged during each call. */
    class Frame(
        @JvmField val width: Int,
        @JvmField val height: Int,
        @JvmField val format: PixelFormat,
        @JvmField val rowStride: Int,
        pixels: ByteBuffer,
    ) {
        private val storage: ByteBuffer

        init {
            val rowBytes = width.toLong() * format.bytesPerPixel
            val required = (height.toLong() - 1) * rowStride + rowBytes
            require(width > 0 && height > 0 && rowStride >= rowBytes && pixels.isDirect && required <= pixels.remaining()) {
                "Invalid dimensions, stride or direct buffer extent"
            }
            storage = pixels.slice().asReadOnlyBuffer()
        }

        /** A read-only view; changing its position does not change the captured range. */
        val pixels: ByteBuffer get() = storage.asReadOnlyBuffer()
    }

    class PlanInfo internal constructor(values: LongArray) {
        @JvmField val frameCount = values[0]
        @JvmField val analysisWidth = values[1].toInt()
        @JvmField val analysisHeight = values[2].toInt()
        @JvmField val originUs = values[3]
    }

    class RendererInfo internal constructor(values: Array<String>) {
        @JvmField val backend = Backend.valueOf(values[0].uppercase(java.util.Locale.ROOT))
        @JvmField val device = values[1]
        @JvmField val fallbackReason = values[2]
        @JvmField val gpu = values[3].toBoolean()
    }


    class StreamSettings {
        private val defaults = nativeStreamDefaults()
        @JvmField var lookaheadSeconds = defaults[0]
        @JvmField var maxBufferedFrames = defaults[1].toInt()
        @JvmField var adaptiveTracking = defaults[2] != 0.0
    }

    /** Immutable output-pixel mapping, independent of native plan lifetime. */
    class WarpMap private constructor(
        @JvmField val width: Int,
        @JvmField val height: Int,
        @get:JvmSynthetic internal val packed: FloatArray,
    ) {
        companion object {
            @JvmSynthetic internal fun own(width: Int, height: Int, packed: FloatArray): WarpMap = WarpMap(width, height, packed)
        }
        @JvmField val columns = packed[0].toInt()
        @JvmField val rows = packed[1].toInt()
        @JvmField val originX = packed[2]
        @JvmField val originY = packed[3]
        @JvmField val stepX = packed[4]
        @JvmField val stepY = packed[5]
        @JvmField val gridX = packed[6]
        @JvmField val gridY = packed[7]
        fun offsetX(column: Int, row: Int): Float = packed[offset(column, row)]
        fun offsetY(column: Int, row: Int): Float = packed[offset(column, row) + 1]
        private fun offset(column: Int, row: Int): Int {
            require(column in 0 until columns && row in 0 until rows)
            return 8 + 2 * (row * columns + column)
        }
    }

    class FrameInfo internal constructor(values: LongArray) {
        @JvmField val index = values[0]
        @JvmField val ptsUs = values[1]
        @JvmField val durationUs = values[2]
        @JvmField val lookaheadSeconds = Double.fromBits(values[3])
    }

    class RecoveryFrame internal constructor(values: LongArray) {
        @JvmField val index = values[0]
        @JvmField val ptsUs = values[1]
    }

    private class Resource(private var handle: Long, private val kind: Int) : AutoCloseable {
        @Synchronized fun <T> access(work: (Long) -> T): T {
            check(handle != 0L) { "StillFlow resource is closed" }
            return work(handle)
        }
        @Synchronized override fun close() {
            if (handle != 0L) {
                nativeDestroy(handle, kind)
                handle = 0
            }
        }
    }

    class Analyzer @JvmOverloads constructor(video: Video, settings: Settings = Settings()) : AutoCloseable {
        private val resource = Resource(create(video, settings, null), 0)
        /** Timestamps and durations use microseconds. Timestamps must increase. */
        fun push(frame: Frame, ptsUs: Long, durationUs: Long) = resource.access {
            nativeAnalyze(it, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, ptsUs, durationUs)
        }
        /** Seals analysis; replay requested frames in order until null. */
        fun nextRecoveryFrame(): RecoveryFrame? = resource.access { nativeNextRecoveryFrame(it)?.let(::RecoveryFrame) }
        fun recover(frame: Frame, ptsUs: Long) = resource.access {
            nativeRecover(it, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, ptsUs)
        }
        /** Transfers an independent plan after all recovery requests have been satisfied. */
        fun finish(): Plan = resource.access { Plan.own(nativeFinish(it)) }
        override fun close() = resource.close()
    }

    class Plan private constructor(handle: Long) : AutoCloseable {
        companion object {
            @JvmSynthetic internal fun own(handle: Long): Plan = Plan(handle)
        }
        private val resource = Resource(handle, 1)
        val info: PlanInfo get() = resource.access { PlanInfo(nativePlanInfo(it)) }
        /** The returned frame plan owns its resources; close it after use. */
        fun frame(index: Long): FramePlan = resource.access {
            require(index >= 0) { "Invalid frame index" }
            FramePlan.own(nativePlanFrame(it, index))
        }
        override fun close() = resource.close()
    }

    class FramePlan private constructor(handle: Long) : AutoCloseable {
        companion object {
            @JvmSynthetic internal fun own(handle: Long): FramePlan = FramePlan(handle)
        }
        private val resource = Resource(handle, 2)
        val info: FrameInfo get() = resource.access { FrameInfo(nativeFrameInfo(it)) }
        fun map(width: Int, height: Int): WarpMap = resource.access {
            require(width > 0 && height > 0)
            WarpMap.own(width, height, nativeFrameMap(it, width, height))
        }
        @JvmSynthetic internal fun <T> access(work: (Long) -> T): T = resource.access(work)
        override fun close() = resource.close()
    }

    sealed class StreamResult {
        class Ready internal constructor(@JvmField val plan: FramePlan) : StreamResult()
        object Pending : StreamResult()
        object End : StreamResult()
    }

    class Stream @JvmOverloads constructor(
        video: Video, settings: Settings = Settings(), streaming: StreamSettings = StreamSettings(),
    ) : AutoCloseable {
        private val resource = Resource(create(video, settings, streaming), 3)
        /** False applies backpressure and consumes nothing. Drain and retry or drop this input. */
        fun tryPush(frame: Frame, ptsUs: Long, durationUs: Long): Boolean = resource.access {
            nativeStreamPush(it, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, ptsUs, durationUs)
        }
        /** Ready plans are independently owned. Retain full input frames by their plan index. */
        fun poll(): StreamResult = resource.access {
            val values = nativeStreamReady(it)
            when (values[0]) {
                0L -> StreamResult.Ready(FramePlan.own(values[1]))
                1L -> StreamResult.Pending
                2L -> StreamResult.End
                else -> error("Unexpected native stream status")
            }
        }
        /** Seals input. Continue polling until End; repeated finish calls are harmless. */
        fun finish() = resource.access { nativeStreamFinish(it) }
        /** Bounded native stage aggregates in JSON. */
        fun profile(): String = resource.access { nativeProfile(it) }
        override fun close() = resource.close()
    }

    class Renderer @JvmOverloads constructor(backend: Backend = Backend.AUTO) : AutoCloseable {
        private val resource = Resource(nativeRendererCreate(backend.ordinal), 4)
        val info: RendererInfo get() = resource.access { RendererInfo(nativeRendererInfo(it)) }
        /** Writes packed output into caller-owned storage, preserving its position and limit. */
        fun render(plan: FramePlan, frame: Frame, destination: ByteBuffer): Frame = resource.access { renderer ->
            val rowBytes = frame.width.toLong() * frame.format.bytesPerPixel
            val size = rowBytes * frame.height
            require(destination.isDirect && !destination.isReadOnly && rowBytes <= Int.MAX_VALUE && size <= destination.remaining()) {
                "Invalid writable direct destination extent"
            }
            val output = destination.slice().apply { limit(size.toInt()) }
            plan.access {
                nativeRender(renderer, it, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, output)
            }
            Frame(frame.width, frame.height, frame.format, rowBytes.toInt(), output)
        }
        override fun close() = resource.close()
    }

    private fun create(video: Video, settings: Settings, stream: StreamSettings?): Long = nativeCreate(
        video.width, video.height, video.fps, video.horizontalScan, settings.motionEstimator.ordinal,
        settings.analysisWidth, settings.features, settings.bands, settings.cropMode.ordinal,
        settings.rollingShutter, settings.smoothSeconds, settings.rsSeconds, settings.crop, settings.rsStrength,
        stream != null, stream?.lookaheadSeconds ?: 0.0, stream?.maxBufferedFrames ?: 0, stream?.adaptiveTracking ?: false,
    )

    /** Configure process-wide OpenCV threads before starting analysis workers. */
    @JvmStatic fun configureThreads(threads: Int) = nativeThreads(threads)
    @JvmStatic fun mobileSettings(): Settings = Settings().apply { analysisWidth = 640; features = 600 }
    @JvmStatic fun buildDescription(): String = nativeBuildDescription()
    @JvmStatic private external fun nativeThreads(threads: Int)
    @JvmStatic private external fun nativeProfile(handle: Long): String
    @JvmStatic private external fun nativeDefaults(): DoubleArray
    @JvmStatic private external fun nativeStreamDefaults(): DoubleArray
    @JvmStatic private external fun nativeCreate(
        width: Int, height: Int, fps: Double, horizontalScan: Boolean, motionEstimator: Int,
        analysisWidth: Int, features: Int, bands: Int, cropMode: Int, rollingShutter: Boolean,
        smoothSeconds: Double, rsSeconds: Double, crop: Double, rsStrength: Double,
        realtime: Boolean, lookaheadSeconds: Double, maxBufferedFrames: Int, adaptiveTracking: Boolean,
    ): Long
    @JvmStatic private external fun nativeStreamPush(
        handle: Long, width: Int, height: Int, format: Int, stride: Int, pixels: ByteBuffer, ptsUs: Long, durationUs: Long,
    ): Boolean
    @JvmStatic private external fun nativeStreamReady(handle: Long): LongArray
    @JvmStatic private external fun nativeStreamFinish(handle: Long)
    @JvmStatic private external fun nativeAnalyze(
        handle: Long, width: Int, height: Int, format: Int, rowStride: Int, pixels: ByteBuffer, ptsUs: Long, durationUs: Long,
    )
    @JvmStatic private external fun nativeNextRecoveryFrame(handle: Long): LongArray?
    @JvmStatic private external fun nativeRecover(
        handle: Long, width: Int, height: Int, format: Int, stride: Int, pixels: ByteBuffer, ptsUs: Long,
    )
    @JvmStatic private external fun nativeFinish(handle: Long): Long
    @JvmStatic private external fun nativePlanInfo(handle: Long): LongArray
    @JvmStatic private external fun nativePlanFrame(handle: Long, index: Long): Long
    @JvmStatic private external fun nativeFrameInfo(handle: Long): LongArray
    @JvmStatic private external fun nativeFrameMap(handle: Long, width: Int, height: Int): FloatArray
    @JvmStatic private external fun nativeRendererCreate(backend: Int): Long
    @JvmStatic private external fun nativeRendererInfo(handle: Long): Array<String>
    @JvmStatic private external fun nativeRender(
        handle: Long, plan: Long, width: Int, height: Int, format: Int, rowStride: Int, pixels: ByteBuffer, destination: ByteBuffer,
    )
    @JvmStatic private external fun nativeDestroy(handle: Long, kind: Int)
    @JvmStatic private external fun nativeBuildDescription(): String
}
