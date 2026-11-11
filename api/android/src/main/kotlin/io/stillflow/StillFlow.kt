package io.stillflow

import java.nio.ByteBuffer

/** Frame stabilization. Calls are synchronous; use a worker thread and close the session. */
class StillFlow(
    width: Int,
    height: Int,
    fps: Double,
    horizontalScan: Boolean,
    settings: Settings,
) : AutoCloseable {
    enum class PixelFormat(internal val nativeValue: Int, @JvmField val bytesPerPixel: Int) {
        GRAY8(0, 1), BGR8(1, 3), RGBA8(2, 4)
    }
    enum class CropMode { AUTO, FIXED, ADAPTIVE }
    enum class Backend { AUTO, CPU, GLES }
    enum class MotionEstimator { TRANSLATION, OPENCV }

    /** Settings are copied when the session is created. Public fields also support Java callers. */
    class Settings {
        @JvmField var backend = Backend.AUTO
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
        @JvmField val backend = values[0]
        @JvmField val device = values[1]
        @JvmField val fallbackReason = values[2]
        @JvmField val gpu = values[3].toBoolean()
    }

    private var handle = nativeCreate(
        width, height, fps, horizontalScan, settings.backend.ordinal, settings.motionEstimator.ordinal,
        settings.analysisWidth, settings.features, settings.bands, settings.cropMode.ordinal,
        settings.rollingShutter, settings.smoothSeconds, settings.rsSeconds, settings.crop, settings.rsStrength,
    )

    constructor(width: Int, height: Int, fps: Double) : this(width, height, fps, false, Settings())

    /** Timestamps and durations use microseconds. Timestamps must increase. */
    @Synchronized
    fun analyze(frame: Frame, ptsUs: Long, durationUs: Long) {
        requireOpen()
        nativeAnalyze(handle, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, ptsUs, durationUs)
    }

    class RecoveryFrame internal constructor(values: LongArray) {
        @JvmField val index = values[0]
        @JvmField val ptsUs = values[1]
    }

    /** Seals analysis; replay requested frames in order until this returns null. */
    @Synchronized
    fun nextRecoveryFrame(): RecoveryFrame? {
        requireOpen()
        return nativeNextRecoveryFrame(handle)?.let { RecoveryFrame(it) }
    }

    @Synchronized
    fun recover(frame: Frame, ptsUs: Long) {
        requireOpen()
        nativeRecover(handle, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, ptsUs)
    }

    /** Requires all requested recovery frames to have been replayed. */
    @Synchronized
    fun finish(): PlanInfo {
        requireOpen()
        return PlanInfo(nativeFinish(handle))
    }

    @Synchronized
    fun rendererInfo(): RendererInfo {
        requireOpen()
        return RendererInfo(nativeRendererInfo(handle))
    }

    /** Writes packed output into caller-owned storage without changing its position or limit. */
    @Synchronized
    fun render(index: Int, frame: Frame, destination: ByteBuffer): Frame {
        requireOpen()
        val rowBytes = frame.width.toLong() * frame.format.bytesPerPixel
        val size = rowBytes * frame.height
        require(index >= 0 && destination.isDirect && !destination.isReadOnly && rowBytes <= Int.MAX_VALUE && size <= destination.remaining()) {
            "Invalid index or writable direct destination extent"
        }
        val output = destination.slice()
        output.limit(size.toInt())
        nativeRender(handle, index, frame.width, frame.height, frame.format.nativeValue, frame.rowStride, frame.pixels, output)
        return Frame(frame.width, frame.height, frame.format, rowBytes.toInt(), output)
    }

    @Synchronized
    override fun close() {
        if (handle != 0L) {
            nativeDestroy(handle)
            handle = 0
        }
    }

    private fun requireOpen() = check(handle != 0L) { "StillFlow session is closed" }

    companion object {
        init { System.loadLibrary("stillflow_jni") }

        @JvmStatic fun buildDescription(): String = nativeBuildDescription()
        @JvmStatic private external fun nativeDefaults(): DoubleArray
        @JvmStatic private external fun nativeCreate(
            width: Int, height: Int, fps: Double, horizontalScan: Boolean, backend: Int, motionEstimator: Int,
            analysisWidth: Int, features: Int, bands: Int, cropMode: Int, rollingShutter: Boolean,
            smoothSeconds: Double, rsSeconds: Double, crop: Double, rsStrength: Double,
        ): Long
        @JvmStatic private external fun nativeAnalyze(
            handle: Long, width: Int, height: Int, format: Int, rowStride: Int, pixels: ByteBuffer, ptsUs: Long, durationUs: Long,
        )
        @JvmStatic private external fun nativeNextRecoveryFrame(handle: Long): LongArray?
        @JvmStatic private external fun nativeRecover(
            handle: Long, width: Int, height: Int, format: Int, stride: Int, pixels: ByteBuffer, ptsUs: Long,
        )
        @JvmStatic private external fun nativeFinish(handle: Long): LongArray
        @JvmStatic private external fun nativeRendererInfo(handle: Long): Array<String>
        @JvmStatic private external fun nativeRender(
            handle: Long, index: Int, width: Int, height: Int, format: Int, rowStride: Int, pixels: ByteBuffer, destination: ByteBuffer,
        )
        @JvmStatic private external fun nativeDestroy(handle: Long)
        @JvmStatic private external fun nativeBuildDescription(): String
    }
}
