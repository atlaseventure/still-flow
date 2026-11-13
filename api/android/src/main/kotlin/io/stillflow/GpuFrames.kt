package io.stillflow

import androidx.annotation.RequiresApi
import android.graphics.Rect
import android.hardware.HardwareBuffer
import android.media.Image
import android.opengl.EGL14
import android.opengl.EGLExt
import android.opengl.EGLContext
import android.opengl.EGLSurface
import android.opengl.GLES11Ext
import android.opengl.GLES30
import android.opengl.GLES31
import android.view.Surface
import java.nio.ByteBuffer
import java.util.concurrent.ConcurrentLinkedQueue

/** Thread-confined GLES context. Frames own textures independently of decoder buffers. */
@RequiresApi(29)
class GpuFrames @JvmOverloads constructor(
    private val width: Int,
    private val height: Int,
    private val capacity: Int,
    encoderSurface: Surface,
    preferComputeReadback: Boolean = true,
) : AutoCloseable {
    internal class Readback(val id: Int) {
        var bytes = 0
        var bufferBytes = 0
        var fence = 0L
        var texture = 0
        var textureWidth = 0
        var textureHeight = 0
        var compute = false
        var pixel = false
    }
    internal data class Texture(val y: Int, val uv: Int)
    class Frame internal constructor(
        internal val owner: GpuFrames, internal val texture: Texture,
        internal val bt709: Boolean, internal val fullRange: Boolean,
        internal val analysisWidth: Int, internal val analysisHeight: Int,
        internal var transfer: Readback?,
    ) {
        internal var rendered = false
        internal var buffer: ByteBuffer? = null
        internal var analysis: StillFlow.Frame? = null
        val luma: StillFlow.Frame get() = analysis ?: checkNotNull(owner.readLuma(this, true))
    }
    private val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
    private val version = IntArray(2)
    private val configs = arrayOfNulls<android.opengl.EGLConfig>(1)
    private val count = IntArray(1)
    private var context: EGLContext = EGL14.EGL_NO_CONTEXT
    private var window: EGLSurface = EGL14.EGL_NO_SURFACE
    private val fbo = IntArray(1)
    private val external = IntArray(1)
    private val horizontal = IntArray(1)
    private val free = ConcurrentLinkedQueue<Texture>()
    private val proxies = ConcurrentLinkedQueue<ByteBuffer>()
    private val owned = mutableListOf<Texture>()
    private var copyProgram = 0
    private var proxyProgram = 0
    private var horizontalProgram = 0
    private var verticalProgram = 0
    private var directCompute = 0
    private var verticalCompute = 0
    private var computeCapture = false
    private var pixelCapture = false
    private var warpProgram = 0
    private var externalWarpProgram = 0
    private var proxyWidth = 0
    private var proxyHeight = 0
    private val readbacks = mutableListOf<Readback>()
    private val availableReadbacks = java.util.ArrayDeque<Readback>()
    private val ownerThread = Thread.currentThread()
    private val previousDisplay = EGL14.eglGetCurrentDisplay()
    private val previousContext = EGL14.eglGetCurrentContext()
    private val previousDraw = EGL14.eglGetCurrentSurface(EGL14.EGL_DRAW)
    private val previousRead = EGL14.eglGetCurrentSurface(EGL14.EGL_READ)
    private var closed = false
    var captureSubmitMs = 0.0
        private set
    var proxyReadbackMs = 0.0
        private set
    var sourceReleaseWaitMs = 0.0
        private set
    var proxyTransferMs = 0.0
        private set
    var renderSubmitMs = 0.0
        private set
    var surfaceSubmitMs = 0.0
        private set
    val retainedTextureBytes: Long get() = owned.size.toLong() * (width.toLong() * height + ((width + 1L) / 2) * ((height + 1L) / 2) * 2)
    val readbackBackend: String get() = if (computeCapture) "compute_buffer" else if (pixelCapture) "pixel_buffer" else "texture_readback"
    var device: String = ""
        private set

    init {
        try {
        require(width > 0 && height > 0 && capacity >= 2)
        check(EGL14.eglInitialize(display, version, 0, version, 1))
        val attributes = intArrayOf(EGL14.EGL_RENDERABLE_TYPE, 0x40, EGL14.EGL_SURFACE_TYPE, EGL14.EGL_WINDOW_BIT,
            EGL14.EGL_RED_SIZE, 8, EGL14.EGL_GREEN_SIZE, 8, EGL14.EGL_BLUE_SIZE, 8, EGL14.EGL_ALPHA_SIZE, 8,
            0x3142, 1, EGL14.EGL_NONE)
        check(EGL14.eglChooseConfig(display, attributes, 0, configs, 0, 1, count, 0) && count[0] > 0)
        val config = checkNotNull(configs[0])
        context = EGL14.eglCreateContext(display, config, EGL14.EGL_NO_CONTEXT,
            intArrayOf(EGL14.EGL_CONTEXT_CLIENT_VERSION, 3, EGL14.EGL_NONE), 0)
        check(context != EGL14.EGL_NO_CONTEXT)
        window = EGL14.eglCreateWindowSurface(display, config, encoderSurface, intArrayOf(EGL14.EGL_NONE), 0)
        check(window != EGL14.EGL_NO_SURFACE)
        check(EGL14.eglMakeCurrent(display, window, window, context))
        check(GLES30.glGetString(GLES30.GL_EXTENSIONS)?.split(' ')?.contains("GL_EXT_YUV_target") == true) {
            "GPU source-luma analysis requires GL_EXT_YUV_target"
        }
        device = GLES30.glGetString(GLES30.GL_RENDERER) ?: "GLES"
        GLES30.glGenFramebuffers(1, fbo, 0)
        GLES30.glGenTextures(1, external, 0)
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, external[0])
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_NEAREST)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_NEAREST)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
        copyProgram = program(YUV_HEADER + COPY)
        proxyProgram = program(PROXY_HEADER + PROXY)
        horizontalProgram = program(PROXY_HEADER.replace("out vec4 color;", "out uvec4 color;") + HORIZONTAL)
        verticalProgram = program(VERTICAL)
        val minor = IntArray(1)
        GLES30.glGetIntegerv(GLES30.GL_MINOR_VERSION, minor, 0)
        if (preferComputeReadback && minor[0] >= 1) {
            directCompute = compute(COMPUTE_HEADER + AREA_LUMA + COMPUTE_DIRECT)
            verticalCompute = compute(COMPUTE_HEADER.replace("sampler2D", "usampler2D") + VERTICAL_LUMA + COMPUTE_VERTICAL)
        }
        warpProgram = program(WARP_HEADER + PLANAR_SOURCE + WARP)
        externalWarpProgram = program(YUV_HEADER.replace("out vec4 color;", "") + WARP_HEADER.substringAfter("precision highp float;") + EXTERNAL_SOURCE + WARP)
        } catch (error: Throwable) {
            close()
            throw error
        }
    }

    /** Captures exactly one acquired Image. Only the reduced source-Y proxy is read by the CPU. */
    fun capture(image: Image, analysisWidth: Int, analysisHeight: Int, bt709: Boolean, fullRange: Boolean): Frame {
        val frame = captureAsync(image, analysisWidth, analysisHeight, bt709, fullRange)
        readLuma(frame, true)
        return frame
    }

    /** Releases source reads before returning; finish at most three pending transfers with readLuma(). */
    fun captureAsync(image: Image, analysisWidth: Int, analysisHeight: Int, bt709: Boolean, fullRange: Boolean): Frame {
        checkThread()
        require(analysisWidth in 1..width && analysisHeight in 1..height)
        val began = System.nanoTime()
        val transfer = availableReadbacks.poll() ?: run {
            check(readbacks.size < minOf(3, capacity)) { "Drain pending luma transfers before capturing another frame" }
            val id = IntArray(1)
            GLES30.glGenBuffers(1, id, 0)
            Readback(id[0]).also { readbacks.add(it) }
        }
        val texture = free.poll() ?: run {
            check(owned.size < capacity) { "GPU frame cache is full" }
            Texture(allocate(width, height, GLES30.GL_R8), allocate((width + 1) / 2, (height + 1) / 2, GLES30.GL_RG8)).also { owned.add(it) }
        }
        val crop = image.cropRect
        require(crop.width() == width && crop.height() == height)
        val buffer = checkNotNull(image.hardwareBuffer) { "Decoder image has no HardwareBuffer" }
        try {
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, external[0])
            val imported = nativeImport(buffer)
            var sourceFence = 0L
            var sourceComplete = false
            try {
                GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_NEAREST)
                GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_NEAREST)
                source(copyProgram, buffer, crop)
                target(texture.y, width, height)
                GLES30.glUniform1i(GLES30.glGetUniformLocation(copyProgram, "chroma"), 0)
                GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                target(texture.uv, (width + 1) / 2, (height + 1) / 2)
                GLES30.glUniform1i(GLES30.glGetUniformLocation(copyProgram, "chroma"), 1)
                GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                sourceFence = GLES30.glFenceSync(GLES30.GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
                check(sourceFence != 0L)
                val packedWidth = (analysisWidth + 3) / 4
                val bytes = packedWidth * analysisHeight * 4
                // Small storage buffers avoid a pixel transfer; larger two-pass proxies defer texture readback.
                computeCapture = directCompute != 0 && bytes <= 64 * 1024
                val separable = width > analysisWidth * 2 || height > analysisHeight * 2
                pixelCapture = !computeCapture && !separable
                if (proxyWidth != packedWidth || proxyHeight != analysisHeight) {
                    if (horizontal[0] != 0) GLES30.glDeleteTextures(1, horizontal, 0)
                    horizontal[0] = if (separable) allocate(packedWidth, height, GLES30.GL_RGBA16UI) else 0
                    proxyWidth = packedWidth
                    proxyHeight = analysisHeight
                }
                transfer.compute = computeCapture
                transfer.pixel = pixelCapture
                transfer.bytes = bytes
                if ((computeCapture || pixelCapture) && transfer.bufferBytes != bytes) {
                    GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, transfer.id)
                    GLES30.glBufferData(GLES30.GL_PIXEL_PACK_BUFFER, bytes, null, GLES30.GL_STREAM_READ)
                    GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, 0)
                    transfer.bufferBytes = bytes
                }
                if (!computeCapture && (transfer.textureWidth != packedWidth || transfer.textureHeight != analysisHeight)) {
                    if (transfer.texture != 0) GLES30.glDeleteTextures(1, intArrayOf(transfer.texture), 0)
                    transfer.texture = allocate(packedWidth, analysisHeight)
                    transfer.textureWidth = packedWidth
                    transfer.textureHeight = analysisHeight
                }
                if (separable) {
                    if (horizontal[0] == 0) horizontal[0] = allocate(packedWidth, height, GLES30.GL_RGBA16UI)
                    target(horizontal[0], packedWidth, height)
                    proxySource(horizontalProgram, texture.y, analysisWidth, analysisHeight)
                    GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                }
                if (computeCapture) {
                    proxySource(if (separable) verticalCompute else directCompute,
                        if (separable) horizontal[0] else texture.y, analysisWidth, analysisHeight)
                    GLES31.glBindBufferBase(GLES31.GL_SHADER_STORAGE_BUFFER, 0, transfer.id)
                    GLES31.glDispatchCompute((packedWidth + 7) / 8, (analysisHeight + 7) / 8, 1)
                    GLES31.glMemoryBarrier(GLES31.GL_BUFFER_UPDATE_BARRIER_BIT)
                    GLES31.glBindBufferBase(GLES31.GL_SHADER_STORAGE_BUFFER, 0, 0)
                    GLES31.glBindBuffer(GLES31.GL_SHADER_STORAGE_BUFFER, 0)
                } else if (separable) {
                    target(transfer.texture, packedWidth, analysisHeight)
                    proxySource(verticalProgram, horizontal[0], analysisWidth, analysisHeight)
                    GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                } else {
                    target(transfer.texture, packedWidth, analysisHeight)
                    proxySource(proxyProgram, texture.y, analysisWidth, analysisHeight)
                    GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                }
                if (pixelCapture) {
                    GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, transfer.id)
                    GLES30.glReadPixels(0, 0, packedWidth, analysisHeight, GLES30.GL_RGBA, GLES30.GL_UNSIGNED_BYTE, 0)
                }
                GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, 0)
                transfer.fence = GLES30.glFenceSync(GLES30.GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
                check(transfer.fence != 0L)
                GLES30.glFlush()
                val submitted = System.nanoTime()
                captureSubmitMs = (submitted - began) / 1e6
                waitFence(sourceFence, true)
                sourceComplete = true
                sourceReleaseWaitMs = (System.nanoTime() - submitted) / 1e6
                checkGl()
                return Frame(this, texture, bt709, fullRange, analysisWidth, analysisHeight, transfer)
            } finally {
                GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, 0)
                if (!sourceComplete) GLES30.glFinish()
                if (sourceFence != 0L) GLES30.glDeleteSync(sourceFence)
                nativeRelease(imported)
            }
        } finally {
            buffer.close()
        }
    }

    /** Poll on the GPU thread; returned luma owns CPU memory until its frame is rendered. */
    @JvmOverloads
    fun readLuma(frame: Frame, wait: Boolean = false): StillFlow.Frame? {
        checkThread()
        require(frame.owner === this && !frame.rendered)
        frame.analysis?.let { return it }
        val transfer = checkNotNull(frame.transfer)
        val began = System.nanoTime()
        if (!waitFence(transfer.fence, wait)) return null
        val ready = System.nanoTime()
        val pixels = proxies.poll()?.takeIf { it.capacity() == transfer.bytes } ?: ByteBuffer.allocateDirect(transfer.bytes)
        pixels.clear()
        if (transfer.compute || transfer.pixel) {
            GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, transfer.id)
            try {
                val mapped = GLES30.glMapBufferRange(GLES30.GL_PIXEL_PACK_BUFFER, 0, transfer.bytes, GLES30.GL_MAP_READ_BIT) as? ByteBuffer
                checkNotNull(mapped) { "GPU luma readback mapping failed" }
                try {
                    pixels.clear()
                    mapped.position(0)
                    mapped.limit(transfer.bytes)
                    pixels.put(mapped)
                    pixels.flip()
                } finally {
                    check(GLES30.glUnmapBuffer(GLES30.GL_PIXEL_PACK_BUFFER)) { "GPU luma readback was invalidated" }
                }
            } finally {
                GLES30.glBindBuffer(GLES30.GL_PIXEL_PACK_BUFFER, 0)
            }
        } else {
            target(transfer.texture, (frame.analysisWidth + 3) / 4, frame.analysisHeight)
            GLES30.glReadPixels(0, 0, (frame.analysisWidth + 3) / 4, frame.analysisHeight,
                GLES30.GL_RGBA, GLES30.GL_UNSIGNED_BYTE, pixels)
            pixels.position(0)
            pixels.limit(transfer.bytes)
        }
        GLES30.glDeleteSync(transfer.fence)
        transfer.fence = 0
        frame.transfer = null
        availableReadbacks.add(transfer)
        frame.buffer = pixels
        val luma = StillFlow.Frame(frame.analysisWidth, frame.analysisHeight, StillFlow.PixelFormat.GRAY8,
            ((frame.analysisWidth + 3) / 4) * 4, pixels)
        frame.analysis = luma
        proxyReadbackMs = (System.nanoTime() - began) / 1e6
        proxyTransferMs = (System.nanoTime() - ready) / 1e6
        checkGl()
        return luma
    }

    private fun waitFence(fence: Long, wait: Boolean): Boolean {
        val deadline = System.nanoTime() + 5_000_000_000L
        do {
            when (GLES30.glClientWaitSync(fence, 0, if (wait) 1_000_000L else 0L)) {
                GLES30.GL_ALREADY_SIGNALED, GLES30.GL_CONDITION_SATISFIED -> return true
                GLES30.GL_TIMEOUT_EXPIRED -> if (!wait) return false
                else -> error("GPU synchronization failed")
            }
        } while (System.nanoTime() < deadline)
        error("GPU synchronization timed out")
    }

    private fun proxySource(program: Int, texture: Int, analysisWidth: Int, analysisHeight: Int) {
        GLES30.glUseProgram(program)
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture)
        GLES30.glUniform1i(GLES30.glGetUniformLocation(program, "source"), 0)
        GLES30.glUniform2f(GLES30.glGetUniformLocation(program, "sourceSize"), width.toFloat(), height.toFloat())
        GLES30.glUniform2f(GLES30.glGetUniformLocation(program, "analysisSize"), analysisWidth.toFloat(), analysisHeight.toFloat())
    }

    fun render(frame: Frame, plan: StreamPlanner.PlannedFrame, originUs: Long) = render(frame, plan.map, plan.ptsUs - originUs)

    /** The map is an independent immutable snapshot obtained from takeReadyMap(). */
    fun render(frame: Frame, map: FloatArray, ptsUs: Long) {
        checkThread()
        require(frame.owner === this && !frame.rendered) { "Frame is foreign or already consumed" }
        readLuma(frame, true)
        renderTexture(frame.texture, map, ptsUs, frame.bt709, frame.fullRange)
        frame.rendered = true
        free.add(frame.texture)
        proxies.add(checkNotNull(frame.buffer))
    }

    /** Render one decoded Image with an offline map; the caller may close the Image on return. */
    fun render(image: Image, map: FloatArray, ptsUs: Long, bt709: Boolean, fullRange: Boolean) {
        checkThread()
        val crop = image.cropRect
        require(crop.width() == width && crop.height() == height)
        val buffer = checkNotNull(image.hardwareBuffer) { "Decoder image has no HardwareBuffer" }
        try {
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, external[0])
            val imported = nativeImport(buffer)
            try {
                GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
                GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
                source(externalWarpProgram, buffer, crop)
                renderTexture(null, map, ptsUs, bt709, fullRange)
            } finally {
                // Complete source reads before the decoder recycles this Image, including on failure.
                GLES30.glFinish()
                nativeRelease(imported)
            }
        } finally {
            buffer.close()
        }
    }

    private fun source(program: Int, buffer: HardwareBuffer, crop: Rect) {
        GLES30.glUseProgram(program)
        GLES30.glUniform1i(GLES30.glGetUniformLocation(program, "source"), 0)
        GLES30.glUniform2f(GLES30.glGetUniformLocation(program, "codedSize"), buffer.width.toFloat(), buffer.height.toFloat())
        GLES30.glUniform4f(GLES30.glGetUniformLocation(program, "crop"), crop.left.toFloat(), crop.top.toFloat(), width.toFloat(), height.toFloat())
    }

    private fun renderTexture(texture: Texture?, map: FloatArray, ptsUs: Long, bt709: Boolean, fullRange: Boolean) {
        val began = System.nanoTime()
        require(map.size >= 8 && ptsUs >= 0 && map.all { it.isFinite() })
        val vertices = map[0].toInt() * map[1].toInt()
        require(map[0].toInt() in 2..32 && map[1].toInt() in 2..32 && vertices <= 288 && map.size == 8 + 2 * vertices)
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)
        GLES30.glViewport(0, 0, width, height)
        val program = if (texture == null) externalWarpProgram else warpProgram
        GLES30.glUseProgram(program)
        if (texture != null) {
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture.y)
            GLES30.glUniform1i(GLES30.glGetUniformLocation(program, "source"), 0)
            GLES30.glActiveTexture(GLES30.GL_TEXTURE1)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture.uv)
            GLES30.glUniform1i(GLES30.glGetUniformLocation(program, "chromaSource"), 1)
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
        }
        GLES30.glUniform2f(GLES30.glGetUniformLocation(program, "coefficients"), if (bt709) 0.2126f else 0.299f, if (bt709) 0.0722f else 0.114f)
        GLES30.glUniform1i(GLES30.glGetUniformLocation(program, "fullRange"), if (fullRange) 1 else 0)
        GLES30.glUniform2f(GLES30.glGetUniformLocation(program, "dimensions"), width.toFloat(), height.toFloat())
        GLES30.glUniform2i(GLES30.glGetUniformLocation(program, "meshSize"), map[0].toInt(), map[1].toInt())
        GLES30.glUniform4fv(GLES30.glGetUniformLocation(program, "viewport"), 1, map, 2)
        GLES30.glUniform2fv(GLES30.glGetUniformLocation(program, "gridScale"), 1, map, 6)
        GLES30.glUniform2fv(GLES30.glGetUniformLocation(program, "offsets"), (map.size - 8) / 2, map, 8)
        GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
        checkGl()
        check(EGLExt.eglPresentationTimeANDROID(display, window, Math.multiplyExact(ptsUs, 1000)))
        val submitted = System.nanoTime()
        renderSubmitMs = (submitted - began) / 1e6
        check(EGL14.eglSwapBuffers(display, window))
        surfaceSubmitMs = (System.nanoTime() - submitted) / 1e6
    }

    private fun target(texture: Int, width: Int, height: Int) {
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, fbo[0])
        GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_COLOR_ATTACHMENT0, GLES30.GL_TEXTURE_2D, texture, 0)
        check(GLES30.glCheckFramebufferStatus(GLES30.GL_FRAMEBUFFER) == GLES30.GL_FRAMEBUFFER_COMPLETE)
        GLES30.glViewport(0, 0, width, height)
    }

    private fun checkThread() { check(!closed && Thread.currentThread() === ownerThread) { "GPU context is closed or used from another thread" } }

    override fun close() {
        if (closed) return
        check(Thread.currentThread() === ownerThread) { "Close GPU context on its owner thread" }
        closed = true
        if (context != EGL14.EGL_NO_CONTEXT && window != EGL14.EGL_NO_SURFACE) {
        EGL14.eglMakeCurrent(display, window, window, context)
        GLES30.glFinish()
        GLES30.glDeleteProgram(copyProgram)
        GLES30.glDeleteProgram(proxyProgram)
        GLES30.glDeleteProgram(horizontalProgram)
        GLES30.glDeleteProgram(verticalProgram)
        GLES30.glDeleteProgram(directCompute)
        GLES30.glDeleteProgram(verticalCompute)
        GLES30.glDeleteProgram(warpProgram)
        GLES30.glDeleteProgram(externalWarpProgram)
        val textures = owned.flatMap { listOf(it.y, it.uv) }.toIntArray()
        GLES30.glDeleteTextures(textures.size, textures, 0)
        GLES30.glDeleteTextures(1, external, 0)
        GLES30.glDeleteTextures(readbacks.size, readbacks.map { it.texture }.toIntArray(), 0)
        GLES30.glDeleteTextures(1, horizontal, 0)
        readbacks.forEach { if (it.fence != 0L) GLES30.glDeleteSync(it.fence) }
        GLES30.glDeleteBuffers(readbacks.size, readbacks.map { it.id }.toIntArray(), 0)
        GLES30.glDeleteFramebuffers(1, fbo, 0)
        }
        EGL14.eglMakeCurrent(display, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_SURFACE, EGL14.EGL_NO_CONTEXT)
        if (window != EGL14.EGL_NO_SURFACE) EGL14.eglDestroySurface(display, window)
        if (context != EGL14.EGL_NO_CONTEXT) EGL14.eglDestroyContext(display, context)
        if (previousDisplay != EGL14.EGL_NO_DISPLAY) EGL14.eglMakeCurrent(previousDisplay, previousDraw, previousRead, previousContext)
    }

    companion object {
        init { System.loadLibrary("stillflow_jni") }
        @JvmStatic private external fun nativeImport(buffer: HardwareBuffer): Long
        @JvmStatic private external fun nativeRelease(image: Long)
        private fun checkGl() { check(GLES30.glGetError() == GLES30.GL_NO_ERROR) { "GLES operation failed" } }
        private fun allocate(width: Int, height: Int, format: Int = GLES30.GL_RGBA8): Int {
            val id = IntArray(1)
            GLES30.glGenTextures(1, id, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, id[0])
            GLES30.glTexStorage2D(GLES30.GL_TEXTURE_2D, 1, format, width, height)
            val filter = if (format == GLES30.GL_RGBA16UI) GLES30.GL_NEAREST else GLES30.GL_LINEAR
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, filter)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, filter)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
            return id[0]
        }
        private fun program(fragment: String): Int {
            fun shader(type: Int, source: String): Int {
                val id = GLES30.glCreateShader(type)
                GLES30.glShaderSource(id, source)
                GLES30.glCompileShader(id)
                val status = IntArray(1)
                GLES30.glGetShaderiv(id, GLES30.GL_COMPILE_STATUS, status, 0)
                check(status[0] != 0) { GLES30.glGetShaderInfoLog(id) }
                return id
            }
            val vertex = shader(GLES30.GL_VERTEX_SHADER, """#version 300 es
                void main() { vec2 p=vec2((gl_VertexID<<1)&2,gl_VertexID&2); gl_Position=vec4(p*2.0-1.0,0,1); }
            """.trimIndent())
            val pixel = shader(GLES30.GL_FRAGMENT_SHADER, fragment)
            val id = GLES30.glCreateProgram()
            GLES30.glAttachShader(id, vertex)
            GLES30.glAttachShader(id, pixel)
            GLES30.glLinkProgram(id)
            GLES30.glDeleteShader(vertex)
            GLES30.glDeleteShader(pixel)
            val status = IntArray(1)
            GLES30.glGetProgramiv(id, GLES30.GL_LINK_STATUS, status, 0)
            check(status[0] != 0) { GLES30.glGetProgramInfoLog(id) }
            return id
        }
        private fun compute(source: String): Int {
            val shader = GLES30.glCreateShader(GLES31.GL_COMPUTE_SHADER)
            GLES30.glShaderSource(shader, source)
            GLES30.glCompileShader(shader)
            val status = IntArray(1)
            GLES30.glGetShaderiv(shader, GLES30.GL_COMPILE_STATUS, status, 0)
            check(status[0] != 0) { GLES30.glGetShaderInfoLog(shader) }
            val id = GLES30.glCreateProgram()
            GLES30.glAttachShader(id, shader)
            GLES30.glLinkProgram(id)
            GLES30.glDeleteShader(shader)
            GLES30.glGetProgramiv(id, GLES30.GL_LINK_STATUS, status, 0)
            check(status[0] != 0) { GLES30.glGetProgramInfoLog(id) }
            return id
        }
        private val YUV_HEADER = """#version 300 es
            #extension GL_EXT_YUV_target : require
            precision highp float;
            uniform __samplerExternal2DY2YEXT source;
            uniform vec2 codedSize;
            uniform vec4 crop;
            out vec4 color;
            vec3 sampleYuv(vec2 p) { return texture(source,(crop.xy+p)/codedSize).xyz; }
        """.trimIndent() + "\n"
        private val COPY = """
            uniform int chroma;
            void main() {
                vec2 p=chroma!=0 ? floor(gl_FragCoord.xy)*2.0+vec2(0.5,1.0) : gl_FragCoord.xy;
                vec3 yuv=sampleYuv(p);
                color=chroma!=0 ? vec4(yuv.yz,0.0,1.0) : vec4(yuv.x,0.0,0.0,1.0);
            }
        """.trimIndent()
        private val PROXY_HEADER = """#version 300 es
            precision highp float;
            precision highp int;
            uniform highp sampler2D source;
            uniform vec2 sourceSize;
            uniform vec2 analysisSize;
            out vec4 color;
        """.trimIndent() + "\n"
        private val AREA_LUMA = """
            float luma(vec2 at) {
                at=min(at,analysisSize-1.0);
                vec2 scale=sourceSize/analysisSize;
                vec2 lo=at*scale,hi=(at+1.0)*scale;
                float sum=0.0;
                for(int y=int(floor(lo.y));y<int(ceil(hi.y));++y)
                    for(int x=int(floor(lo.x));x<int(ceil(hi.x));++x) {
                        vec2 p=vec2(x,y);
                        vec2 weight=max(vec2(0),min(hi,p+1.0)-max(lo,p));
                        sum+=texelFetch(source,min(ivec2(x,y),ivec2(sourceSize)-1),0).r*255.0*weight.x*weight.y;
                    }
                return floor(sum/(scale.x*scale.y)+0.5)/255.0;
            }
        """.trimIndent() + "\n"
        private val PROXY = AREA_LUMA + """
            void main() { vec2 at=vec2(floor(gl_FragCoord.x)*4.0,floor(gl_FragCoord.y)); color=vec4(luma(at),luma(at+vec2(1,0)),luma(at+vec2(2,0)),luma(at+vec2(3,0))); }
        """.trimIndent()
        // Preserve fractional horizontal averages in 8.8 fixed point, rounding to gray8 only after the vertical pass.
        private val HORIZONTAL = """
            uint luma(int at,int y) {
                at=min(at,int(analysisSize.x)-1);
                float scale=sourceSize.x/analysisSize.x;
                float lo=float(at)*scale,hi=float(at+1)*scale,sum=0.0;
                for(int x=int(floor(lo));x<int(ceil(hi));++x)
                    sum+=texelFetch(source,min(ivec2(x,y),ivec2(sourceSize)-1),0).r*255.0*max(0.0,min(hi,float(x+1))-max(lo,float(x)));
                return uint(floor(sum*256.0/scale+0.5));
            }
            void main() {
                ivec2 at=ivec2(gl_FragCoord.xy);
                int x=at.x*4;
                color=uvec4(luma(x,at.y),luma(x+1,at.y),luma(x+2,at.y),luma(x+3,at.y));
            }
        """.trimIndent()
        private val VERTICAL_LUMA = """
            vec4 verticalLuma(ivec2 at) {
                float scale=sourceSize.y/analysisSize.y;
                float lo=float(at.y)*scale,hi=float(at.y+1)*scale;
                vec4 sum=vec4(0);
                for(int y=int(floor(lo));y<int(ceil(hi));++y)
                    sum+=vec4(texelFetch(source,ivec2(at.x,min(y,int(sourceSize.y)-1)),0))*max(0.0,min(hi,float(y+1))-max(lo,float(y)));
                return floor(sum/(scale*256.0)+0.5)/255.0;
            }
        """.trimIndent() + "\n"
        private val VERTICAL = PROXY_HEADER.replace("sampler2D", "usampler2D") + VERTICAL_LUMA + """
            void main() { color=verticalLuma(ivec2(gl_FragCoord.xy)); }
        """.trimIndent()
        private val COMPUTE_HEADER = """#version 310 es
            precision highp float;
            precision highp int;
            layout(local_size_x=8,local_size_y=8) in;
            uniform highp sampler2D source;
            uniform vec2 sourceSize;
            uniform vec2 analysisSize;
            layout(std430,binding=0) writeonly buffer Luma { uint pixels[]; };
            void storeLuma(ivec2 at,vec4 value) {
                uvec4 bytes=uvec4(floor(value*255.0+0.5));
                int stride=(int(analysisSize.x)+3)/4;
                pixels[at.y*stride+at.x]=bytes.x|(bytes.y<<8)|(bytes.z<<16)|(bytes.w<<24);
            }
        """.trimIndent() + "\n"
        private val COMPUTE_DIRECT = """
            void main() {
                ivec2 at=ivec2(gl_GlobalInvocationID.xy);
                if(at.x>=(int(analysisSize.x)+3)/4 || at.y>=int(analysisSize.y)) return;
                vec2 p=vec2(at.x*4,at.y);
                storeLuma(at,vec4(luma(p),luma(p+vec2(1,0)),luma(p+vec2(2,0)),luma(p+vec2(3,0))));
            }
        """.trimIndent()
        private val COMPUTE_VERTICAL = """
            void main() {
                ivec2 at=ivec2(gl_GlobalInvocationID.xy);
                if(at.x>=(int(analysisSize.x)+3)/4 || at.y>=int(analysisSize.y)) return;
                storeLuma(at,verticalLuma(at));
            }
        """.trimIndent()
        private val WARP_HEADER = """#version 300 es
            precision highp float;
            uniform vec2 dimensions;
            uniform ivec2 meshSize;
            uniform vec2 gridScale;
            uniform vec4 viewport;
            uniform vec2 offsets[288];
            out vec4 color;
            uniform vec2 coefficients;
            uniform int fullRange;
        """.trimIndent() + "\n"
        private val PLANAR_SOURCE = """
            uniform sampler2D source;
            uniform sampler2D chromaSource;
            float sourceLuma(vec2 p) { return texture(source,(p+0.5)/dimensions).r; }
            vec2 sourceChroma(vec2 p) {
                vec2 size=ceil(dimensions*0.5);
                return texture(chromaSource,((p-vec2(0.0,0.5))*0.5+0.5)/size).rg;
            }
        """.trimIndent() + "\n"
        private val EXTERNAL_SOURCE = """
            float sourceLuma(vec2 p) { return sampleYuv(clamp(p,vec2(0),dimensions-1.0)+0.5).x; }
            vec2 sourceChroma(vec2 p) { return sampleYuv(clamp(p,vec2(0),dimensions-1.0)+0.5).yz; }
        """.trimIndent() + "\n"
        private val WARP = """
            vec4 weights(float t) {
                float t2=t*t,t3=t2*t;
                return vec4(-0.5*t+t2-0.5*t3,1.0-2.5*t2+1.5*t3,0.5*t+2.0*t2-1.5*t3,-0.5*t2+0.5*t3);
            }
            float reconstruct(vec2 p) {
                p=clamp(p,vec2(0),dimensions-1.0);
                vec2 lo=floor(p),f=p-lo;
                vec4 wx=weights(f.x),wy=weights(f.y);
                // Combine the two positive center taps with hardware linear
                // filtering: nine reads implement the sixteen-tap cubic kernel.
                vec3 x=vec3(lo.x-1.0,lo.x+wx.z/(wx.y+wx.z),lo.x+2.0);
                vec3 y=vec3(lo.y-1.0,lo.y+wy.z/(wy.y+wy.z),lo.y+2.0);
                vec3 ax=vec3(wx.x,wx.y+wx.z,wx.w),ay=vec3(wy.x,wy.y+wy.z,wy.w);
                float value=0.0;
                for(int j=0;j<3;++j)
                    for(int i=0;i<3;++i)
                        value+=sourceLuma(vec2(x[i],y[j]))*ax[i]*ay[j];
                return clamp(value,0.0,1.0);
            }
            void main() {
                vec2 at=vec2(gl_FragCoord.x-0.5,dimensions.y-gl_FragCoord.y-0.5);
                vec2 target=viewport.xy+at*viewport.zw;
                vec2 p=target;
                for(int iteration=0;iteration<8;++iteration) {
                    vec2 g=clamp(p*gridScale,vec2(0),vec2(meshSize-1));
                    ivec2 cell=min(ivec2(g),meshSize-2);
                    vec2 f=g-vec2(cell);
                    int a=cell.y*meshSize.x+cell.x,b=a+meshSize.x;
                    p=target-mix(mix(offsets[a],offsets[a+1],f.x),mix(offsets[b],offsets[b+1],f.x),f.y);
                }
                float y=reconstruct(p);
                y=fullRange!=0 ? y : (y-16.0/255.0)*255.0/219.0;
                vec2 uv=(sourceChroma(p)-vec2(128.0/255.0))*(fullRange!=0 ? 1.0 : 255.0/224.0);
                float kr=coefficients.x,kb=coefficients.y,kg=1.0-kr-kb;
                color=vec4(y+2.0*(1.0-kr)*uv.y,y-2.0*kb*(1.0-kb)/kg*uv.x-2.0*kr*(1.0-kr)/kg*uv.y,y+2.0*(1.0-kb)*uv.x,1.0);
            }
        """.trimIndent()
    }
}
