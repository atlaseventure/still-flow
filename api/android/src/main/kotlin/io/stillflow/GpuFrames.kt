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
import android.view.Surface
import java.nio.ByteBuffer
import java.util.concurrent.ConcurrentLinkedQueue

/** Thread-confined GLES context. Frames own textures independently of decoder buffers. */
@RequiresApi(29)
class GpuFrames(
    private val width: Int,
    private val height: Int,
    private val capacity: Int,
    encoderSurface: Surface,
) : AutoCloseable {
    class Frame internal constructor(internal val owner: GpuFrames, internal val texture: Int, internal val buffer: ByteBuffer, val luma: StillFlow.Frame) {
        internal var rendered = false
    }
    private val display = EGL14.eglGetDisplay(EGL14.EGL_DEFAULT_DISPLAY)
    private val version = IntArray(2)
    private val configs = arrayOfNulls<android.opengl.EGLConfig>(1)
    private val count = IntArray(1)
    private var context: EGLContext = EGL14.EGL_NO_CONTEXT
    private var window: EGLSurface = EGL14.EGL_NO_SURFACE
    private val fbo = IntArray(1)
    private val external = IntArray(1)
    private val proxy = IntArray(1)
    private val free = ConcurrentLinkedQueue<Int>()
    private val proxies = ConcurrentLinkedQueue<ByteBuffer>()
    private val owned = mutableListOf<Int>()
    private var copyProgram = 0
    private var proxyProgram = 0
    private var warpProgram = 0
    private var offlineTexture = 0
    private var proxyWidth = 0
    private var proxyHeight = 0
    private val ownerThread = Thread.currentThread()
    private val previousDisplay = EGL14.eglGetCurrentDisplay()
    private val previousContext = EGL14.eglGetCurrentContext()
    private val previousDraw = EGL14.eglGetCurrentSurface(EGL14.EGL_DRAW)
    private val previousRead = EGL14.eglGetCurrentSurface(EGL14.EGL_READ)
    private var closed = false
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
        proxyProgram = program(YUV_HEADER + PROXY)
        warpProgram = program(WARP)
        } catch (error: Throwable) {
            close()
            throw error
        }
    }

    /** Captures exactly one acquired Image. Only the reduced source-Y proxy is read by the CPU. */
    fun capture(image: Image, analysisWidth: Int, analysisHeight: Int, bt709: Boolean, fullRange: Boolean): Frame {
        checkThread()
        val texture = free.poll() ?: run {
            check(owned.size < capacity) { "GPU frame cache is full" }
            allocate(width, height).also { owned.add(it) }
        }
        val crop = image.cropRect
        require(crop.width() == width && crop.height() == height)
        val buffer = checkNotNull(image.hardwareBuffer) { "Decoder image has no HardwareBuffer" }
        try {
            GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
            GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, external[0])
            val imported = nativeImport(buffer)
            try {
                target(texture, width, height)
                source(copyProgram, buffer, crop)
                GLES30.glUniform2f(GLES30.glGetUniformLocation(copyProgram, "coefficients"), if (bt709) 0.2126f else 0.299f, if (bt709) 0.0722f else 0.114f)
                GLES30.glUniform1i(GLES30.glGetUniformLocation(copyProgram, "fullRange"), if (fullRange) 1 else 0)
                GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                val packedWidth = (analysisWidth + 3) / 4
                if (proxyWidth != packedWidth || proxyHeight != analysisHeight) {
                    if (proxy[0] != 0) GLES30.glDeleteTextures(1, proxy, 0)
                    proxy[0] = allocate(packedWidth, analysisHeight)
                    proxyWidth = packedWidth
                    proxyHeight = analysisHeight
                }
                target(proxy[0], packedWidth, analysisHeight)
                source(proxyProgram, buffer, crop)
                GLES30.glUniform2f(GLES30.glGetUniformLocation(proxyProgram, "analysisSize"), analysisWidth.toFloat(), analysisHeight.toFloat())
                GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                val bytes = packedWidth * analysisHeight * 4
                val pixels = proxies.poll()?.takeIf { it.capacity() == bytes } ?: ByteBuffer.allocateDirect(bytes)
                pixels.clear()
                GLES30.glReadPixels(0, 0, packedWidth, analysisHeight, GLES30.GL_RGBA, GLES30.GL_UNSIGNED_BYTE, pixels)
                checkGl()
                // Readback orders both capture draws before the decoder buffer is released.
                return Frame(this, texture, pixels, StillFlow.Frame(analysisWidth, analysisHeight, StillFlow.PixelFormat.GRAY8, packedWidth * 4, pixels))
            } finally {
                nativeRelease(imported)
            }
        } finally {
            buffer.close()
        }
    }

    fun render(frame: Frame, plan: StreamPlanner.PlannedFrame, originUs: Long) = render(frame, plan.map, plan.ptsUs - originUs)

    /** The map is an independent immutable snapshot obtained from takeReadyMap(). */
    fun render(frame: Frame, map: FloatArray, ptsUs: Long) {
        checkThread()
        require(frame.owner === this && !frame.rendered) { "Frame is foreign or already consumed" }
        renderTexture(frame.texture, map, ptsUs)
        frame.rendered = true
        free.add(frame.texture)
        proxies.add(frame.buffer)
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
                if (offlineTexture == 0) offlineTexture = allocate(width, height)
                target(offlineTexture, width, height)
                source(copyProgram, buffer, crop)
                GLES30.glUniform2f(GLES30.glGetUniformLocation(copyProgram, "coefficients"), if (bt709) 0.2126f else 0.299f, if (bt709) 0.0722f else 0.114f)
                GLES30.glUniform1i(GLES30.glGetUniformLocation(copyProgram, "fullRange"), if (fullRange) 1 else 0)
                GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
                renderTexture(offlineTexture, map, ptsUs)
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

    private fun renderTexture(texture: Int, map: FloatArray, ptsUs: Long) {
        require(map.size >= 8 && ptsUs >= 0 && map.all { it.isFinite() })
        val vertices = map[0].toInt() * map[1].toInt()
        require(map[0].toInt() in 2..32 && map[1].toInt() in 2..32 && vertices <= 288 && map.size == 8 + 2 * vertices)
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0)
        GLES30.glViewport(0, 0, width, height)
        GLES30.glUseProgram(warpProgram)
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texture)
        GLES30.glUniform1i(GLES30.glGetUniformLocation(warpProgram, "source"), 0)
        GLES30.glUniform2f(GLES30.glGetUniformLocation(warpProgram, "dimensions"), width.toFloat(), height.toFloat())
        GLES30.glUniform2i(GLES30.glGetUniformLocation(warpProgram, "meshSize"), map[0].toInt(), map[1].toInt())
        GLES30.glUniform4fv(GLES30.glGetUniformLocation(warpProgram, "viewport"), 1, map, 2)
        GLES30.glUniform2fv(GLES30.glGetUniformLocation(warpProgram, "gridScale"), 1, map, 6)
        GLES30.glUniform2fv(GLES30.glGetUniformLocation(warpProgram, "offsets"), (map.size - 8) / 2, map, 8)
        GLES30.glDrawArrays(GLES30.GL_TRIANGLES, 0, 3)
        checkGl()
        check(EGLExt.eglPresentationTimeANDROID(display, window, Math.multiplyExact(ptsUs, 1000)))
        check(EGL14.eglSwapBuffers(display, window))
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
        GLES30.glDeleteProgram(warpProgram)
        GLES30.glDeleteTextures(owned.size, owned.toIntArray(), 0)
        GLES30.glDeleteTextures(1, intArrayOf(offlineTexture), 0)
        GLES30.glDeleteTextures(1, external, 0)
        GLES30.glDeleteTextures(1, proxy, 0)
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
        private fun allocate(width: Int, height: Int): Int {
            val id = IntArray(1)
            GLES30.glGenTextures(1, id, 0)
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, id[0])
            GLES30.glTexStorage2D(GLES30.GL_TEXTURE_2D, 1, GLES30.GL_RGBA8, width, height)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
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
            uniform vec2 coefficients;
            uniform int fullRange;
            void main() {
                vec3 yuv=sampleYuv(gl_FragCoord.xy);
                float y=fullRange!=0 ? yuv.x : (yuv.x-16.0/255.0)*255.0/219.0;
                vec2 uv=(yuv.yz-vec2(128.0/255.0))*(fullRange!=0 ? 1.0 : 255.0/224.0);
                float kr=coefficients.x,kb=coefficients.y,kg=1.0-kr-kb;
                color=vec4(y+2.0*(1.0-kr)*uv.y,y-2.0*kb*(1.0-kb)/kg*uv.x-2.0*kr*(1.0-kr)/kg*uv.y,y+2.0*(1.0-kb)*uv.x,1);
            }
        """.trimIndent()
        private val PROXY = """
            uniform vec2 analysisSize;
            float luma(vec2 at) {
                vec2 scale=crop.zw/analysisSize;
                vec2 lo=at*scale,hi=(at+1.0)*scale;
                float sum=0.0;
                for(int y=int(floor(lo.y));y<int(ceil(hi.y));++y)
                    for(int x=int(floor(lo.x));x<int(ceil(hi.x));++x) {
                        vec2 p=vec2(x,y);
                        vec2 weight=max(vec2(0),min(hi,p+1.0)-max(lo,p));
                        sum+=sampleYuv(p+0.5).x*255.0*weight.x*weight.y;
                    }
                return floor(sum/(scale.x*scale.y)+0.5)/255.0;
            }
            void main() { vec2 at=vec2(floor(gl_FragCoord.x)*4.0,floor(gl_FragCoord.y)); color=vec4(luma(at),luma(at+vec2(1,0)),luma(at+vec2(2,0)),luma(at+vec2(3,0))); }
        """.trimIndent()
        private val WARP = """#version 300 es
            precision highp float;
            uniform sampler2D source;
            uniform vec2 dimensions;
            uniform ivec2 meshSize;
            uniform vec2 gridScale;
            uniform vec4 viewport;
            uniform vec2 offsets[288];
            out vec4 color;
            vec4 weights(float t) {
                float t2=t*t,t3=t2*t;
                return vec4(-0.5*t+t2-0.5*t3,1.0-2.5*t2+1.5*t3,0.5*t+2.0*t2-1.5*t3,-0.5*t2+0.5*t3);
            }
            vec4 reconstruct(vec2 p) {
                p=clamp(p,vec2(0),dimensions-1.0);
                vec2 lo=floor(p),f=p-lo;
                vec4 wx=weights(f.x),wy=weights(f.y);
                // Combine the two positive center taps with hardware linear
                // filtering: nine reads implement the sixteen-tap cubic kernel.
                vec3 x=vec3(lo.x-1.0,lo.x+wx.z/(wx.y+wx.z),lo.x+2.0);
                vec3 y=vec3(lo.y-1.0,lo.y+wy.z/(wy.y+wy.z),lo.y+2.0);
                vec3 ax=vec3(wx.x,wx.y+wx.z,wx.w),ay=vec3(wy.x,wy.y+wy.z,wy.w);
                vec4 value=vec4(0);
                for(int j=0;j<3;++j)
                    for(int i=0;i<3;++i)
                        value+=texture(source,(vec2(x[i],y[j])+0.5)/dimensions)*ax[i]*ay[j];
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
                color=reconstruct(p);
            }
        """.trimIndent()
    }
}
