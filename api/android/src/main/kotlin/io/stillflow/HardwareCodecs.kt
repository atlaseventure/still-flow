package io.stillflow

import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaCodecList
import android.media.MediaFormat
import android.os.Build
import android.view.Surface
import java.io.IOException
import java.util.Locale

/** Configured hardware codecs. The caller starts, feeds, drains and releases each codec. */
object HardwareCodecs {
    /** Pass null for Image output and analyze its luma before releasing the output buffer. */
    @JvmStatic
    @Throws(IOException::class)
    fun decoder(format: MediaFormat, surface: Surface?): MediaCodec = configure(format, false, surface)

    /** Configure Surface or flexible YUV input and provide the corresponding pixels. */
    @JvmStatic
    @Throws(IOException::class)
    fun encoder(format: MediaFormat): MediaCodec = configure(format, true, null)

    private fun configure(format: MediaFormat, encoder: Boolean, surface: Surface?): MediaCodec {
        val mime = requireNotNull(format.getString(MediaFormat.KEY_MIME)) { "mime" }
        val failure = IOException("No compatible hardware ${if (encoder) "encoder" else "decoder"}: $mime")
        for (info in MediaCodecList(MediaCodecList.REGULAR_CODECS).codecInfos) {
            if (info.isEncoder != encoder || !isHardware(info)) continue
            val type = info.supportedTypes.firstOrNull { it.equals(mime, ignoreCase = true) } ?: continue
            if (!info.getCapabilitiesForType(type).isFormatSupported(format)) continue
            var codec: MediaCodec? = null
            try {
                codec = MediaCodec.createByCodecName(info.name)
                codec.configure(format, surface, null, if (encoder) MediaCodec.CONFIGURE_FLAG_ENCODE else 0)
                return codec
            } catch (error: IOException) {
                failure.addSuppressed(error)
                codec?.release()
            } catch (error: RuntimeException) {
                failure.addSuppressed(error)
                codec?.release()
            }
        }
        throw failure
    }

    private fun isHardware(info: MediaCodecInfo): Boolean {
        if (Build.VERSION.SDK_INT >= 29) return info.isHardwareAccelerated
        // API 26-28 expose no hardware flag; exclude platform software codecs.
        val name = info.name.lowercase(Locale.ROOT)
        return (name.startsWith("omx.") || name.startsWith("c2.")) &&
            !name.startsWith("omx.google.") && !name.startsWith("c2.android.") && !name.startsWith("c2.google.") &&
            !name.endsWith(".sw.dec") && !name.endsWith(".sw.enc") && !name.contains(".sw.")
    }
}
