package io.stillflow;

import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaCodecList;
import android.media.MediaFormat;
import android.os.Build;
import android.view.Surface;
import java.io.IOException;
import java.util.Locale;
import java.util.Objects;

/** Hardware codec creation for host-managed decode/encode pipelines. */
public final class HardwareCodecs {
    private HardwareCodecs() {}

    /**
     * Creates a configured hardware decoder. Pass null for Image output, then
     * start the codec and feed compressed samples. Analyze each decoded Image
     * through AndroidFrames.luma before releasing its output buffer.
     * The caller owns the codec and must release it.
     */
    public static MediaCodec decoder(MediaFormat format, Surface surface) throws IOException {
        return configure(format, false, surface);
    }

    /**
     * Creates a configured hardware encoder. For Surface input, set
     * COLOR_FormatSurface, call createInputSurface, then start. For Image input,
     * use COLOR_FormatYUV420Flexible and fill each getInputImage plane respecting
     * its row/pixel stride. The caller owns the codec and must release it.
     */
    public static MediaCodec encoder(MediaFormat format) throws IOException {
        return configure(format, true, null);
    }

    private static MediaCodec configure(MediaFormat format, boolean encoder, Surface surface) throws IOException {
        Objects.requireNonNull(format, "format");
        String mime = Objects.requireNonNull(format.getString(MediaFormat.KEY_MIME), "mime");
        IOException failure = new IOException("No compatible hardware " + (encoder ? "encoder: " : "decoder: ") + mime);
        for (MediaCodecInfo info : new MediaCodecList(MediaCodecList.REGULAR_CODECS).getCodecInfos()) {
            if (info.isEncoder() != encoder || !isHardware(info)) continue;
            boolean supported = false;
            for (String type : info.getSupportedTypes()) {
                if (type.equalsIgnoreCase(mime)) {
                    supported = info.getCapabilitiesForType(type).isFormatSupported(format);
                    break;
                }
            }
            if (!supported) continue;
            MediaCodec codec = null;
            try {
                codec = MediaCodec.createByCodecName(info.getName());
                codec.configure(format, surface, null, encoder ? MediaCodec.CONFIGURE_FLAG_ENCODE : 0);
                return codec;
            } catch (IOException | RuntimeException error) {
                failure.addSuppressed(error);
                if (codec != null) codec.release();
            }
        }
        throw failure;
    }

    private static boolean isHardware(MediaCodecInfo info) {
        if (Build.VERSION.SDK_INT >= 29) return info.isHardwareAccelerated();
        // API 26-28 expose no hardware flag; exclude platform software codecs.
        String name = info.getName().toLowerCase(Locale.ROOT);
        return (name.startsWith("omx.") || name.startsWith("c2."))
                && !name.startsWith("omx.google.") && !name.startsWith("c2.android.")
                && !name.startsWith("c2.google.") && !name.endsWith(".sw.dec")
                && !name.endsWith(".sw.enc") && !name.contains(".sw.");
    }
}
