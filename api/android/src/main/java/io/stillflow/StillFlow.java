package io.stillflow;

import java.nio.ByteBuffer;
import java.util.Objects;

/**
 * Two-pass, in-memory stabilization. Analyze frames in presentation order, finish,
 * then render the original frames by index. Calls are synchronous; use a worker
 * thread. Close the session with try-with-resources or Kotlin use.
 */
public final class StillFlow implements AutoCloseable {
    static {
        System.loadLibrary("stillflow_jni");
    }

    public enum PixelFormat {
        GRAY8(0, 1), BGR8(1, 3), RGBA8(2, 4);

        private final int nativeValue;
        public final int bytesPerPixel;

        PixelFormat(int nativeValue, int bytesPerPixel) {
            this.nativeValue = nativeValue;
            this.bytesPerPixel = bytesPerPixel;
        }
    }

    public enum CropMode { AUTO, FIXED, ADAPTIVE }
    public enum Backend { AUTO, CPU, GLES }

    /** Settings are copied when the session is created. */
    public static final class Settings {
        public Backend backend = Backend.AUTO;
        public int analysisWidth;
        public int features;
        public int bands;
        public CropMode cropMode;
        public boolean rollingShutter;
        public double smoothSeconds;
        public double rsSeconds;
        public double crop;
        public double rsStrength;

        public Settings() {
            double[] values = nativeDefaults();
            analysisWidth = (int) values[0];
            features = (int) values[1];
            bands = (int) values[2];
            cropMode = CropMode.values()[(int) values[3]];
            rollingShutter = values[4] != 0;
            smoothSeconds = values[5];
            rsSeconds = values[6];
            crop = values[7];
            rsStrength = values[8];
        }
    }

    /**
     * A packed image view over a direct buffer's remaining bytes. Position and
     * limit are captured at construction; subsequent changes to the original
     * buffer's position/limit do not affect this view. Keep the storage alive and
     * unchanged during a call. Input storage may be reused after analyze returns.
     */
    public static final class Frame {
        public final int width;
        public final int height;
        public final int rowStride;
        public final PixelFormat format;
        private final ByteBuffer pixels;

        public Frame(int width, int height, PixelFormat format, int rowStride, ByteBuffer pixels) {
            this.format = Objects.requireNonNull(format, "format");
            Objects.requireNonNull(pixels, "pixels");
            long rowBytes = (long) width * format.bytesPerPixel;
            long required = (long) (height - 1) * rowStride + rowBytes;
            if (width <= 0 || height <= 0 || rowStride < rowBytes || !pixels.isDirect()
                    || required > pixels.remaining()) {
                throw new IllegalArgumentException("Invalid dimensions, stride or direct buffer extent");
            }
            this.width = width;
            this.height = height;
            this.rowStride = rowStride;
            this.pixels = pixels.slice().asReadOnlyBuffer();
        }

        /** Returns a read-only view without changing the stored buffer position. */
        public ByteBuffer getPixels() {
            return pixels.asReadOnlyBuffer();
        }
    }

    public static final class PlanInfo {
        public final long frameCount;
        public final int analysisWidth;
        public final int analysisHeight;
        public final long originUs;

        private PlanInfo(long[] values) {
            frameCount = values[0];
            analysisWidth = (int) values[1];
            analysisHeight = (int) values[2];
            originUs = values[3];
        }
    }

    public static final class RendererInfo {
        public final String backend;
        public final String device;
        public final String fallbackReason;
        public final boolean gpu;

        private RendererInfo(String[] values) {
            backend = values[0];
            device = values[1];
            fallbackReason = values[2];
            gpu = Boolean.parseBoolean(values[3]);
        }
    }

    private long handle;

    public StillFlow(int width, int height, double fps) {
        this(width, height, fps, false, new Settings());
    }

    /** horizontalScan selects left-to-right rolling shutter readout. */
    public StillFlow(int width, int height, double fps, boolean horizontalScan, Settings settings) {
        Objects.requireNonNull(settings, "settings");
        handle = nativeCreate(width, height, fps, horizontalScan,
                Objects.requireNonNull(settings.backend, "backend").ordinal(),
                settings.analysisWidth, settings.features, settings.bands,
                Objects.requireNonNull(settings.cropMode, "cropMode").ordinal(), settings.rollingShutter,
                settings.smoothSeconds, settings.rsSeconds, settings.crop, settings.rsStrength);
    }

    /** Timestamps and durations are in microseconds; timestamps must increase. */
    public synchronized void analyze(Frame frame, long ptsUs, long durationUs) {
        requireOpen();
        Objects.requireNonNull(frame, "frame");
        nativeAnalyze(handle, frame.width, frame.height, frame.format.nativeValue,
                frame.rowStride, frame.pixels, ptsUs, durationUs);
    }

    /** Completes analysis once and retains the plan until close. */
    public synchronized PlanInfo finish() {
        requireOpen();
        return new PlanInfo(nativeFinish(handle));
    }

    /** Reports the current backend, device and last automatic fallback reason. */
    public synchronized RendererInfo rendererInfo() {
        requireOpen();
        return new RendererInfo(nativeRendererInfo(handle));
    }

    /**
     * Renders into caller-owned direct storage, starting at its position. The
     * returned frame shares that storage. Destination position/limit are unchanged.
     * Output has the input dimensions and format, with tightly packed rows.
     */
    public synchronized Frame render(int index, Frame frame, ByteBuffer destination) {
        requireOpen();
        Objects.requireNonNull(frame, "frame");
        Objects.requireNonNull(destination, "destination");
        long rowBytes = (long) frame.width * frame.format.bytesPerPixel;
        long size = rowBytes * frame.height;
        if (index < 0 || !destination.isDirect() || destination.isReadOnly()
                || rowBytes > Integer.MAX_VALUE || size > destination.remaining()) {
            throw new IllegalArgumentException("Invalid index or writable direct destination extent");
        }
        ByteBuffer output = destination.slice();
        output.limit((int) size);
        nativeRender(handle, index, frame.width, frame.height, frame.format.nativeValue,
                frame.rowStride, frame.pixels, output);
        return new Frame(frame.width, frame.height, frame.format, (int) rowBytes, output);
    }

    /** Releases all native resources. Repeated close calls are safe. */
    @Override
    public synchronized void close() {
        if (handle != 0) {
            nativeDestroy(handle);
            handle = 0;
        }
    }

    public static String buildDescription() {
        return nativeBuildDescription();
    }

    private void requireOpen() {
        if (handle == 0) {
            throw new IllegalStateException("StillFlow session is closed");
        }
    }

    private static native double[] nativeDefaults();
    private static native long nativeCreate(int width, int height, double fps, boolean horizontalScan,
            int backend,
            int analysisWidth, int features, int bands, int cropMode, boolean rollingShutter,
            double smoothSeconds, double rsSeconds, double crop, double rsStrength);
    private static native void nativeAnalyze(long handle, int width, int height, int format,
            int rowStride, ByteBuffer pixels, long ptsUs, long durationUs);
    private static native long[] nativeFinish(long handle);
    private static native String[] nativeRendererInfo(long handle);
    private static native void nativeRender(long handle, int index, int width, int height, int format,
            int rowStride, ByteBuffer pixels, ByteBuffer destination);
    private static native void nativeDestroy(long handle);
    private static native String nativeBuildDescription();
}
