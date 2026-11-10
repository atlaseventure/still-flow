package io.stillflow;

import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.media.Image;
import java.nio.ByteBuffer;
import java.util.Objects;

/** Views of Android image storage for synchronous frame analysis. */
public final class AndroidFrames {
    private AndroidFrames() {}

    /**
     * Borrows the cropped Y plane of a YUV_420_888 image. Keep the Image open until
     * analyze returns. Uses the crop rectangle and preserves the plane row stride.
     * The session dimensions must match the crop rectangle.
     */
    public static StillFlow.Frame luma(Image image) {
        Objects.requireNonNull(image, "image");
        if (image.getFormat() != ImageFormat.YUV_420_888) {
            throw new IllegalArgumentException("Expected YUV_420_888");
        }
        Image.Plane plane = image.getPlanes()[0];
        if (plane.getPixelStride() != 1) {
            throw new IllegalArgumentException("Expected a packed luma plane");
        }
        Rect crop = image.getCropRect();
        if (crop.isEmpty() || crop.left < 0 || crop.top < 0
                || crop.right > image.getWidth() || crop.bottom > image.getHeight()) {
            throw new IllegalArgumentException("Invalid image crop rectangle");
        }
        ByteBuffer pixels = plane.getBuffer().duplicate();
        long offset = (long) crop.top * plane.getRowStride() + crop.left;
        if (offset > pixels.remaining()) {
            throw new IllegalArgumentException("Luma crop exceeds plane storage");
        }
        pixels.position(pixels.position() + (int) offset);
        return new StillFlow.Frame(crop.width(), crop.height(), StillFlow.PixelFormat.GRAY8,
                plane.getRowStride(), pixels);
    }
}
