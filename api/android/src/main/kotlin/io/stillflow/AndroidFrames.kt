package io.stillflow

import android.graphics.ImageFormat
import android.media.Image

/** Views of Android image storage for synchronous frame analysis. */
object AndroidFrames {
    /** Borrow the cropped Y plane. Keep the Image open until analyze returns. */
    @JvmStatic
    fun luma(image: Image): StillFlow.Frame {
        require(image.format == ImageFormat.YUV_420_888) { "Expected YUV_420_888" }
        val plane = image.planes[0]
        require(plane.pixelStride == 1) { "Expected a packed luma plane" }
        val crop = image.cropRect
        require(!crop.isEmpty && crop.left >= 0 && crop.top >= 0 && crop.right <= image.width && crop.bottom <= image.height) {
            "Invalid image crop rectangle"
        }
        val pixels = plane.buffer.duplicate()
        val offset = crop.top.toLong() * plane.rowStride + crop.left
        require(offset <= pixels.remaining()) { "Luma crop exceeds plane storage" }
        pixels.position(pixels.position() + offset.toInt())
        return StillFlow.Frame(crop.width(), crop.height(), StillFlow.PixelFormat.GRAY8, plane.rowStride, pixels)
    }
}
