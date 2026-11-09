#pragma once
#include "stillflow/application/image.hpp"
#include <opencv2/core.hpp>
#include <stdexcept>

namespace stillflow {
class OpenCvImage final : public ImageData {
public:
    explicit OpenCvImage(cv::Mat value) : pixels_(std::move(value)) { (void)format(); }
    Size size() const override { return {pixels_.cols, pixels_.rows}; }
    PixelFormat format() const override {
        if (pixels_.type() == CV_8UC3) return PixelFormat::bgr8;
        if (pixels_.type() == CV_8UC1) return PixelFormat::gray8;
        throw std::invalid_argument("Unsupported OpenCV pixel format");
    }
    HostImageView map_read() const override {
        auto owner = std::make_shared<const cv::Mat>(pixels_);
        const size_t bytes = pixels_.empty() ? 0 : (pixels_.rows - 1) * pixels_.step + pixels_.cols * pixels_.elemSize();
        return {size(), format(), pixels_.step, {pixels_.data, bytes}, owner};
    }
private:
    cv::Mat pixels_;
};
struct CvReadView {
    HostImageView mapping;
    cv::Mat pixels;
};
inline CvReadView read_pixels(const Image& image) {
    auto mapping = map_host_image(image);
    const int type = CV_MAKETYPE(CV_8U, static_cast<int>(pixel_bytes(mapping.format)));
    // OpenCV's input-only APIs accept a mutable Mat header; callers keep pixels read-only.
    cv::Mat pixels(mapping.size.height, mapping.size.width, type,
        const_cast<uint8_t*>(mapping.bytes.data()), mapping.row_stride);
    return {std::move(mapping), std::move(pixels)};
}
inline Image image(cv::Mat pixels) { return std::make_shared<OpenCvImage>(std::move(pixels)); }
} // namespace stillflow
