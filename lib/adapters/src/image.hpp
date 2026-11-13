#pragma once
#include "stillflow/application/image.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <stdexcept>

namespace stillflow {
class OpenCvImage final : public ImageData {
  public:
    explicit OpenCvImage(cv::Mat value) : pixels_(std::move(value)) { (void)format(); }
    Size size() const override { return {pixels_.cols, pixels_.rows}; }
    PixelFormat format() const override {
        if (pixels_.type() == CV_8UC3)
            return PixelFormat::bgr8;
        if (pixels_.type() == CV_8UC1)
            return PixelFormat::gray8;
        throw std::invalid_argument("Unsupported OpenCV pixel format");
    }
    HostImageView map_read() const override {
        auto owner = std::make_shared<const cv::Mat>(pixels_);
        const size_t bytes =
            pixels_.empty() ? 0 : (pixels_.rows - 1) * pixels_.step + pixels_.cols * pixels_.elemSize();
        return {size(), format(), {{pixels_.step, {pixels_.data, bytes}}}, owner, {}};
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
    if (mapping.format == PixelFormat::i420 || mapping.format == PixelFormat::nv12 ||
        mapping.format == PixelFormat::nv21) {
        const auto size = mapping.size;
        cv::Mat bgr(size.height, size.width, CV_8UC3);
        const auto& color = mapping.color;
        const int cw = (size.width + 1) / 2, ch = (size.height + 1) / 2;
        const auto chroma = [&](int component, int x, int y) {
            const bool planar = mapping.format == PixelFormat::i420;
            const auto& plane = mapping.planes[planar ? component + 1 : 1];
            const size_t stride = planar ? 1 : 2;
            const size_t offset = planar ? 0 : (mapping.format == PixelFormat::nv21 ? 1 - component : component);
            const double cx = std::clamp((x - color.chroma_x) * 0.5, 0.0, double(cw - 1));
            const double cy = std::clamp((y - color.chroma_y) * 0.5, 0.0, double(ch - 1));
            const int ix = static_cast<int>(cx), iy = static_cast<int>(cy);
            const double fx = cx - ix, fy = cy - iy;
            const auto sample = [&](int sx, int sy) {
                return plane.bytes[static_cast<size_t>(std::min(sy, ch - 1)) * plane.row_stride +
                                   static_cast<size_t>(std::min(sx, cw - 1)) * stride + offset];
            };
            return ((sample(ix, iy) * (1 - fx) + sample(ix + 1, iy) * fx) * (1 - fy) +
                    (sample(ix, iy + 1) * (1 - fx) + sample(ix + 1, iy + 1) * fx) * fy - 128) /
                   (color.full_range ? 255 : 224);
        };
        for (int y = 0; y < size.height; ++y)
            for (int x = 0; x < size.width; ++x) {
                const double yy =
                    (mapping.planes[0].bytes[y * mapping.planes[0].row_stride + x] - (color.full_range ? 0 : 16)) /
                    double(color.full_range ? 255 : 219);
                const double r = yy + 2 * (1 - color.kr) * chroma(1, x, y);
                const double b = yy + 2 * (1 - color.kb) * chroma(0, x, y);
                const double g = (yy - color.kr * r - color.kb * b) / (1 - color.kr - color.kb);
                auto* pixel = bgr.ptr<uint8_t>(y) + static_cast<size_t>(x) * 3;
                pixel[0] = static_cast<uint8_t>(std::clamp(std::lround(b * 255), 0L, 255L));
                pixel[1] = static_cast<uint8_t>(std::clamp(std::lround(g * 255), 0L, 255L));
                pixel[2] = static_cast<uint8_t>(std::clamp(std::lround(r * 255), 0L, 255L));
            }
        auto converted = std::make_shared<OpenCvImage>(std::move(bgr));
        return read_pixels(converted);
    }
    const int type = CV_MAKETYPE(CV_8U, static_cast<int>(pixel_bytes(mapping.format)));
    // OpenCV's input-only APIs accept a mutable Mat header; callers keep pixels read-only.
    cv::Mat pixels(mapping.size.height, mapping.size.width, type, const_cast<uint8_t*>(mapping.planes[0].bytes.data()),
                   mapping.planes[0].row_stride);
    return {std::move(mapping), std::move(pixels)};
}
inline Image image(cv::Mat pixels) { return std::make_shared<OpenCvImage>(std::move(pixels)); }
} // namespace stillflow
