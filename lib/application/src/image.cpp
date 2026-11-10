#include "stillflow/application/image.hpp"
#include <cmath>
#include <limits>

namespace stillflow {
size_t pixel_bytes(PixelFormat format) {
    switch (format) {
    case PixelFormat::gray8:
        return 1;
    case PixelFormat::bgr8:
        return 3;
    case PixelFormat::rgba8:
        return 4;
    default:
        throw std::invalid_argument("Pixel format is not packed");
    }
}
std::string_view pixel_format_name(PixelFormat format) {
    switch (format) {
    case PixelFormat::gray8:
        return "gray8";
    case PixelFormat::bgr8:
        return "bgr8";
    case PixelFormat::rgba8:
        return "rgba8";
    case PixelFormat::nv12:
        return "nv12";
    case PixelFormat::nv21:
        return "nv21";
    case PixelFormat::i420:
        return "i420";
    }
    throw std::invalid_argument("Unknown pixel format");
}
namespace {
bool planar(PixelFormat f) { return f == PixelFormat::nv12 || f == PixelFormat::nv21 || f == PixelFormat::i420; }
void validate(const HostImageView& v) {
    if (v.size.width <= 0 || v.size.height <= 0 || !v.lifetime)
        throw std::invalid_argument("Invalid image dimensions or lifetime");
    const size_t count = v.format == PixelFormat::i420 ? 3 : planar(v.format) ? 2 : 1;
    if (v.planes.size() != count)
        throw std::invalid_argument("Invalid plane count");
    if (!std::isfinite(v.color.kr) || !std::isfinite(v.color.kb) || v.color.kr <= 0 || v.color.kb <= 0 ||
        v.color.kr + v.color.kb >= 1 || !std::isfinite(v.color.chroma_x) || !std::isfinite(v.color.chroma_y))
        throw std::invalid_argument("Invalid color metadata");
    for (size_t i = 0; i < count; ++i) {
        const auto& plane = v.planes[i];
        const size_t width = i ? (static_cast<size_t>(v.size.width) + 1) / 2 : v.size.width;
        const size_t height = i ? (static_cast<size_t>(v.size.height) + 1) / 2 : v.size.height;
        const size_t channels =
            !planar(v.format) ? pixel_bytes(v.format) : (i && v.format != PixelFormat::i420 ? 2 : 1);
        if (width > std::numeric_limits<size_t>::max() / channels)
            throw std::invalid_argument("Image row overflow");
        const size_t row_bytes = width * channels, rows = height - 1;
        if (!plane.bytes.data() || plane.row_stride < row_bytes ||
            (rows && plane.row_stride > (std::numeric_limits<size_t>::max() - row_bytes) / rows) ||
            plane.bytes.size() < rows * plane.row_stride + row_bytes)
            throw std::invalid_argument("Image plane has an invalid stride or extent");
    }
}
class HostImage final : public ImageData {
  public:
    explicit HostImage(HostImageView view) : view_(std::move(view)) { validate(view_); }
    Size size() const override { return view_.size; }
    PixelFormat format() const override { return view_.format; }
    HostImageView map_read() const override { return view_; }

  private:
    HostImageView view_;
};
} // namespace
HostImageView map_host_image(const Image& image) {
    if (!image)
        throw std::invalid_argument("Missing image");
    auto view = image->map_read();
    validate(view);
    if (view.size.width != image->size().width || view.size.height != image->size().height ||
        view.format != image->format())
        throw std::invalid_argument("Image mapping does not match metadata");
    return view;
}
Image borrow_host_image(HostImageView view) { return std::make_shared<HostImage>(std::move(view)); }
Image luma_view(const Image& image) {
    auto view = map_host_image(image);
    if (view.format != PixelFormat::gray8 && !planar(view.format))
        throw std::invalid_argument("Image has no luma plane");
    view.format = PixelFormat::gray8;
    view.planes.resize(1);
    return borrow_host_image(std::move(view));
}
Image make_host_image(Size size, PixelFormat format, std::vector<uint8_t> bytes, size_t stride) {
    if (size.width <= 0 || size.height <= 0)
        throw std::invalid_argument("Invalid image dimensions");
    if (!stride)
        stride = static_cast<size_t>(size.width) * pixel_bytes(format);
    auto owner = std::make_shared<const std::vector<uint8_t>>(std::move(bytes));
    return borrow_host_image({size, format, {{stride, *owner}}, owner, {}});
}
} // namespace stillflow
