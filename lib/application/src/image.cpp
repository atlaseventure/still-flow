#include "stillflow/application/image.hpp"
#include <limits>
#include <stdexcept>

namespace stillflow {
size_t pixel_bytes(PixelFormat format) {
    switch (format) {
    case PixelFormat::gray8: return 1;
    case PixelFormat::bgr8: return 3;
    case PixelFormat::rgba8: return 4;
    }
    throw std::invalid_argument("Unknown pixel format");
}
std::string_view pixel_format_name(PixelFormat format) {
    switch (format) {
    case PixelFormat::gray8: return "gray8";
    case PixelFormat::bgr8: return "bgr8";
    case PixelFormat::rgba8: return "rgba8";
    }
    throw std::invalid_argument("Unknown pixel format");
}
namespace {
void validate(const HostImageView& view) {
    if (view.size.width <= 0 || view.size.height <= 0 || !view.lifetime || !view.bytes.data())
        throw std::invalid_argument("Invalid image mapping or lifetime");
    const auto channels = pixel_bytes(view.format);
    const auto width = static_cast<size_t>(view.size.width);
    if (width > std::numeric_limits<size_t>::max() / channels)
        throw std::invalid_argument("Image row is too large");
    const size_t row_bytes = width * channels, rows = view.size.height - 1;
    if (view.row_stride < row_bytes || (rows && view.row_stride > (std::numeric_limits<size_t>::max() - row_bytes) / rows)
        || view.bytes.size() < rows * view.row_stride + row_bytes)
        throw std::invalid_argument("Image mapping has an invalid stride or extent");
}
class HostImage final : public ImageData {
public:
    HostImage(Size size, PixelFormat format, std::vector<uint8_t> bytes, size_t stride)
        : size_(size), format_(format), stride_(stride), bytes_(std::make_shared<const std::vector<uint8_t>>(std::move(bytes))) {
        validate(map_read());
    }
    Size size() const override { return size_; }
    PixelFormat format() const override { return format_; }
    HostImageView map_read() const override { return {size_, format_, stride_, *bytes_, bytes_}; }
private:
    Size size_;
    PixelFormat format_;
    size_t stride_;
    std::shared_ptr<const std::vector<uint8_t>> bytes_;
};
}
HostImageView map_host_image(const Image& image) {
    if (!image) throw std::invalid_argument("Missing image");
    auto view = image->map_read();
    validate(view);
    if (view.size.width != image->size().width || view.size.height != image->size().height || view.format != image->format())
        throw std::invalid_argument("Image mapping does not match its metadata");
    return view;
}
Image make_host_image(Size size, PixelFormat format, std::vector<uint8_t> bytes, size_t stride) {
    if (size.width <= 0 || size.height <= 0) throw std::invalid_argument("Invalid image dimensions");
    if (!stride) stride = static_cast<size_t>(size.width) * pixel_bytes(format);
    return std::make_shared<HostImage>(size, format, std::move(bytes), stride);
}
} // namespace stillflow
