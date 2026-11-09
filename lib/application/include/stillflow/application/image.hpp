#pragma once
#include "stillflow/domain/geometry.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace stillflow {
enum class PixelFormat { gray8, bgr8, rgba8 };
size_t pixel_bytes(PixelFormat format);
std::string_view pixel_format_name(PixelFormat format);

// A read lease: pixels are ready before mapping returns and remain immutable
// until lifetime is released. An owner may unmap or release a fence on destruction.
struct HostImageView {
    Size size;
    PixelFormat format;
    size_t row_stride;
    std::span<const uint8_t> bytes;
    std::shared_ptr<const void> lifetime;
};
// API-specific read handle (for example an Android buffer). The producer waits
// for writes before returning; the consumer retains the lease until its GPU work
// completes. The api string identifies the handle's platform-defined type.
struct NativeImageView {
    std::string api;
    const void* handle;
    std::shared_ptr<const void> lifetime;
};
class ImageAccessError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};
class ImageData {
  public:
    virtual ~ImageData() = default;
    virtual Size size() const = 0;
    virtual PixelFormat format() const = 0;
    virtual HostImageView map_read() const { throw ImageAccessError("Host mapping is unavailable"); }
    virtual std::optional<NativeImageView> native_read(std::string_view) const { return std::nullopt; }
};
using Image = std::shared_ptr<const ImageData>;
HostImageView map_host_image(const Image& image);
Image make_host_image(Size size, PixelFormat format, std::vector<uint8_t> bytes, size_t row_stride = 0);
} // namespace stillflow
