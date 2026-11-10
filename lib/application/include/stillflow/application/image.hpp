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
enum class PixelFormat { gray8, bgr8, rgba8, nv12, nv21, i420 };
struct ColorInfo {
    float kr = 0.2126f, kb = 0.0722f;
    bool full_range = false;
    float chroma_x = 0, chroma_y = 0.5f;
};
struct ImagePlane {
    size_t row_stride = 0;
    std::span<const uint8_t> bytes;
};
inline constexpr std::string_view android_image_api = "android.hbuffer";
inline constexpr std::string_view dmabuf_image_api = "linux.dmabuf";
inline constexpr std::string_view vulkan_image_api = "vulkan.image";
size_t pixel_bytes(PixelFormat format);
std::string_view pixel_format_name(PixelFormat format);

// A read lease: pixels are ready before mapping returns and remain immutable
// until lifetime is released. An owner may unmap or release a fence on destruction.
struct HostImageView {
    Size size;
    PixelFormat format;
    std::vector<ImagePlane> planes;
    std::shared_ptr<const void> lifetime;
    ColorInfo color;
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
Image borrow_host_image(HostImageView view);
Image luma_view(const Image& image);
Image make_host_image(Size size, PixelFormat format, std::vector<uint8_t> bytes, size_t row_stride = 0);
} // namespace stillflow
