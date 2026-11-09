#pragma once
#include "stillflow/application/image.hpp"
#include "stillflow/domain/motion.hpp"
#include <functional>
#include <map>
#include <stdexcept>

namespace stillflow {
struct BackendCapabilities {
    std::vector<PixelFormat> formats;
    // "host" uses map_read; other identifiers use native_read with that API.
    std::vector<std::string> image_access;
};
struct BackendInfo {
    std::string id, device;
    BackendCapabilities capabilities;
    std::map<std::string, std::string> implementation;
};
struct BackendRun {
    BackendInfo info;
    size_t frames = 0;
    bool gpu = false;
};
struct BackendFailure {
    std::string backend, stage, reason;
};
struct RenderingReport {
    std::string requested, backend;
    std::vector<BackendRun> backends;
    std::vector<BackendFailure> failures;
};
// Only backend availability/execution failures trigger automatic fallback.
// Invalid input, invalid geometry and programming errors propagate unchanged.
class BackendError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};
class RenderBackend {
  public:
    virtual ~RenderBackend() = default;
    virtual BackendInfo info() const = 0;
    // Retain any input lease needed until device reads have completed.
    virtual void upload(const Image& input) = 0;
    // Returns the completed fraction of out-of-bounds pixels in [0, 1].
    virtual double warp(const MapParameters& map) = 0;
    // Return a ready image whose storage survives subsequent backend calls.
    virtual Image download() = 0;
};
struct BackendProvider {
    std::string id;
    std::function<std::unique_ptr<RenderBackend>()> create;
    // Renders on a GPU; "auto" codec selection then prefers hardware codecs.
    bool gpu = false;
};
class Renderer {
  public:
    virtual ~Renderer() = default;
    virtual Image render(const Image& input, Warp& warp, Size analysis_size, double crop, bool horizontal,
                         Profile& profile) = 0;
    virtual RenderingReport report() const = 0;
};
class RendererFactory {
  public:
    virtual ~RendererFactory() = default;
    virtual std::unique_ptr<Renderer> create(const std::string& requested) const = 0;
    // Whether the request is expected to render on a GPU before create()
    // resolves it; for "auto" this is the most preferred provider.
    virtual bool prefers_gpu(const std::string& /*requested*/) const { return false; }
};
// Registration order is the preference/fallback order for "auto". Explicit
// selection uses only that provider. Providers and their captures are copied.
std::unique_ptr<RendererFactory> make_renderer_factory(std::vector<BackendProvider> providers);
} // namespace stillflow
