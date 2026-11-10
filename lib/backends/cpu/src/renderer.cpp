#include "stillflow/backends/cpu.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace stillflow {
namespace {
class CpuBackend final : public RenderBackend {
  public:
    explicit CpuBackend(ExecutionSettings execution) : execution_(execution) {}
    BackendInfo info() const override {
        return {"cpu",
                "host",
                {{PixelFormat::gray8, PixelFormat::bgr8, PixelFormat::rgba8}, {"host"}},
                {{"sampler", "bilinear-5bit"}}};
    }
    void upload(const Image& input) override {
        execution_.check();
        mapping_ = map_host_image(input);
        channels_ = pixel_bytes(mapping_.format);
    }
    double warp(const MapParameters& map) override {
        if (mapping_.planes.empty())
            throw std::logic_error("No uploaded image");
        const auto size = mapping_.size;
        const size_t stride = static_cast<size_t>(size.width) * channels_;
        std::vector<uint8_t> pixels(stride * size.height);
        std::vector<size_t> invalid(size.height);
        const auto& plane = mapping_.planes[0];
        execution_.for_each(size.height, [&](size_t row) {
            for (int x = 0; x < size.width; ++x) {
                auto p = source_point(map, {map.origin_x + x * map.step_x, map.origin_y + row * map.step_y});
                const float fx = static_cast<float>(p.x), fy = static_cast<float>(p.y);
                if (!std::isfinite(fx) || !std::isfinite(fy) || std::abs(fx) > 10000000 || std::abs(fy) > 10000000)
                    throw std::invalid_argument("Invalid sample coordinate");
                invalid[row] += !(fx >= 0 && fx <= size.width - 1 && fy >= 0 && fy <= size.height - 1);
                const int qx = static_cast<int>(std::nearbyint(fx * 32)),
                          qy = static_cast<int>(std::nearbyint(fy * 32));
                const int ix = static_cast<int>(std::floor(qx / 32.0)), iy = static_cast<int>(std::floor(qy / 32.0));
                const int ax = qx - ix * 32, ay = qy - iy * 32;
                for (size_t c = 0; c < channels_; ++c) {
                    const auto sample = [&](int sx, int sy) -> int {
                        return sx < 0 || sy < 0 || sx >= size.width || sy >= size.height
                                   ? 0
                                   : plane.bytes[static_cast<size_t>(sy) * plane.row_stride +
                                                 static_cast<size_t>(sx) * channels_ + c];
                    };
                    const int value = sample(ix, iy) * (32 - ax) * (32 - ay) + sample(ix + 1, iy) * ax * (32 - ay) +
                                      sample(ix, iy + 1) * (32 - ax) * ay + sample(ix + 1, iy + 1) * ax * ay;
                    pixels[row * stride + static_cast<size_t>(x) * channels_ + c] =
                        static_cast<uint8_t>((value + 512) >> 10);
                }
            }
        });
        output_ = make_host_image(size, mapping_.format, std::move(pixels));
        size_t count = 0;
        for (auto n : invalid)
            count += n;
        return double(count) / (static_cast<size_t>(size.width) * size.height);
    }
    Image download() override {
        execution_.check();
        if (!output_)
            throw std::logic_error("No rendered image");
        return output_;
    }

  private:
    ExecutionSettings execution_;
    HostImageView mapping_{};
    Image output_;
    size_t channels_ = 0;
};
} // namespace
std::unique_ptr<RenderBackend> make_cpu_backend(ExecutionSettings execution) {
    return std::make_unique<CpuBackend>(execution);
}
} // namespace stillflow
