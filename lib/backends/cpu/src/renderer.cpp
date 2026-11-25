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
                {{PixelFormat::gray8, PixelFormat::bgr8, PixelFormat::rgba8, PixelFormat::nv12, PixelFormat::nv21,
                  PixelFormat::i420},
                 {"host"}},
                {{"sampler", "bilinear-5bit"}}};
    }
    void upload(const Image& input) override {
        execution_.check();
        mapping_ = map_host_image(input);
        planar_ = mapping_.format == PixelFormat::nv12 || mapping_.format == PixelFormat::nv21 ||
                  mapping_.format == PixelFormat::i420;
        channels_ = planar_ ? 1 : pixel_bytes(mapping_.format);
    }
    double warp_gyro(const GyroFrameMap&) override {
        throw NotImplementedError("Gyro stabilization rendering is not implemented for CPU backend");
    }
    double warp(const MapParameters& map) override {
        if (mapping_.planes.empty())
            throw std::logic_error("No uploaded image");
        if (planar_)
            return warp_yuv(map);
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
        auto output = map_host_image(make_host_image(size, mapping_.format, std::move(pixels)));
        output.color = mapping_.color;
        output_ = borrow_host_image(std::move(output));
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
    double warp_yuv(const MapParameters& map) {
        const auto size = mapping_.size;
        const int cw = (size.width + 1) / 2, ch = (size.height + 1) / 2;
        const size_t ybytes = static_cast<size_t>(size.width) * size.height;
        const size_t cbytes = static_cast<size_t>(cw) * ch;
        auto pixels = std::make_shared<std::vector<uint8_t>>(ybytes + 2 * cbytes);
        HostImageView output{size,
                             mapping_.format,
                             {{static_cast<size_t>(size.width), {pixels->data(), ybytes}}},
                             pixels,
                             mapping_.color};
        const bool split = mapping_.format == PixelFormat::i420;
        if (split) {
            output.planes.push_back({static_cast<size_t>(cw), {pixels->data() + ybytes, cbytes}});
            output.planes.push_back({static_cast<size_t>(cw), {pixels->data() + ybytes + cbytes, cbytes}});
        } else
            output.planes.push_back({static_cast<size_t>(cw) * 2, {pixels->data() + ybytes, 2 * cbytes}});
        std::vector<size_t> invalid(size.height);
        execution_.for_each(static_cast<size_t>(size.height) + ch, [&](size_t index) {
            const int plane = index >= static_cast<size_t>(size.height);
            const size_t row = plane ? index - size.height : index;
            const int width = plane ? cw : size.width, height = plane ? ch : size.height;
            const size_t channels = plane ? 2 : 1;
            const size_t input_step = plane && !split ? 2 : 1;
            for (int x = 0; x < width; ++x) {
                const double tx = plane ? 2 * x + mapping_.color.chroma_x : x;
                const double ty = plane ? 2 * row + mapping_.color.chroma_y : row;
                auto p = source_point(map, {map.origin_x + tx * map.step_x, map.origin_y + ty * map.step_y});
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 10000000 || std::abs(p.y) > 10000000)
                    throw std::invalid_argument("Invalid sample coordinate");
                if (!plane)
                    invalid[row] += !(p.x >= 0 && p.x <= size.width - 1 && p.y >= 0 && p.y <= size.height - 1);
                else
                    p = {(p.x - mapping_.color.chroma_x) * 0.5, (p.y - mapping_.color.chroma_y) * 0.5};
                const int qx =
                    static_cast<int>(std::nearbyint(static_cast<float>(std::clamp(p.x, 0.0, double(width - 1))) * 32));
                const int qy =
                    static_cast<int>(std::nearbyint(static_cast<float>(std::clamp(p.y, 0.0, double(height - 1))) * 32));
                const int ix = qx / 32, iy = qy / 32, ax = qx % 32, ay = qy % 32;
                const int right = std::min(ix + 1, width - 1), bottom = std::min(iy + 1, height - 1);
                for (size_t c = 0; c < channels; ++c) {
                    const auto& input = mapping_.planes[plane && split ? c + 1 : plane];
                    const size_t component = plane && !split ? c : 0;
                    const auto* upper = input.bytes.data() + static_cast<size_t>(iy) * input.row_stride + component;
                    const auto* lower = input.bytes.data() + static_cast<size_t>(bottom) * input.row_stride + component;
                    const int value = upper[ix * input_step] * (32 - ax) * (32 - ay) +
                                      upper[right * input_step] * ax * (32 - ay) +
                                      lower[ix * input_step] * (32 - ax) * ay + lower[right * input_step] * ax * ay;
                    const size_t offset = !plane  ? row * size.width + x
                                          : split ? ybytes + c * cbytes + row * cw + x
                                                  : ybytes + (row * cw + x) * 2 + c;
                    (*pixels)[offset] = static_cast<uint8_t>((value + 512) >> 10);
                }
            }
        });
        output_ = borrow_host_image(std::move(output));
        size_t count = 0;
        for (auto n : invalid)
            count += n;
        return double(count) / (static_cast<double>(size.width) * size.height);
    }
    ExecutionSettings execution_;
    HostImageView mapping_{};
    Image output_;
    size_t channels_ = 0;
    bool planar_ = false;
};
} // namespace
std::unique_ptr<RenderBackend> make_cpu_backend(ExecutionSettings execution) {
    return std::make_unique<CpuBackend>(execution);
}
} // namespace stillflow
