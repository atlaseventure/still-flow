#include "stillflow/backends/cpu.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>

namespace stillflow {
namespace {
void validate_gyro_map(const GyroFrameMap& map, Size uploaded) {
    const auto finite = [](double v) { return std::isfinite(static_cast<float>(v)); };
    const auto focal = [&](double v) { return finite(v) && static_cast<float>(v) > 0; };
    const auto& lens = map.lens;
    if (uploaded.width < 2 || uploaded.height < 2 || map.input_size.width != uploaded.width ||
        map.input_size.height != uploaded.height || map.output_size.width < 2 || map.output_size.height < 2 ||
        map.row_rotations.empty() || map.row_rotations.size() > std::numeric_limits<int>::max() / 9 ||
        !std::ranges::all_of(
            map.row_rotations, [&](const Matrix3& m) { return std::ranges::all_of(m.values, finite); }) ||
        !focal(map.out_fx) || !focal(map.out_fy) || !focal(lens.fx) || !focal(lens.fy) || !finite(map.out_cx) ||
        !finite(map.out_cy) || !finite(lens.cx) || !finite(lens.cy) || !std::ranges::all_of(lens.k, finite) ||
        !(lens.lens_correction_amount >= 0 && lens.lens_correction_amount <= 1) ||
        (lens.model != LensModel::none && lens.model != LensModel::opencv_fisheye))
        throw std::invalid_argument("Invalid CPU gyro map");
}

std::array<double, 8> lanczos_weights(double t) {
    const bool reverse = t > 0.5;
    t = reverse ? 1 - t : t;
    std::array<double, 8> weights{};
    if (t < 1e-7) {
        weights[reverse ? 4 : 3] = 1;
        return weights;
    }
    const double angle = t * (std::numbers::pi / 4);
    const double sine = std::sin(angle), cosine = std::cos(angle);
    constexpr double diagonal = std::numbers::sqrt2 / 2;
    const std::array phases{sine, -(sine + cosine) * diagonal, cosine, (sine - cosine) * diagonal};
    double sum = 0;
    for (int i = 0; i < 8; ++i) {
        const int shift = 3 - i;
        const double d = t + shift;
        const double w = phases[(shift + 4) % 4] * (shift < 0 ? -1 : 1) / (d * d);
        weights[reverse ? 7 - i : i] = w;
        sum += w;
    }
    for (double& w : weights)
        w /= sum;
    return weights;
}

uint8_t round_byte(double value) { return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0, 255.0))); }

void sample_lanczos(const ImagePlane& plane, Size size, size_t channels, Point p, uint8_t* output) {
    const auto ix = static_cast<int>(p.x), iy = static_cast<int>(p.y);
    if (p.x == ix && p.y == iy) {
        std::copy_n(plane.bytes.data() + iy * plane.row_stride + ix * channels, channels, output);
        return;
    }
    const auto wx = lanczos_weights(p.x - ix), wy = lanczos_weights(p.y - iy);
    std::array<size_t, 8> offsets;
    for (int i = 0; i < 8; ++i)
        offsets[i] = std::clamp<int64_t>(int64_t{ix} + i - 3, 0, size.width - 1) * channels;
    std::array<double, 4> sums{};
    for (int j = 0; j < 8; ++j) {
        const auto y = std::clamp<int64_t>(int64_t{iy} + j - 3, 0, size.height - 1);
        const auto* row = plane.bytes.data() + y * plane.row_stride;
        for (size_t c = 0; c < channels; ++c) {
            double line = 0;
            for (int i = 0; i < 8; ++i)
                line += wx[i] * row[offsets[i] + c];
            sums[c] += line * wy[j];
        }
    }
    for (size_t c = 0; c < channels; ++c)
        output[c] = round_byte(sums[c]);
}

Size chroma_size(Size size) { return {size.width / 2 + size.width % 2, size.height / 2 + size.height % 2}; }

uint8_t sample_chroma(const ImagePlane& plane, Size size, size_t step, size_t component, Point p) {
    const double x = std::clamp(p.x, 0.0, double(size.width - 1)), y = std::clamp(p.y, 0.0, double(size.height - 1));
    const auto ix = static_cast<int>(x), iy = static_cast<int>(y);
    const auto right = std::min(ix + 1, size.width - 1);
    const auto* upper = plane.bytes.data() + iy * plane.row_stride + component;
    const auto* lower = plane.bytes.data() + std::min(iy + 1, size.height - 1) * plane.row_stride + component;
    return round_byte(std::lerp(std::lerp(upper[ix * step], upper[right * step], x - ix),
                                std::lerp(lower[ix * step], lower[right * step], x - ix), y - iy));
}

Image yuv_image(Size size, PixelFormat format, ColorInfo color, const std::shared_ptr<std::vector<uint8_t>>& pixels) {
    const auto [cw, ch] = chroma_size(size);
    const size_t ybytes = static_cast<size_t>(size.width) * size.height, cbytes = static_cast<size_t>(cw) * ch;
    HostImageView output{size, format, {{static_cast<size_t>(size.width), {pixels->data(), ybytes}}}, pixels, color};
    const bool split = format == PixelFormat::i420;
    if (split) {
        output.planes.push_back({static_cast<size_t>(cw), {pixels->data() + ybytes, cbytes}});
        output.planes.push_back({static_cast<size_t>(cw), {pixels->data() + ybytes + cbytes, cbytes}});
    } else
        output.planes.push_back({static_cast<size_t>(cw) * 2, {pixels->data() + ybytes, 2 * cbytes}});
    return borrow_host_image(std::move(output));
}

class CpuBackend final : public RenderBackend {
  public:
    explicit CpuBackend(ExecutionSettings execution) : execution_(execution) {}
    BackendInfo info() const override {
        return {"cpu",
                "host",
                {{PixelFormat::gray8, PixelFormat::bgr8, PixelFormat::rgba8, PixelFormat::nv12, PixelFormat::nv21,
                  PixelFormat::i420},
                 {"host"}},
                {{"sampler", "visual bilinear-5bit, gyro Lanczos4 luma/RGB, bilinear chroma"}}};
    }
    void upload(const Image& input) override {
        execution_.check();
        mapping_ = map_host_image(input);
        planar_ = mapping_.format == PixelFormat::nv12 || mapping_.format == PixelFormat::nv21 ||
                  mapping_.format == PixelFormat::i420;
        channels_ = planar_ ? 1 : pixel_bytes(mapping_.format);
    }
    double warp_gyro(const GyroFrameMap& map) override {
        if (mapping_.planes.empty())
            throw std::invalid_argument("No uploaded CPU gyro input");
        execution_.check();
        validate_gyro_map(map, mapping_.size);
        const auto size = map.output_size;
        const auto [cw, ch] = chroma_size(size);
        const auto input_chroma = chroma_size(mapping_.size);
        const size_t stride = static_cast<size_t>(size.width) * channels_, plane_bytes = stride * size.height;
        const size_t cbytes = planar_ ? static_cast<size_t>(cw) * ch : 0;
        auto pixels = std::make_shared<std::vector<uint8_t>>(plane_bytes + 2 * cbytes);
        const auto color = mapping_.color;
        const bool split = mapping_.format == PixelFormat::i420;
        std::vector<size_t> invalid(size.height);
        execution_.for_each(size.height, [&](size_t row) {
            size_t count = 0;
            for (int x = 0; x < size.width; ++x) {
                Point p;
                bool valid = project_output_to_input(map, {double(x), double(row)}, p);
                auto* dst = pixels->data() + row * stride + x * channels_;
                if (valid)
                    sample_lanczos(mapping_.planes.front(), mapping_.size, channels_, p, dst);
                else
                    std::fill_n(dst, channels_, planar_ && !color.full_range ? 16 : 0);

                if (planar_ && row % 2 == 0 && x % 2 == 0) {
                    const Point site{std::min(x + double(color.chroma_x), double(size.width - 1)),
                                     std::min(row + double(color.chroma_y), double(size.height - 1))};
                    const bool chroma_valid = project_output_to_input(map, site, p);
                    if (chroma_valid)
                        p = {(p.x - color.chroma_x) * 0.5, (p.y - color.chroma_y) * 0.5};
                    const size_t index = row / 2 * cw + x / 2;
                    for (size_t c = 0; c < 2; ++c) {
                        const auto offset = split ? c * cbytes + index : 2 * index + c;
                        (*pixels)[plane_bytes + offset] =
                            chroma_valid ? sample_chroma(mapping_.planes[split ? 1 + c : 1], input_chroma,
                                                         split ? 1 : 2, split ? 0 : c, p)
                                         : 128;
                    }
                    valid = valid && chroma_valid;
                }
                count += !valid;
            }
            invalid[row] = count;
        });
        if (planar_)
            output_ = yuv_image(size, mapping_.format, color, pixels);
        else
            output_ = borrow_host_image({size, mapping_.format, {{stride, *pixels}}, pixels, color});
        return double(std::accumulate(invalid.begin(), invalid.end(), size_t{0})) / (double(size.width) * size.height);
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
        const bool split = mapping_.format == PixelFormat::i420;
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
        output_ = yuv_image(size, mapping_.format, mapping_.color, pixels);
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
