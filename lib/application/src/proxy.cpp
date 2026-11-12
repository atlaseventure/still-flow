#include "stillflow/application/image.hpp"
#include <algorithm>
#include <cmath>

namespace stillflow {
namespace {
struct AreaSpan {
    int first, end;
    size_t offset;
};
struct AreaAxis {
    std::vector<AreaSpan> spans;
    std::vector<double> weights;
    AreaAxis(int source, int target) {
        const double scale = double(source) / target;
        spans.reserve(target);
        weights.reserve(static_cast<size_t>(source) + target);
        for (int at = 0; at < target; ++at) {
            const double lo = at * scale, hi = (at + 1) * scale;
            const int first = int(lo), end = std::min(source, int(std::ceil(hi)));
            spans.push_back({first, end, weights.size()});
            for (int sample = first; sample < end; ++sample)
                weights.push_back(std::min(hi, double(sample + 1)) - std::max(lo, double(sample)));
        }
    }
};
} // namespace
Image gray_proxy(const Image& input, Size size) {
    if (size.width < 2 || size.height < 2)
        throw std::invalid_argument("Invalid analysis size");
    auto view = map_host_image(input);
    if (view.format == PixelFormat::gray8 || view.format == PixelFormat::nv12 || view.format == PixelFormat::nv21 ||
        view.format == PixelFormat::i420) {
        if (size.width == view.size.width && size.height == view.size.height)
            return luma_view(input);
    }
    const auto& plane = view.planes[0];
    const auto sample = [&](int x, int y) -> double {
        const auto* row = plane.bytes.data() + static_cast<size_t>(y) * plane.row_stride;
        if (view.format == PixelFormat::bgr8)
            return (1868 * row[static_cast<size_t>(x) * 3] + 9617 * row[static_cast<size_t>(x) * 3 + 1] +
                    4899 * row[static_cast<size_t>(x) * 3 + 2] + 8192) >>
                   14;
        if (view.format == PixelFormat::rgba8)
            return (4899 * row[static_cast<size_t>(x) * 4] + 9617 * row[static_cast<size_t>(x) * 4 + 1] +
                    1868 * row[static_cast<size_t>(x) * 4 + 2] + 8192) >>
                   14;
        return row[x];
    };
    std::vector<uint8_t> pixels(static_cast<size_t>(size.width) * size.height);
    if (view.format != PixelFormat::bgr8 && view.format != PixelFormat::rgba8 && view.size.width == size.width * 2 &&
        view.size.height == size.height * 2) {
        for (int y = 0; y < size.height; ++y) {
            const auto* a = plane.bytes.data() + static_cast<size_t>(2 * y) * plane.row_stride;
            const auto* b = a + plane.row_stride;
            auto* out = pixels.data() + static_cast<size_t>(y) * size.width;
            for (size_t x = 0; x < static_cast<size_t>(size.width); ++x)
                out[x] = static_cast<uint8_t>((unsigned(a[2 * x]) + a[2 * x + 1] + b[2 * x] + b[2 * x + 1] + 2) >> 2);
        }
        return make_host_image(size, PixelFormat::gray8, std::move(pixels));
    }
    // Area sampling preserves thin features when downscaling camera luma.
    const double sx = double(view.size.width) / size.width, sy = double(view.size.height) / size.height;
    const AreaAxis horizontal(view.size.width, size.width), vertical(view.size.height, size.height);
    for (int y = 0; y < size.height; ++y)
        for (int x = 0; x < size.width; ++x) {
            const auto& xs = horizontal.spans[x];
            const auto& ys = vertical.spans[y];
            double sum = 0;
            // Keep the original accumulation order and rounding while reusing axis weights.
            for (int iy = ys.first; iy < ys.end; ++iy)
                for (int ix = xs.first; ix < xs.end; ++ix)
                    sum += sample(ix, iy) * horizontal.weights[xs.offset + ix - xs.first] *
                           vertical.weights[ys.offset + iy - ys.first];
            pixels[static_cast<size_t>(y) * size.width + x] =
                static_cast<uint8_t>(std::clamp(std::lround(sum / (sx * sy)), 0L, 255L));
        }
    return make_host_image(size, PixelFormat::gray8, std::move(pixels));
}
} // namespace stillflow
