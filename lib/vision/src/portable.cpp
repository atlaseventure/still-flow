#include "stillflow/vision/vision.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace stillflow {
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
    for (int y = 0; y < size.height; ++y)
        for (int x = 0; x < size.width; ++x) {
            const double left = x * sx, right = (x + 1) * sx, top = y * sy, bottom = (y + 1) * sy;
            double sum = 0;
            for (int iy = int(top); iy < std::min(view.size.height, int(std::ceil(bottom))); ++iy)
                for (int ix = int(left); ix < std::min(view.size.width, int(std::ceil(right))); ++ix)
                    sum += sample(ix, iy) * (std::min(right, double(ix + 1)) - std::max(left, double(ix))) *
                           (std::min(bottom, double(iy + 1)) - std::max(top, double(iy)));
            pixels[static_cast<size_t>(y) * size.width + x] =
                static_cast<uint8_t>(std::clamp(std::lround(sum / (sx * sy)), 0L, 255L));
        }
    return make_host_image(size, PixelFormat::gray8, std::move(pixels));
}
namespace {
class TranslationAnalyzer final : public MotionAnalyzer {
  public:
    explicit TranslationAnalyzer(int bands) : bands_(bands) {}
    Motion estimate(const Image& previous, const Image& current, Profile&) override {
        auto a = map_host_image(previous), b = map_host_image(current);
        if (a.format != PixelFormat::gray8 || b.format != PixelFormat::gray8 || a.size.width != b.size.width ||
            a.size.height != b.size.height || a.size.width < 32 || a.size.height < 32)
            throw std::invalid_argument("Translation estimation requires equal gray8 images of at least 32 pixels");
        const auto& ap = a.planes[0];
        const auto& bp = b.planes[0];
        const int radius = std::min({12, a.size.width / 4, a.size.height / 4});
        const int stride = std::max(1, std::max(a.size.width, a.size.height) / 96);
        double best = std::numeric_limits<double>::infinity(), second = best;
        int dx = 0, dy = 0, count = 0;
        double mean = 0, square = 0;
        for (int y = radius; y < a.size.height - radius; y += stride)
            for (int x = radius; x < a.size.width - radius; x += stride) {
                double value = ap.bytes[static_cast<size_t>(y) * ap.row_stride + x];
                mean += value;
                square += value * value;
                ++count;
            }
        for (int oy = -radius; oy <= radius; ++oy)
            for (int ox = -radius; ox <= radius; ++ox) {
                double error = 0;
                for (int y = radius; y < a.size.height - radius; y += stride)
                    for (int x = radius; x < a.size.width - radius; x += stride) {
                        const double delta = double(ap.bytes[static_cast<size_t>(y) * ap.row_stride + x]) -
                                             bp.bytes[static_cast<size_t>(y + oy) * bp.row_stride + x + ox];
                        error += delta * delta;
                    }
                error /= count;
                if (error < best) {
                    second = best;
                    best = error;
                    dx = ox;
                    dy = oy;
                } else
                    second = std::min(second, error);
            }
        Motion motion;
        motion.row_delta.resize(bands_);
        motion.residual_px = std::sqrt(best);
        if (square / count - std::pow(mean / count, 2) < 16 || second <= best * 1.05 || best > 1600) {
            motion.state = MotionState::missing;
            motion.failure = "translation_unreliable";
            return motion;
        }
        motion.step(0, 2) = dx;
        motion.step(1, 2) = dy;
        motion.confidence = std::clamp(1 - best / 1600, 0.0, 1.0);
        motion.state = MotionState::global_fallback;
        motion.tracks = count;
        return motion;
    }

  private:
    int bands_;
};
} // namespace
std::unique_ptr<MotionAnalyzer> make_translation_analyzer(const StabilizationSettings& settings) {
    return std::make_unique<TranslationAnalyzer>(settings.bands);
}
} // namespace stillflow
