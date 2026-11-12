#include "stillflow/vision/portable.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace stillflow {
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
