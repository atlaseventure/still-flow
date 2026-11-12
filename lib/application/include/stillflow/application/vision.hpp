#pragma once
#include "stillflow/application/image.hpp"
#include "stillflow/application/profile.hpp"
#include "stillflow/domain/motion.hpp"
#include "stillflow/domain/settings.hpp"

namespace stillflow {
class MotionAnalyzer {
  public:
    virtual ~MotionAnalyzer() = default;
    virtual void adaptive_tracking(bool) {}
    virtual Motion estimate(const Image& previous, const Image& current, Profile& profile) = 0;
};
class ImageProcessing {
  public:
    virtual ~ImageProcessing() = default;
    virtual std::unique_ptr<MotionAnalyzer> analyzer(const StabilizationSettings& settings, bool horizontal) = 0;
    virtual std::unique_ptr<MotionAnalyzer> stream_analyzer(const StabilizationSettings& settings, bool horizontal) {
        return analyzer(settings, horizontal);
    }
    virtual Image gray_proxy(const Image& image, Size size) = 0;
    virtual Image comparison(const Image& input, const Image& stabilized) = 0;
};

} // namespace stillflow
