#pragma once
#include "stillflow/application/rendering.hpp"
namespace stillflow {
class MotionAnalyzer {
  public:
    virtual ~MotionAnalyzer() = default;
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

// Dependency-free translation estimator for constrained hosts.
std::unique_ptr<MotionAnalyzer> make_translation_analyzer(const StabilizationSettings& settings);
// Optional OpenCV implementation of the full tracking algorithm.
std::unique_ptr<MotionAnalyzer> make_opencv_analyzer(const StabilizationSettings& settings, bool horizontal,
                                                     bool incremental = false);
Image gray_proxy(const Image& image, Size size);
} // namespace stillflow
