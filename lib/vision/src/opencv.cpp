#include "motion.hpp"
#include "stillflow/vision/vision.hpp"
namespace stillflow {
namespace {
class OpenCvMotionAnalyzer final : public MotionAnalyzer {
  public:
    OpenCvMotionAnalyzer(const StabilizationSettings& settings, bool horizontal) : estimator_(settings, horizontal) {}
    Motion estimate(const Image& previous, const Image& current, Profile& profile) override {
        auto before = map_host_image(previous), after = map_host_image(current);
        if (before.format != PixelFormat::gray8 || after.format != PixelFormat::gray8)
            throw std::invalid_argument("Motion estimation requires gray8");
        cv::Mat a(before.size.height, before.size.width, CV_8UC1, const_cast<uint8_t*>(before.planes[0].bytes.data()),
                  before.planes[0].row_stride);
        cv::Mat b(after.size.height, after.size.width, CV_8UC1, const_cast<uint8_t*>(after.planes[0].bytes.data()),
                  after.planes[0].row_stride);
        return estimator_.estimate(a, b, profile);
    }

  private:
    MotionEstimator estimator_;
};

} // namespace
std::unique_ptr<MotionAnalyzer> make_opencv_analyzer(const StabilizationSettings& settings, bool horizontal) {
    return std::make_unique<OpenCvMotionAnalyzer>(settings, horizontal);
}
} // namespace stillflow
