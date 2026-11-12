#include "stillflow/vision/opencv.hpp"
#include "motion.hpp"
#include <deque>
namespace stillflow {
namespace {
class OpenCvMotionAnalyzer final : public MotionAnalyzer {
  public:
    OpenCvMotionAnalyzer(const StabilizationSettings& settings, bool horizontal, bool incremental,
                         std::unique_ptr<SparseTracker> tracker)
        : incremental_(incremental), estimator_(settings, horizontal, incremental, std::move(tracker)) {}
    void adaptive_tracking(bool enabled) override { estimator_.adaptive_tracking(enabled); }
    Motion estimate(const Image& previous, const Image& current, Profile& profile) override {
        auto before = map_host_image(previous), after = map_host_image(current);
        if (before.format != PixelFormat::gray8 || after.format != PixelFormat::gray8)
            throw std::invalid_argument("Motion estimation requires gray8");
        cv::Mat a(before.size.height, before.size.width, CV_8UC1, const_cast<uint8_t*>(before.planes[0].bytes.data()),
                  before.planes[0].row_stride);
        cv::Mat b(after.size.height, after.size.width, CV_8UC1, const_cast<uint8_t*>(after.planes[0].bytes.data()),
                  after.planes[0].row_stride);
        if (!incremental_ && frames_.size() > 6) {
            frames_.pop_front();
            frames_.pop_front();
        }
        auto* first = incremental_ ? nullptr : &features(previous);
        auto* second = incremental_ ? nullptr : &features(current);
        auto result = estimator_.estimate(a, b, profile, previous == cached_, &prediction_, first, second);
        prediction_ = result;
        cached_ = current;
        return result;
    }

  private:
    struct CachedFrame {
        Image image;
        FeatureFrame features;
    };
    std::deque<CachedFrame> frames_;
    bool incremental_;
    FeatureFrame& features(const Image& image) {
        for (auto& frame : frames_)
            if (frame.image == image)
                return frame.features;
        // Each entry owns the image backing cached OpenCV pyramid views.
        // Leave room for both endpoints of the current pair before eviction.
        if (frames_.size() == 8)
            frames_.pop_front();
        frames_.push_back({image, {}});
        return frames_.back().features;
    }
    MotionEstimator estimator_;
    Image cached_;
    Motion prediction_;
};

} // namespace
std::unique_ptr<MotionAnalyzer> make_opencv_analyzer(const StabilizationSettings& settings, bool horizontal,
                                                     bool incremental, std::unique_ptr<SparseTracker> tracker) {
    return std::make_unique<OpenCvMotionAnalyzer>(settings, horizontal, incremental, std::move(tracker));
}
void configure_opencv_threads(int threads) {
    if (threads < 1 || threads > 64)
        throw std::invalid_argument("Invalid OpenCV thread count");
    cv::setNumThreads(threads);
}
} // namespace stillflow
