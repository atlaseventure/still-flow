#pragma once
#include "stillflow/application/profile.hpp"
#include "stillflow/domain/motion.hpp"
#include <opencv2/core.hpp>
namespace stillflow {
struct FeatureFrame {
    std::vector<cv::Mat> pyramid;
    std::vector<cv::Point2f> corners;
    std::vector<cv::KeyPoint> keys;
    cv::Mat descriptors;
    bool corners_ready = false, descriptors_ready = false;
};
class MotionEstimator {
  public:
    MotionEstimator(const StabilizationSettings& options, bool horizontal_scan, bool incremental = false);
    Motion estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile, bool consecutive = false,
                    const Motion* prediction = nullptr, FeatureFrame* before = nullptr, FeatureFrame* after = nullptr);

  private:
    StabilizationSettings options_;
    bool horizontal_, incremental_;
    size_t pairs_ = 0;
    std::vector<cv::Mat> pyramid_, before_;
    std::vector<cv::Point2f> tracked_;
    std::vector<cv::KeyPoint> descriptor_keys_;
    cv::Mat descriptors_;
    bool descriptors_ready_ = false;
};
} // namespace stillflow
