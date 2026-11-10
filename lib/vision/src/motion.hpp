#pragma once
#include "stillflow/application/profile.hpp"
#include "stillflow/domain/motion.hpp"
#include <opencv2/core.hpp>
namespace stillflow {
class MotionEstimator {
  public:
    MotionEstimator(const StabilizationSettings& options, bool horizontal_scan);
    Motion estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile);

  private:
    StabilizationSettings options_;
    bool horizontal_;
};
} // namespace stillflow
