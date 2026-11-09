#pragma once
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
void build_maps(const Warp& warp, Size analysis_size, cv::Size output_size,
    double crop, bool horizontal_scan, cv::Mat& map_x, cv::Mat& map_y);
} // namespace stillflow
