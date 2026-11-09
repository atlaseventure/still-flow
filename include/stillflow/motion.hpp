#pragma once
#include "stillflow/types.hpp"
#include "stillflow/profile.hpp"

namespace stillflow {
class MotionEstimator {
public:
    MotionEstimator(const Options& options, bool horizontal_scan);
    Motion estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile);
private:
    Options options_;
    bool horizontal_;
};

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames,
    cv::Size analysis_size, const Options& options, bool horizontal_scan, Profile& profile);

// Maps output pixel centers into the original input in one resampling step.
struct MapParameters {
    double origin_x = 0, origin_y = 0;
    double xx = 0, xy = 0, yx = 0, yy = 0;
    double scan[32]{}, offset_x[32]{}, offset_y[32]{};
    int bands = 0;
    bool horizontal = false;
};
MapParameters prepare_map(const Warp& warp, cv::Size analysis_size,
    cv::Size output_size, double crop, bool horizontal_scan);

void build_maps(const Warp& warp, cv::Size analysis_size, cv::Size output_size,
    double crop, bool horizontal_scan, cv::Mat& map_x, cv::Mat& map_y);
} // namespace stillflow
