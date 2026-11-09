#pragma once

#include <opencv2/core.hpp>
#include <filesystem>
#include <string>
#include <vector>

namespace stillflow {

struct Options {
    std::filesystem::path input, output, compare, report;
    int analysis_width = 960;
    int features = 1200;
    int bands = 12;
    int threads = 12;
    int crf = 18;
    double smooth_seconds = 0.45;
    double rs_seconds = 0.30;
    double crop = 0.08; // Fraction removed from EACH edge; fixed across the clip.
    double rs_strength = 1.0;
    std::string scan_axis = "auto";
    std::string preset = "fast";
    bool rolling_shutter = true;
    bool audio = true;
    bool overwrite = false;
};

struct Motion {
    cv::Matx33d step = cv::Matx33d::eye();
    std::vector<cv::Point2d> row_delta;
    double confidence = 0;
    double rs_confidence = 0;
    double residual_px = 0;
    int tracks = 0;
    bool reset = false;
};

struct FrameMotion {
    double seconds = 0;
    double duration = 0;
    Motion motion;
    int segment = 0;
};

struct Warp {
    // Forward mapping from input to stable image, before the fixed output crop.
    cv::Matx33d global = cv::Matx33d::eye();
    std::vector<cv::Point2d> row_offset;
    double strength = 1;
    double invalid_fraction = 0;
};

inline cv::Point2d transform(const cv::Matx33d& m, cv::Point2d p) {
    const auto v = m * cv::Vec3d(p.x, p.y, 1);
    return {v[0] / v[2], v[1] / v[2]};
}

} // namespace stillflow
