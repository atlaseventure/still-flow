#pragma once
#include "stillflow/domain/geometry.hpp"
#include "stillflow/domain/settings.hpp"
#include <vector>

namespace stillflow {
struct Motion {
    Matrix3 step = Matrix3::eye();
    std::vector<Point> row_delta;
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
    Matrix3 global = Matrix3::eye();
    std::vector<Point> row_offset;
    double strength = 1;
    double invalid_fraction = 0;
};

} // namespace stillflow
