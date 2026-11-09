#pragma once
#include "stillflow/domain/profile.hpp"
#include "stillflow/domain/types.hpp"

namespace stillflow {
std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size analysis_size,
                             const StabilizationSettings& options, bool horizontal_scan, Profile& profile);

// Maps output pixel centers into the original input in one resampling step.
struct MapParameters {
    double origin_x = 0, origin_y = 0;
    double xx = 0, xy = 0, yx = 0, yy = 0;
    double scan[32]{}, offset_x[32]{}, offset_y[32]{};
    int bands = 0;
    bool horizontal = false;
};
MapParameters prepare_map(const Warp& warp, Size analysis_size, Size output_size, double crop, bool horizontal_scan);

} // namespace stillflow
