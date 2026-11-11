#pragma once
#include "stillflow/domain/types.hpp"
#include <span>

namespace stillflow {
// Set first_shot_start=false when a sliding window has evicted the shot beginning.
std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size analysis_size,
                             const StabilizationSettings& options, bool horizontal_scan, bool first_shot_start = true);
// Geometry-constrained, shot-local viewport and compensation planning.
void plan_crops(std::span<const FrameMotion> frames, std::span<Warp> warps, Size size,
                const StabilizationSettings& options);

// Stateful viewport follower. Two convex low-pass stages bound viewport
// velocity and acceleration while preserving crop budget and image bounds.
class StreamCrop {
  public:
    void apply(Warp& warp, std::span<const Warp> future, Size size, const StabilizationSettings& settings, double dt,
               bool reset);

  private:
    CropWindow target_, window_;
    double strength_ = 1;
    bool initialized_ = false;
};

struct TemporalLink {
    size_t from = 0, to = 0;
    Motion motion;
};
// Fuses measured connections within one short, continuous shot interval.
bool recover_motion(std::span<FrameMotion> frames, const std::vector<TemporalLink>& links, Size analysis_size);

inline constexpr int max_mesh_vertices = 9 * 32;
inline constexpr int mesh_inverse_iterations = 8;
inline constexpr int trajectory_smoothing_passes = 50;

// Forward mesh in output-resolution pixels; invert before the single resampling.
struct MapParameters {
    double origin_x = 0, origin_y = 0;
    double step_x = 1, step_y = 1, grid_x = 0, grid_y = 0;
    double offset_x[max_mesh_vertices]{}, offset_y[max_mesh_vertices]{};
    int columns = 0, rows = 0;
};
Point source_point(const MapParameters& map, Point target);
MapParameters prepare_map(const Warp& warp, Size analysis_size, Size output_size, double crop, bool horizontal_scan);

} // namespace stillflow
