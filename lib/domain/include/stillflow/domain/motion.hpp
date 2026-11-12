#pragma once
#include "stillflow/domain/types.hpp"
#include <memory>
#include <span>

namespace stillflow {
// Per-stream forward trajectory state. Unchanged evidence is reused after exact FIR support.
class TrajectoryCache {
  public:
    struct Impl;
    TrajectoryCache();
    ~TrajectoryCache();
    TrajectoryCache(const TrajectoryCache&) = delete;
    TrajectoryCache& operator=(const TrajectoryCache&) = delete;
    std::unique_ptr<Impl> impl;
};
// Set first_shot_start=false when a sliding window has evicted the shot beginning.
std::vector<Warp> plan_warps(std::span<const FrameMotion> frames, Size analysis_size,
                             const StabilizationSettings& options, bool horizontal_scan, bool first_shot_start = true,
                             TrajectoryCache* cache = nullptr, size_t output_first = 0,
                             size_t output_last = std::dynamic_extent);
// Geometry-constrained, shot-local viewport and compensation planning.
void plan_crops(std::span<const FrameMotion> frames, std::span<Warp> warps, Size size,
                const StabilizationSettings& options);

// Preview-driven viewport with bounded zoom speed and acceleration. A constrained
// translation path preserves shake correction when the viewport is under pressure.
class StreamCrop {
  public:
    void apply(Warp& warp, std::span<const Warp> future, std::span<const FrameMotion> frames, Size size,
               const StabilizationSettings& settings, double dt, bool reset);

  private:
    double inset_ = 0, velocity_ = 0;
    Point previous_bias_;
    std::vector<Point> bias_;
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
