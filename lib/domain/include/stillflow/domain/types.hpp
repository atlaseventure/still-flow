#pragma once
#include "stillflow/domain/geometry.hpp"
#include "stillflow/domain/settings.hpp"
#include <string>
#include <vector>

namespace stillflow {
enum class MotionState { tracked, global_fallback, descriptor_fallback, missing, bridged, cut, time_gap };
inline const char* motion_state_name(MotionState state) {
    switch (state) {
    case MotionState::tracked:
        return "tracked";
    case MotionState::global_fallback:
        return "global_fallback";
    case MotionState::descriptor_fallback:
        return "descriptor_fallback";
    case MotionState::missing:
        return "missing";
    case MotionState::bridged:
        return "bridged";
    case MotionState::cut:
        return "cut";
    case MotionState::time_gap:
        return "time_gap";
    }
    return "unknown";
}
struct Motion {
    Matrix3 step = Matrix3::eye();
    std::vector<Point> row_delta;
    double confidence = 0;
    double rs_confidence = 0;
    double residual_px = 0;
    int tracks = 0;
    // Only cuts and timestamp gaps start a new trajectory. Missing observations
    // remain in the current shot so recovery can preserve temporal continuity.
    bool reset = false;
    MotionState state = MotionState::tracked;
    std::string failure;
    double fit_confidence = 0;
    double coverage = 0;
    double match_reliability = 0;
    double model_consistency = 0;
    double spatial_span = 0;
    double residual_normalized = 0;
    int temporal_links = 0;
    double temporal_residual = 0;
};

struct FrameMotion {
    double seconds = 0;
    double duration = 0;
    Motion motion;
    int segment = 0;
};

struct Warp {
    // Forward displacement S - O on a regular grid spanning the analysis image.
    int columns = 0, rows = 0;
    std::vector<Point> displacement;
    Point global_offset;
    double row_correction_rms = 0;
    double strength = 1;
    double crop_strength = 1;
    double tracking_strength = 1;
    double invalid_fraction = 0;
};

} // namespace stillflow
