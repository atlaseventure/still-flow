#pragma once

namespace stillflow {
enum class CropMode { automatic, fixed, adaptive };
inline const char* crop_mode_name(CropMode mode) {
    switch (mode) {
    case CropMode::automatic:
        return "auto";
    case CropMode::fixed:
        return "fixed";
    case CropMode::adaptive:
        return "adaptive";
    }
    return "unknown";
}
struct StabilizationSettings {
    int analysis_width = 960;
    int features = 1200;
    int bands = 12;
    double smooth_seconds = 0.45;
    double rs_seconds = 0.30;
    double crop = 0.08; // Maximum equivalent per-edge crop; exact in fixed mode.
    CropMode crop_mode = CropMode::automatic;
    double rs_strength = 1.0;
    bool rolling_shutter = true;
};
} // namespace stillflow
