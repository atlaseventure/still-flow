#pragma once

namespace stillflow {
struct StabilizationSettings {
    int analysis_width = 960;
    int features = 1200;
    int bands = 12;
    double smooth_seconds = 0.45;
    double rs_seconds = 0.30;
    double crop = 0.08; // Fraction removed from EACH edge; fixed across the clip.
    double rs_strength = 1.0;
    bool rolling_shutter = true;
};
} // namespace stillflow
