#pragma once
#include "stillflow/domain/motion.hpp"
#include <cmath>
#include <vector>

inline std::vector<stillflow::FrameMotion> trajectory_fixture() {
    std::vector<stillflow::FrameMotion> frames(36);
    for (size_t i = 0; i < frames.size(); ++i) {
        auto& f = frames[i];
        f.seconds = i / 30.0 + (i > 20 ? 0.013 : 0);
        f.duration = 1 / 30.0;
        f.segment = i >= 24 ? 1 : 0;
        auto& m = f.motion;
        m.step(0, 2) = i ? 2 * std::sin(i * 1.3) + 0.2 : 0;
        m.step(1, 2) = i ? std::cos(i * 0.8) : 0;
        m.confidence = 0.9;
        m.rs_confidence = 0.8;
        for (int row = 0; row < 6; ++row)
            m.row_delta.push_back({0.3 * std::sin(i + row), 0.2 * std::cos(i - row)});
        if (i == 10 || i == 11) {
            m.state = stillflow::MotionState::missing;
            m.confidence = m.rs_confidence = 0;
        }
        if (i == 24) {
            m.state = stillflow::MotionState::cut;
            m.reset = true;
        }
    }
    return frames;
}
inline std::vector<double> golden_values() {
    std::vector<double> values;
    for (const auto mode : {stillflow::CropMode::automatic, stillflow::CropMode::fixed, stillflow::CropMode::adaptive})
        for (bool horizontal : {false, true}) {
            stillflow::StabilizationSettings settings;
            settings.bands = 6;
            settings.crop_mode = mode;
            auto frames = trajectory_fixture();
            auto warps = stillflow::plan_warps(frames, {320, 180}, settings, horizontal);
            for (const auto& w : warps) {
                for (auto p : w.displacement) {
                    values.push_back(p.x);
                    values.push_back(p.y);
                }
                for (auto v : {w.global_offset.x, w.global_offset.y, w.row_correction_rms, w.strength,
                               w.tracking_strength, w.crop_strength, w.window.left, w.window.top, w.window.scale})
                    values.push_back(v);
            }
        }
    return values;
}
