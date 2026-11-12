#include "stillflow/application/engine.hpp"
#include "recovery.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace stillflow {
void validate_settings(const StabilizationSettings& s) {
    auto range = [](double value, double low, double high) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument("Invalid stabilization settings");
    };
    range(s.analysis_width, 32, 4096);
    range(s.features, 96, 10000);
    range(s.bands, 4, 32);
    range(s.smooth_seconds, 0, 10);
    range(s.rs_seconds, 0, 10);
    range(s.rs_strength, 0, 1);
    range(s.crop, 0, 0.25);
    if (s.crop_mode != CropMode::automatic && s.crop_mode != CropMode::fixed && s.crop_mode != CropMode::adaptive)
        throw std::invalid_argument("Invalid crop mode");
}
Plan Planner::plan(AnalysisResult analysis, StabilizationSettings settings, ExecutionSettings execution) {
    validate_settings(settings);
    execution.check();
    if (analysis.frames.empty() || analysis.size.width < 32 || analysis.size.height < 32)
        throw std::invalid_argument("Empty or invalid analysis");
    if (execution.logger)
        execution.logger->log("Planning stabilization");
    auto warps = plan_warps(analysis.frames, analysis.size, settings, analysis.horizontal);
    execution.check();
    return {std::move(analysis), settings, std::move(warps)};
}
Analyzer::Analyzer(StabilizationSettings settings, Size size, double fps, bool horizontal,
                   std::unique_ptr<MotionAnalyzer> motion, ExecutionSettings execution)
    : settings_(settings), result_{size, horizontal, 0, {}}, fps_(fps), motion_(std::move(motion)),
      execution_(execution) {
    validate_settings(settings_);
    if (size.width < 32 || size.height < 32 || size.width > 4096 || size.height > 4096 || !std::isfinite(fps) ||
        fps <= 0)
        throw std::invalid_argument("Invalid analysis geometry or frame rate");
    execution_.check();
}
Analyzer::~Analyzer() = default;
std::optional<RecoveryFrame> Analyzer::next_recovery_frame() {
    execution_.check();
    if (finished_ || result_.frames.empty())
        throw std::logic_error("Analyzer is empty or already finished");
    if (!recovery_) {
        recovery_ = std::make_unique<TrackingRecovery>(result_);
        previous_.reset();
    }
    return recovery_->next();
}
void Analyzer::recover(const Image& image, int64_t pts, Profile& profile) {
    execution_.check();
    if (finished_ || !recovery_ || !motion_)
        throw std::logic_error("Request recovery frames before replaying them");
    recovery_->push(image, pts, *motion_, profile, execution_);
}
void Analyzer::push(const Image& image, int64_t pts, int64_t duration, Profile& profile) {
    execution_.check();
    if (finished_ || recovery_)
        throw std::logic_error("Analyzer input is sealed");
    if ((!result_.frames.empty() && pts <= last_pts_) || duration < 0)
        throw std::invalid_argument("Invalid analyzer timestamp or duration");
    auto current = gray_proxy(image, result_.size);
    Motion observation;
    observation.row_delta.resize(settings_.bands);
    if (previous_ && (static_cast<long double>(pts) - last_pts_) / 1000000 <= std::max(0.5, 5 / fps_)) {
        if (!motion_)
            throw std::logic_error("No motion estimator registered");
        execution_.for_each(1, [&](size_t) { observation = motion_->estimate(previous_, current, profile); });
    }
    push_observation(std::move(observation), pts, duration);
    previous_ = std::move(current);
}
void Analyzer::push_observation(Motion motion, int64_t pts, int64_t duration) {
    execution_.check();
    if (finished_ || recovery_)
        throw std::logic_error("Analyzer input is sealed");
    if (duration < 0 || (!result_.frames.empty() && pts <= last_pts_))
        throw std::invalid_argument("Invalid analyzer timestamp or duration");
    if (result_.frames.empty())
        result_.origin_us = pts;
    const double seconds = static_cast<double>((static_cast<long double>(pts) - result_.origin_us) / 1000000);
    int segment = result_.frames.empty() ? 0 : result_.frames.back().segment;
    if (!result_.frames.empty()) {
        const double gap = seconds - result_.frames.back().seconds;
        result_.frames.back().duration = gap;
        if (gap > std::max(0.5, 5 / fps_)) {
            motion.reset = true;
            motion.state = MotionState::time_gap;
            motion.failure = "timestamp_gap";
        }
        if (motion.reset)
            ++segment;
    }
    result_.frames.push_back({seconds, duration / 1000000.0, std::move(motion), segment});
    last_pts_ = pts;
}
AnalysisResult Analyzer::finish_analysis() {
    execution_.check();
    if (finished_ || result_.frames.empty())
        throw std::logic_error("Analyzer is empty or already finished");
    if (next_recovery_frame())
        throw std::logic_error("Replay requested recovery frames before finishing analysis");
    finished_ = true;
    recovery_.reset();
    previous_.reset();
    return std::move(result_);
}
Plan Analyzer::finish() { return Planner::plan(finish_analysis(), settings_, execution_); }
Image RenderSession::render(size_t index, const Image& image, Profile& profile) {
    execution_.check();
    if (index >= plan_.warps.size())
        throw std::out_of_range("Frame index is outside the plan");
    auto output = renderer_.render(image, plan_.warps[index], plan_.analysis.size, plan_.settings.crop,
                                   plan_.analysis.horizontal, profile);
    execution_.check();
    return output;
}
} // namespace stillflow
