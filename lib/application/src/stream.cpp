#include "stillflow/application/stream.hpp"
#include <algorithm>
#include <cmath>

namespace stillflow {
void validate_stream_settings(const StreamSettings& s) {
    if (!std::isfinite(s.lookahead_seconds) || s.lookahead_seconds < 0 || s.lookahead_seconds > 10 ||
        s.max_buffered_frames < 2 || s.max_buffered_frames > 600)
        throw std::invalid_argument("Invalid streaming lookahead or buffer capacity");
}
StreamAnalyzer::StreamAnalyzer(StabilizationSettings settings, StreamSettings stream, Size size, double fps,
                               bool horizontal, std::unique_ptr<MotionAnalyzer> motion, ExecutionSettings execution)
    : settings_(settings), stream_(stream), analysis_{size, horizontal, 0, {}}, fps_(fps), execution_(execution) {
    // Share validation with the offline analyzer.
    Analyzer validate(settings, size, fps, horizontal, std::move(motion), execution);
    validate_stream_settings(stream);
}
bool StreamAnalyzer::push_observation(Motion motion, int64_t pts, int64_t duration) {
    execution_.check();
    if (finished_)
        throw std::logic_error("Stream is already finished");
    if (duration < 0 || (!analysis_.frames.empty() && pts <= last_pts_))
        throw std::invalid_argument("Stream timestamps must increase and durations must be nonnegative");
    if (full())
        return false;
    if (analysis_.frames.empty())
        analysis_.origin_us = pts;
    const double seconds = static_cast<double>((static_cast<long double>(pts) - analysis_.origin_us) / 1000000);
    int segment = analysis_.frames.empty() ? 0 : analysis_.frames.back().segment;
    if (!analysis_.frames.empty()) {
        const double gap = seconds - analysis_.frames.back().seconds;
        analysis_.frames.back().duration = gap;
        if (gap > std::max(0.5, 5 / fps_)) {
            motion.reset = true;
            motion.state = MotionState::time_gap;
            motion.failure = "timestamp_gap";
        }
        if (motion.reset)
            ++segment;
    }
    analysis_.frames.push_back({seconds, duration / 1000000.0, std::move(motion), segment});
    last_pts_ = pts;
    return true;
}
void StreamAnalyzer::finish() {
    execution_.check();
    if (finished_)
        throw std::logic_error("Stream is already finished");
    finished_ = true;
}
bool StreamAnalyzer::take_ready(ReadyWarp& output) {
    execution_.check();
    if (!buffered())
        return false;
    const auto& frames = analysis_.frames;
    const auto& current = frames[next_];
    const double ahead = frames.back().seconds - current.seconds;
    const bool shot_ended = frames.back().segment != current.segment;
    // One future observation finalizes the PTS-derived duration even at zero lag.
    if (!finished_ && !shot_ended && !full() && (buffered() < 2 || ahead + 1e-9 < stream_.lookahead_seconds))
        return false;
    auto fixed = settings_;
    fixed.crop_mode = CropMode::fixed;
    auto warps = plan_warps(frames, analysis_.size, fixed, analysis_.horizontal);
    const bool reset = base_ + next_ == 0 || current.motion.reset;
    const double dt = reset ? std::max(1 / fps_, current.duration) : current.seconds - frames[next_ - 1].seconds;
    size_t end = next_ + 1;
    while (end < frames.size() && frames[end].segment == current.segment &&
           frames[end].seconds - current.seconds <= 0.5)
        ++end;
    auto warp = warps[next_];
    if (settings_.crop_mode != CropMode::fixed)
        crop_.apply(warp, std::span<const Warp>(warps).subspan(next_, end - next_), analysis_.size, settings_, dt,
                    reset);
    output = {base_ + next_,
              analysis_.origin_us + static_cast<int64_t>(std::llround(current.seconds * 1000000)),
              static_cast<int64_t>(std::llround(current.duration * 1000000)),
              current,
              std::move(warp),
              ahead};
    ++next_;
    // Exact trajectory support plus crop neighborhood. Full-resolution images
    // belong to the caller and can be released as soon as their warp is consumed.
    constexpr size_t history = 180;
    if (next_ > history) {
        const size_t count = next_ - history;
        analysis_.frames.erase(analysis_.frames.begin(), analysis_.frames.begin() + count);
        base_ += count;
        next_ -= count;
    }
    return true;
}
} // namespace stillflow
