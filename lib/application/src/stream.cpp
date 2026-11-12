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
                               bool horizontal, std::unique_ptr<MotionAnalyzer> motion, ExecutionSettings execution,
                               std::unique_ptr<MotionAnalyzer> recovery)
    : settings_(settings), stream_(stream), analysis_{size, horizontal, 0, {}}, fps_(fps), execution_(execution),
      motion_(std::move(motion)), recovery_(std::move(recovery)) {
    // Share validation with the offline analyzer.
    Analyzer validate(settings, size, fps, horizontal, nullptr, execution);
    validate_stream_settings(stream);
}
bool StreamAnalyzer::push(const Image& image, int64_t pts, int64_t duration, Profile& profile) {
    execution_.check();
    if (finished_)
        throw std::logic_error("Stream is already finished");
    if (duration < 0 || (!analysis_.frames.empty() && pts <= last_pts_))
        throw std::invalid_argument("Stream timestamps must increase and durations must be nonnegative");
    if (full())
        return false;
    Image current;
    {
        ScopeTimer timer(profile, "analysis.proxy");
        current = gray_proxy(image, analysis_.size);
    }
    Motion observation;
    observation.row_delta.resize(settings_.bands);
    if (previous_ && (static_cast<long double>(pts) - last_pts_) / 1000000 <= std::max(0.5, 5 / fps_)) {
        if (!motion_)
            throw std::logic_error("No streaming motion estimator registered");
        observation = motion_->estimate(previous_, current, profile);
    }
    push_observation(std::move(observation), pts, duration);
    previous_ = current;
    proxies_.emplace_back(base_ + analysis_.frames.size() - 1, std::move(current));
    if (proxies_.size() > 24)
        proxies_.pop_front();
    recover_pending(profile, false);
    return true;
}
void StreamAnalyzer::recover_pending(Profile& profile, bool flush) {
    if (!recovery_ || proxies_.size() < 3)
        return;
    const auto weak = [](const Motion& m) {
        return m.state == MotionState::missing || (m.model_consistency > 0 && m.model_consistency < 0.40);
    };
    auto& frames = analysis_.frames;
    const size_t start = proxies_.front().first - base_;
    size_t budget = 4;
    for (size_t first = std::max(start + 1, next_); first < frames.size(); ++first) {
        if (!weak(frames[first].motion) || frames[first].motion.reset || weak(frames[first - 1].motion))
            continue;
        size_t last = first + 1;
        while (last < frames.size() && weak(frames[last].motion) && !frames[last].motion.reset)
            ++last;
        if (last == frames.size())
            break;
        if (frames[last].motion.reset || frames[last].seconds - frames[first - 1].seconds > 0.20 ||
            base_ + last <= recovered_until_ || (!flush && last + 2 >= frames.size())) {
            first = last - 1;
            continue;
        }
        size_t begin = first - 1, end = last;
        for (int k = 0; k < 2 && begin > start && !frames[begin].motion.reset; ++k)
            --begin;
        for (int k = 0; k < 2 && end + 1 < frames.size() && !frames[end + 1].motion.reset; ++k)
            ++end;
        AnalysisResult local{analysis_.size,
                             analysis_.horizontal,
                             analysis_.origin_us,
                             {frames.begin() + begin, frames.begin() + end + 1}};
        // Four independent endpoint links constrain a short weak interval.
        // A fixed estimate budget makes recovery independent of processor speed.
        ScopeTimer timer(profile, "analysis.bridge");
        std::vector<TemporalLink> links;
        bool exhausted = false;
        for (size_t offset = 0; offset < 2 && !exhausted; ++offset) {
            if (first <= begin + offset)
                break;
            const size_t from = first - 1 - offset;
            for (size_t to = last; to <= std::min(end, last + 1); ++to) {
                if (frames[to].seconds - frames[from].seconds > 0.20)
                    continue;
                if (budget == 0) {
                    profile.add("analysis.bridge_budget_exhausted", 0);
                    exhausted = true;
                    break;
                }
                execution_.check();
                --budget;
                auto observation =
                    recovery_->estimate(proxies_[from - start].second, proxies_[to - start].second, profile);
                if (!observation.reset && observation.state != MotionState::missing)
                    links.push_back({from - begin, to - begin, std::move(observation)});
            }
        }
        recover_motion(local.frames, links, analysis_.size);
        for (size_t i = std::max(begin, next_); i <= end; ++i)
            frames[i].motion = std::move(local.frames[i - begin].motion);
        recovered_until_ = base_ + last;
        first = last;
        if (exhausted)
            break;
    }
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
    Profile profile;
    recover_pending(profile, true);
    finished_ = true;
    previous_.reset();
    proxies_.clear();
}
bool StreamAnalyzer::take_ready(ReadyWarp& output) {
    execution_.check();
    if (!buffered())
        return false;
    // Microsecond timestamps can round an exact lookahead down by one tick.
    constexpr double pts_tolerance = 2e-6;
    const double required = std::max(stream_.lookahead_seconds, motion_ ? 0.20 + 2 / fps_ : 0.0);
    size_t limit = next_ + 1;
    while (limit < analysis_.frames.size()) {
        const auto& candidate = analysis_.frames[limit++];
        if (candidate.segment != analysis_.frames[next_].segment ||
            candidate.seconds - analysis_.frames[next_].seconds + pts_tolerance >= required)
            break;
    }
    const auto frames = std::span<const FrameMotion>(analysis_.frames).first(limit);
    const auto& current = frames[next_];
    const double ahead = frames.back().seconds - current.seconds;
    const bool shot_ended = frames.back().segment != current.segment;
    // One future observation finalizes the PTS-derived duration even at zero lag.
    if (!finished_ && !shot_ended && !full() && (buffered() < 2 || ahead + pts_tolerance < required))
        return false;
    auto fixed = settings_;
    fixed.crop_mode = CropMode::fixed;
    // Fixed cropping retains the offline constraint neighborhoods. Adaptive
    // streams constrain only the preview meshes with their own controller.
    const auto first = std::lower_bound(frames.begin(), frames.end(), current.seconds - 2.0 - 2e-6,
                                        [](const auto& frame, double seconds) { return frame.seconds < seconds; });
    size_t shot_start = next_;
    while (shot_start > 0 && frames[shot_start - 1].segment == current.segment && !frames[shot_start].motion.reset)
        --shot_start;
    const bool known_start = base_ == 0 || shot_start > 0 || frames[shot_start].motion.reset;
    const bool initialize_path =
        known_start && current.seconds - frames[shot_start].seconds < 3 * settings_.smooth_seconds;
    // Keep endpoint path relaxation for fast opening pans; steady-state
    // streaming needs no offline viewport or historical crop meshes.
    const bool plan_viewports = settings_.crop_mode == CropMode::fixed || initialize_path;
    const size_t output_first = plan_viewports ? static_cast<size_t>(first - frames.begin()) : next_;
    auto warps = plan_warps(frames, analysis_.size, fixed, analysis_.horizontal, base_ == 0, &planning_, output_first,
                            std::dynamic_extent, plan_viewports);
    const bool reset = base_ + next_ == 0 || current.motion.reset;
    const double dt = reset ? std::max(1 / fps_, current.duration) : current.seconds - frames[next_ - 1].seconds;
    size_t end = next_ + 1;
    while (end < frames.size() && frames[end].segment == current.segment)
        ++end;
    auto warp = warps[next_];
    warp.boundary_search_steps = 0;
    if (settings_.crop_mode != CropMode::fixed)
        crop_.apply(warp, std::span<const Warp>(warps).subspan(next_, end - next_), frames.subspan(next_, end - next_),
                    analysis_.size, settings_, dt, reset);
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
