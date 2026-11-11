#include "recovery.hpp"
#include "stillflow/domain/motion.hpp"
#include <algorithm>
#include <cmath>

namespace stillflow {
TrackingRecovery::TrackingRecovery(AnalysisResult& analysis) : analysis_(analysis) {
    const auto& frames = analysis.frames;
    const auto weak = [](const Motion& motion) {
        return motion.state == MotionState::missing ||
               (motion.model_consistency > 0 && motion.model_consistency < 0.40);
    };
    for (size_t first = 1; first < frames.size();) {
        if (!weak(frames[first].motion) || frames[first].motion.reset) {
            ++first;
            continue;
        }
        size_t last = first + 1;
        while (last < frames.size() && !frames[last].motion.reset && weak(frames[last].motion))
            ++last;
        // Long failures use the gradual tracking fade. Never measure across cuts
        // or timestamp gaps, even when a neighboring proxy happens to look alike.
        if (last < frames.size() && !frames[last].motion.reset &&
            frames[last].seconds - frames[first - 1].seconds <= 0.20) {
            size_t begin = first - 1, end = last;
            for (int k = 0; k < 2 && begin > 0 && !frames[begin].motion.reset; ++k)
                --begin;
            for (int k = 0; k < 2 && end + 1 < frames.size() && !frames[end + 1].motion.reset; ++k)
                ++end;
            // Cap both proxy storage and the dense local solve. Adjacent
            // windows share one anchor when a long chain of weak fits occurs.
            if (!windows_.empty() && begin <= windows_.back().second && end - windows_.back().first < 24) {
                windows_.back().second = end;
            } else {
                if (!windows_.empty())
                    begin = std::max(begin, windows_.back().second);
                if (end - begin < 24)
                    windows_.emplace_back(begin, end);
            }
        }
        first = last;
    }
}
std::optional<RecoveryFrame> TrackingRecovery::next() const {
    if (window_ == windows_.size())
        return std::nullopt;
    const size_t index = windows_[window_].first + proxies_.size();
    return RecoveryFrame{index, analysis_.origin_us +
                                    static_cast<int64_t>(std::llround(analysis_.frames[index].seconds * 1000000))};
}
void TrackingRecovery::push(const Image& image, int64_t pts, MotionAnalyzer& motion, Profile& profile,
                            const ExecutionSettings& execution) {
    const auto request = next();
    if (!request)
        throw std::logic_error("No recovery frame requested");
    if (pts != request->pts_us)
        throw std::invalid_argument("Recovery frame timestamp does not match analysis");
    auto proxy = gray_proxy(image, analysis_.size);
    const auto [begin, end] = windows_[window_];
    proxies_.push_back(std::move(proxy));
    if (request->index != end)
        return;
    try {
        ScopeTimer timer(profile, "analysis.bridge");
        auto& frames = analysis_.frames;
        std::vector<TemporalLink> links;
        for (size_t to = 2; to < proxies_.size(); ++to)
            for (size_t from = to > 3 ? to - 3 : 0; from + 2 <= to; ++from) {
                execution.check();
                auto observation = motion.estimate(proxies_[from], proxies_[to], profile);
                if (!observation.reset && observation.state != MotionState::missing)
                    links.push_back({from, to, std::move(observation)});
            }
        // With two or more blank frames, some visible endpoint connections
        // are longer than three frames. Add independent neighboring anchors
        // within the same short recovery horizon.
        for (size_t first = begin + 1; first <= end; ++first) {
            if (frames[first].motion.state != MotionState::missing)
                continue;
            size_t last = first;
            while (last <= end && frames[last].motion.state == MotionState::missing)
                ++last;
            if (last <= end)
                for (size_t from = first > begin + 1 ? first - 2 : first - 1; from < first; ++from)
                    for (size_t to = last - 1; to <= std::min(end, last + 1); ++to) {
                        if (to - from <= 3 || frames[to].seconds - frames[from].seconds > 0.20)
                            continue;
                        execution.check();
                        auto observation = motion.estimate(proxies_[from - begin], proxies_[to - begin], profile);
                        if (!observation.reset && observation.state != MotionState::missing)
                            links.push_back({from - begin, to - begin, std::move(observation)});
                    }
            first = last - 1;
        }
        recover_motion(std::span<FrameMotion>(frames).subspan(begin, end - begin + 1), links, analysis_.size);
    } catch (...) {
        proxies_.pop_back();
        throw;
    }
    const auto anchor = proxies_.back();
    proxies_.clear();
    ++window_;
    if (window_ < windows_.size() && windows_[window_].first == end)
        proxies_.push_back(anchor);
}
} // namespace stillflow
