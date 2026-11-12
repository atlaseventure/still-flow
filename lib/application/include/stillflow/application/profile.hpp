#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>

namespace stillflow {
class Profile {
  public:
    // Fixed logarithmic range, with separate underflow and overflow buckets.
    // Midpoints have <0.55% relative error within the range; sub-nanosecond
    // timings use the underflow midpoint. Counts, totals and extrema remain exact.
    static constexpr double histogram_min_ms = 0.000001;
    static constexpr int histogram_bins_per_octave = 64;
    static constexpr int histogram_bins = 4096;
    struct Statistics {
        uint64_t count = 0;
        double total = 0, minimum = 0, maximum = 0;

        void add(double ms, uint64_t occurrences = 1) {
            int bucket = 0;
            if (ms >= histogram_min_ms) {
                const double position = std::log2(ms / histogram_min_ms) * histogram_bins_per_octave;
                bucket = position >= histogram_bins ? histogram_bins + 1 : 1 + static_cast<int>(position);
            }
            buckets_[bucket] += occurrences;
            minimum = count ? std::min(minimum, ms) : ms;
            maximum = count ? std::max(maximum, ms) : ms;
            total += ms * occurrences;
            count += occurrences;
        }
        void merge(const Statistics& other) {
            if (!other.count)
                return;
            for (const auto& [bucket, calls] : other.buckets_)
                buckets_[bucket] += calls;
            minimum = count ? std::min(minimum, other.minimum) : other.minimum;
            maximum = count ? std::max(maximum, other.maximum) : other.maximum;
            total += other.total;
            count += other.count;
        }
        [[nodiscard]] double percentile(double fraction) const {
            if (!count)
                return 0;
            if (fraction <= 0)
                return minimum;
            if (fraction >= 1)
                return maximum;
            const auto rank = static_cast<uint64_t>(fraction * (count - 1));
            uint64_t seen = 0;
            for (const auto& [bucket, calls] : buckets_) {
                seen += calls;
                if (seen > rank) {
                    const double midpoint =
                        bucket == 0 ? histogram_min_ms / 2
                        : bucket > histogram_bins
                            ? maximum
                            : histogram_min_ms * std::exp2((bucket - 0.5) / histogram_bins_per_octave);
                    return std::clamp(midpoint, minimum, maximum);
                }
            }
            return maximum;
        }

      private:
        // Sparse storage avoids allocating the full histogram for per-call profiles.
        // At most histogram_bins + 2 entries, independent of the number of calls.
        std::map<int, uint64_t> buckets_;
    };
    using Timer = std::function<double()>;
    explicit Profile(Timer timer = {}) : timer_(std::move(timer)) {}
    double now() const { return timer_ ? timer_() : 0; }
    const Timer& timer() const { return timer_; }
    void add(const std::string& stage, double ms) {
        if (std::isfinite(ms) && ms >= 0)
            stages_[stage].add(ms);
    }
    void count(const std::string& stage, uint64_t occurrences) {
        if (occurrences)
            stages_[stage].add(0, occurrences);
    }
    // Profiles are single-threaded; each pipeline thread records its own and merges after joining.
    void merge(const Profile& other) {
        for (const auto& [stage, statistics] : other.stages_)
            stages_[stage].merge(statistics);
    }
    [[nodiscard]] const std::map<std::string, Statistics>& stages() const { return stages_; }

  private:
    Timer timer_;
    std::map<std::string, Statistics> stages_;
};

class ScopeTimer {
  public:
    ScopeTimer(Profile& profile, std::string stage)
        : profile_(profile), stage_(std::move(stage)), start_(profile.now()) {}
    ~ScopeTimer() noexcept {
        try {
            profile_.add(stage_, profile_.now() - start_);
        } catch (...) {
            // Optional profiling must not terminate processing, including during unwinding.
            return;
        }
    }

  private:
    Profile& profile_;
    std::string stage_;
    double start_;
};
} // namespace stillflow
