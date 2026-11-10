#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace stillflow {
class Profile {
  public:
    using Timer = std::function<double()>;
    explicit Profile(Timer timer = {}) : timer_(std::move(timer)) {}
    double now() const { return timer_ ? timer_() : 0; }
    const Timer& timer() const { return timer_; }
    void add(const std::string& stage, double ms) { samples_[stage].push_back(ms); }
    // Profiles are single-threaded; each pipeline thread records its own and merges after joining.
    void merge(const Profile& other) {
        for (const auto& [stage, values] : other.samples_) {
            auto& target = samples_[stage];
            target.insert(target.end(), values.begin(), values.end());
        }
    }
    [[nodiscard]] const std::map<std::string, std::vector<double>>& samples() const { return samples_; }

  private:
    Timer timer_;
    std::map<std::string, std::vector<double>> samples_;
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
