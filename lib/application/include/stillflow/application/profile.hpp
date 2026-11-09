#pragma once

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace stillflow {
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

class Profile {
  public:
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
    std::map<std::string, std::vector<double>> samples_;
};

class ScopeTimer {
  public:
    ScopeTimer(Profile& profile, std::string stage)
        : profile_(profile), stage_(std::move(stage)), start_(Clock::now()) {}
    ~ScopeTimer() { profile_.add(stage_, milliseconds(start_)); }

  private:
    Profile& profile_;
    std::string stage_;
    Clock::time_point start_;
};
} // namespace stillflow
