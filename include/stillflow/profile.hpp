#pragma once

#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace stillflow {
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

class Profile {
public:
    void add(const std::string& stage, double ms) { samples_[stage].push_back(ms); }
    [[nodiscard]] nlohmann::json json() const {
        auto result = nlohmann::json::object();
        for (const auto& [name, values] : samples_) {
            auto sorted = values;
            std::sort(sorted.begin(), sorted.end());
            const double total = std::accumulate(sorted.begin(), sorted.end(), 0.0);
            auto percentile = [&](double p) { return sorted[static_cast<size_t>(p * (sorted.size() - 1))]; };
            result[name] = {{"calls", sorted.size()}, {"total_ms", total},
                {"mean_ms", total / sorted.size()}, {"p50_ms", percentile(0.50)},
                {"p95_ms", percentile(0.95)}, {"max_ms", sorted.back()}};
        }
        return result;
    }
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
