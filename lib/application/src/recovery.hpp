#pragma once
#include "stillflow/application/engine.hpp"

namespace stillflow {
class TrackingRecovery {
  public:
    explicit TrackingRecovery(AnalysisResult& analysis);
    std::optional<RecoveryFrame> next() const;
    void push(const Image& image, int64_t pts_us, MotionAnalyzer& motion, Profile& profile,
              const ExecutionSettings& execution);

  private:
    AnalysisResult& analysis_;
    std::vector<std::pair<size_t, size_t>> windows_;
    size_t window_ = 0;
    std::vector<Image> proxies_;
};
} // namespace stillflow
