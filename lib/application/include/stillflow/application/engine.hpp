#pragma once
#include "stillflow/application/runtime.hpp"
#include "stillflow/vision/vision.hpp"

namespace stillflow {
class TrackingRecovery;
struct RecoveryFrame {
    size_t index;
    int64_t pts_us;
};
void validate_settings(const StabilizationSettings& settings);
struct AnalysisResult {
    Size size;
    bool horizontal = false;
    int64_t origin_us = 0;
    std::vector<FrameMotion> frames;
};
struct Plan {
    AnalysisResult analysis;
    StabilizationSettings settings;
    std::vector<Warp> warps;
};
class Planner {
  public:
    static Plan plan(AnalysisResult analysis, StabilizationSettings settings, ExecutionSettings execution = {});
};
// Push frames or measured observations in presentation order. Replay requested
// recovery frames before finishing; at most 24 proxies are retained. take_ready() reserves
// incremental plan delivery for future lookahead implementations.
class Analyzer {
  public:
    Analyzer(StabilizationSettings settings, Size analysis_size, double fps, bool horizontal,
             std::unique_ptr<MotionAnalyzer> motion, ExecutionSettings execution = {});
    ~Analyzer();
    // Seals analysis and requests recovery frames in increasing index order.
    std::optional<RecoveryFrame> next_recovery_frame();
    void recover(const Image& image, int64_t pts_us, Profile& profile);
    void push(const Image& image, int64_t pts_us, int64_t duration_us, Profile& profile);
    void push_observation(Motion motion, int64_t pts_us, int64_t duration_us);
    AnalysisResult finish_analysis();
    Plan finish();
    bool take_ready(Warp&) {
        execution_.check();
        return false;
    }

  private:
    StabilizationSettings settings_;
    AnalysisResult result_;
    double fps_;
    std::unique_ptr<MotionAnalyzer> motion_;
    ExecutionSettings execution_;
    Image previous_;
    std::unique_ptr<TrackingRecovery> recovery_;
    bool finished_ = false;
    int64_t last_pts_ = 0;
};
class RenderSession {
  public:
    RenderSession(Plan& plan, Renderer& renderer, ExecutionSettings execution = {})
        : plan_(plan), renderer_(renderer), execution_(execution) {}
    Image render(size_t index, const Image& image, Profile& profile);

  private:
    Plan& plan_;
    Renderer& renderer_;
    ExecutionSettings execution_;
};
} // namespace stillflow
