#pragma once
#include "stillflow/application/engine.hpp"
#include <deque>

namespace stillflow {
struct StreamSettings {
    double lookahead_seconds = 1.5;
    size_t max_buffered_frames = 120;
};
void validate_stream_settings(const StreamSettings& settings);
struct ReadyWarp {
    size_t index = 0;
    int64_t pts_us = 0, duration_us = 0;
    FrameMotion observation;
    Warp warp;
    double lookahead_seconds = 0;
};
// Single-caller fixed-lag planner. Drain take_ready() after each push and after
// finish(). A full queue rejects pushes without consuming them; draining at the
// capacity limit shortens lookahead. Storage is bounded independently of duration.
class StreamAnalyzer {
  public:
    StreamAnalyzer(StabilizationSettings settings, StreamSettings stream, Size size, double fps, bool horizontal,
                   std::unique_ptr<MotionAnalyzer> motion = {}, ExecutionSettings execution = {},
                   std::unique_ptr<MotionAnalyzer> recovery = {});
    // Only reduced gray proxies are retained (at most 24); callers own the
    // delayed full-resolution frames until take_ready returns their index.
    bool push(const Image& image, int64_t pts_us, int64_t duration_us, Profile& profile);
    bool push_observation(Motion motion, int64_t pts_us, int64_t duration_us);
    bool take_ready(ReadyWarp& output);
    void finish();
    size_t buffered() const { return analysis_.frames.size() - next_; }
    bool full() const { return buffered() >= stream_.max_buffered_frames; }

  private:
    StabilizationSettings settings_;
    StreamSettings stream_;
    AnalysisResult analysis_;
    double fps_;
    ExecutionSettings execution_;
    size_t base_ = 0, next_ = 0;
    int64_t last_pts_ = 0;
    bool finished_ = false;
    StreamCrop crop_;
    std::unique_ptr<MotionAnalyzer> motion_, recovery_;
    Image previous_;
    std::deque<std::pair<size_t, Image>> proxies_;
    size_t recovered_until_ = 0;
    void recover_pending(Profile& profile, bool flush);
};
} // namespace stillflow
