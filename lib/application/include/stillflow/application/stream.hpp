#pragma once
#include "stillflow/application/engine.hpp"
#include <atomic>
#include <deque>
#include <future>

namespace stillflow {
struct StreamSettings {
    double lookahead_seconds = 1.35;
    size_t max_buffered_frames = 120;
    bool background_recovery = false;
    bool adaptive_tracking = false;
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
    ~StreamAnalyzer();
    // Only reduced gray proxies are retained (at most 24); callers own the
    // delayed full-resolution frames until take_ready returns their index.
    bool push(const Image& image, int64_t pts_us, int64_t duration_us, Profile& profile);
    bool push_observation(Motion motion, int64_t pts_us, int64_t duration_us);
    bool take_ready(ReadyWarp& output, Profile* profile = nullptr);
    void finish(Profile* profile = nullptr);
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
    TrajectoryCache planning_;
    Warp previous_warp_;
    double previous_picture_seconds_ = 0;
    std::unique_ptr<MotionAnalyzer> motion_, recovery_;
    Image previous_;
    std::deque<std::pair<size_t, Image>> proxies_;
    size_t recovered_until_ = 0;
    struct RecoveryResult {
        size_t begin = 0, apply_at = 0;
        int segment = 0;
        std::vector<FrameMotion> frames;
        Profile profile;
    };
    std::atomic<bool> stop_recovery_{false};
    std::future<RecoveryResult> recovery_task_;
    size_t recovery_apply_at_ = 0, recovery_begin_ = 0;
    void settle_recovery(Profile& profile);
    void recover_pending(Profile& profile, bool flush);
    void consume_ready();
};
} // namespace stillflow
