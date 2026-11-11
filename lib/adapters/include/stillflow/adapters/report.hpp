#pragma once
#include "stillflow/adapters/options.hpp"
#include "stillflow/application/media.hpp"
#include "stillflow/application/profile.hpp"
#include "stillflow/application/rendering.hpp"
#include "stillflow/domain/types.hpp"

namespace stillflow {
struct ResourceUsage {
    double cpu_seconds = 0, peak_rss_mib = 0;
};
struct RunSummary {
    size_t frames = 0;
    double wall_ms = 0;
    ResourceUsage resources;
};
struct StreamStatistics {
    size_t peak_buffered_frames = 0, shortened_lookahead_frames = 0, frame_budget_overruns = 0;
};
struct RunReport {
    const Options& options;
    std::filesystem::path frames_path;
    const VideoInfo& info;
    Size analysis_size;
    bool horizontal;
    const std::vector<FrameMotion>& frames;
    const std::vector<Warp>& warps;
    const Profile& profile;
    RenderingReport rendering;
    // Keyed by stream: analysis_decoder, recovery_decoder (when needed), render_decoder,
    // encoder, comparison_encoder.
    std::vector<std::pair<std::string, CodecReport>> codecs;
    RunSummary summary;
    double analysis_ms;
    StreamStatistics streaming;
};
} // namespace stillflow
