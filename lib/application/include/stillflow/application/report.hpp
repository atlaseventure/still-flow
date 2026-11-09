#pragma once
#include "stillflow/application/media.hpp"
#include "stillflow/application/options.hpp"
#include "stillflow/application/rendering.hpp"
#include "stillflow/domain/profile.hpp"
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
struct RunReport {
    const Options& options;
    const VideoInfo& info;
    Size analysis_size;
    bool horizontal;
    const std::vector<FrameMotion>& frames;
    const std::vector<Warp>& warps;
    const Profile& profile;
    RenderingReport rendering;
    RunSummary summary;
    double analysis_ms;
};
} // namespace stillflow
