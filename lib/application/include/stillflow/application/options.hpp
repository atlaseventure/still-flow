#pragma once
#include "stillflow/application/media.hpp"
#include "stillflow/application/runtime.hpp"
#include "stillflow/application/stream.hpp"
#include "stillflow/domain/settings.hpp"
#include <filesystem>
#include <string>

namespace stillflow {
struct Options : StabilizationSettings {
    std::filesystem::path input, output, compare, report;
    ExecutionSettings execution;
    int threads = 12;
    int crf = 18;
    std::string scan_axis = "auto";
    std::string preset = "fast";
    std::string backend = "auto";
    // Media adapter codec names; "auto" prefers hardware with GPU rendering.
    std::string decoder = "auto", encoder = "auto";
    bool realtime = false;
    StreamSettings stream;
    AudioMode audio = AudioMode::automatic;
    bool overwrite = false;
};
// Resolve derived paths and validate the use-case request before invoking services.
Options resolve_options(Options options);
std::filesystem::path frames_report_path(const Options& options);
} // namespace stillflow
