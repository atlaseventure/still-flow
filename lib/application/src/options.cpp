#include "stillflow/application/options.hpp"
#include <cmath>
#include <stdexcept>

namespace stillflow {
namespace {
void validate_options(const Options& o) {
    if (o.input.empty() || o.output.empty())
        throw std::invalid_argument("input and output paths are required");
    if (o.backend.empty())
        throw std::invalid_argument("backend must be named; use auto for backend selection");
    auto range = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument(std::string(name) + " is out of range");
    };
    range(o.analysis_width, 160, 4096, "analysis_width");
    range(o.features, 96, 10000, "features");
    range(o.bands, 4, 32, "bands");
    range(o.threads, 1, 64, "threads");
    range(o.crf, 0, 51, "crf");
    range(o.smooth_seconds, 0, 10, "smooth_seconds");
    range(o.rs_seconds, 0, 10, "rs_seconds");
    range(o.rs_strength, 0, 1, "rs_strength");
    range(o.crop, 0, 0.25, "crop");
    if (o.crop_mode != CropMode::automatic && o.crop_mode != CropMode::fixed && o.crop_mode != CropMode::adaptive)
        throw std::invalid_argument("crop_mode must be auto, fixed or adaptive");
    if (o.scan_axis != "auto" && o.scan_axis != "x" && o.scan_axis != "y")
        throw std::invalid_argument("scan_axis must be auto, x or y");
    if (o.decoder.empty() || o.encoder.empty())
        throw std::invalid_argument("decoder and encoder must be named; use auto for adapter selection");
}
} // namespace

Options resolve_options(Options options) {
    validate_options(options);
    if (options.report.empty())
        options.report = options.output.string() + ".profile.json";
    return options;
}

std::filesystem::path frames_report_path(const Options& options) {
    auto path = options.report;
    path.replace_extension(".frames.csv");
    return path;
}
} // namespace stillflow
