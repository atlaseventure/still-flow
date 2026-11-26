#include "stillflow/application/options.hpp"
#include "stillflow/application/rendering.hpp"
#include <cmath>
#include <stdexcept>

namespace stillflow {
namespace {
void validate_options(const Options& o) {
    if (o.gyro) {
        if (o.realtime)
            throw NotImplementedError("Realtime streaming mode is not implemented for gyro stabilization");
        validate_gyro_settings(o.gyro->settings);
        if (o.gyro->imu.empty())
            throw std::invalid_argument("Gyro stabilization requires IMU samples");
    }
    validate_settings(o);
    if (o.input.empty() || o.output.empty())
        throw std::invalid_argument("input and output paths are required");
    if (o.backend.empty())
        throw std::invalid_argument("backend must be named; use auto for backend selection");
    auto range = [](double value, double low, double high, const char* name) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::invalid_argument(std::string(name) + " is out of range");
    };
    range(o.analysis_width, 160, 4096, "analysis_width");
    range(o.threads, 1, 64, "threads");
    range(o.crf, 0, 51, "crf");
    if (o.scan_axis != "auto" && o.scan_axis != "x" && o.scan_axis != "y")
        throw std::invalid_argument("scan_axis must be auto, x or y");
    if (o.audio < AudioMode::automatic || o.audio > AudioMode::drop)
        throw std::invalid_argument("Invalid audio mode");
    if (o.decoder.empty() || o.encoder.empty())
        throw std::invalid_argument("decoder and encoder must be named; use auto for adapter selection");
}
} // namespace

Options resolve_options(Options options) {
    validate_options(options);
    validate_stream_settings(options.stream);
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
