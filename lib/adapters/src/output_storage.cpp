#include "stillflow/adapters/backends.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <numeric>
#include <opencv2/core/version.hpp>
#include <set>
#include <stdexcept>
#include <thread>
extern "C" {
#include <libavutil/avutil.h>
}

namespace stillflow {
namespace fs = std::filesystem;
namespace {
class LocalOutputFile final : public OutputFile {
  public:
    explicit LocalOutputFile(fs::path path) : target_(std::move(path)) {
        if (!target_.parent_path().empty())
            fs::create_directories(target_.parent_path());
        temporary_ = target_.parent_path() /
                     ("." + target_.stem().string() + ".tmp-" +
                      std::to_string(Clock::now().time_since_epoch().count()) + target_.extension().string());
    }
    ~LocalOutputFile() override {
        std::error_code ignored;
        fs::remove(temporary_, ignored);
    }
    const fs::path& temporary() const override { return temporary_; }
    void publish() override { fs::rename(temporary_, target_); }

  private:
    fs::path target_, temporary_;
};

void validate_paths(const Options& o, const fs::path& frames_path) {
    if (!fs::is_regular_file(o.input))
        throw std::runtime_error("Input must be a readable local video file");
    std::set<fs::path> used{fs::weakly_canonical(o.input)};
    for (const auto& path : {o.output, o.compare, o.report, frames_path}) {
        if (path.empty())
            continue;
        if (!used.insert(fs::weakly_canonical(path)).second)
            throw std::runtime_error("Input, output, comparison and report paths must be distinct");
        if (fs::exists(path) && (!o.overwrite || !fs::is_regular_file(path)))
            throw std::runtime_error("Output exists: " + path.string() + " (use --overwrite to replace files)");
    }
    for (const auto& path : {o.output, o.compare}) {
        if (!path.empty() && path.extension() != ".mp4" && path.extension() != ".mkv")
            throw std::runtime_error("Output must use .mp4 or .mkv");
    }
}

void write_csv(const fs::path& path, const std::vector<FrameMotion>& frames, const std::vector<Warp>& warps) {
    std::ofstream file(path);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << "frame,time_s,segment,reset,tracks,confidence,rs_confidence,fit_residual_px,step_x_px,step_y_px,step_angle_"
            "rad,warp_x_px,warp_y_px,row_correction_rms_px,strength,invalid_fraction\n";
    file << std::setprecision(9);
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        const auto& w = warps[i];
        double row_energy = 0;
        for (const auto& point : w.row_offset)
            row_energy += point.dot(point);
        const double rms = w.row_offset.empty() ? 0 : std::sqrt(row_energy / w.row_offset.size());
        file << i << ',' << f.seconds << ',' << f.segment << ',' << f.motion.reset << ',' << f.motion.tracks << ','
             << f.motion.confidence << ',' << f.motion.rs_confidence << ',' << f.motion.residual_px << ','
             << f.motion.step(0, 2) << ',' << f.motion.step(1, 2) << ','
             << std::atan2(f.motion.step(1, 0), f.motion.step(0, 0)) << ',' << w.global(0, 2) << ',' << w.global(1, 2)
             << ',' << rms << ',' << w.strength << ',' << w.invalid_fraction << '\n';
    }
}

nlohmann::json profile_json(const Profile& profile) {
    auto result = nlohmann::json::object();
    for (const auto& [name, values] : profile.samples()) {
        if (values.empty())
            continue;
        auto sorted = values;
        std::sort(sorted.begin(), sorted.end());
        const double total = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        auto percentile = [&](double p) { return sorted[static_cast<size_t>(p * (sorted.size() - 1))]; };
        result[name] = {{"calls", sorted.size()},     {"total_ms", total},          {"mean_ms", total / sorted.size()},
                        {"p50_ms", percentile(0.50)}, {"p95_ms", percentile(0.95)}, {"max_ms", sorted.back()}};
    }
    return result;
}
nlohmann::json rendering_json(const RenderingReport& data) {
    auto backends = nlohmann::json::array(), failures = nlohmann::json::array();
    for (const auto& backend : data.backends) {
        auto formats = nlohmann::json::array();
        for (auto format : backend.info.capabilities.formats)
            formats.push_back(pixel_format_name(format));
        backends.push_back(
            {{"id", backend.info.id},
             {"device", backend.info.device},
             {"frames", backend.frames},
             {"gpu", backend.gpu},
             {"capabilities", {{"formats", formats}, {"image_access", backend.info.capabilities.image_access}}},
             {"implementation", backend.info.implementation}});
    }
    for (const auto& failure : data.failures)
        failures.push_back({{"backend", failure.backend}, {"stage", failure.stage}, {"reason", failure.reason}});
    return {{"requested", data.requested}, {"backend", data.backend}, {"backends", backends}, {"failures", failures}};
}
nlohmann::json codecs_json(const std::vector<std::pair<std::string, CodecReport>>& codecs) {
    auto result = nlohmann::json::object();
    for (const auto& [stream, codec] : codecs)
        result[stream] = {{"requested", codec.request.name},
                          {"prefer_hardware", codec.request.prefer_hardware},
                          {"implementation", codec.implementation},
                          {"device", codec.device},
                          {"frames", codec.frames},
                          {"hardware_frames", codec.hardware_frames},
                          {"fallbacks", codec.fallbacks}};
    return result;
}
class LocalOutputStorage final : public OutputStorage {
  public:
    void validate(const Options& options, const fs::path& frames_path) override {
        validate_paths(options, frames_path);
    }
    std::unique_ptr<OutputFile> stage(const fs::path& path) override { return std::make_unique<LocalOutputFile>(path); }
    void write_report(const fs::path& json_path, const fs::path& csv_path, const RunReport& data) override {
        const auto& options = data.options;
        const auto& info = data.info;
        const auto& frames = data.frames;
        const auto& warps = data.warps;
        const auto analysis_size = data.analysis_size;
        const bool horizontal = data.horizontal;
        const double total_ms = data.summary.wall_ms, analysis_ms = data.analysis_ms;
        auto frames_path = options.report;
        frames_path.replace_extension(".frames.csv");
        const double duration = frames.back().seconds + frames.back().duration;
        const auto resets = std::count_if(frames.begin(), frames.end(), [](const auto& f) { return f.motion.reset; });
        const auto limited =
            std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.strength < 0.999; });
        nlohmann::json report{
            {"schema_version", 3},
            {"version", STILLFLOW_VERSION},
            {"input", fs::absolute(options.input).string()},
            {"output", fs::absolute(options.output).string()},
            {"frames_csv", fs::absolute(frames_path).string()},
            {"rendering", rendering_json(data.rendering)},
            {"codecs", codecs_json(data.codecs)},
            {"video",
             {{"width", info.width},
              {"height", info.height},
              {"nominal_fps", info.fps},
              {"frames", frames.size()},
              {"duration_s", duration},
              {"input_codec", info.codec},
              {"display_rotation_applied", info.rotation},
              {"analysis_width", analysis_size.width},
              {"analysis_height", analysis_size.height}}},
            {"settings",
             {{"rolling_shutter", options.rolling_shutter},
              {"scan_axis", horizontal ? "x" : "y"},
              {"bands", options.bands},
              {"features", options.features},
              {"smooth_seconds", options.smooth_seconds},
              {"rs_seconds", options.rs_seconds},
              {"rs_strength", options.rs_strength},
              {"crop_per_edge", options.crop},
              {"retained_area_ratio", std::pow(1 - 2 * options.crop, 2)},
              {"threads", options.threads},
              {"crf", options.crf},
              {"preset", options.preset},
              {"decoder", options.decoder},
              {"encoder", options.encoder},
              {"audio_copy", options.audio},
              {"comparison", !options.compare.empty()}}},
            {"performance",
             {{"wall_ms", total_ms},
              {"analysis_and_plan_ms", analysis_ms},
              {"end_to_end_fps", frames.size() * 1000.0 / total_ms},
              {"processing_to_duration_ratio", total_ms / 1000 / duration},
              {"process_cpu_s", data.summary.resources.cpu_seconds},
              {"peak_rss_mib", data.summary.resources.peak_rss_mib}}},
            {"diagnostics",
             {{"tracking_resets", resets},
              {"crop_limited_frames", limited},
              {"note", "Motion residuals are fit diagnostics, not ground-truth image quality metrics."}}},
            {"build",
             {{"compiler", __VERSION__},
              {"cplusplus", __cplusplus},
              {"opencv", CV_VERSION},
              {"ffmpeg", av_version_info()},
              {"logical_cpus", std::thread::hardware_concurrency()},
#ifdef NDEBUG
              {"configuration", "release"}
#else
              {"configuration", "debug"}
#endif
             }},
            {"stages", profile_json(data.profile)}};
        write_csv(csv_path, frames, warps);
        std::ofstream file(json_path);
        file.exceptions(std::ios::badbit | std::ios::failbit);
        file << report.dump(2) << '\n';
    }
};
} // namespace
std::unique_ptr<OutputStorage> make_output_storage() { return std::make_unique<LocalOutputStorage>(); }
} // namespace stillflow
