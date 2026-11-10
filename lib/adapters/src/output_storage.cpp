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

void validate_paths(const fs::path& input, std::span<const fs::path> outputs, bool overwrite) {
    if (!fs::is_regular_file(input))
        throw std::runtime_error("Input must be a readable local video file");
    std::set<fs::path> used{fs::weakly_canonical(input)};
    for (const auto& path : outputs) {
        if (path.empty())
            continue;
        if (!used.insert(fs::weakly_canonical(path)).second)
            throw std::runtime_error("Input, output, comparison and report paths must be distinct");
        if (fs::exists(path) && (!overwrite || !fs::is_regular_file(path)))
            throw std::runtime_error("Output exists: " + path.string() + " (use --overwrite to replace files)");
    }
}

void write_csv(const fs::path& path, const std::vector<FrameMotion>& frames, const std::vector<Warp>& warps) {
    std::ofstream file(path);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << "frame,time_s,segment,reset,tracks,confidence,rs_confidence,fit_residual_px,step_x_px,step_y_px,step_angle_"
            "rad,warp_x_px,warp_y_px,row_correction_rms_px,strength,invalid_fraction,motion_state,failure_reason,"
            "fit_confidence,coverage,crop_strength,tracking_strength,match_reliability,model_consistency,spatial_span,"
            "residual_normalized,temporal_links,temporal_residual,crop_left,crop_top,crop_right,crop_bottom,"
            "retained_area_ratio,geometry_strength,planned_strength,render_strength,boundary_retries,boundary_invalid_"
            "fraction\n";
    file << std::setprecision(9);
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i];
        const auto& w = warps[i];
        file << i << ',' << f.seconds << ',' << f.segment << ',' << f.motion.reset << ',' << f.motion.tracks << ','
             << f.motion.confidence << ',' << f.motion.rs_confidence << ',' << f.motion.residual_px << ','
             << f.motion.step(0, 2) << ',' << f.motion.step(1, 2) << ','
             << std::atan2(f.motion.step(1, 0), f.motion.step(0, 0)) << ',' << w.global_offset.x << ','
             << w.global_offset.y << ',' << w.row_correction_rms << ',' << w.strength << ',' << w.invalid_fraction
             << ',' << motion_state_name(f.motion.state) << ',' << f.motion.failure << ',' << f.motion.fit_confidence
             << ',' << f.motion.coverage << ',' << w.crop_strength << ',' << w.tracking_strength << ','
             << f.motion.match_reliability << ',' << f.motion.model_consistency << ',' << f.motion.spatial_span << ','
             << f.motion.residual_normalized << ',' << f.motion.temporal_links << ',' << f.motion.temporal_residual
             << ',' << w.window.left << ',' << w.window.top << ',' << 1 - w.window.left - w.window.scale << ','
             << 1 - w.window.top - w.window.scale << ',' << w.window.scale * w.window.scale << ','
             << w.geometry_strength << ',' << w.planned_strength << ',' << w.render_strength << ','
             << w.boundary_retries << ',' << w.boundary_invalid_fraction << '\n';
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
    void validate(const fs::path& input, std::span<const fs::path> outputs, bool overwrite) override {
        validate_paths(input, outputs, overwrite);
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
        const auto& frames_path = data.frames_path;
        const double duration = frames.back().seconds + frames.back().duration;
        const auto resets = std::count_if(frames.begin(), frames.end(), [](const auto& f) { return f.motion.reset; });
        const auto limited =
            std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.crop_strength < 0.999; });
        const auto tracking_limited =
            std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.tracking_strength < 0.999; });
        const auto geometry_limited =
            std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.geometry_strength < 0.999; });
        const auto boundary_fallback =
            std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.boundary_retries > 0; });
        double minimum_area = 1, maximum_area = 0, mean_area = 0;
        for (const auto& w : warps) {
            const double area = w.window.scale * w.window.scale;
            minimum_area = std::min(minimum_area, area);
            maximum_area = std::max(maximum_area, area);
            mean_area += area / warps.size();
        }
        auto states = nlohmann::json::object(), failures = nlohmann::json::object();
        for (const auto& frame : frames) {
            const std::string state = motion_state_name(frame.motion.state);
            states[state] = states.value(state, 0) + 1;
            if (!frame.motion.failure.empty()) {
                const auto& reason = frame.motion.failure;
                failures[reason] = failures.value(reason, 0) + 1;
            }
        }
        nlohmann::json report{
            {"schema_version", 7},
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
              {"crop_mode", crop_mode_name(options.crop_mode)},
              {"crop_budget_per_edge", options.crop},
              {"minimum_retained_area_ratio", std::pow(1 - 2 * options.crop, 2)},
              {"threads", options.threads},
              {"crf", options.crf},
              {"preset", options.preset},
              {"decoder", options.decoder},
              {"encoder", options.encoder},
              {"audio_copy", options.audio},
              {"comparison", !options.compare.empty()}}},
            {"cropping",
             {{"method", "geometric_viewport"},
              {"retained_area_ratio", {{"minimum", minimum_area}, {"mean", mean_area}, {"maximum", maximum_area}}},
              {"strength_note",
               "Geometry and tracking are caps; crop_strength is the remaining boundary fraction. Planned strength "
               "includes temporal smoothing; render_strength records final boundary fallback."}}},
            {"trajectory",
             {{"method", "lightstab_demo_causal_mesh"},
              {"kernel", "gaussian_motion_confidence"},
              {"global", "zero_phase_forward_backward"},
              {"global_boundary", "point_reflected_about_fitted_shot_ends"},
              {"initialization", "reflected_scanline_history"},
              {"passes", trajectory_smoothing_passes}}},
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
              {"tracking_limited_frames", tracking_limited},
              {"geometry_limited_frames", geometry_limited},
              {"boundary_fallback_frames", boundary_fallback},
              {"motion_states", states},
              {"tracking_failure_reasons", failures},
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
