#include "stillflow/stabilizer.hpp"
#include "stillflow/media.hpp"
#include "stillflow/motion.hpp"
#include "stillflow/profile.hpp"
#include "stillflow/renderer.hpp"

#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <sys/resource.h>

extern "C" {
#include <libavcodec/avcodec.h>
}

namespace stillflow {
namespace fs = std::filesystem;
namespace {
class OutputFile {
public:
    explicit OutputFile(fs::path path) : target_(std::move(path)) {
        if (!target_.parent_path().empty()) fs::create_directories(target_.parent_path());
        temporary_ = target_.parent_path() / ("." + target_.stem().string() + ".tmp-" +
            std::to_string(Clock::now().time_since_epoch().count()) + target_.extension().string());
    }
    ~OutputFile() { std::error_code ignored; fs::remove(temporary_, ignored); }
    const fs::path& temporary() const { return temporary_; }
    void publish() { fs::rename(temporary_, target_); }
private:
    fs::path target_, temporary_;
};

void validate_paths(const Options& o, const fs::path& frames_path) {
    if (!fs::is_regular_file(o.input)) throw std::runtime_error("Input must be a readable local video file");
    std::set<fs::path> used{fs::weakly_canonical(o.input)};
    for (const auto& path : {o.output, o.compare, o.report, frames_path}) {
        if (path.empty()) continue;
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

cv::Mat gray_proxy(const cv::Mat& image, cv::Size size) {
    cv::Mat gray, small;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    if (gray.size() == size) return gray;
    cv::resize(gray, small, size, 0, 0, cv::INTER_AREA);
    return small;
}

void write_csv(const fs::path& path, const std::vector<FrameMotion>& frames, const std::vector<Warp>& warps) {
    std::ofstream file(path);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << "frame,time_s,segment,reset,tracks,confidence,rs_confidence,fit_residual_px,step_x_px,step_y_px,step_angle_rad,warp_x_px,warp_y_px,row_correction_rms_px,strength,invalid_fraction\n";
    file << std::setprecision(9);
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& f = frames[i]; const auto& w = warps[i];
        double row_energy = 0;
        for (const auto& point : w.row_offset) row_energy += point.dot(point);
        const double rms = w.row_offset.empty() ? 0 : std::sqrt(row_energy / w.row_offset.size());
        file << i << ',' << f.seconds << ',' << f.segment << ',' << f.motion.reset << ',' << f.motion.tracks
             << ',' << f.motion.confidence << ',' << f.motion.rs_confidence << ',' << f.motion.residual_px
             << ',' << f.motion.step(0, 2) << ',' << f.motion.step(1, 2)
             << ',' << std::atan2(f.motion.step(1, 0), f.motion.step(0, 0))
             << ',' << w.global(0, 2) << ',' << w.global(1, 2) << ',' << rms
             << ',' << w.strength << ',' << w.invalid_fraction << '\n';
    }
}
}

int run(const Options& options) {
    const auto start = Clock::now();
    auto frames_path = options.report;
    frames_path.replace_extension(".frames.csv");
    validate_paths(options, frames_path);
    cv::setNumThreads(options.threads);
    cv::setRNGSeed(42);
    av_log_set_level(AV_LOG_WARNING);
    Profile profile;
    const auto backend_start = Clock::now();
    FrameRenderer renderer(options.gpu);
    profile.add("render.backend_init", milliseconds(backend_start));
    VideoInfo info;
    cv::Size analysis_size;
    std::vector<FrameMotion> frames;
    int64_t origin = 0;
    bool horizontal = false;
    {
        Reader reader(options.input, options.threads);
        info = reader.info();
        horizontal = options.scan_axis == "x" || (options.scan_axis == "auto" && info.horizontal_scan);
        // Keep original raster geometry; odd dimensions are rejected before writing H.264 4:2:0.
        if ((info.width & 1) || (info.height & 1))
            throw std::runtime_error("Output H.264 4:2:0 requires even input dimensions");
        const double scale = std::min(1.0, static_cast<double>(options.analysis_width) / info.width);
        analysis_size = {std::max(32, static_cast<int>(std::lround(info.width * scale))),
                         std::max(32, static_cast<int>(std::lround(info.height * scale)))};
        MotionEstimator estimator(options, horizontal);
        VideoFrame decoded;
        cv::Mat previous;
        int segment = 0;
        std::cerr << "Analyze " << info.width << 'x' << info.height << " at " << analysis_size.width
                  << 'x' << analysis_size.height << ", scan axis " << (horizontal ? 'x' : 'y') << '\n';
        for (;;) {
            bool available;
            { ScopeTimer timer(profile, "analysis.decode"); available = reader.next(decoded); }
            if (!available) break;
            if (frames.empty()) origin = decoded.pts_us;
            cv::Mat current;
            { ScopeTimer timer(profile, "analysis.preprocess"); current = gray_proxy(decoded.bgr, analysis_size); }
            FrameMotion frame;
            frame.seconds = (decoded.pts_us - origin) / 1000000.0;
            frame.duration = decoded.duration_us / 1000000.0;
            frame.motion.row_delta.resize(options.bands);
            if (!previous.empty()) {
                const double gap = frame.seconds - frames.back().seconds;
                if (gap > std::max(0.5, 5 / info.fps)) frame.motion.reset = true;
                else frame.motion = estimator.estimate(previous, current, profile);
                if (frame.motion.reset) ++segment;
            }
            frame.segment = segment;
            frames.push_back(std::move(frame));
            previous = std::move(current);
            if (frames.size() % 120 == 0) std::cerr << "\rAnalyzed " << frames.size() << " frames" << std::flush;
        }
        std::cerr << "\rAnalyzed " << frames.size() << " frames\n";
    }
    if (frames.empty()) throw std::runtime_error("Input contains no decoded video frames");
    // Actual PTS differences, rather than nominal FPS, define frame durations for VFR files.
    for (size_t i = 0; i + 1 < frames.size(); ++i) frames[i].duration = frames[i + 1].seconds - frames[i].seconds;
    auto warps = plan_warps(frames, analysis_size, options, horizontal, profile);
    const auto analysis_ms = milliseconds(start);
    OutputFile output(options.output);
    std::unique_ptr<OutputFile> comparison;
    if (!options.compare.empty()) comparison = std::make_unique<OutputFile>(options.compare);
    size_t rendered = 0;
    {
        Reader reader(options.input, options.threads);
        const cv::Size size(info.width, info.height);
        Writer writer(output.temporary(), size, info.fps, reader.context(), origin,
            options.threads, options.crf, options.preset, options.audio);
        std::unique_ptr<Writer> compare_writer;
        if (comparison) compare_writer = std::make_unique<Writer>(comparison->temporary(), cv::Size(size.width * 2, size.height),
            info.fps, reader.context(), origin, options.threads, options.crf, options.preset, options.audio);
        reader.set_audio_callback([&](const AVPacket* packet) {
            writer.audio(packet);
            if (compare_writer) compare_writer->audio(packet);
        });
        VideoFrame frame;
        cv::Mat stabilized;
        std::unique_ptr<cv::FontFace> label_font;
        if (compare_writer) label_font = std::make_unique<cv::FontFace>("sans");
        for (;;) {
            bool available;
            { ScopeTimer timer(profile, "render.decode_and_audio"); available = reader.next(frame); }
            if (!available) break;
            if (rendered >= frames.size() || std::abs((frame.pts_us - origin) / 1000000.0 - frames[rendered].seconds) > 0.000001)
                throw std::runtime_error("Input changed between analysis and render passes");
            auto& warp = warps[rendered];
            renderer.render(frame.bgr, stabilized, warp, analysis_size, options.crop, horizontal, profile);
            const auto duration = static_cast<int64_t>(std::llround(frames[rendered].duration * 1000000));
            { ScopeTimer timer(profile, "render.encode"); writer.video(stabilized, frame.pts_us, duration); }
            if (compare_writer) {
                ScopeTimer timer(profile, "comparison.compose_and_encode");
                cv::Mat combined;
                cv::hconcat(frame.bgr, stabilized, combined);
                cv::putText(combined, "INPUT", {20, 35}, {255, 255, 255}, *label_font, 24, 600);
                cv::putText(combined, "STABILIZED", {size.width + 20, 35}, {255, 255, 255}, *label_font, 24, 600);
                compare_writer->video(combined, frame.pts_us, duration);
            }
            ++rendered;
            if (rendered % 120 == 0) std::cerr << "\rRendered " << rendered << '/' << frames.size() << std::flush;
        }
        if (rendered != frames.size()) throw std::runtime_error("Frame count changed between passes");
        { ScopeTimer timer(profile, "render.encode_flush"); writer.finish(); }
        if (compare_writer) { ScopeTimer timer(profile, "comparison.flush"); compare_writer->finish(); }
    }
    std::cerr << "\rRendered " << rendered << '/' << frames.size() << '\n';
    const double total_ms = milliseconds(start);
    const double duration = frames.back().seconds + frames.back().duration;
    const auto resets = std::count_if(frames.begin(), frames.end(), [](const auto& f) { return f.motion.reset; });
    const auto limited = std::count_if(warps.begin(), warps.end(), [](const auto& w) { return w.strength < 0.999; });
    struct rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const double cpu_s = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
                         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
    nlohmann::json report{
        {"schema_version", 1}, {"version", STILLFLOW_VERSION},
        {"input", fs::absolute(options.input).string()}, {"output", fs::absolute(options.output).string()},
        {"frames_csv", fs::absolute(frames_path).string()},
        {"acceleration", renderer.report()},
        {"video", {{"width", info.width}, {"height", info.height}, {"nominal_fps", info.fps},
            {"frames", frames.size()}, {"duration_s", duration}, {"input_codec", info.codec},
            {"display_rotation_applied", info.rotation}, {"analysis_width", analysis_size.width}, {"analysis_height", analysis_size.height}}},
        {"settings", {{"rolling_shutter", options.rolling_shutter}, {"scan_axis", horizontal ? "x" : "y"},
            {"bands", options.bands}, {"features", options.features}, {"smooth_seconds", options.smooth_seconds},
            {"rs_seconds", options.rs_seconds}, {"rs_strength", options.rs_strength}, {"crop_per_edge", options.crop},
            {"retained_area_ratio", std::pow(1 - 2 * options.crop, 2)}, {"threads", options.threads},
            {"crf", options.crf}, {"preset", options.preset}, {"audio_copy", options.audio}, {"comparison", bool(comparison)}}},
        {"performance", {{"wall_ms", total_ms}, {"analysis_and_plan_ms", analysis_ms},
            {"end_to_end_fps", frames.size() * 1000.0 / total_ms}, {"processing_to_duration_ratio", total_ms / 1000 / duration},
            {"process_cpu_s", cpu_s}, {"peak_rss_mib", usage.ru_maxrss / 1024.0}}},
        {"diagnostics", {{"tracking_resets", resets}, {"crop_limited_frames", limited},
            {"note", "Motion residuals are fit diagnostics, not ground-truth image quality metrics."}}},
        {"build", {{"compiler", __VERSION__}, {"cplusplus", __cplusplus}, {"opencv", CV_VERSION},
            {"ffmpeg", av_version_info()}, {"logical_cpus", std::thread::hardware_concurrency()},
#ifdef NDEBUG
            {"configuration", "release"}
#else
            {"configuration", "debug"}
#endif
        }},
        {"stages", profile.json()}
    };
    OutputFile report_file(options.report), csv_file(frames_path);
    write_csv(csv_file.temporary(), frames, warps);
    {
        std::ofstream file(report_file.temporary());
        file.exceptions(std::ios::badbit | std::ios::failbit);
        file << report.dump(2) << '\n';
    }
    output.publish();
    if (comparison) comparison->publish();
    csv_file.publish();
    report_file.publish();
    std::cout << std::fixed << std::setprecision(2) << frames.size() << " frames, " << total_ms / 1000
              << " s, " << frames.size() * 1000.0 / total_ms << " end-to-end fps, " << usage.ru_maxrss / 1024.0
              << " MiB peak RSS\nReport: " << options.report << '\n';
    return 0;
}
} // namespace stillflow
