#include "stillflow/application/stabilize.hpp"
#include "stillflow/domain/motion.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace stillflow {
namespace {
void validate_options(const Options& o) {
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
    if (o.scan_axis != "auto" && o.scan_axis != "x" && o.scan_axis != "y")
        throw std::invalid_argument("scan_axis must be auto, x or y");
}
} // namespace
RunSummary stabilize(const Options& request, Services services) {
    auto options = request;
    if (options.report.empty())
        options.report = options.output.string() + ".profile.json";
    validate_options(options);
    const auto start = Clock::now();
    auto frames_path = options.report;
    frames_path.replace_extension(".frames.csv");
    services.outputs.validate(options, frames_path);
    services.runtime.configure(options.threads);
    Profile profile;
    const auto backend_start = Clock::now();
    auto renderer = services.renderers.create(options.backend);
    profile.add("render.backend_init", milliseconds(backend_start));
    services.progress.backend(renderer->report());
    VideoInfo info;
    Size analysis_size;
    std::vector<FrameMotion> frames;
    int64_t origin = 0;
    bool horizontal = false;
    {
        auto reader = services.media.reader(options.input, options.threads);
        info = reader->info();
        horizontal = options.scan_axis == "x" || (options.scan_axis == "auto" && info.horizontal_scan);
        const double scale = std::min(1.0, static_cast<double>(options.analysis_width) / info.width);
        analysis_size = {std::max(32, static_cast<int>(std::lround(info.width * scale))),
                         std::max(32, static_cast<int>(std::lround(info.height * scale)))};
        auto estimator = services.images.analyzer(options, horizontal);
        VideoFrame decoded;
        Image previous;
        int segment = 0;
        services.progress.analyzing(info, analysis_size, horizontal);
        for (;;) {
            bool available;
            {
                ScopeTimer timer(profile, "analysis.decode");
                available = reader->next(decoded);
            }
            if (!available)
                break;
            if (frames.empty())
                origin = decoded.pts_us;
            Image current;
            {
                ScopeTimer timer(profile, "analysis.preprocess");
                current = services.images.gray_proxy(decoded.bgr, analysis_size);
            }
            FrameMotion frame;
            frame.seconds = (decoded.pts_us - origin) / 1000000.0;
            frame.duration = decoded.duration_us / 1000000.0;
            frame.motion.row_delta.resize(options.bands);
            if (previous) {
                const double gap = frame.seconds - frames.back().seconds;
                if (gap > std::max(0.5, 5 / info.fps))
                    frame.motion.reset = true;
                else
                    frame.motion = estimator->estimate(previous, current, profile);
                if (frame.motion.reset)
                    ++segment;
            }
            frame.segment = segment;
            frames.push_back(std::move(frame));
            previous = std::move(current);
            if (frames.size() % 120 == 0)
                services.progress.analyzed(frames.size(), false);
        }
        services.progress.analyzed(frames.size(), true);
    }
    if (frames.empty())
        throw std::runtime_error("Input contains no decoded video frames");
    // Actual PTS differences, rather than nominal FPS, define frame durations for VFR files.
    for (size_t i = 0; i + 1 < frames.size(); ++i)
        frames[i].duration = frames[i + 1].seconds - frames[i].seconds;
    auto warps = plan_warps(frames, analysis_size, options, horizontal, profile);
    const auto analysis_ms = milliseconds(start);
    auto output = services.outputs.stage(options.output);
    std::unique_ptr<OutputFile> comparison;
    if (!options.compare.empty())
        comparison = services.outputs.stage(options.compare);
    size_t rendered = 0;
    {
        auto reader = services.media.reader(options.input, options.threads);
        const Size size(info.width, info.height);
        auto writer = services.media.writer(output->temporary(), size, info.fps, *reader, origin, options);
        std::unique_ptr<VideoWriter> compare_writer;
        if (comparison)
            compare_writer = services.media.writer(comparison->temporary(), {size.width * 2, size.height}, info.fps,
                                                   *reader, origin, options);
        reader->set_audio_callback([&](const AudioPacket& packet) {
            writer->audio(packet);
            if (compare_writer)
                compare_writer->audio(packet);
        });
        VideoFrame frame;
        for (;;) {
            bool available;
            {
                ScopeTimer timer(profile, "render.decode_and_audio");
                available = reader->next(frame);
            }
            if (!available)
                break;
            if (rendered >= frames.size() ||
                std::abs((frame.pts_us - origin) / 1000000.0 - frames[rendered].seconds) > 0.000001)
                throw std::runtime_error("Input changed between analysis and render passes");
            auto& warp = warps[rendered];
            auto stabilized = renderer->render(frame.bgr, warp, analysis_size, options.crop, horizontal, profile);
            const auto duration = static_cast<int64_t>(std::llround(frames[rendered].duration * 1000000));
            {
                ScopeTimer timer(profile, "render.encode");
                writer->video(stabilized, frame.pts_us, duration);
            }
            if (compare_writer) {
                ScopeTimer timer(profile, "comparison.compose_and_encode");
                auto combined = services.images.comparison(frame.bgr, stabilized);
                compare_writer->video(combined, frame.pts_us, duration);
            }
            ++rendered;
            if (rendered % 120 == 0)
                services.progress.rendered(rendered, frames.size(), false);
        }
        if (rendered != frames.size())
            throw std::runtime_error("Frame count changed between passes");
        {
            ScopeTimer timer(profile, "render.encode_flush");
            writer->finish();
        }
        if (compare_writer) {
            ScopeTimer timer(profile, "comparison.flush");
            compare_writer->finish();
        }
    }
    services.progress.rendered(rendered, frames.size(), true);
    const double total_ms = milliseconds(start);
    RunSummary summary{frames.size(), total_ms, services.runtime.resources()};
    auto report_file = services.outputs.stage(options.report);
    auto csv_file = services.outputs.stage(frames_path);
    services.outputs.write_report(
        report_file->temporary(), csv_file->temporary(),
        {options, info, analysis_size, horizontal, frames, warps, profile, renderer->report(), summary, analysis_ms});
    output->publish();
    if (comparison)
        comparison->publish();
    csv_file->publish();
    report_file->publish();
    return summary;
}
} // namespace stillflow
