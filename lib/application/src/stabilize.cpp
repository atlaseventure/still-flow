#include "stillflow/application/stabilize.hpp"
#include "pipeline.hpp"
#include "stillflow/domain/motion.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <optional>
#include <stdexcept>
#include <utility>

namespace stillflow {
namespace {
// Frames buffered between stages. Small depths keep memory bounded while
// letting decode, rendering and each encoder run concurrently.
constexpr size_t frame_queue_depth = 4;

using AudioPackets = std::vector<std::shared_ptr<const AudioPacket>>;

// Backend initialization (for example CUDA context creation) runs while the
// analysis pass decodes, so its latency is hidden behind CPU work.
class PendingRenderer {
  public:
    PendingRenderer(RendererFactory& factory, const std::string& backend, Progress& progress) : progress_(progress) {
        future_ = std::async(std::launch::async, [&factory, backend] {
            const auto start = Clock::now();
            auto renderer = factory.create(backend);
            return std::pair{std::move(renderer), milliseconds(start)};
        });
    }
    // Surfaces initialization failures without waiting for the analysis pass.
    void poll(Profile& profile) {
        if (!renderer_ && future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            get(profile);
    }
    Renderer& get(Profile& profile) {
        if (!renderer_) {
            auto [renderer, ms] = future_.get();
            renderer_ = std::move(renderer);
            profile.add("render.backend_init", ms);
            progress_.backend(renderer_->report());
        }
        return *renderer_;
    }

  private:
    Progress& progress_;
    std::future<std::pair<std::unique_ptr<Renderer>, double>> future_;
    std::unique_ptr<Renderer> renderer_;
};

struct Analysis {
    VideoInfo info;
    Size size;
    bool horizontal = false;
    int64_t origin = 0;
    std::vector<FrameMotion> frames;
    CodecReport decoder;
    std::optional<CodecReport> recovery_decoder;
};

// Re-decode only when short missing spans need a bridge. Retain two endpoint
// proxies instead of keeping an entire clip of analysis images in memory.
void bridge_gaps(Analysis& analysis, const Options& options, Services& services, Profile& profile) {
    auto& frames = analysis.frames;
    std::vector<std::pair<size_t, size_t>> gaps;
    for (size_t first = 1; first < frames.size();) {
        if (frames[first].motion.state != MotionState::missing) {
            ++first;
            continue;
        }
        size_t last = first + 1;
        while (last < frames.size() && frames[last].motion.state == MotionState::missing)
            ++last;
        if (last < frames.size() && !frames[last].motion.reset &&
            frames[last].seconds - frames[first - 1].seconds <= 0.20)
            gaps.emplace_back(first, last);
        first = last;
    }
    if (gaps.empty())
        return;
    ScopeTimer timer(profile, "analysis.bridge");
    auto reader = services.media.reader(options.input, options.threads,
                                        {options.decoder, services.renderers.prefers_gpu(options.backend)});
    auto analyzer = services.images.analyzer(options, analysis.horizontal);
    const Point center((analysis.size.width - 1) * 0.5, (analysis.size.height - 1) * 0.5);
    Image anchor;
    size_t gap = 0, index = 0;
    VideoFrame source;
    while (gap < gaps.size() && reader->next(source)) {
        const auto [first, last] = gaps[gap];
        if (index >= frames.size() ||
            std::abs((source.pts_us - analysis.origin) / 1000000.0 - frames[index].seconds) > 0.000001)
            throw std::runtime_error("Input changed during tracking recovery");
        if (index == first - 1)
            anchor = services.images.gray_proxy(source.bgr, analysis.size);
        if (index == last) {
            const auto current = services.images.gray_proxy(source.bgr, analysis.size);
            const auto bridge = analyzer->estimate(anchor, current, profile);
            if (bridge.state == MotionState::tracked || bridge.state == MotionState::global_fallback ||
                bridge.state == MotionState::descriptor_fallback) {
                // Preserve the reliable outgoing step. Interpolate the missing
                // camera poses so their product exactly reaches the measured anchor.
                const auto end = frames[last].motion.step.inv() * bridge.step;
                const double duration = frames[last - 1].seconds - frames[first - 1].seconds;
                auto previous = Matrix3::eye();
                for (size_t i = first; i < last; ++i) {
                    const double t = (frames[i].seconds - frames[first - 1].seconds) / duration;
                    const auto camera = interpolate_similarity(end, center, t);
                    auto& motion = frames[i].motion;
                    motion.step = camera * previous.inv();
                    motion.state = MotionState::bridged;
                    motion.confidence = bridge.confidence * 0.5;
                    previous = camera;
                }
            }
            ++gap;
            anchor = gap < gaps.size() && gaps[gap].first - 1 == index ? current : Image{};
        }
        ++index;
    }
    if (gap != gaps.size())
        throw std::runtime_error("Input changed during tracking recovery");
    analysis.recovery_decoder = reader->codec();
}

// Decoding runs ahead on its own thread and proxies are generated in order;
// motion estimation between consecutive proxies is independent per pair and
// runs on a worker pool.
Analysis analyze(const Options& options, Services& services, PendingRenderer& renderer, Profile& profile) {
    Analysis result;
    // The backend is still initializing, so hardware decoding follows the
    // provider expected to win rather than the one that will be selected.
    auto reader = services.media.reader(options.input, options.threads,
                                        {options.decoder, services.renderers.prefers_gpu(options.backend)});
    result.info = reader->info();
    const auto& info = result.info;
    result.horizontal = options.scan_axis == "x" || (options.scan_axis == "auto" && info.horizontal_scan);
    const double scale = std::min(1.0, static_cast<double>(options.analysis_width) / info.width);
    result.size = {std::max(32, static_cast<int>(std::lround(info.width * scale))),
                   std::max(32, static_cast<int>(std::lround(info.height * scale)))};
    services.progress.analyzing(info, result.size, result.horizontal);

    struct Task {
        Image previous, current;
        Motion* output;
    };
    const int workers =
        std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, std::max(1, options.threads));
    std::vector<std::unique_ptr<MotionAnalyzer>> analyzers;
    analyzers.reserve(workers);
    for (int i = 0; i < workers; ++i)
        analyzers.push_back(services.images.analyzer(options, result.horizontal));
    std::vector<Profile> worker_profiles(workers);
    // Indexed by frame; owned separately so workers write through stable pointers.
    std::vector<std::unique_ptr<Motion>> estimated;
    Channel<VideoFrame> decoded(frame_queue_depth);
    Channel<Task> tasks(static_cast<size_t>(workers) * 2);
    Profile decode_profile;
    ThreadGroup group;
    group.watch(decoded);
    group.watch(tasks);
    group.spawn([&] {
        VideoFrame frame;
        for (;;) {
            {
                ScopeTimer timer(decode_profile, "analysis.decode");
                if (!reader->next(frame))
                    break;
            }
            if (!decoded.push(std::exchange(frame, {})))
                break;
        }
        decoded.close();
    });
    for (int i = 0; i < workers; ++i)
        group.spawn([&, i] {
            while (auto task = tasks.pop())
                *task->output = analyzers[i]->estimate(task->previous, task->current, worker_profiles[i]);
        });

    auto& frames = result.frames;
    try {
        Image previous;
        while (auto decoded_frame = decoded.pop()) {
            renderer.poll(profile);
            const auto& source = *decoded_frame;
            if (frames.empty())
                result.origin = source.pts_us;
            Image current;
            {
                ScopeTimer timer(profile, "analysis.preprocess");
                current = services.images.gray_proxy(source.bgr, result.size);
            }
            FrameMotion frame;
            frame.seconds = (source.pts_us - result.origin) / 1000000.0;
            frame.duration = source.duration_us / 1000000.0;
            frame.motion.row_delta.resize(options.bands);
            estimated.emplace_back();
            if (previous) {
                const double gap = frame.seconds - frames.back().seconds;
                if (gap > std::max(0.5, 5 / info.fps)) {
                    frame.motion.reset = true;
                    frame.motion.state = MotionState::time_gap;
                    frame.motion.failure = "timestamp_gap";
                } else {
                    estimated.back() = std::make_unique<Motion>();
                    if (!tasks.push({previous, current, estimated.back().get()}))
                        break;
                }
            }
            frames.push_back(std::move(frame));
            previous = std::move(current);
            if (frames.size() % 120 == 0)
                services.progress.analyzed(frames.size(), false);
        }
        tasks.close();
    } catch (...) {
        group.fail(std::current_exception());
    }
    group.finish();
    result.decoder = reader->codec();
    profile.merge(decode_profile);
    for (const auto& worker : worker_profiles)
        profile.merge(worker);
    for (size_t i = 0; i < frames.size(); ++i) {
        if (estimated[i])
            frames[i].motion = std::move(*estimated[i]);
    }
    bridge_gaps(result, options, services, profile);
    int segment = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (i && frames[i].motion.reset)
            ++segment;
        frames[i].segment = segment;
    }
    services.progress.analyzed(frames.size(), true);
    services.progress.codec("analysis_decoder", result.decoder);
    return result;
}

struct DecodedItem {
    VideoFrame frame; // Empty after the last frame; carries trailing audio only.
    AudioPackets audio;
};
struct EncodeItem {
    Image input, stabilized; // stabilized is empty for trailing audio.
    int64_t pts_us = 0, duration_us = 0;
    AudioPackets audio;
};

// Per writer, audio packets precede the frame they were demuxed with, matching
// the order of a sequential decode/encode loop so outputs are byte-identical.
void write_stream(Channel<EncodeItem>& queue, VideoWriter& writer, ImageProcessing* comparison, Profile& profile,
                  const char* encode_stage, const char* flush_stage) {
    while (auto item = queue.pop()) {
        for (const auto& packet : item->audio)
            writer.audio(*packet);
        if (!item->stabilized)
            continue;
        ScopeTimer timer(profile, encode_stage);
        writer.video(comparison ? comparison->comparison(item->input, item->stabilized) : item->stabilized,
                     item->pts_us, item->duration_us);
    }
    if (queue.cancelled())
        return; // Another stage failed; the staged output is discarded unpublished.
    ScopeTimer timer(profile, flush_stage);
    writer.finish();
}
} // namespace

RunSummary stabilize(const Options& request, Services services) {
    const auto options = resolve_options(request);
    const auto start = Clock::now();
    const auto frames_path = frames_report_path(options);
    const std::array outputs{options.output, options.compare, options.report, frames_path};
    services.outputs.validate(options.input, outputs, options.overwrite);
    services.media.validate_output(options.output);
    if (!options.compare.empty())
        services.media.validate_output(options.compare);
    services.runtime.configure(options.threads);
    Profile profile;
    PendingRenderer pending(services.renderers, options.backend, services.progress);
    auto analysis = analyze(options, services, pending, profile);
    const auto& info = analysis.info;
    const auto& frames = analysis.frames;
    const auto analysis_size = analysis.size;
    const bool horizontal = analysis.horizontal;
    const auto origin = analysis.origin;
    if (frames.empty())
        throw std::runtime_error("Input contains no decoded video frames");
    // Actual PTS differences, rather than nominal FPS, define frame durations for VFR files.
    for (size_t i = 0; i + 1 < frames.size(); ++i)
        analysis.frames[i].duration = frames[i + 1].seconds - frames[i].seconds;
    std::vector<Warp> warps;
    {
        ScopeTimer timer(profile, "plan.optimize");
        warps = plan_warps(frames, analysis_size, options, horizontal);
    }
    const auto analysis_ms = milliseconds(start);
    auto& renderer = pending.get(profile);
    auto output = services.outputs.stage(options.output);
    std::unique_ptr<OutputFile> comparison;
    if (!options.compare.empty())
        comparison = services.outputs.stage(options.compare);
    size_t rendered = 0;
    std::vector<std::pair<std::string, CodecReport>> codecs{{"analysis_decoder", analysis.decoder}};
    if (analysis.recovery_decoder)
        codecs.emplace_back("recovery_decoder", *analysis.recovery_decoder);
    {
        const auto backends = renderer.report().backends;
        const bool gpu = !backends.empty() && backends.back().gpu;
        auto reader = services.media.reader(options.input, options.threads, {options.decoder, gpu});
        const Size size(info.width, info.height);
        const CodecRequest encoder{options.encoder, gpu};
        const EncodingSettings encoding{options.threads, options.crf, options.preset, options.audio};
        auto writer = services.media.writer(output->temporary(), size, info.fps, *reader, origin, encoding, encoder);
        std::unique_ptr<VideoWriter> compare_writer;
        if (comparison)
            compare_writer = services.media.writer(comparison->temporary(), {size.width * 2, size.height}, info.fps,
                                                   *reader, origin, encoding, encoder);
        Channel<DecodedItem> decoded(frame_queue_depth);
        Channel<EncodeItem> encode(frame_queue_depth), compare(frame_queue_depth);
        Profile decode_profile, encode_profile, compare_profile;
        ThreadGroup group;
        group.watch(decoded);
        group.watch(encode);
        group.watch(compare);
        group.spawn([&] {
            AudioPackets audio;
            reader->set_audio_callback([&](const AudioPacket& packet) { audio.push_back(packet.retain()); });
            for (;;) {
                DecodedItem item;
                bool available;
                {
                    ScopeTimer timer(decode_profile, "render.decode_and_audio");
                    available = reader->next(item.frame);
                }
                item.audio = std::move(audio);
                audio.clear();
                if (!decoded.push(std::move(item)) || !available)
                    break;
            }
            reader->set_audio_callback(nullptr);
            decoded.close();
        });
        group.spawn(
            [&] { write_stream(encode, *writer, nullptr, encode_profile, "render.encode", "render.encode_flush"); });
        if (compare_writer)
            group.spawn([&] {
                write_stream(compare, *compare_writer, &services.images, compare_profile,
                             "comparison.compose_and_encode", "comparison.flush");
            });
        try {
            while (auto item = decoded.pop()) {
                EncodeItem out{item->frame.bgr, nullptr, item->frame.pts_us, 0, std::move(item->audio)};
                if (out.input) {
                    const auto pts = item->frame.pts_us;
                    if (rendered >= frames.size() ||
                        std::abs((pts - origin) / 1000000.0 - frames[rendered].seconds) > 0.000001)
                        throw std::runtime_error("Input changed between analysis and render passes");
                    out.stabilized =
                        renderer.render(out.input, warps[rendered], analysis_size, options.crop, horizontal, profile);
                    out.duration_us = static_cast<int64_t>(std::llround(frames[rendered].duration * 1000000));
                    ++rendered;
                    if (rendered % 120 == 0)
                        services.progress.rendered(rendered, frames.size(), false);
                }
                if (compare_writer && !compare.push(out))
                    break;
                if (!encode.push(std::move(out)))
                    break;
            }
            encode.close();
            compare.close();
        } catch (...) {
            group.fail(std::current_exception());
        }
        group.finish();
        codecs.emplace_back("render_decoder", reader->codec());
        codecs.emplace_back("encoder", writer->codec());
        if (compare_writer)
            codecs.emplace_back("comparison_encoder", compare_writer->codec());
        profile.merge(decode_profile);
        profile.merge(encode_profile);
        profile.merge(compare_profile);
        if (rendered != frames.size())
            throw std::runtime_error("Frame count changed between passes");
    }
    services.progress.rendered(rendered, frames.size(), true);
    for (auto it = codecs.begin() + 1; it != codecs.end(); ++it)
        services.progress.codec(it->first, it->second);
    const double total_ms = milliseconds(start);
    RunSummary summary{frames.size(), total_ms, services.runtime.resources()};
    auto report_file = services.outputs.stage(options.report);
    auto csv_file = services.outputs.stage(frames_path);
    services.outputs.write_report(report_file->temporary(), csv_file->temporary(),
                                  {options, frames_path, info, analysis_size, horizontal, frames, warps, profile,
                                   renderer.report(), std::move(codecs), summary, analysis_ms});
    output->publish();
    if (comparison)
        comparison->publish();
    csv_file->publish();
    report_file->publish();
    return summary;
}
} // namespace stillflow
