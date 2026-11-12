#include "stillflow/adapters/stabilize.hpp"
#include "pipeline.hpp"
#include "stillflow/application/engine.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <future>
#include <optional>
#include <stdexcept>
#include <utility>

namespace stillflow {
namespace {
using Clock = std::chrono::steady_clock;
double now_ms() { return std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count(); }
double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
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

void progress(Progress& sink, ProgressStage stage, size_t frames = 0, size_t total = 0, int64_t position_us = 0,
              int64_t duration_us = 0, bool complete = false) {
    constexpr std::array starts{0.0, 0.40, 0.50, 0.55, 0.98, 1.0};
    constexpr std::array weights{0.40, 0.10, 0.05, 0.43, 0.02, 0.0};
    double ratio = total ? double(frames) / total : duration_us > 0 ? double(position_us) / duration_us : 0;
    ratio = complete ? 1.0 : std::clamp(ratio, 0.0, 0.99);
    const auto index = static_cast<size_t>(stage);
    sink.event({stage, complete, frames, total, position_us, duration_us, starts[index] + weights[index] * ratio});
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
    progress(services.progress, ProgressStage::analyze, 0, info.estimated_frames, 0, info.duration_us);

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
    std::vector<Profile> worker_profiles(workers, Profile(now_ms));
    // Indexed by frame; owned separately so workers write through stable pointers.
    std::vector<std::unique_ptr<Motion>> estimated;
    Channel<VideoFrame> decoded(frame_queue_depth);
    Channel<Task> tasks(static_cast<size_t>(workers) * 2);
    Profile decode_profile(now_ms);
    ThreadGroup group;
    group.watch(decoded);
    group.watch(tasks);
    group.spawn([&] {
        VideoFrame frame;
        for (;;) {
            options.execution.check();
            {
                ScopeTimer timer(decode_profile, "analysis.decode");
                if (!reader->next_luma(frame))
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
            options.execution.check();
            renderer.poll(profile);
            const auto& source = *decoded_frame;
            if (frames.empty())
                result.origin = source.pts_us;
            Image current;
            {
                ScopeTimer timer(profile, "analysis.preprocess");
                current = services.images.gray_proxy(source.image, result.size);
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
            progress(services.progress, ProgressStage::analyze, frames.size(), info.estimated_frames,
                     source.pts_us - result.origin + source.duration_us, info.duration_us);
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
    const auto duration_us =
        frames.empty() ? 0
                       : static_cast<int64_t>(std::llround((frames.back().seconds + frames.back().duration) * 1000000));
    progress(services.progress, ProgressStage::analyze, frames.size(), frames.size(), duration_us, duration_us, true);
    progress(services.progress, ProgressStage::recover, 0, frames.size(), 0, duration_us);
    Analyzer collector(options, result.size, info.fps, result.horizontal,
                       services.images.analyzer(options, result.horizontal), options.execution);
    for (auto& frame : frames)
        collector.push_observation(std::move(frame.motion),
                                   result.origin + static_cast<int64_t>(std::llround(frame.seconds * 1000000)),
                                   static_cast<int64_t>(std::llround(frame.duration * 1000000)));
    size_t recovery_scanned = 0;
    int64_t recovery_position_us = 0;
    if (auto request = collector.next_recovery_frame()) {
        auto recovery_reader = services.media.reader(
            options.input, options.threads, {options.decoder, services.renderers.prefers_gpu(options.backend)});
        VideoFrame source;
        for (size_t index = 0; request && recovery_reader->next_luma(source); ++index) {
            options.execution.check();
            recovery_scanned = index + 1;
            recovery_position_us = source.pts_us - result.origin + source.duration_us;
            progress(services.progress, ProgressStage::recover, recovery_scanned, frames.size(), recovery_position_us,
                     duration_us);
            if (index == request->index) {
                collector.recover(services.images.gray_proxy(source.image, result.size), source.pts_us, profile);
                request = collector.next_recovery_frame();
            }
        }
        if (request)
            throw std::runtime_error("Input ended during tracking recovery");
        result.recovery_decoder = recovery_reader->codec();
    }
    result.frames = collector.finish_analysis().frames;
    progress(services.progress, ProgressStage::recover, recovery_scanned, frames.size(), recovery_position_us,
             duration_us, true);
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
RunSummary stabilize_stream(const Options& options, Services& services, Clock::time_point start) {
    Profile profile(now_ms);
    auto renderer = services.renderers.create(options.backend);
    services.progress.backend(renderer->report());
    const bool gpu = services.renderers.prefers_gpu(renderer->report().backend);
    auto reader = services.media.reader(options.input, options.threads, {options.decoder, gpu, true});
    const auto info = reader->info();
    const Size size(info.width, info.height);
    const double scale = std::min(1.0, double(options.analysis_width) / info.width);
    const Size analysis_size{std::max(32, int(std::lround(info.width * scale))),
                             std::max(32, int(std::lround(info.height * scale)))};
    const bool horizontal = options.scan_axis == "x" || (options.scan_axis == "auto" && info.horizontal_scan);
    services.progress.analyzing(info, analysis_size, horizontal);
    StreamAnalyzer analyzer(options, options.stream, analysis_size, info.fps, horizontal,
                            services.images.stream_analyzer(options, horizontal), options.execution,
                            services.images.analyzer(options, horizontal));
    auto output = services.outputs.stage(options.output);
    std::unique_ptr<OutputFile> comparison;
    if (!options.compare.empty())
        comparison = services.outputs.stage(options.compare);
    std::unique_ptr<VideoWriter> writer, compare_writer;
    AudioPackets audio;
    reader->set_audio_callback([&](const AudioPacket& packet) { audio.push_back(packet.retain()); });
    struct Pending {
        VideoFrame frame;
        AudioPackets audio;
        double received_ms;
    };
    std::deque<Pending> delayed;
    std::vector<FrameMotion> frames;
    std::vector<Warp> warps;
    StreamStatistics statistics;
    size_t decoded = 0, planned = 0;
    struct RenderItem {
        Pending input;
        ReadyWarp plan;
    };
    Channel<RenderItem> render_queue(2);
    Profile render_profile(now_ms);
    ThreadGroup group;
    group.watch(render_queue);
    const auto render = [&] {
        while (auto work = render_queue.pop()) {
            options.execution.check();
            auto& item = work->input;
            auto& ready = work->plan;
            Image stabilized =
                renderer->render(item.frame.image, ready.warp, analysis_size, options.crop, horizontal, render_profile);
            for (const auto& packet : item.audio) {
                writer->audio(*packet);
                if (compare_writer)
                    compare_writer->audio(*packet);
            }
            {
                ScopeTimer timer(render_profile, "render.encode");
                writer->video(stabilized, ready.pts_us, ready.duration_us);
            }
            if (compare_writer) {
                ScopeTimer timer(render_profile, "comparison.compose_and_encode");
                compare_writer->video(services.images.comparison(item.frame.image, stabilized), ready.pts_us,
                                      ready.duration_us);
            }
            render_profile.add("stream.residence", now_ms() - item.received_ms);
            render_profile.add("stream.lookahead", ready.lookahead_seconds * 1000);
            if (ready.lookahead_seconds + 2e-6 < options.stream.lookahead_seconds)
                ++statistics.shortened_lookahead_frames;
            frames.push_back(std::move(ready.observation));
            warps.push_back(std::move(ready.warp));
        }
    };
    const auto drain = [&] {
        ReadyWarp ready;
        for (;;) {
            const double plan_start = now_ms();
            if (!analyzer.take_ready(ready))
                break;
            profile.add("stream.plan", now_ms() - plan_start);
            if (delayed.empty() || ready.index != planned || ready.pts_us != delayed.front().frame.pts_us)
                throw std::logic_error("Streaming frame and plan are out of order");
            auto item = std::move(delayed.front());
            delayed.pop_front();
            if (!render_queue.push({std::move(item), std::move(ready)}))
                return;
            ready = {};
            ++planned;
        }
    };
    try {
        for (;;) {
            options.execution.check();
            if (render_queue.cancelled())
                break;
            const double work_start = now_ms();
            VideoFrame frame;
            {
                ScopeTimer timer(profile, "stream.decode");
                if (!reader->next(frame))
                    break;
            }
            if (!writer) {
                const EncodingSettings encoding{options.threads, options.crf, options.preset, options.audio};
                writer = services.media.writer(output->temporary(), size, info.fps, *reader, frame.pts_us, encoding,
                                               {options.encoder, gpu});
                if (comparison)
                    compare_writer =
                        services.media.writer(comparison->temporary(), {size.width * 2, size.height}, info.fps, *reader,
                                              frame.pts_us, encoding, {options.encoder, gpu});
                group.spawn(render);
            }
            Image proxy;
            {
                ScopeTimer timer(profile, "stream.proxy");
                proxy = services.images.gray_proxy(frame.analysis_image ? frame.analysis_image : frame.image,
                                                   analysis_size);
            }
            {
                ScopeTimer timer(profile, "stream.analyze");
                if (!analyzer.push(proxy, frame.pts_us, frame.duration_us, profile))
                    throw std::logic_error("Streaming queue was not drained");
            }
            frame.analysis_image.reset();
            delayed.push_back({std::move(frame), std::move(audio), work_start});
            frame = {};
            audio.clear();
            ++decoded;
            statistics.peak_buffered_frames = std::max(statistics.peak_buffered_frames, delayed.size());
            drain();
            const double work_ms = now_ms() - work_start;
            profile.add("stream.frame_work", work_ms);
            if (work_ms > 1000 / info.fps)
                ++statistics.frame_budget_overruns;
        }
        if (!writer)
            throw std::runtime_error("Input contains no decoded video frames");
        analyzer.finish();
        drain();
        render_queue.close();
    } catch (...) {
        group.fail(std::current_exception());
    }
    group.finish();
    profile.merge(render_profile);
    for (const auto& packet : audio) {
        writer->audio(*packet);
        if (compare_writer)
            compare_writer->audio(*packet);
    }
    writer->finish();
    if (compare_writer)
        compare_writer->finish();
    reader->set_audio_callback(nullptr);
    if (!delayed.empty() || frames.size() != decoded)
        throw std::logic_error("Streaming flush lost frames");
    std::vector<std::pair<std::string, CodecReport>> codecs{{"stream_decoder", reader->codec()},
                                                            {"encoder", writer->codec()}};
    if (compare_writer)
        codecs.emplace_back("comparison_encoder", compare_writer->codec());
    for (const auto& [name, codec] : codecs)
        services.progress.codec(name, codec);
    services.progress.rendered(decoded, decoded, true);
    RunSummary summary{decoded, milliseconds(start), services.runtime.resources()};
    const auto frames_path = frames_report_path(options);
    auto report_file = services.outputs.stage(options.report), csv_file = services.outputs.stage(frames_path);
    services.outputs.write_report(report_file->temporary(), csv_file->temporary(),
                                  {options, frames_path, info, analysis_size, horizontal, frames, warps, profile,
                                   renderer->report(), std::move(codecs), summary, 0, statistics});
    options.execution.check();
    output->publish();
    if (comparison)
        comparison->publish();
    csv_file->publish();
    report_file->publish();
    return summary;
}
} // namespace

RunSummary stabilize(const Options& request, Services services) {
    const auto options = resolve_options(request);
    options.execution.check();
    const auto start = Clock::now();
    const auto frames_path = frames_report_path(options);
    const std::array outputs{options.output, options.compare, options.report, frames_path};
    services.outputs.validate(options.input, outputs, options.overwrite);
    services.media.validate_output(options.output);
    if (!options.compare.empty())
        services.media.validate_output(options.compare);
    services.runtime.configure(options.threads);
    if (options.realtime)
        return stabilize_stream(options, services, start);
    Profile profile(now_ms);
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
    Plan plan;
    progress(services.progress, ProgressStage::plan);
    {
        ScopeTimer timer(profile, "plan.optimize");
        plan = Planner::plan({analysis_size, horizontal, origin, frames}, options, options.execution);
    }
    progress(services.progress, ProgressStage::plan, 0, 0, 0, 0, true);
    auto& warps = plan.warps;
    const auto analysis_ms = milliseconds(start);
    auto& renderer = pending.get(profile);
    RenderSession session(plan, renderer, options.execution);
    auto output = services.outputs.stage(options.output);
    std::unique_ptr<OutputFile> comparison;
    if (!options.compare.empty())
        comparison = services.outputs.stage(options.compare);
    size_t rendered = 0;
    const auto duration_us =
        static_cast<int64_t>(std::llround((frames.back().seconds + frames.back().duration) * 1000000));
    progress(services.progress, ProgressStage::render, 0, frames.size(), 0, duration_us);
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
        Profile decode_profile(now_ms), encode_profile(now_ms), compare_profile(now_ms);
        ThreadGroup group;
        group.watch(decoded);
        group.watch(encode);
        group.watch(compare);
        group.spawn([&] {
            AudioPackets audio;
            reader->set_audio_callback([&](const AudioPacket& packet) { audio.push_back(packet.retain()); });
            for (;;) {
                options.execution.check();
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
                options.execution.check();
                EncodeItem out{item->frame.image, nullptr, item->frame.pts_us, 0, std::move(item->audio)};
                if (out.input) {
                    const auto pts = item->frame.pts_us;
                    if (rendered >= frames.size() ||
                        std::abs((pts - origin) / 1000000.0 - frames[rendered].seconds) > 0.000001)
                        throw std::runtime_error("Input changed between analysis and render passes");
                    out.stabilized = session.render(rendered, out.input, profile);
                    out.duration_us = static_cast<int64_t>(std::llround(frames[rendered].duration * 1000000));
                    ++rendered;
                    progress(services.progress, ProgressStage::render, rendered, frames.size(),
                             pts - origin + out.duration_us, duration_us);
                    if (rendered % 120 == 0)
                        services.progress.rendered(rendered, frames.size(), false);
                }
                if (compare_writer && !compare.push(out))
                    break;
                if (!encode.push(std::move(out)))
                    break;
            }
            progress(services.progress, ProgressStage::render, rendered, frames.size(), duration_us, duration_us, true);
            progress(services.progress, ProgressStage::finalize);
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
                                  {options,
                                   frames_path,
                                   info,
                                   analysis_size,
                                   horizontal,
                                   frames,
                                   warps,
                                   profile,
                                   renderer.report(),
                                   std::move(codecs),
                                   summary,
                                   analysis_ms,
                                   {}});
    options.execution.check();
    output->publish();
    if (comparison)
        comparison->publish();
    csv_file->publish();
    report_file->publish();
    progress(services.progress, ProgressStage::complete, rendered, frames.size(), duration_us, duration_us, true);
    return summary;
}
} // namespace stillflow
