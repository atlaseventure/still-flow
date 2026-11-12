#include "stillflow/desktop.h"
#include "internal.hpp"
#include "stillflow/adapters/backends.hpp"
#include "stillflow/adapters/stabilize.hpp"
#include "stillflow/backends/cpu.hpp"
#ifdef STILLFLOW_C_CUDA
#include "stillflow/backends/cuda.hpp"
#endif
#include <sstream>

namespace {
class Progress final : public stillflow::Progress {
  public:
    explicit Progress(const sf_file_callbacks* callbacks) : callbacks_(callbacks ? *callbacks : sf_file_callbacks{}) {}
    void event(const stillflow::ProgressEvent& value) override {
        if (callbacks_.progress) {
            const sf_progress_event event{static_cast<sf_progress_stage>(value.stage),
                                          value.stage_complete,
                                          value.processed_frames,
                                          value.total_frames,
                                          value.processed_us,
                                          value.duration_us,
                                          value.fraction};
            callbacks_.progress(callbacks_.runtime.user_data, &event);
        }
    }
    void send(const std::string& message) {
        if (callbacks_.runtime.log)
            callbacks_.runtime.log(callbacks_.runtime.user_data, message.c_str());
    }
    void backend(const stillflow::RenderingReport& report) override {
        std::string text = "Render backend: " + report.backend;
        if (!report.backends.empty())
            text += " (" + report.backends.back().info.device + ")";
        for (const auto& failure : report.failures)
            text += "; " + failure.backend + ": " + failure.reason;
        send(text);
    }
    void codec(const std::string& stream, const stillflow::CodecReport& report) override {
        std::string text = "Codec " + stream + ": " + report.implementation;
        if (!report.device.empty())
            text += " (" + report.device + ", " + std::to_string(report.hardware_frames) + "/" +
                    std::to_string(report.frames) + " hardware frames)";
        for (const auto& reason : report.fallbacks)
            text += "; " + reason;
        send(text);
    }
    void analyzing(const stillflow::VideoInfo& info, stillflow::Size size, bool horizontal) override {
        send("Analyze " + std::to_string(info.width) + "x" + std::to_string(info.height) + " at " +
             std::to_string(size.width) + "x" + std::to_string(size.height) + ", scan axis " +
             (horizontal ? "x" : "y"));
    }
    void analyzed(size_t frames, bool) override { send("Analyzed " + std::to_string(frames) + " frames"); }
    void rendered(size_t frames, size_t total, bool) override {
        send("Rendered " + std::to_string(frames) + "/" + std::to_string(total));
    }

  private:
    sf_file_callbacks callbacks_{};
};
std::filesystem::path path(const char* text) {
    return text ? std::filesystem::path(reinterpret_cast<const char8_t*>(text)) : std::filesystem::path{};
}
} // namespace
extern "C" {
sf_file_options sf_file_options_default(void) {
    sf_file_options value{};
    value.stabilization = sf_settings_default();
    value.stabilization.motion_estimator = SF_MOTION_OPENCV;
    value.stream = sf_stream_settings_default();
    value.preset = "fast";
    value.threads = 12;
    value.crf = 18;
    return value;
}
const char* sf_desktop_build_description(void) {
    try {
        static const auto description = stillflow::build_description();
        return description.c_str();
    } catch (...) {
        return "stillflow (build description unavailable)";
    }
}
sf_status sf_stabilize_file(const sf_file_options* value, const sf_file_callbacks* callbacks, sf_run_summary* summary,
                            sf_error* error) {
    if (summary)
        *summary = {};
    return stillflow::c_api::boundary(error, [&] {
        stillflow::c_api::require(value);
        stillflow::c_api::require(summary);
        stillflow::c_api::boolean(value->overwrite);
        if (!value->preset)
            throw std::invalid_argument("Missing encoder preset");
        if (value->stabilization.motion_estimator != SF_MOTION_OPENCV)
            throw std::invalid_argument("Desktop processing requires the OpenCV estimator");
        if (value->mode != SF_FILE_OFFLINE && value->mode != SF_FILE_STREAM)
            throw std::invalid_argument("Invalid file processing mode");
        if (value->scan_axis < SF_SCAN_AUTO || value->scan_axis > SF_SCAN_Y || value->decoder < SF_DECODER_AUTO ||
            value->decoder > SF_DECODER_NVDEC || value->encoder < SF_ENCODER_AUTO ||
            value->encoder > SF_ENCODER_NVENC || value->audio < SF_AUDIO_AUTO || value->audio > SF_AUDIO_DROP)
            throw std::invalid_argument("Invalid media selector");
        constexpr const char* scans[]{"auto", "x", "y"};
        constexpr const char* decoders[]{"auto", "cpu", "nvdec"};
        constexpr const char* encoders[]{"auto", "openh264", "nvenc"};
        stillflow::Options options;
        static_cast<stillflow::StabilizationSettings&>(options) = stillflow::c_api::settings(&value->stabilization);
        options.input = path(value->input);
        options.output = path(value->output);
        options.compare = path(value->compare);
        options.report = path(value->report);
        options.scan_axis = scans[value->scan_axis];
        options.preset = value->preset;
        options.backend = stillflow::c_api::backend_name(value->backend);
        if (value->backend == SF_BACKEND_GLES)
            throw stillflow::BackendError("GLES is unavailable in the desktop file adapter");
#ifndef STILLFLOW_C_CUDA
        if (value->backend == SF_BACKEND_CUDA)
            throw stillflow::BackendError("CUDA is unavailable in this build");
#endif
        options.decoder = decoders[value->decoder];
        options.encoder = encoders[value->encoder];
        options.threads = value->threads;
        options.crf = value->crf;
        options.audio = static_cast<stillflow::AudioMode>(value->audio);
        options.overwrite = value->overwrite != 0;
        options.realtime = value->mode == SF_FILE_STREAM;
        options.stream = {value->stream.lookahead_seconds, value->stream.max_buffered_frames};
        const auto* runtime_callbacks = callbacks ? &callbacks->runtime : nullptr;
        stillflow::c_api::Runtime runtime(runtime_callbacks);
        auto executor = stillflow::make_thread_executor(options.threads);
        options.execution = runtime.settings();
        if (!runtime_callbacks || !runtime_callbacks->execute)
            options.execution.executor = executor.get();
        auto media = stillflow::make_media_factory();
        auto images = stillflow::make_image_processing();
#ifdef STILLFLOW_C_CUDA
        images = stillflow::make_cuda_image_processing(std::move(images));
#endif
        std::vector<stillflow::BackendProvider> providers;
#ifdef STILLFLOW_C_CUDA
        providers.push_back({"cuda", stillflow::make_cuda_backend, true});
#endif
        providers.push_back({"cpu", [&] { return stillflow::make_cpu_backend(options.execution); }});
        auto renderers = stillflow::make_renderer_factory(std::move(providers));
        auto outputs = stillflow::make_output_storage();
        auto host = stillflow::make_runtime();
        Progress events(callbacks);
        auto result = stillflow::stabilize(options, {*media, *images, *renderers, *outputs, *host, events});
        summary->frames = result.frames;
        summary->wall_ms = result.wall_ms;
        summary->cpu_seconds = result.resources.cpu_seconds;
        summary->peak_rss_mib = result.resources.peak_rss_mib;
    });
}
}
