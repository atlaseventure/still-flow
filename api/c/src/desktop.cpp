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
    explicit Progress(const sf_progress* value) {
        if (value) {
            stillflow::c_api::require(value);
            value_ = *value;
        }
    }
    void send(const std::string& message) {
        if (value_.message)
            value_.message(value_.user_data, message.c_str());
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
    sf_progress value_{};
};
std::filesystem::path path(const char* text) {
    return text ? std::filesystem::path(reinterpret_cast<const char8_t*>(text)) : std::filesystem::path{};
}
} // namespace
extern "C" {
sf_status sf_file_options_default(sf_file_options* value) {
    return stillflow::c_api::boundary(nullptr, [&] {
        if (!value || value->struct_size < offsetof(sf_file_options, realtime))
            throw std::invalid_argument("Missing or undersized file options");
        auto size = value->struct_size;
        std::memset(value, 0, std::min(size, sizeof(sf_file_options)));
        value->struct_size = size;
        value->stabilization.struct_size = sizeof(sf_settings);
        sf_settings_default(&value->stabilization);
        value->stabilization.motion_estimator = SF_MOTION_OPENCV;
        value->scan_axis = "auto";
        value->preset = "fast";
        value->backend = "auto";
        value->decoder = "auto";
        value->encoder = "auto";
        value->threads = 12;
        value->crf = 18;
        value->audio = 1;
        if (size >= sizeof(sf_file_options)) {
            value->lookahead_seconds = 1.5;
            value->max_buffered_frames = 120;
        }
    });
}
const char* sf_desktop_build_description(void) {
    try {
        static const auto description = stillflow::build_description();
        return description.c_str();
    } catch (...) {
        return "stillflow (build description unavailable)";
    }
}
sf_status sf_stabilize_file(const sf_file_options* value, const sf_runtime* callbacks, const sf_progress* progress,
                            sf_run_summary* summary, sf_error* error) {
    return stillflow::c_api::boundary(error, [&] {
        if (!value || value->struct_size < offsetof(sf_file_options, realtime))
            throw std::invalid_argument("Missing or undersized file options");
        stillflow::c_api::require(summary);
        if (!value->scan_axis || !value->preset || !value->backend || !value->decoder || !value->encoder)
            throw std::invalid_argument("Missing file option string");
        if (value->stabilization.motion_estimator != SF_MOTION_OPENCV)
            throw std::invalid_argument("Desktop processing requires the OpenCV estimator");
        stillflow::Options options;
        static_cast<stillflow::StabilizationSettings&>(options) = stillflow::c_api::settings(&value->stabilization);
        options.input = path(value->input);
        options.output = path(value->output);
        options.compare = path(value->compare);
        options.report = path(value->report);
        options.scan_axis = value->scan_axis;
        options.preset = value->preset;
        options.backend = value->backend;
        options.decoder = value->decoder;
        options.encoder = value->encoder;
        options.threads = value->threads;
        options.crf = value->crf;
        options.audio = value->audio != 0;
        options.overwrite = value->overwrite != 0;
        if (value->struct_size >= sizeof(sf_file_options)) {
            options.realtime = value->realtime != 0;
            options.stream = {value->lookahead_seconds, value->max_buffered_frames};
        }
        stillflow::c_api::Runtime runtime(callbacks);
        auto executor = stillflow::make_thread_executor(options.threads);
        options.execution = runtime.settings();
        if (!callbacks || !callbacks->execute)
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
        Progress events(progress);
        auto result = stillflow::stabilize(options, {*media, *images, *renderers, *outputs, *host, events});
        summary->frames = result.frames;
        summary->wall_ms = result.wall_ms;
        summary->cpu_seconds = result.resources.cpu_seconds;
        summary->peak_rss_mib = result.resources.peak_rss_mib;
    });
}
}
