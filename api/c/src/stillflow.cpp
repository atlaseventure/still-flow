#include "internal.hpp"
#include "stillflow/application/stream.hpp"
#include "stillflow/backends/cpu.hpp"
#ifdef STILLFLOW_C_GLES
#include "stillflow/backends/gles.hpp"
#endif
#include <chrono>
#include <cmath>

using namespace stillflow;
namespace {
double profile_now() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Image import_image(const sf_image* value) {
    c_api::require(value);
    if (value->plane_count < 1 || value->plane_count > 3 || value->format < SF_GRAY8 || value->format > SF_I420 ||
        bool(value->retain) != bool(value->release))
        throw std::invalid_argument("Invalid image descriptor");
    c_api::boolean(value->full_range);
    HostImageView view{{value->width, value->height},
                       static_cast<PixelFormat>(value->format),
                       {},
                       {},
                       {value->kr, value->kb, value->full_range != 0, value->chroma_x, value->chroma_y}};
    for (int i = 0; i < value->plane_count; ++i) {
        if (!value->planes[i].data || !value->planes[i].size_bytes)
            throw std::invalid_argument("Missing plane data");
        view.planes.push_back({value->planes[i].row_stride, {value->planes[i].data, value->planes[i].size_bytes}});
    }
    // Validate extents before reading or retaining caller memory.
    view.lifetime = std::shared_ptr<const void>(value, [](const void*) {});
    (void)borrow_host_image(view);
    if (value->retain) {
        struct Lease {
            void* data;
            void (*release)(void*);
            explicit Lease(const sf_image& image) : data(image.user_data), release(image.release) {
                image.retain(data);
            }
            ~Lease() { release(data); }
        };
        view.lifetime = std::make_shared<Lease>(*value);
    } else {
        auto storage = std::make_shared<std::array<std::vector<uint8_t>, 3>>();
        for (size_t i = 0; i < view.planes.size(); ++i) {
            (*storage)[i].assign(view.planes[i].bytes.begin(), view.planes[i].bytes.end());
            view.planes[i].bytes = (*storage)[i];
        }
        view.lifetime = storage;
    }
    return borrow_host_image(std::move(view));
}
} // namespace
struct sf_analyzer {
    c_api::Runtime runtime;
    std::unique_ptr<Analyzer> analyzer;
    Size source, analysis;
    explicit sf_analyzer(const sf_runtime* callbacks) : runtime(callbacks) {}
};
struct sf_stream {
    c_api::Runtime runtime;
    std::unique_ptr<StreamAnalyzer> analyzer;
    struct Statistics {
        uint64_t count = 0;
        double total = 0, minimum = 0, maximum = 0;
    };
    std::map<std::string, Statistics> profile;
    bool finished = false;
    Size source;
    void record(const Profile& call) {
        for (const auto& [name, sample] : call.stages()) {
            if (!sample.count)
                continue;
            auto& stats = profile[name];
            stats.minimum = stats.count ? std::min(stats.minimum, sample.minimum) : sample.minimum;
            stats.maximum = stats.count ? std::max(stats.maximum, sample.maximum) : sample.maximum;
            stats.total += sample.total;
            stats.count += sample.count;
        }
    }
    StabilizationSettings settings;
    Size size;
    bool horizontal = false;
    explicit sf_stream(const sf_runtime* callbacks) : runtime(callbacks) {}
};
struct sf_plan {
    Plan plan;
    Size source;
};
struct sf_frame_plan {
    StabilizationSettings settings;
    Size analysis, source;
    bool horizontal = false;
    Warp warp;
    sf_frame_info info{};
};
struct sf_renderer {
    c_api::Runtime runtime;
    std::unique_ptr<Renderer> renderer;
    explicit sf_renderer(const sf_runtime* callbacks, const std::string& backend) : runtime(callbacks) {
        std::vector<BackendProvider> providers;
#ifdef STILLFLOW_C_GLES
        providers.push_back({"gles", [] { return make_gles_backend(); }, true});
#endif
        providers.push_back({"cpu", [this] { return make_cpu_backend(runtime.settings()); }});
        if (backend != "auto" && std::none_of(providers.begin(), providers.end(),
                                              [&](const auto& provider) { return provider.id == backend; }))
            throw BackendError("Rendering backend is unavailable in this build: " + backend);
        auto factory = make_renderer_factory(std::move(providers));
        renderer = factory->create(backend);
    }
};
struct sf_frame {
    Image image;
    HostImageView view;
    sf_frame_info info{};
};
namespace {
Size analysis_size(const sf_video* video, const StabilizationSettings& settings) {
    c_api::require(video);
    c_api::boolean(video->horizontal_scan);
    if (video->width < 32 || video->height < 32 || !std::isfinite(video->fps) || video->fps <= 0)
        throw std::invalid_argument("Invalid video geometry or frame rate");
    const double scale = std::min(1.0, double(settings.analysis_width) / video->width);
    return {std::max(32, int(std::lround(video->width * scale))),
            std::max(32, int(std::lround(video->height * scale)))};
}
void validate_image_size(const sf_image* image, Size source, Size analysis) {
    c_api::require(image);
    if (!((image->width == source.width && image->height == source.height) ||
          (image->format == SF_GRAY8 && image->width == analysis.width && image->height == analysis.height)))
        throw std::invalid_argument("Frame dimensions do not match the video or analysis proxy");
}
} // namespace
extern "C" {
sf_status sf_opencv_set_threads(int32_t threads, sf_error* error) {
    return c_api::boundary(error, [&] {
#ifdef STILLFLOW_C_OPENCV
        configure_opencv_threads(threads);
#else
        (void)threads;
        throw BackendError("OpenCV is unavailable");
#endif
    });
}
sf_settings sf_settings_default(void) {
    return {960, 1200, 12, SF_CROP_AUTO, 1, SF_MOTION_TRANSLATION, 0.45, 0.30, 0.08, 1.0};
}
sf_image sf_image_default(void) {
    sf_image image{};
    image.kr = 0.2126f;
    image.kb = 0.0722f;
    image.chroma_y = 0.5f;
    return image;
}
const char* sf_build_description(void) { return "stillflow " STILLFLOW_VERSION " (pre-release C API, MIT core)"; }
sf_status sf_analyzer_create(const sf_settings* value, const sf_video* video, const sf_runtime* callbacks,
                             sf_analyzer** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out)
            throw std::invalid_argument("Missing analyzer output");
        auto settings = c_api::settings(value);
        const auto size = analysis_size(video, settings);
        std::unique_ptr<MotionAnalyzer> motion;
        if (value->motion_estimator == SF_MOTION_TRANSLATION)
            motion = make_translation_analyzer(settings);
#ifdef STILLFLOW_C_OPENCV
        else if (value->motion_estimator == SF_MOTION_OPENCV)
            motion = make_opencv_analyzer(settings, video->horizontal_scan != 0);
#endif
        else
            throw BackendError("Requested motion estimator is unavailable");
        auto result = std::make_unique<sf_analyzer>(callbacks);
        result->source = {video->width, video->height};
        result->analysis = size;
        result->analyzer = std::make_unique<Analyzer>(settings, size, video->fps, video->horizontal_scan != 0,
                                                      std::move(motion), result->runtime.settings());
        *out = result.release();
    });
}
sf_status sf_analyzer_push(sf_analyzer* analyzer, const sf_image* image, int64_t pts, int64_t duration,
                           sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!analyzer)
            throw std::invalid_argument("Missing analyzer");
        analyzer->runtime.settings().check();
        validate_image_size(image, analyzer->source, analyzer->analysis);
        Profile profile;
        analyzer->analyzer->push(import_image(image), pts, duration, profile);
    });
}
sf_status sf_analyzer_next_recovery_frame(sf_analyzer* analyzer, sf_recovery_frame* frame, sf_error* error) {
    if (frame)
        *frame = {};
    return c_api::boundary(error, [&]() -> sf_status {
        c_api::require(analyzer);
        c_api::require(frame);
        if (auto request = analyzer->analyzer->next_recovery_frame()) {
            *frame = {request->index, request->pts_us};
            return SF_OK;
        }
        return SF_END;
    });
}
sf_status sf_analyzer_recover(sf_analyzer* analyzer, const sf_image* image, int64_t pts, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!analyzer)
            throw std::invalid_argument("Missing analyzer");
        analyzer->runtime.settings().check();
        validate_image_size(image, analyzer->source, analyzer->analysis);
        Profile profile;
        analyzer->analyzer->recover(import_image(image), pts, profile);
    });
}
sf_status sf_analyzer_finish(sf_analyzer* analyzer, sf_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!analyzer || !out)
            throw std::invalid_argument("Missing analyzer or plan output");
        auto plan = std::make_unique<sf_plan>();
        plan->source = analyzer->source;
        plan->plan = analyzer->analyzer->finish();
        *out = plan.release();
    });
}
void sf_analyzer_destroy(sf_analyzer* value) { delete value; }
sf_status sf_plan_get_info(const sf_plan* plan, sf_plan_info* info, sf_error* error) {
    if (info)
        *info = {};
    return c_api::boundary(error, [&] {
        if (!plan)
            throw std::invalid_argument("Missing plan");
        c_api::require(info);
        info->frames = plan->plan.warps.size();
        info->analysis_width = plan->plan.analysis.size.width;
        info->analysis_height = plan->plan.analysis.size.height;
        info->origin_us = plan->plan.analysis.origin_us;
    });
}
sf_status sf_plan_get_frame(const sf_plan* plan, size_t index, sf_frame_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        c_api::require(plan);
        c_api::require(out);
        const auto& p = plan->plan;
        const auto& observation = p.analysis.frames.at(index);
        auto frame = std::make_unique<sf_frame_plan>();
        frame->settings = p.settings;
        frame->source = plan->source;
        frame->analysis = p.analysis.size;
        frame->horizontal = p.analysis.horizontal;
        frame->warp = p.warps.at(index);
        frame->info = {index, p.analysis.origin_us + static_cast<int64_t>(std::llround(observation.seconds * 1000000)),
                       static_cast<int64_t>(std::llround(observation.duration * 1000000)), 0};
        *out = frame.release();
    });
}
sf_status sf_frame_plan_get_info(const sf_frame_plan* plan, sf_frame_info* info, sf_error* error) {
    if (info)
        *info = {};
    return c_api::boundary(error, [&] {
        c_api::require(plan);
        c_api::require(info);
        *info = plan->info;
    });
}
sf_status sf_frame_plan_get_map(const sf_frame_plan* plan, int32_t width, int32_t height, sf_map* out,
                                sf_error* error) {
    if (out)
        *out = {};
    return c_api::boundary(error, [&] {
        c_api::require(out);
        if (!plan || width < 2 || height < 2)
            throw std::invalid_argument("Invalid map geometry");
        const auto map =
            prepare_map(plan->warp, plan->analysis, {width, height}, plan->settings.crop, plan->horizontal);
        if (static_cast<size_t>(map.columns) * map.rows > SF_MAP_MAX_VERTICES)
            throw std::logic_error("Render map exceeds the public vertex capacity");
        out->columns = map.columns;
        out->rows = map.rows;
        out->origin_x = map.origin_x;
        out->origin_y = map.origin_y;
        out->step_x = map.step_x;
        out->step_y = map.step_y;
        out->grid_x = map.grid_x;
        out->grid_y = map.grid_y;
        for (size_t i = 0; i < static_cast<size_t>(map.columns) * map.rows; ++i) {
            out->offsets[2 * i] = map.offset_x[i];
            out->offsets[2 * i + 1] = map.offset_y[i];
        }
    });
}
void sf_frame_plan_destroy(sf_frame_plan* plan) { delete plan; }
void sf_plan_destroy(sf_plan* plan) { delete plan; }
sf_stream_settings sf_stream_settings_default(void) { return {1.35, 120}; }
sf_status sf_stream_create(const sf_settings* value, const sf_stream_settings* stream, const sf_video* video,
                           const sf_runtime* callbacks, sf_stream** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out)
            throw std::invalid_argument("Missing stream output");
        c_api::require(stream);
        auto settings = c_api::settings(value);
        const auto size = analysis_size(video, settings);
        std::unique_ptr<MotionAnalyzer> motion;
        if (value->motion_estimator == SF_MOTION_TRANSLATION)
            motion = make_translation_analyzer(settings);
#ifdef STILLFLOW_C_OPENCV
        else if (value->motion_estimator == SF_MOTION_OPENCV)
            motion = make_opencv_analyzer(settings, video->horizontal_scan != 0, true);
#endif
        else
            throw BackendError("Requested motion estimator is unavailable");
        auto result = std::make_unique<sf_stream>(callbacks);
        result->source = {video->width, video->height};
        result->settings = settings;
        result->size = size;
        result->horizontal = video->horizontal_scan != 0;
        result->analyzer = std::make_unique<StreamAnalyzer>(
            settings, StreamSettings{stream->lookahead_seconds, stream->max_buffered_frames}, size, video->fps,
            result->horizontal, std::move(motion), result->runtime.settings(),
#ifdef STILLFLOW_C_OPENCV
            value->motion_estimator == SF_MOTION_OPENCV ? make_opencv_analyzer(settings, result->horizontal) :
#endif
                                                        make_translation_analyzer(settings));
        *out = result.release();
    });
}
sf_status sf_stream_push(sf_stream* stream, const sf_image* image, int64_t pts, int64_t duration, sf_error* error) {
    return c_api::boundary(error, [&]() -> sf_status {
        c_api::require(stream);
        stream->runtime.settings().check();
        if (stream->finished)
            throw std::logic_error("Stream input is sealed");
        validate_image_size(image, stream->source, stream->size);
        Profile call{profile_now};
        bool accepted;
        {
            ScopeTimer timer(call, "stream.analyze");
            accepted = stream->analyzer->push(import_image(image), pts, duration, call);
        }
        stream->record(call);
        return accepted ? SF_OK : SF_AGAIN;
    });
}
sf_status sf_stream_finish(sf_stream* stream, sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(stream);
        stream->runtime.settings().check();
        if (!stream->finished) {
            stream->analyzer->finish();
            stream->finished = true;
        }
    });
}
sf_status sf_stream_take_ready(sf_stream* stream, sf_frame_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&]() -> sf_status {
        c_api::require(stream);
        c_api::require(out);
        stream->runtime.settings().check();
        auto plan = std::make_unique<sf_frame_plan>();
        ReadyWarp ready;
        Profile call{profile_now};
        bool available;
        {
            ScopeTimer timer(call, "stream.plan");
            available = stream->analyzer->take_ready(ready);
        }
        stream->record(call);
        if (!available)
            return stream->finished ? SF_END : SF_AGAIN;
        plan->settings = stream->settings;
        plan->source = stream->source;
        plan->analysis = stream->size;
        plan->horizontal = stream->horizontal;
        plan->warp = std::move(ready.warp);
        plan->info = {ready.index, ready.pts_us, ready.duration_us, ready.lookahead_seconds};
        *out = plan.release();
        return SF_OK;
    });
}
sf_status sf_stream_get_profile(const sf_stream* stream, sf_profile_visitor visitor, void* data, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!stream || !visitor)
            throw std::invalid_argument("Missing stream or profile visitor");
        for (const auto& [stage, stats] : stream->profile) {
            const sf_profile_entry entry{stage.c_str(), stats.count, stats.total, stats.minimum, stats.maximum};
            visitor(data, &entry);
        }
    });
}
void sf_stream_destroy(sf_stream* stream) { delete stream; }
sf_status sf_renderer_create(sf_backend backend, const sf_runtime* callbacks, sf_renderer** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        c_api::require(out);
        auto result = std::make_unique<sf_renderer>(callbacks, c_api::backend_name(backend));
        result->runtime.settings().check();
        *out = result.release();
    });
}
sf_status sf_renderer_get_info(const sf_renderer* renderer, sf_renderer_info* info, sf_error* error) {
    if (info)
        *info = {};
    return c_api::boundary(error, [&] {
        if (!renderer)
            throw std::invalid_argument("Missing renderer");
        c_api::require(info);
        const auto report = renderer->renderer->report();
        *info = {};
        if (!report.backends.empty()) {
            const auto& active = report.backends.back();
            info->backend = active.info.id == "cpu"    ? SF_BACKEND_CPU
                            : active.info.id == "gles" ? SF_BACKEND_GLES
                                                       : SF_BACKEND_CUDA;
            std::strncpy(info->device, active.info.device.c_str(), sizeof(info->device) - 1);
            info->gpu = active.gpu;
        }
        if (!report.failures.empty())
            std::strncpy(info->fallback_reason, report.failures.back().reason.c_str(),
                         sizeof(info->fallback_reason) - 1);
    });
}
sf_status sf_renderer_render(sf_renderer* renderer, const sf_frame_plan* plan, const sf_image* image, sf_frame** out,
                             sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!renderer || !plan || !out)
            throw std::invalid_argument("Missing render argument");
        validate_image_size(image, plan->source, plan->source);
        renderer->runtime.settings().check();
        Profile profile;
        auto warp = plan->warp;
        auto frame = std::make_unique<sf_frame>();
        frame->image = renderer->renderer->render(import_image(image), warp, plan->analysis, plan->settings.crop,
                                                  plan->horizontal, profile);
        renderer->runtime.settings().check();
        frame->view = map_host_image(frame->image);
        frame->info = plan->info;
        *out = frame.release();
    });
}
void sf_renderer_destroy(sf_renderer* renderer) { delete renderer; }
sf_status sf_frame_get_info(const sf_frame* frame, sf_frame_info* info, sf_error* error) {
    if (info)
        *info = {};
    return c_api::boundary(error, [&] {
        c_api::require(frame);
        c_api::require(info);
        *info = frame->info;
    });
}
sf_status sf_frame_get_image(const sf_frame* frame, sf_image* image, sf_error* error) {
    if (image)
        *image = {};
    return c_api::boundary(error, [&] {
        if (!frame)
            throw std::invalid_argument("Missing frame");
        c_api::require(image);
        *image = sf_image_default();
        const auto& view = frame->view;
        image->width = view.size.width;
        image->height = view.size.height;
        image->format = static_cast<int32_t>(view.format);
        image->plane_count = static_cast<int32_t>(view.planes.size());
        image->kr = view.color.kr;
        image->kb = view.color.kb;
        image->full_range = view.color.full_range;
        image->chroma_x = view.color.chroma_x;
        image->chroma_y = view.color.chroma_y;
        for (size_t i = 0; i < view.planes.size(); ++i)
            image->planes[i] = {view.planes[i].bytes.data(), view.planes[i].bytes.size(), view.planes[i].row_stride};
    });
}
void sf_frame_destroy(sf_frame* frame) { delete frame; }
}
