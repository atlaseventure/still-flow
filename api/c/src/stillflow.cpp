#include "internal.hpp"
#include "stillflow/application/stream.hpp"
#include "stillflow/backends/cpu.hpp"
#ifdef STILLFLOW_C_GLES
#include "stillflow/backends/gles.hpp"
#endif
#include <cmath>

using namespace stillflow;
namespace {
Image import_image(const sf_image* value) {
    c_api::require(value);
    if (value->plane_count < 1 || value->plane_count > 3 || value->format < SF_GRAY8 || value->format > SF_I420 ||
        bool(value->retain) != bool(value->release))
        throw std::invalid_argument("Invalid image descriptor");
    HostImageView view{{value->width, value->height},
                       static_cast<PixelFormat>(value->format),
                       {},
                       {},
                       {value->kr, value->kb, value->full_range != 0, value->chroma_x, value->chroma_y}};
    for (int i = 0; i < value->plane_count; ++i) {
        c_api::require(&value->planes[i]);
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
    explicit sf_analyzer(const sf_runtime* callbacks) : runtime(callbacks) {}
};
struct sf_stream {
    c_api::Runtime runtime;
    std::unique_ptr<StreamAnalyzer> analyzer;
    StabilizationSettings settings;
    Size size;
    bool horizontal = false;
    explicit sf_stream(const sf_runtime* callbacks) : runtime(callbacks) {}
};
struct sf_plan {
    Plan plan;
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
        auto factory = make_renderer_factory(std::move(providers));
        renderer = factory->create(backend);
    }
};
struct sf_frame {
    Image image;
    HostImageView view;
};
extern "C" {
sf_status sf_settings_default(sf_settings* value) {
    return c_api::boundary(nullptr, [&] {
        c_api::require(value);
        const auto size = value->struct_size;
        *value = {size, 960, 1200, 12, SF_CROP_AUTO, 1, SF_MOTION_TRANSLATION, 0.45, 0.30, 0.08, 1.0};
    });
}
sf_status sf_image_default(sf_image* value) {
    return c_api::boundary(nullptr, [&] {
        c_api::require(value);
        const auto size = value->struct_size;
        *value = {};
        value->struct_size = size;
        value->kr = 0.2126f;
        value->kb = 0.0722f;
        value->chroma_y = 0.5f;
        for (auto& plane : value->planes)
            plane.struct_size = sizeof(sf_plane);
    });
}
const char* sf_build_description(void) { return "stillflow " STILLFLOW_VERSION " (C ABI 1, MIT core)"; }
sf_status sf_analyzer_create(const sf_settings* value, const sf_video* video, const sf_runtime* callbacks,
                             sf_analyzer** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out)
            throw std::invalid_argument("Missing analyzer output");
        auto settings = c_api::settings(value);
        c_api::require(video);
        if (video->width < 32 || video->height < 32)
            throw std::invalid_argument("Invalid video dimensions");
        const double scale = std::min(1.0, double(settings.analysis_width) / video->width);
        Size size{std::max(32, int(std::lround(video->width * scale))),
                  std::max(32, int(std::lround(video->height * scale)))};
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
        Profile profile;
        analyzer->analyzer->push(import_image(image), pts, duration, profile);
    });
}
sf_status sf_analyzer_next_recovery_frame(sf_analyzer* analyzer, sf_recovery_frame* frame, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!analyzer)
            throw std::invalid_argument("Missing analyzer");
        c_api::require(frame);
        frame->available = 0;
        frame->index = 0;
        frame->pts_us = 0;
        if (auto request = analyzer->analyzer->next_recovery_frame()) {
            frame->index = request->index;
            frame->pts_us = request->pts_us;
            frame->available = 1;
        }
    });
}
sf_status sf_analyzer_recover(sf_analyzer* analyzer, const sf_image* image, int64_t pts, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!analyzer)
            throw std::invalid_argument("Missing analyzer");
        analyzer->runtime.settings().check();
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
        plan->plan = analyzer->analyzer->finish();
        *out = plan.release();
    });
}
void sf_analyzer_destroy(sf_analyzer* value) { delete value; }
sf_status sf_plan_get_info(const sf_plan* plan, sf_plan_info* info, sf_error* error) {
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
void sf_plan_destroy(sf_plan* plan) { delete plan; }
sf_status sf_stream_settings_default(sf_stream_settings* value) {
    return c_api::boundary(nullptr, [&] {
        c_api::require(value);
        const auto size = value->struct_size;
        *value = {size, 1.5, 120};
    });
}
sf_status sf_stream_create(const sf_settings* value, const sf_stream_settings* stream, const sf_video* video,
                           const sf_runtime* callbacks, sf_stream** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out)
            throw std::invalid_argument("Missing stream output");
        c_api::require(stream);
        c_api::require(video);
        auto settings = c_api::settings(value);
        if (video->width < 32 || video->height < 32)
            throw std::invalid_argument("Invalid video dimensions");
        const double scale = std::min(1.0, double(settings.analysis_width) / video->width);
        const Size size{std::max(32, int(std::lround(video->width * scale))),
                        std::max(32, int(std::lround(video->height * scale)))};
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
        result->settings = settings;
        result->size = size;
        result->horizontal = video->horizontal_scan != 0;
        result->analyzer = std::make_unique<StreamAnalyzer>(
            settings, StreamSettings{stream->lookahead_seconds, stream->max_buffered_frames}, size, video->fps,
            result->horizontal, std::move(motion), result->runtime.settings());
        *out = result.release();
    });
}
sf_status sf_stream_push(sf_stream* stream, const sf_image* image, int64_t pts, int64_t duration, int32_t* accepted,
                         sf_error* error) {
    if (accepted)
        *accepted = 0;
    return c_api::boundary(error, [&] {
        if (!stream || !accepted)
            throw std::invalid_argument("Missing stream push argument");
        stream->runtime.settings().check();
        Profile profile;
        *accepted = stream->analyzer->push(import_image(image), pts, duration, profile);
    });
}
sf_status sf_stream_finish(sf_stream* stream, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!stream)
            throw std::invalid_argument("Missing stream");
        stream->analyzer->finish();
    });
}
sf_status sf_stream_take_ready(sf_stream* stream, sf_ready_frame* frame, sf_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!stream || !out)
            throw std::invalid_argument("Missing stream output");
        c_api::require(frame);
        frame->available = 0;
        auto plan = std::make_unique<sf_plan>();
        plan->plan.settings = stream->settings;
        plan->plan.analysis.size = stream->size;
        plan->plan.analysis.horizontal = stream->horizontal;
        plan->plan.analysis.frames.resize(1);
        plan->plan.warps.resize(1);
        ReadyWarp ready;
        if (!stream->analyzer->take_ready(ready))
            return;
        plan->plan.analysis.origin_us = ready.pts_us;
        plan->plan.analysis.frames[0] = std::move(ready.observation);
        plan->plan.analysis.frames[0].seconds = 0;
        plan->plan.warps[0] = std::move(ready.warp);
        frame->index = ready.index;
        frame->pts_us = ready.pts_us;
        frame->duration_us = ready.duration_us;
        frame->lookahead_seconds = ready.lookahead_seconds;
        frame->available = 1;
        *out = plan.release();
    });
}
void sf_stream_destroy(sf_stream* stream) { delete stream; }

sf_status sf_renderer_create(const sf_runtime* callbacks, sf_renderer** out, sf_error* error) {
    return sf_renderer_create_backend("cpu", callbacks, out, error);
}
sf_status sf_renderer_create_backend(const char* backend, const sf_runtime* callbacks, sf_renderer** out,
                                     sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out || !backend)
            throw std::invalid_argument("Missing renderer output");
        auto result = std::make_unique<sf_renderer>(callbacks, backend);
        result->runtime.settings().check();
        *out = result.release();
    });
}
sf_status sf_renderer_get_info(const sf_renderer* renderer, sf_renderer_info* info, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!renderer)
            throw std::invalid_argument("Missing renderer");
        c_api::require(info);
        const auto report = renderer->renderer->report();
        const auto size = info->struct_size;
        *info = {};
        info->struct_size = size;
        if (!report.backends.empty()) {
            const auto& active = report.backends.back();
            std::strncpy(info->backend, active.info.id.c_str(), sizeof(info->backend) - 1);
            std::strncpy(info->device, active.info.device.c_str(), sizeof(info->device) - 1);
            info->gpu = active.gpu;
        }
        if (!report.failures.empty())
            std::strncpy(info->fallback_reason, report.failures.back().reason.c_str(),
                         sizeof(info->fallback_reason) - 1);
    });
}
sf_status sf_renderer_render(sf_renderer* renderer, sf_plan* plan, size_t index, const sf_image* image, sf_frame** out,
                             sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!renderer || !plan || !out)
            throw std::invalid_argument("Missing render argument");
        Profile profile;
        RenderSession session(plan->plan, *renderer->renderer, renderer->runtime.settings());
        auto frame = std::make_unique<sf_frame>();
        frame->image = session.render(index, import_image(image), profile);
        frame->view = map_host_image(frame->image);
        *out = frame.release();
    });
}
void sf_renderer_destroy(sf_renderer* renderer) { delete renderer; }
sf_status sf_frame_get_image(const sf_frame* frame, sf_image* image, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!frame)
            throw std::invalid_argument("Missing frame");
        c_api::require(image);
        sf_image_default(image);
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
            image->planes[i] = {sizeof(sf_plane), view.planes[i].bytes.data(), view.planes[i].bytes.size(),
                                view.planes[i].row_stride};
    });
}
void sf_frame_destroy(sf_frame* frame) { delete frame; }
}
