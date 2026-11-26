#include "stillflow/vision/portable.hpp"
#ifdef STILLFLOW_C_OPENCV
#include "stillflow/vision/opencv.hpp"
#endif
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
    c_api::boolean(value->color.full_range);
    HostImageView view{
        {value->width, value->height},
        static_cast<PixelFormat>(value->format),
        {},
        {},
        {value->color.kr, value->color.kb, value->color.full_range != 0, value->color.chroma_x, value->color.chroma_y}};
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
namespace {
struct StreamState {
    explicit StreamState(const sf_runtime* callbacks) : runtime(callbacks) {}
    c_api::Runtime runtime;
    std::map<std::string, Profile::Statistics> profile;
    bool finished = false;
    Size source;
    void record(const Profile& call) {
        for (const auto& [name, sample] : call.stages()) {
            if (!sample.count)
                continue;
            auto& stats = profile[name];
            stats.merge(sample);
        }
    }
};
} // namespace
struct sf_gyro_stream : StreamState {
    using StreamState::StreamState;
    std::unique_ptr<GyroStream> analyzer;
};
struct sf_stream : StreamState {
    using StreamState::StreamState;
    std::unique_ptr<StreamAnalyzer> analyzer;
    StabilizationSettings settings;
    Size size;
    bool horizontal = false;
};
struct sf_plan {
    Plan plan;
    Size source, output;
    bool gyro = false;
    int64_t origin_us = 0;
    std::vector<sf_frame_info> gyro_infos;
    std::vector<GyroFrameMap> gyro_maps;
};
struct sf_frame_plan {
    bool gyro = false;
    GyroFrameMap gyro_map;
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
void validate_public_map(const GyroFrameMap& map) {
    if (map.row_rotations.size() > SF_GYRO_MAX_ROW_ROTATIONS)
        throw std::logic_error("Gyro map exceeds the public row capacity");
}
GyroSettings gyro_settings(const sf_gyro_settings* settings) {
    c_api::require(settings);
    const auto& s = *settings;
    if ((s.output_width != 0 || s.output_height != 0) && (s.output_width < 2 || s.output_height < 2))
        throw std::invalid_argument("Gyro output dimensions must both be zero or at least two");
    c_api::boolean(s.per_axis);
    c_api::boolean(s.readout_top_to_bottom);
    c_api::boolean(s.adaptive_zoom);
    c_api::boolean(s.zoom_envelope);
    if (s.integration_method != SF_IMU_AUTO && s.integration_method != SF_IMU_GYRO &&
        s.integration_method != SF_IMU_COMPLEMENTARY)
        throw std::invalid_argument("Invalid IMU integration method");
    return {.smoothness = s.smoothness,
            .per_axis = s.per_axis != 0,
            .smoothness_pitch = s.smoothness_pitch,
            .smoothness_yaw = s.smoothness_yaw,
            .smoothness_roll = s.smoothness_roll,
            .max_smoothness = s.max_smoothness,
            .alpha_0_1s = s.alpha_0_1s,
            .readout_ms = s.readout_ms,
            .readout_top_to_bottom = s.readout_top_to_bottom != 0,
            .fov = s.fov,
            .max_zoom = s.max_zoom,
            .max_zoom_iterations = s.max_zoom_iterations,
            .adaptive_zoom_window_s = s.adaptive_zoom_window_s,
            .adaptive_zoom = s.adaptive_zoom != 0,
            .time_offset_s = s.time_offset_s,
            .integration_method = s.integration_method == SF_IMU_COMPLEMENTARY ? ImuIntegrationMethod::complementary
                                                                               : ImuIntegrationMethod::gyro_only,
            .output_size = {s.output_width, s.output_height},
            .zoom_envelope = s.zoom_envelope != 0};
}

LensProfile lens_profile(const sf_lens_profile* lens) {
    c_api::require(lens);
    if (lens->model != SF_LENS_NONE && lens->model != SF_LENS_OPENCV_FISHEYE)
        throw std::invalid_argument("Invalid lens model");
    return {static_cast<LensModel>(lens->model),
            {lens->calib_width, lens->calib_height},
            lens->fx,
            lens->fy,
            lens->cx,
            lens->cy,
            std::to_array(lens->k),
            lens->lens_correction_amount};
}

std::vector<ImuSample> import_imu(const sf_imu_sample* imu_samples, size_t imu_count) {
    if (imu_count)
        c_api::require(imu_samples);
    std::vector<ImuSample> samples;
    samples.reserve(imu_count);
    for (const auto& sample : std::span(imu_samples, imu_count)) {
        c_api::boolean(sample.has_accel);
        samples.push_back(
            {sample.time_s, sample.wx, sample.wy, sample.wz, sample.ax, sample.ay, sample.az, sample.has_accel != 0});
    }
    return samples;
}

StreamSettings stream_settings(const sf_stream_settings* stream) {
    c_api::require(stream);
    c_api::boolean(stream->adaptive_tracking);
    return StreamSettings{stream->lookahead_seconds, stream->max_buffered_frames,
#ifdef __ANDROID__
                          true,
#else
                          false,
#endif
                          stream->adaptive_tracking != 0};
}
sf_status stream_profile(const StreamState* stream, sf_profile_visitor visitor, void* data, sf_error* error) {
    return c_api::boundary(error, [&] {
        if (!stream || !visitor)
            throw std::invalid_argument("Missing stream or profile visitor");
        for (const auto& [stage, stats] : stream->profile) {
            const sf_profile_entry entry{stage.c_str(),          stats.count,           stats.total,
                                         stats.minimum,          stats.maximum,         stats.percentile(0.50),
                                         stats.percentile(0.95), stats.percentile(0.99)};
            visitor(data, &entry);
        }
    });
}
template <class Stream> sf_status finish_stream(Stream* stream, sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(stream);
        stream->runtime.settings().check();
        if (!stream->finished) {
            Profile call{profile_now};
            stream->analyzer->finish(&call);
            stream->record(call);
            stream->finished = true;
        }
    });
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
sf_lens_profile sf_lens_profile_default(void) {
    sf_lens_profile lens{};
    lens.model = SF_LENS_NONE;
    lens.lens_correction_amount = 1;
    return lens;
}
sf_gyro_settings sf_gyro_settings_default(void) {
    const GyroSettings s;
    return {s.smoothness,
            s.per_axis,
            s.smoothness_pitch,
            s.smoothness_yaw,
            s.smoothness_roll,
            s.max_smoothness,
            s.alpha_0_1s,
            s.readout_ms,
            s.readout_top_to_bottom,
            s.fov,
            s.max_zoom,
            s.max_zoom_iterations,
            s.adaptive_zoom_window_s,
            s.adaptive_zoom,
            s.time_offset_s,
            SF_IMU_GYRO,
            s.output_size.width,
            s.output_size.height,
            s.zoom_envelope};
}
sf_status sf_gyro_plan_create(const sf_gyro_settings* settings, const sf_lens_profile* lens, int32_t input_width,
                              int32_t input_height, const sf_frame_timestamp* frames, size_t frame_count,
                              const sf_imu_sample* imu_samples, size_t imu_count, const sf_runtime* callbacks,
                              sf_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        c_api::require(out);
        c_api::require(frames);
        c_api::require(imu_samples);
        if (input_width < 2 || input_height < 2 || frame_count == 0 || imu_count == 0)
            throw std::invalid_argument("Gyro planning requires valid input dimensions, frames and IMU samples");
        const auto config = gyro_settings(settings);
        const auto profile = lens_profile(lens);
        c_api::Runtime runtime(callbacks);
        runtime.settings().check();
        runtime.log("Planning gyro stabilization");
        auto plan = std::make_unique<sf_plan>();
        plan->gyro = true;
        plan->source = {input_width, input_height};
        plan->origin_us = frames[0].pts_us;
        plan->gyro_infos.reserve(frame_count);
        std::vector<double> times;
        times.reserve(frame_count);
        for (size_t i = 0; i < frame_count; ++i) {
            const auto [pts, duration] = frames[i];
            if (duration < 0 || (i && pts <= frames[i - 1].pts_us))
                throw std::invalid_argument("Invalid gyro frame timestamp");
            // Unsigned subtraction covers the entire distance between two ordered int64 timestamps.
            times.push_back((static_cast<uint64_t>(pts) - static_cast<uint64_t>(plan->origin_us)) / 1e6);
            plan->gyro_infos.push_back({i, pts, duration, 0});
        }
        const auto samples = import_imu(imu_samples, imu_count);
        const auto track = integrate_imu(samples, config);
        runtime.settings().check();
        plan->gyro_maps = plan_gyro_frames(times, track, profile, plan->source, config);
        runtime.settings().check();
        std::ranges::for_each(plan->gyro_maps, validate_public_map);
        plan->output = plan->gyro_maps.front().output_size;
        *out = plan.release();
    });
}
sf_image sf_image_default(void) {
    sf_image image{};
    image.color.kr = 0.2126f;
    image.color.kb = 0.0722f;
    image.color.chroma_y = 0.5f;
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
        if (plan->gyro) {
            *info = {plan->gyro_maps.size(), plan->output.width, plan->output.height, plan->origin_us};
            return;
        }
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
        auto frame = std::make_unique<sf_frame_plan>();
        frame->source = plan->source;
        if (plan->gyro) {
            frame->gyro = true;
            frame->gyro_map = plan->gyro_maps.at(index);
            frame->info = plan->gyro_infos.at(index);
            *out = frame.release();
            return;
        }
        const auto& p = plan->plan;
        const auto& observation = p.analysis.frames.at(index);
        frame->settings = p.settings;
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
        c_api::require(plan);
        if (plan->gyro)
            throw std::logic_error("Gyro frame plan has no visual displacement mesh");
        if (width < 2 || height < 2)
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
sf_status sf_frame_plan_get_gyro_map(const sf_frame_plan* plan, sf_gyro_map* out, sf_error* error) {
    if (out)
        *out = {};
    return c_api::boundary(error, [&] {
        c_api::require(plan);
        c_api::require(out);
        if (!plan->gyro)
            throw std::logic_error("Visual frame plan has no gyro map");
        const auto& m = plan->gyro_map;
        out->input_width = m.input_size.width;
        out->input_height = m.input_size.height;
        out->output_width = m.output_size.width;
        out->output_height = m.output_size.height;
        out->lens_model = static_cast<sf_lens_model>(m.lens.model);
        out->in_fx = m.lens.fx;
        out->in_fy = m.lens.fy;
        out->in_cx = m.lens.cx;
        out->in_cy = m.lens.cy;
        std::ranges::copy(m.lens.k, out->k);
        out->lens_correction_amount = m.lens.lens_correction_amount;
        out->out_fx = m.out_fx;
        out->out_fy = m.out_fy;
        out->out_cx = m.out_cx;
        out->out_cy = m.out_cy;
        out->fov_scale = m.fov_scale;
        out->zoom_ratio = m.zoom_ratio;
        out->smooth_factor = m.smooth_factor;
        out->row_count = static_cast<int32_t>(m.row_rotations.size());
        out->sensor_rows = m.sensor_rows;
        for (size_t i = 0; i < m.row_rotations.size(); ++i)
            std::ranges::copy(m.row_rotations[i].values, out->row_rotations + 9 * i);
    });
}
sf_status sf_frame_plan_get_output_size(const sf_frame_plan* plan, int32_t* width, int32_t* height, sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(plan);
        c_api::require(width);
        c_api::require(height);
        const auto size = plan->gyro ? plan->gyro_map.output_size : plan->source;
        *width = size.width;
        *height = size.height;
    });
}
void sf_frame_plan_destroy(sf_frame_plan* plan) { delete plan; }
void sf_plan_destroy(sf_plan* plan) { delete plan; }
sf_stream_settings sf_stream_settings_default(void) {
    return {1.35, 120,
#ifdef __ANDROID__
            1
#else
            0
#endif
    };
}
sf_status sf_stream_create(const sf_settings* value, const sf_stream_settings* stream, const sf_video* video,
                           const sf_runtime* callbacks, sf_stream** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        if (!out)
            throw std::invalid_argument("Missing stream output");
        const auto buffering = stream_settings(stream);
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
            settings, buffering, size, video->fps, result->horizontal, std::move(motion), result->runtime.settings(),
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
sf_status sf_stream_finish(sf_stream* stream, sf_error* error) { return finish_stream(stream, error); }
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
            available = stream->analyzer->take_ready(ready, &call);
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
    return stream_profile(stream, visitor, data, error);
}
void sf_stream_destroy(sf_stream* stream) { delete stream; }
sf_status sf_gyro_stream_create(const sf_gyro_settings* settings, const sf_lens_profile* lens,
                                const sf_stream_settings* stream, int32_t input_width, int32_t input_height,
                                const sf_runtime* callbacks, sf_gyro_stream** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&] {
        c_api::require(out);
        auto result = std::make_unique<sf_gyro_stream>(callbacks);
        result->runtime.settings().check();
        result->source = {input_width, input_height};
        result->analyzer =
            std::make_unique<GyroStream>(gyro_settings(settings), lens_profile(lens), stream_settings(stream),
                                         result->source, result->runtime.settings());
        *out = result.release();
    });
}
sf_status sf_gyro_stream_push_imu(sf_gyro_stream* stream, const sf_imu_sample* samples, size_t count, sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(stream);
        stream->runtime.settings().check();
        if (stream->finished)
            throw std::logic_error("Stream input is sealed");
        stream->analyzer->push_imu(import_imu(samples, count));
    });
}
sf_status sf_gyro_stream_push_frame(sf_gyro_stream* stream, int64_t pts, int64_t duration, sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(stream);
        return stream->analyzer->push_frame(pts, duration) ? SF_OK : SF_AGAIN;
    });
}
sf_status sf_gyro_stream_finish(sf_gyro_stream* stream, sf_error* error) { return finish_stream(stream, error); }
sf_status sf_gyro_stream_take_ready(sf_gyro_stream* stream, sf_frame_plan** out, sf_error* error) {
    if (out)
        *out = nullptr;
    return c_api::boundary(error, [&]() -> sf_status {
        c_api::require(stream);
        c_api::require(out);
        auto plan = std::make_unique<sf_frame_plan>();
        ReadyGyroFrame ready;
        Profile call{profile_now};
        const bool available = stream->analyzer->take_ready(ready, &call);
        stream->record(call);
        if (!available)
            return stream->finished ? SF_END : SF_AGAIN;
        plan->gyro = true;
        plan->source = stream->source;
        validate_public_map(ready.map);
        plan->gyro_map = std::move(ready.map);
        plan->info = {ready.index, ready.pts_us, ready.duration_us, ready.lookahead_seconds};
        *out = plan.release();
        return SF_OK;
    });
}
sf_status sf_gyro_stream_get_output_size(const sf_gyro_stream* stream, int32_t* width, int32_t* height,
                                         sf_error* error) {
    return c_api::boundary(error, [&] {
        c_api::require(stream);
        c_api::require(width);
        c_api::require(height);
        const auto size = stream->analyzer->output_size();
        *width = size.width;
        *height = size.height;
    });
}
sf_status sf_gyro_stream_get_profile(const sf_gyro_stream* stream, sf_profile_visitor visitor, void* data,
                                     sf_error* error) {
    return stream_profile(stream, visitor, data, error);
}
void sf_gyro_stream_destroy(sf_gyro_stream* stream) { delete stream; }
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
        c_api::require(image);
        if (image->width != plan->source.width || image->height != plan->source.height)
            throw std::invalid_argument("Render input dimensions do not match the plan");
        renderer->runtime.settings().check();
        Profile profile;
        auto frame = std::make_unique<sf_frame>();
        if (plan->gyro) {
            double invalid_fraction = 0;
            frame->image =
                renderer->renderer->render_gyro(import_image(image), plan->gyro_map, invalid_fraction, profile);
        } else {
            auto warp = plan->warp;
            frame->image = renderer->renderer->render(import_image(image), warp, plan->analysis, plan->settings.crop,
                                                      plan->horizontal, profile);
        }
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
        image->color.kr = view.color.kr;
        image->color.kb = view.color.kb;
        image->color.full_range = view.color.full_range;
        image->color.chroma_x = view.color.chroma_x;
        image->color.chroma_y = view.color.chroma_y;
        for (size_t i = 0; i < view.planes.size(); ++i)
            image->planes[i] = {view.planes[i].bytes.data(), view.planes[i].bytes.size(), view.planes[i].row_stride};
    });
}
void sf_frame_destroy(sf_frame* frame) { delete frame; }
}
