#include "stillflow/stillflow.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <jni.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template <class T, auto Destroy> using Handle = std::unique_ptr<T, decltype(Destroy)>;

struct Session {
    Handle<sf_stream, sf_stream_destroy> stream{nullptr, sf_stream_destroy};
    Handle<sf_analyzer, sf_analyzer_destroy> analyzer{nullptr, sf_analyzer_destroy};
    Handle<sf_plan, sf_plan_destroy> plan{nullptr, sf_plan_destroy};
    Handle<sf_frame_plan, sf_frame_plan_destroy> ready{nullptr, sf_frame_plan_destroy};
    Handle<sf_renderer, sf_renderer_destroy> renderer{nullptr, sf_renderer_destroy};
    int width{}, height{}, analysis_width{}, analysis_height{};
};

struct Failure : std::runtime_error {
    sf_status status;
    Failure(sf_status code, const char* message) : std::runtime_error(message), status(code) {}
};

void check(sf_status status, const sf_error& error) {
    if (status != SF_OK)
        throw Failure(status, error.message);
}

void raise(JNIEnv* env, const char* type, const char* message) {
    if (env->ExceptionCheck())
        return;
    if (jclass cls = env->FindClass(type)) {
        env->ThrowNew(cls, message);
        env->DeleteLocalRef(cls);
    }
}

template <class F> void boundary(JNIEnv* env, F work) noexcept {
    try {
        work();
    } catch (const Failure& error) {
        const char* type = "java/lang/RuntimeException";
        switch (error.status) {
        case SF_INVALID_ARGUMENT:
            type = "java/lang/IllegalArgumentException";
            break;
        case SF_INVALID_STATE:
            type = "java/lang/IllegalStateException";
            break;
        case SF_OUT_OF_MEMORY:
            type = "java/lang/OutOfMemoryError";
            break;
        case SF_UNSUPPORTED:
            type = "java/lang/UnsupportedOperationException";
            break;
        default:
            break;
        }
        raise(env, type, error.what());
    } catch (const std::bad_alloc&) {
        raise(env, "java/lang/OutOfMemoryError", "Native allocation failed");
    } catch (const std::exception& error) {
        raise(env, "java/lang/RuntimeException", error.what());
    } catch (...) {
        raise(env, "java/lang/RuntimeException", "Unknown native failure");
    }
}

Session& session(jlong handle) {
    if (!handle)
        throw Failure(SF_INVALID_STATE, "StillFlow session is closed");
    // JNI represents opaque native handles as jlong.
    return *reinterpret_cast<Session*>(handle); // NOLINT(performance-no-int-to-ptr)
}

sf_image image(JNIEnv* env, const Session& owner, jint width, jint height, jint format, jint stride, jobject pixels) {
    if (!((width == owner.width && height == owner.height) ||
          (format == SF_GRAY8 && width == owner.analysis_width && height == owner.analysis_height)) ||
        format < SF_GRAY8 || format > SF_RGBA8 || stride <= 0)
        throw Failure(SF_INVALID_ARGUMENT, "Frame dimensions or format do not match the session");
    auto* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(pixels));
    const auto capacity = env->GetDirectBufferCapacity(pixels);
    const int channels = format == SF_RGBA8 ? 4 : (format == SF_BGR8 ? 3 : 1);
    const auto row_bytes = static_cast<int64_t>(width) * channels;
    const auto required = static_cast<int64_t>(height - 1) * stride + row_bytes;
    if (!data || stride < row_bytes || capacity < required)
        throw Failure(SF_INVALID_ARGUMENT, "Invalid direct frame buffer extent");
    auto value = sf_image_default();
    value.width = width;
    value.height = height;
    value.format = format;
    value.plane_count = 1;
    value.planes[0] = {data, static_cast<size_t>(required), static_cast<size_t>(stride)};
    return value;
}
void render(JNIEnv* env, const Session& owner, const sf_frame_plan* plan, jint width, jint height, jint format,
            jint stride, jobject pixels, jobject destination) {
    auto input = image(env, owner, width, height, format, stride, pixels);
    const size_t channels = format == SF_RGBA8 ? 4 : (format == SF_BGR8 ? 3 : 1);
    const size_t row_bytes = static_cast<size_t>(width) * channels;
    const size_t required = row_bytes * static_cast<size_t>(height);
    auto* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(destination));
    const auto capacity = env->GetDirectBufferCapacity(destination);
    if (!data || capacity < 0 || required > static_cast<uint64_t>(capacity))
        throw Failure(SF_INVALID_ARGUMENT, "Invalid direct output buffer extent");
    sf_error error{};
    sf_frame* raw_frame = nullptr;
    const auto status = sf_renderer_render(owner.renderer.get(), plan, &input, &raw_frame, &error);
    Handle<sf_frame, sf_frame_destroy> frame(raw_frame, sf_frame_destroy);
    check(status, error);
    sf_image output{};
    check(sf_frame_get_image(frame.get(), &output, &error), error);
    if (output.width != width || output.height != height || output.format != format || output.plane_count != 1)
        throw Failure(SF_INTERNAL_ERROR, "Unexpected renderer output layout");
    for (int row = 0; row < height; ++row)
        std::memcpy(data + static_cast<size_t>(row) * row_bytes,
                    output.planes[0].data + static_cast<size_t>(row) * output.planes[0].row_stride, row_bytes);
}
jfloatArray frame_map(JNIEnv* env, const Session& owner, const sf_frame_plan* plan) {
    sf_error error{};
    sf_map map{};
    check(sf_frame_plan_get_map(plan, owner.width, owner.height, &map, &error), error);
    std::vector<jfloat> values{float(map.columns), float(map.rows),   float(map.origin_x), float(map.origin_y),
                               float(map.step_x),  float(map.step_y), float(map.grid_x),   float(map.grid_y)};
    for (int i = 0; i < map.columns * map.rows * 2; ++i)
        values.push_back(static_cast<float>(map.offsets[i]));
    auto result = env->NewFloatArray(static_cast<jsize>(values.size()));
    if (result)
        env->SetFloatArrayRegion(result, 0, static_cast<jsize>(values.size()), values.data());
    return result;
}
} // namespace

extern "C" {
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeThreads(JNIEnv* env, jclass, jint threads) {
    boundary(env, [&] {
        sf_error error{};
        check(sf_opencv_set_threads(threads, &error), error);
    });
}
JNIEXPORT jstring JNICALL Java_io_stillflow_StillFlow_nativeProfile(JNIEnv* env, jclass, jlong handle) {
    jstring result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        std::string json = "[";
        sf_error error{};
        check(sf_stream_get_profile(
                  owner.stream.get(),
                  [](void* data, const sf_profile_entry* entry) {
                      auto& output = *static_cast<std::string*>(data);
                      if (output.size() > 1)
                          output += ',';
                      output += "{\"stage\":\"" + std::string(entry->stage) +
                                "\",\"count\":" + std::to_string(entry->count) +
                                ",\"total_ms\":" + std::to_string(entry->total_ms) +
                                ",\"min_ms\":" + std::to_string(entry->min_ms) +
                                ",\"max_ms\":" + std::to_string(entry->max_ms) +
                                ",\"p50_ms\":" + std::to_string(entry->p50_ms) +
                                ",\"p95_ms\":" + std::to_string(entry->p95_ms) +
                                ",\"p99_ms\":" + std::to_string(entry->p99_ms) + "}";
                  },
                  &json, &error),
              error);
        json += ']';
        result = env->NewStringUTF(json.c_str());
    });
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_io_stillflow_StillFlow_nativeDefaults(JNIEnv* env, jclass) {
    const auto settings = sf_settings_default();
    const jdouble values[] = {double(settings.analysis_width),
                              double(settings.features),
                              double(settings.bands),
                              double(settings.crop_mode),
                              double(settings.rolling_shutter),
                              settings.smooth_seconds,
                              settings.rs_seconds,
                              settings.crop,
                              settings.rs_strength};
    auto result = env->NewDoubleArray(9);
    if (result)
        env->SetDoubleArrayRegion(result, 0, 9, values);
    return result;
}

JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeCreate(
    JNIEnv* env, jclass, jint width, jint height, jdouble fps, jboolean horizontal_scan, jint backend,
    jint motion_estimator, jint analysis_width, jint features, jint bands, jint crop_mode, jboolean rolling_shutter,
    jdouble smooth_seconds, jdouble rs_seconds, jdouble crop, jdouble rs_strength, jboolean realtime,
    jdouble lookahead_seconds, jint max_buffered_frames) {
    jlong result = 0;
    boundary(env, [&] {
        sf_settings settings{analysis_width,   features,       bands,      crop_mode, rolling_shutter,
                             motion_estimator, smooth_seconds, rs_seconds, crop,      rs_strength};
        sf_video video{width, height, horizontal_scan, fps};
        sf_error error{};
        if (backend < 0 || backend > 2)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid rendering backend");
        auto owner = std::make_unique<Session>();
        owner->width = width;
        owner->height = height;
        const double scale = std::min(1.0, double(analysis_width) / width);
        owner->analysis_width = std::max(32, int(std::lround(width * scale)));
        owner->analysis_height = std::max(32, int(std::lround(height * scale)));
        if (realtime) {
            if (max_buffered_frames < 2)
                throw Failure(SF_INVALID_ARGUMENT, "Streaming capacity must be at least two frames");
            sf_stream_settings streaming{lookahead_seconds, static_cast<size_t>(max_buffered_frames)};
            sf_stream* stream = nullptr;
            const auto status = sf_stream_create(&settings, &streaming, &video, nullptr, &stream, &error);
            owner->stream.reset(stream);
            check(status, error);
        } else {
            sf_analyzer* analyzer = nullptr;
            const auto status = sf_analyzer_create(&settings, &video, nullptr, &analyzer, &error);
            owner->analyzer.reset(analyzer);
            check(status, error);
        }
        sf_renderer* renderer = nullptr;
        const auto renderer_status = sf_renderer_create(backend, nullptr, &renderer, &error);
        owner->renderer.reset(renderer);
        check(renderer_status, error);
        result = reinterpret_cast<jlong>(owner.release());
    });
    return result;
}

JNIEXPORT jboolean JNICALL Java_io_stillflow_StillFlow_nativeStreamPush(JNIEnv* env, jclass, jlong handle, jint width,
                                                                        jint height, jint format, jint stride,
                                                                        jobject pixels, jlong pts, jlong duration) {
    jboolean result = false;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.stream)
            throw Failure(SF_INVALID_STATE, "Enable realtime in session settings");
        auto input = image(env, owner, width, height, format, stride, pixels);
        sf_error error{};
        const auto status = sf_stream_push(owner.stream.get(), &input, pts, duration, &error);
        if (status == SF_AGAIN)
            return;
        check(status, error);
        result = true;
    });
    return result;
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeStreamFinish(JNIEnv* env, jclass, jlong handle) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.stream)
            throw Failure(SF_INVALID_STATE, "Enable realtime in session settings");
        sf_error error{};
        check(sf_stream_finish(owner.stream.get(), &error), error);
    });
}
JNIEXPORT jfloatArray JNICALL Java_io_stillflow_StillFlow_nativeReadyMap(JNIEnv* env, jclass, jlong handle) {
    jfloatArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.stream || !owner.ready)
            throw Failure(SF_INVALID_STATE, "No ready streaming plan");
        result = frame_map(env, owner, owner.ready.get());
        if (result && !env->ExceptionCheck())
            owner.ready.reset();
    });
    return result;
}
JNIEXPORT jfloatArray JNICALL Java_io_stillflow_StillFlow_nativeFrameMap(JNIEnv* env, jclass, jlong handle,
                                                                         jint index) {
    jfloatArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Finish offline analysis before extracting maps");
        if (index < 0)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid frame index");
        sf_error error{};
        sf_frame_plan* raw = nullptr;
        const auto status = sf_plan_get_frame(owner.plan.get(), static_cast<size_t>(index), &raw, &error);
        Handle<sf_frame_plan, sf_frame_plan_destroy> plan(raw, sf_frame_plan_destroy);
        check(status, error);
        result = frame_map(env, owner, plan.get());
    });
    return result;
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeStreamReady(JNIEnv* env, jclass, jlong handle) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.stream || owner.ready)
            throw Failure(SF_INVALID_STATE, "Streaming requires rendering each ready frame before taking another");
        auto values = env->NewLongArray(3);
        if (!values)
            return;
        sf_error error{};
        sf_frame_info frame{};
        sf_frame_plan* plan = nullptr;
        const auto status = sf_stream_take_ready(owner.stream.get(), &plan, &error);
        owner.ready.reset(plan);
        if (status == SF_AGAIN || status == SF_END) {
            env->DeleteLocalRef(values);
            return;
        }
        check(status, error);
        check(sf_frame_plan_get_info(owner.ready.get(), &frame, &error), error);
        const jlong fields[] = {static_cast<jlong>(frame.index), frame.pts_us, frame.duration_us};
        env->SetLongArrayRegion(values, 0, 3, fields);
        result = values;
    });
    return result;
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeAnalyze(JNIEnv* env, jclass, jlong handle, jint width,
                                                                 jint height, jint format, jint stride, jobject pixels,
                                                                 jlong pts, jlong duration) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Offline analysis is unavailable in this state");
        auto input = image(env, owner, width, height, format, stride, pixels);
        sf_error error{};
        check(sf_analyzer_push(owner.analyzer.get(), &input, pts, duration, &error), error);
    });
}

JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeNextRecoveryFrame(JNIEnv* env, jclass, jlong handle) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Offline analysis is unavailable in this state");
        sf_error error{};
        sf_recovery_frame frame{};
        const auto status = sf_analyzer_next_recovery_frame(owner.analyzer.get(), &frame, &error);
        if (status == SF_END)
            return;
        check(status, error);
        result = env->NewLongArray(2);
        if (!result)
            return;
        const jlong values[] = {static_cast<jlong>(frame.index), frame.pts_us};
        env->SetLongArrayRegion(result, 0, 2, values);
    });
    return result;
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeRecover(JNIEnv* env, jclass, jlong handle, jint width,
                                                                 jint height, jint format, jint stride, jobject pixels,
                                                                 jlong pts) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Offline analysis is unavailable in this state");
        auto input = image(env, owner, width, height, format, stride, pixels);
        sf_error error{};
        check(sf_analyzer_recover(owner.analyzer.get(), &input, pts, &error), error);
    });
}

JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeFinish(JNIEnv* env, jclass, jlong handle) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Offline analysis is unavailable in this state");
        // Allocate the Java result before consuming the analyzer.
        result = env->NewLongArray(4);
        if (!result)
            return;
        sf_error error{};
        sf_plan* plan = nullptr;
        const auto status = sf_analyzer_finish(owner.analyzer.get(), &plan, &error);
        owner.plan.reset(plan);
        check(status, error);
        sf_plan_info info{};
        check(sf_plan_get_info(owner.plan.get(), &info, &error), error);
        const jlong values[] = {static_cast<jlong>(info.frames), info.analysis_width, info.analysis_height,
                                info.origin_us};
        env->SetLongArrayRegion(result, 0, 4, values);
        owner.analyzer.reset();
    });
    return result;
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeRender(JNIEnv* env, jclass, jlong handle, jint index,
                                                                jint width, jint height, jint format, jint stride,
                                                                jobject pixels, jobject destination) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.plan || owner.stream)
            throw Failure(SF_INVALID_STATE, "Finish offline analysis before rendering");
        if (index < 0)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid frame index");
        sf_error error{};
        sf_frame_plan* raw = nullptr;
        const auto status = sf_plan_get_frame(owner.plan.get(), static_cast<size_t>(index), &raw, &error);
        Handle<sf_frame_plan, sf_frame_plan_destroy> plan(raw, sf_frame_plan_destroy);
        check(status, error);
        render(env, owner, plan.get(), width, height, format, stride, pixels, destination);
    });
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeRenderReady(JNIEnv* env, jclass, jlong handle, jint width,
                                                                     jint height, jint format, jint stride,
                                                                     jobject pixels, jobject destination) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (!owner.stream || !owner.ready)
            throw Failure(SF_INVALID_STATE, "No ready streaming plan");
        render(env, owner, owner.ready.get(), width, height, format, stride, pixels, destination);
        owner.ready.reset();
    });
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeDestroy(JNIEnv*, jclass, jlong handle) {
    delete reinterpret_cast<Session*>(handle); // NOLINT(performance-no-int-to-ptr)
}

JNIEXPORT jobjectArray JNICALL Java_io_stillflow_StillFlow_nativeRendererInfo(JNIEnv* env, jclass, jlong handle) {
    jobjectArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        sf_error error{};
        sf_renderer_info info{};
        check(sf_renderer_get_info(owner.renderer.get(), &info, &error), error);
        jclass string_class = env->FindClass("java/lang/String");
        if (!string_class)
            return;
        result = env->NewObjectArray(4, string_class, nullptr);
        env->DeleteLocalRef(string_class);
        if (!result)
            return;
        const char* backend = info.backend == SF_BACKEND_CPU    ? "cpu"
                              : info.backend == SF_BACKEND_GLES ? "gles"
                                                                : "cuda";
        const char* values[] = {backend, info.device, info.fallback_reason, info.gpu ? "true" : "false"};
        for (jsize i = 0; i < 4; ++i) {
            jstring value = env->NewStringUTF(values[i]);
            if (!value)
                return;
            env->SetObjectArrayElement(result, i, value);
            env->DeleteLocalRef(value);
            if (env->ExceptionCheck())
                return;
        }
    });
    return result;
}

JNIEXPORT jstring JNICALL Java_io_stillflow_StillFlow_nativeBuildDescription(JNIEnv* env, jclass) {
    return env->NewStringUTF(sf_build_description());
}
}
