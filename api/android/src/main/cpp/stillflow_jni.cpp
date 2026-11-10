#include "stillflow/stillflow.h"
#include <cstring>
#include <jni.h>
#include <limits>
#include <memory>
#include <stdexcept>

namespace {
template <class T, auto Destroy> using Handle = std::unique_ptr<T, decltype(Destroy)>;

struct Session {
    Handle<sf_analyzer, sf_analyzer_destroy> analyzer{nullptr, sf_analyzer_destroy};
    Handle<sf_plan, sf_plan_destroy> plan{nullptr, sf_plan_destroy};
    Handle<sf_renderer, sf_renderer_destroy> renderer{nullptr, sf_renderer_destroy};
    int width{}, height{};
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
    if (width != owner.width || height != owner.height || format < SF_GRAY8 || format > SF_RGBA8 || stride <= 0)
        throw Failure(SF_INVALID_ARGUMENT, "Frame dimensions or format do not match the session");
    auto* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(pixels));
    const auto capacity = env->GetDirectBufferCapacity(pixels);
    const int channels = format == SF_RGBA8 ? 4 : (format == SF_BGR8 ? 3 : 1);
    const auto row_bytes = static_cast<int64_t>(width) * channels;
    const auto required = static_cast<int64_t>(height - 1) * stride + row_bytes;
    if (!data || stride < row_bytes || capacity < required)
        throw Failure(SF_INVALID_ARGUMENT, "Invalid direct frame buffer extent");
    sf_image value{};
    value.struct_size = sizeof(value);
    sf_image_default(&value);
    value.width = width;
    value.height = height;
    value.format = format;
    value.plane_count = 1;
    value.planes[0] = {sizeof(sf_plane), data, static_cast<size_t>(required), static_cast<size_t>(stride)};
    return value;
}
} // namespace

extern "C" {
JNIEXPORT jdoubleArray JNICALL Java_io_stillflow_StillFlow_nativeDefaults(JNIEnv* env, jclass) {
    sf_settings settings{};
    settings.struct_size = sizeof(settings);
    sf_settings_default(&settings);
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
    jdouble smooth_seconds, jdouble rs_seconds, jdouble crop, jdouble rs_strength) {
    jlong result = 0;
    boundary(env, [&] {
        sf_settings settings{sizeof(sf_settings), analysis_width, features,   bands, crop_mode,  rolling_shutter,
                             motion_estimator,    smooth_seconds, rs_seconds, crop,  rs_strength};
        sf_video video{sizeof(sf_video), width, height, horizontal_scan, fps};
        sf_error error{sizeof(sf_error), {}};
        if (backend < 0 || backend > 2)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid rendering backend");
        const char* backends[] = {"auto", "cpu", "gles"};
        auto owner = std::make_unique<Session>();
        owner->width = width;
        owner->height = height;
        sf_analyzer* analyzer = nullptr;
        const auto status = sf_analyzer_create(&settings, &video, nullptr, &analyzer, &error);
        owner->analyzer.reset(analyzer);
        check(status, error);
        sf_renderer* renderer = nullptr;
        const auto renderer_status = sf_renderer_create_backend(backends[backend], nullptr, &renderer, &error);
        owner->renderer.reset(renderer);
        check(renderer_status, error);
        result = reinterpret_cast<jlong>(owner.release());
    });
    return result;
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeAnalyze(JNIEnv* env, jclass, jlong handle, jint width,
                                                                 jint height, jint format, jint stride, jobject pixels,
                                                                 jlong pts, jlong duration) {
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan)
            throw Failure(SF_INVALID_STATE, "Analysis is already finished");
        auto input = image(env, owner, width, height, format, stride, pixels);
        sf_error error{sizeof(sf_error), {}};
        check(sf_analyzer_push(owner.analyzer.get(), &input, pts, duration, &error), error);
    });
}

JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeFinish(JNIEnv* env, jclass, jlong handle) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        if (owner.plan)
            throw Failure(SF_INVALID_STATE, "Analysis is already finished");
        // Allocate the Java result before consuming the analyzer.
        result = env->NewLongArray(4);
        if (!result)
            return;
        sf_error error{sizeof(sf_error), {}};
        sf_plan* plan = nullptr;
        const auto status = sf_analyzer_finish(owner.analyzer.get(), &plan, &error);
        owner.plan.reset(plan);
        check(status, error);
        sf_plan_info info{sizeof(sf_plan_info), 0, 0, 0, 0};
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
        if (!owner.plan)
            throw Failure(SF_INVALID_STATE, "Finish analysis before rendering");
        if (index < 0)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid frame index");
        auto input = image(env, owner, width, height, format, stride, pixels);
        sf_error error{sizeof(sf_error), {}};
        sf_frame* raw_frame = nullptr;
        const auto status = sf_renderer_render(owner.renderer.get(), owner.plan.get(), static_cast<size_t>(index),
                                               &input, &raw_frame, &error);
        Handle<sf_frame, sf_frame_destroy> frame(raw_frame, sf_frame_destroy);
        check(status, error);
        sf_image output{};
        output.struct_size = sizeof(output);
        check(sf_frame_get_image(frame.get(), &output, &error), error);
        const size_t channels = format == SF_RGBA8 ? 4 : (format == SF_BGR8 ? 3 : 1);
        const size_t row_bytes = static_cast<size_t>(width) * channels;
        const size_t required = row_bytes * static_cast<size_t>(height);
        auto* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(destination));
        const auto capacity = env->GetDirectBufferCapacity(destination);
        if (!data || capacity < 0 || required > static_cast<uint64_t>(capacity))
            throw Failure(SF_INVALID_ARGUMENT, "Invalid direct output buffer extent");
        if (output.width != width || output.height != height || output.format != format || output.plane_count != 1)
            throw Failure(SF_INTERNAL_ERROR, "Unexpected renderer output layout");
        for (int row = 0; row < height; ++row)
            std::memcpy(data + static_cast<size_t>(row) * row_bytes,
                        output.planes[0].data + static_cast<size_t>(row) * output.planes[0].row_stride, row_bytes);
    });
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeDestroy(JNIEnv*, jclass, jlong handle) {
    delete reinterpret_cast<Session*>(handle); // NOLINT(performance-no-int-to-ptr)
}

JNIEXPORT jobjectArray JNICALL Java_io_stillflow_StillFlow_nativeRendererInfo(JNIEnv* env, jclass, jlong handle) {
    jobjectArray result = nullptr;
    boundary(env, [&] {
        auto& owner = session(handle);
        sf_error error{sizeof(sf_error), {}};
        sf_renderer_info info{};
        info.struct_size = sizeof(info);
        check(sf_renderer_get_info(owner.renderer.get(), &info, &error), error);
        jclass string_class = env->FindClass("java/lang/String");
        if (!string_class)
            return;
        result = env->NewObjectArray(4, string_class, nullptr);
        env->DeleteLocalRef(string_class);
        if (!result)
            return;
        const char* values[] = {info.backend, info.device, info.fallback_reason, info.gpu ? "true" : "false"};
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
