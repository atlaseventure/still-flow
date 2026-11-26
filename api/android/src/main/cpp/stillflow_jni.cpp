#include "stillflow/stillflow.h"
#include <algorithm>
#include <bit>
#include <cstring>
#include <jni.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template <class T, auto Destroy> using Handle = std::unique_ptr<T, decltype(Destroy)>;

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

template <class T> T* handle(jlong value) {
    if (!value)
        throw Failure(SF_INVALID_STATE, "StillFlow resource is closed");
    return reinterpret_cast<T*>(value); // NOLINT(performance-no-int-to-ptr)
}

sf_image image(JNIEnv* env, jint width, jint height, jint format, jint stride, jobject pixels) {
    if (width <= 0 || height <= 0 || format < SF_GRAY8 || format > SF_RGBA8 || stride <= 0)
        throw Failure(SF_INVALID_ARGUMENT, "Invalid frame dimensions or format");
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
void render(JNIEnv* env, sf_renderer* renderer, const sf_frame_plan* plan, jint width, jint height, jint format,
            jint stride, jobject pixels, jobject destination) {
    auto input = image(env, width, height, format, stride, pixels);
    sf_error error{};
    int32_t out_width, out_height;
    check(sf_frame_plan_get_output_size(plan, &out_width, &out_height, &error), error);
    const size_t channels = format == SF_RGBA8 ? 4 : (format == SF_BGR8 ? 3 : 1);
    const size_t row_bytes = static_cast<size_t>(out_width) * channels;
    const size_t required = row_bytes * static_cast<size_t>(out_height);
    auto* data = static_cast<uint8_t*>(env->GetDirectBufferAddress(destination));
    const auto capacity = env->GetDirectBufferCapacity(destination);
    if (!data || capacity < 0 || required > static_cast<uint64_t>(capacity))
        throw Failure(SF_INVALID_ARGUMENT, "Invalid direct output buffer extent");
    sf_frame* raw_frame = nullptr;
    const auto status = sf_renderer_render(renderer, plan, &input, &raw_frame, &error);
    Handle<sf_frame, sf_frame_destroy> frame(raw_frame, sf_frame_destroy);
    check(status, error);
    sf_image output{};
    check(sf_frame_get_image(frame.get(), &output, &error), error);
    if (output.width != out_width || output.height != out_height || output.format != format || output.plane_count != 1)
        throw Failure(SF_INTERNAL_ERROR, "Unexpected renderer output layout");
    for (int row = 0; row < out_height; ++row)
        std::memcpy(data + static_cast<size_t>(row) * row_bytes,
                    output.planes[0].data + static_cast<size_t>(row) * output.planes[0].row_stride, row_bytes);
}
jfloatArray frame_map(JNIEnv* env, const sf_frame_plan* plan, jint width, jint height) {
    sf_error error{};
    sf_map map{};
    check(sf_frame_plan_get_map(plan, width, height, &map, &error), error);
    std::vector<jfloat> values{float(map.columns), float(map.rows),   float(map.origin_x), float(map.origin_y),
                               float(map.step_x),  float(map.step_y), float(map.grid_x),   float(map.grid_y)};
    for (int i = 0; i < map.columns * map.rows * 2; ++i)
        values.push_back(static_cast<float>(map.offsets[i]));
    auto result = env->NewFloatArray(static_cast<jsize>(values.size()));
    if (result)
        env->SetFloatArrayRegion(result, 0, static_cast<jsize>(values.size()), values.data());
    return result;
}

sf_lens_profile gyro_lens(JNIEnv* env, jint lens_model, jint calib_width, jint calib_height, jdouble fx, jdouble fy,
                          jdouble cx, jdouble cy, jdoubleArray distortion, jdouble lens_correction_amount) {
    if (!distortion || env->GetArrayLength(distortion) != 4)
        throw Failure(SF_INVALID_ARGUMENT, "Expected four lens distortion coefficients");
    sf_lens_profile lens{lens_model, calib_width, calib_height, fx, fy, cx, cy, {}, lens_correction_amount};
    env->GetDoubleArrayRegion(distortion, 0, 4, lens.k);
    return lens;
}

sf_gyro_settings gyro_settings(jdouble smoothness, jboolean per_axis, jdouble smoothness_pitch, jdouble smoothness_yaw,
                               jdouble smoothness_roll, jdouble max_smoothness, jdouble alpha_0_1s, jdouble readout_ms,
                               jboolean readout_top_to_bottom, jdouble fov, jdouble max_zoom, jint max_zoom_iterations,
                               jdouble adaptive_zoom_window_s, jboolean adaptive_zoom, jdouble time_offset_s,
                               jint integration_method, jint output_width, jint output_height, jboolean zoom_envelope) {
    return {smoothness,
            per_axis,
            smoothness_pitch,
            smoothness_yaw,
            smoothness_roll,
            max_smoothness,
            alpha_0_1s,
            readout_ms,
            readout_top_to_bottom,
            fov,
            max_zoom,
            max_zoom_iterations,
            adaptive_zoom_window_s,
            adaptive_zoom,
            time_offset_s,
            integration_method,
            output_width,
            output_height,
            zoom_envelope};
}

std::vector<sf_imu_sample> imu_samples(JNIEnv* env, jdoubleArray imu, jbooleanArray accel) {
    if (!imu || !accel)
        throw Failure(SF_INVALID_ARGUMENT, "Missing IMU arrays");
    const auto samples = env->GetArrayLength(accel);
    const auto length = env->GetArrayLength(imu);
    if (length != 7LL * samples)
        throw Failure(SF_INVALID_ARGUMENT, "Expected seven values per IMU sample");
    if (!samples)
        return {};
    std::vector<jdouble> data(length);
    std::vector<jboolean> flags(samples);
    env->GetDoubleArrayRegion(imu, 0, length, data.data());
    env->GetBooleanArrayRegion(accel, 0, samples, flags.data());
    if (env->ExceptionCheck())
        return {};
    std::vector<sf_imu_sample> measurements(samples);
    for (size_t i = 0; i < measurements.size(); ++i) {
        const auto* v = &data[i * 7];
        measurements[i] = {v[0], v[1], v[2], v[3], v[4], v[5], v[6], flags[i]};
    }
    return measurements;
}

template <class T, class F> jstring profile_json(JNIEnv* env, jlong value, F query) {
    jstring result = nullptr;
    boundary(env, [&] {
        std::string json = "[";
        sf_error error{};
        check(query(
                  handle<T>(value),
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

template <class T, class F> jlongArray stream_ready(JNIEnv* env, jlong value, F query) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        result = env->NewLongArray(2);
        if (!result)
            return;
        sf_error error{};
        sf_frame_plan* plan = nullptr;
        const auto status = query(handle<T>(value), &plan, &error);
        if (status != SF_AGAIN && status != SF_END)
            check(status, error);
        const jlong values[] = {status, reinterpret_cast<jlong>(plan)};
        env->SetLongArrayRegion(result, 0, 2, values);
        if (env->ExceptionCheck())
            sf_frame_plan_destroy(plan);
    });
    return result;
}

template <class T, class F> jintArray output_size(JNIEnv* env, jlong value, F query) {
    jintArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        jint size[2];
        check(query(handle<T>(value), &size[0], &size[1], &error), error);
        result = env->NewIntArray(2);
        if (result)
            env->SetIntArrayRegion(result, 0, 2, size);
    });
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
JNIEXPORT jstring JNICALL Java_io_stillflow_StillFlow_nativeProfile(JNIEnv* env, jclass, jlong value) {
    return profile_json<sf_stream>(env, value, sf_stream_get_profile);
}
JNIEXPORT jstring JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamProfile(JNIEnv* env, jclass, jlong value) {
    return profile_json<sf_gyro_stream>(env, value, sf_gyro_stream_get_profile);
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

JNIEXPORT jdoubleArray JNICALL Java_io_stillflow_StillFlow_nativeGyroDefaults(JNIEnv* env, jclass) {
    const auto s = sf_gyro_settings_default();
    const auto lens = sf_lens_profile_default();
    const jdouble values[] = {s.smoothness,
                              double(s.per_axis),
                              s.smoothness_pitch,
                              s.smoothness_yaw,
                              s.smoothness_roll,
                              s.max_smoothness,
                              s.alpha_0_1s,
                              s.readout_ms,
                              double(s.readout_top_to_bottom),
                              s.fov,
                              s.max_zoom,
                              double(s.max_zoom_iterations),
                              s.adaptive_zoom_window_s,
                              double(s.adaptive_zoom),
                              s.time_offset_s,
                              double(s.integration_method),
                              double(s.output_width),
                              double(s.output_height),
                              double(s.zoom_envelope),
                              double(lens.model),
                              double(lens.calib_width),
                              double(lens.calib_height),
                              lens.fx,
                              lens.fy,
                              lens.cx,
                              lens.cy,
                              lens.k[0],
                              lens.k[1],
                              lens.k[2],
                              lens.k[3],
                              lens.lens_correction_amount};
    auto result = env->NewDoubleArray(std::size(values));
    if (result)
        env->SetDoubleArrayRegion(result, 0, std::size(values), values);
    return result;
}

JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeGyroPlanCreate(
    JNIEnv* env, jclass, jint width, jint height, jlongArray pts, jlongArray durations, jdoubleArray imu,
    jbooleanArray accel, jint lens_model, jint calib_width, jint calib_height, jdouble fx, jdouble fy, jdouble cx,
    jdouble cy, jdoubleArray distortion, jdouble lens_correction_amount, jdouble smoothness, jboolean per_axis,
    jdouble smoothness_pitch, jdouble smoothness_yaw, jdouble smoothness_roll, jdouble max_smoothness,
    jdouble alpha_0_1s, jdouble readout_ms, jboolean readout_top_to_bottom, jdouble fov, jdouble max_zoom,
    jint max_zoom_iterations, jdouble adaptive_zoom_window_s, jboolean adaptive_zoom, jdouble time_offset_s,
    jint integration_method, jint output_width, jint output_height, jboolean zoom_envelope) {
    jlong result = 0;
    boundary(env, [&] {
        if (!pts || !durations || !env->GetArrayLength(pts) ||
            env->GetArrayLength(pts) != env->GetArrayLength(durations))
            throw Failure(SF_INVALID_ARGUMENT, "Invalid gyro timestamp arrays");
        const auto frames = env->GetArrayLength(pts);
        const auto lens =
            gyro_lens(env, lens_model, calib_width, calib_height, fx, fy, cx, cy, distortion, lens_correction_amount);
        if (env->ExceptionCheck())
            return;
        const auto settings = gyro_settings(
            smoothness, per_axis, smoothness_pitch, smoothness_yaw, smoothness_roll, max_smoothness, alpha_0_1s,
            readout_ms, readout_top_to_bottom, fov, max_zoom, max_zoom_iterations, adaptive_zoom_window_s,
            adaptive_zoom, time_offset_s, integration_method, output_width, output_height, zoom_envelope);
        const auto measurements = imu_samples(env, imu, accel);
        if (env->ExceptionCheck())
            return;
        if (measurements.empty())
            throw Failure(SF_INVALID_ARGUMENT, "Expected IMU samples");
        std::vector<jlong> times(frames), lengths(frames);
        env->GetLongArrayRegion(pts, 0, frames, times.data());
        env->GetLongArrayRegion(durations, 0, frames, lengths.data());
        if (env->ExceptionCheck())
            return;
        std::vector<sf_frame_timestamp> timestamps(frames);
        for (size_t i = 0; i < timestamps.size(); ++i)
            timestamps[i] = {times[i], lengths[i]};
        sf_error error{};
        sf_plan* plan = nullptr;
        check(sf_gyro_plan_create(&settings, &lens, width, height, timestamps.data(), timestamps.size(),
                                  measurements.data(), measurements.size(), nullptr, &plan, &error),
              error);
        result = reinterpret_cast<jlong>(plan);
    });
    return result;
}

JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamCreate(
    JNIEnv* env, jclass, jint width, jint height, jint lens_model, jint calib_width, jint calib_height, jdouble fx,
    jdouble fy, jdouble cx, jdouble cy, jdoubleArray distortion, jdouble lens_correction_amount, jdouble smoothness,
    jboolean per_axis, jdouble smoothness_pitch, jdouble smoothness_yaw, jdouble smoothness_roll,
    jdouble max_smoothness, jdouble alpha_0_1s, jdouble readout_ms, jboolean readout_top_to_bottom, jdouble fov,
    jdouble max_zoom, jint max_zoom_iterations, jdouble adaptive_zoom_window_s, jboolean adaptive_zoom,
    jdouble time_offset_s, jint integration_method, jint output_width, jint output_height, jboolean zoom_envelope,
    jdouble lookahead_seconds, jint max_buffered_frames, jboolean adaptive_tracking) {
    jlong result = 0;
    boundary(env, [&] {
        if (max_buffered_frames < 2)
            throw Failure(SF_INVALID_ARGUMENT, "Streaming capacity must be at least two frames");
        const auto lens =
            gyro_lens(env, lens_model, calib_width, calib_height, fx, fy, cx, cy, distortion, lens_correction_amount);
        if (env->ExceptionCheck())
            return;
        const auto settings = gyro_settings(
            smoothness, per_axis, smoothness_pitch, smoothness_yaw, smoothness_roll, max_smoothness, alpha_0_1s,
            readout_ms, readout_top_to_bottom, fov, max_zoom, max_zoom_iterations, adaptive_zoom_window_s,
            adaptive_zoom, time_offset_s, integration_method, output_width, output_height, zoom_envelope);
        sf_stream_settings streaming{lookahead_seconds, static_cast<size_t>(max_buffered_frames), adaptive_tracking};
        sf_error error{};
        sf_gyro_stream* stream = nullptr;
        check(sf_gyro_stream_create(&settings, &lens, &streaming, width, height, nullptr, &stream, &error), error);
        result = reinterpret_cast<jlong>(stream);
    });
    return result;
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamPushImu(JNIEnv* env, jclass, jlong value,
                                                                           jdoubleArray imu, jbooleanArray accel) {
    boundary(env, [&] {
        const auto measurements = imu_samples(env, imu, accel);
        if (env->ExceptionCheck())
            return;
        sf_error error{};
        check(sf_gyro_stream_push_imu(handle<sf_gyro_stream>(value), measurements.data(), measurements.size(), &error),
              error);
    });
}

JNIEXPORT jboolean JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamPushFrame(JNIEnv* env, jclass, jlong value,
                                                                                 jlong pts, jlong duration) {
    jboolean result = false;
    boundary(env, [&] {
        sf_error error{};
        const auto status = sf_gyro_stream_push_frame(handle<sf_gyro_stream>(value), pts, duration, &error);
        if (status == SF_AGAIN)
            return;
        check(status, error);
        result = true;
    });
    return result;
}

JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamFinish(JNIEnv* env, jclass, jlong value) {
    boundary(env, [&] {
        sf_error error{};
        check(sf_gyro_stream_finish(handle<sf_gyro_stream>(value), &error), error);
    });
}

JNIEXPORT jintArray JNICALL Java_io_stillflow_StillFlow_nativeFrameOutputSize(JNIEnv* env, jclass, jlong value) {
    return output_size<sf_frame_plan>(env, value, sf_frame_plan_get_output_size);
}
JNIEXPORT jintArray JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamOutputSize(JNIEnv* env, jclass, jlong value) {
    return output_size<sf_gyro_stream>(env, value, sf_gyro_stream_get_output_size);
}

JNIEXPORT jdoubleArray JNICALL Java_io_stillflow_StillFlow_nativeFrameGyroMap(JNIEnv* env, jclass, jlong value) {
    jdoubleArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        sf_gyro_map map{};
        check(sf_frame_plan_get_gyro_map(handle<sf_frame_plan>(value), &map, &error), error);
        const jdouble header[] = {double(map.input_width),
                                  double(map.input_height),
                                  double(map.output_width),
                                  double(map.output_height),
                                  double(map.lens_model),
                                  map.in_fx,
                                  map.in_fy,
                                  map.in_cx,
                                  map.in_cy,
                                  map.k[0],
                                  map.k[1],
                                  map.k[2],
                                  map.k[3],
                                  map.lens_correction_amount,
                                  map.out_fx,
                                  map.out_fy,
                                  map.out_cx,
                                  map.out_cy,
                                  map.fov_scale,
                                  map.zoom_ratio,
                                  map.smooth_factor,
                                  double(map.row_count),
                                  double(map.sensor_rows)};
        const jsize count = 9 * map.row_count;
        result = env->NewDoubleArray(std::size(header) + count);
        if (result) {
            env->SetDoubleArrayRegion(result, 0, std::size(header), header);
            env->SetDoubleArrayRegion(result, std::size(header), count, map.row_rotations);
        }
    });
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_io_stillflow_StillFlow_nativeStreamDefaults(JNIEnv* env, jclass) {
    const auto settings = sf_stream_settings_default();
    const jdouble values[] = {settings.lookahead_seconds, double(settings.max_buffered_frames),
                              double(settings.adaptive_tracking)};
    auto result = env->NewDoubleArray(3);
    if (result)
        env->SetDoubleArrayRegion(result, 0, 3, values);
    return result;
}

JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeCreate(
    JNIEnv* env, jclass, jint width, jint height, jdouble fps, jboolean horizontal_scan, jint motion_estimator,
    jint analysis_width, jint features, jint bands, jint crop_mode, jboolean rolling_shutter, jdouble smooth_seconds,
    jdouble rs_seconds, jdouble crop, jdouble rs_strength, jboolean realtime, jdouble lookahead_seconds,
    jint max_buffered_frames, jboolean adaptive_tracking) {
    jlong result = 0;
    boundary(env, [&] {
        sf_settings settings{analysis_width,   features,       bands,      crop_mode, rolling_shutter,
                             motion_estimator, smooth_seconds, rs_seconds, crop,      rs_strength};
        sf_video video{width, height, horizontal_scan, fps};
        sf_error error{};
        if (realtime) {
            if (max_buffered_frames < 2)
                throw Failure(SF_INVALID_ARGUMENT, "Streaming capacity must be at least two frames");
            sf_stream_settings streaming{lookahead_seconds, static_cast<size_t>(max_buffered_frames),
                                         adaptive_tracking};
            sf_stream* stream = nullptr;
            check(sf_stream_create(&settings, &streaming, &video, nullptr, &stream, &error), error);
            result = reinterpret_cast<jlong>(stream);
        } else {
            sf_analyzer* analyzer = nullptr;
            check(sf_analyzer_create(&settings, &video, nullptr, &analyzer, &error), error);
            result = reinterpret_cast<jlong>(analyzer);
        }
    });
    return result;
}

JNIEXPORT jboolean JNICALL Java_io_stillflow_StillFlow_nativeStreamPush(JNIEnv* env, jclass, jlong value, jint width,
                                                                        jint height, jint format, jint stride,
                                                                        jobject pixels, jlong pts, jlong duration) {
    jboolean result = false;
    boundary(env, [&] {
        auto input = image(env, width, height, format, stride, pixels);
        sf_error error{};
        const auto status = sf_stream_push(handle<sf_stream>(value), &input, pts, duration, &error);
        if (status == SF_AGAIN)
            return;
        check(status, error);
        result = true;
    });
    return result;
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeStreamFinish(JNIEnv* env, jclass, jlong value) {
    boundary(env, [&] {
        sf_error error{};
        check(sf_stream_finish(handle<sf_stream>(value), &error), error);
    });
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeStreamReady(JNIEnv* env, jclass, jlong value) {
    return stream_ready<sf_stream>(env, value, sf_stream_take_ready);
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeGyroStreamReady(JNIEnv* env, jclass, jlong value) {
    return stream_ready<sf_gyro_stream>(env, value, sf_gyro_stream_take_ready);
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeAnalyze(JNIEnv* env, jclass, jlong value, jint width,
                                                                 jint height, jint format, jint stride, jobject pixels,
                                                                 jlong pts, jlong duration) {
    boundary(env, [&] {
        auto input = image(env, width, height, format, stride, pixels);
        sf_error error{};
        check(sf_analyzer_push(handle<sf_analyzer>(value), &input, pts, duration, &error), error);
    });
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeNextRecoveryFrame(JNIEnv* env, jclass, jlong value) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        sf_recovery_frame frame{};
        const auto status = sf_analyzer_next_recovery_frame(handle<sf_analyzer>(value), &frame, &error);
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
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeRecover(JNIEnv* env, jclass, jlong value, jint width,
                                                                 jint height, jint format, jint stride, jobject pixels,
                                                                 jlong pts) {
    boundary(env, [&] {
        auto input = image(env, width, height, format, stride, pixels);
        sf_error error{};
        check(sf_analyzer_recover(handle<sf_analyzer>(value), &input, pts, &error), error);
    });
}
JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeFinish(JNIEnv* env, jclass, jlong value) {
    jlong result = 0;
    boundary(env, [&] {
        sf_error error{};
        sf_plan* plan = nullptr;
        check(sf_analyzer_finish(handle<sf_analyzer>(value), &plan, &error), error);
        result = reinterpret_cast<jlong>(plan);
    });
    return result;
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativePlanInfo(JNIEnv* env, jclass, jlong value) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        sf_plan_info info{};
        check(sf_plan_get_info(handle<sf_plan>(value), &info, &error), error);
        result = env->NewLongArray(4);
        if (!result)
            return;
        const jlong values[] = {static_cast<jlong>(info.frames), info.analysis_width, info.analysis_height,
                                info.origin_us};
        env->SetLongArrayRegion(result, 0, 4, values);
    });
    return result;
}
JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativePlanFrame(JNIEnv* env, jclass, jlong value, jlong index) {
    jlong result = 0;
    boundary(env, [&] {
        if (index < 0)
            throw Failure(SF_INVALID_ARGUMENT, "Invalid frame index");
        sf_error error{};
        sf_frame_plan* plan = nullptr;
        check(sf_plan_get_frame(handle<sf_plan>(value), static_cast<size_t>(index), &plan, &error), error);
        result = reinterpret_cast<jlong>(plan);
    });
    return result;
}
JNIEXPORT jlongArray JNICALL Java_io_stillflow_StillFlow_nativeFrameInfo(JNIEnv* env, jclass, jlong value) {
    jlongArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        sf_frame_info info{};
        check(sf_frame_plan_get_info(handle<sf_frame_plan>(value), &info, &error), error);
        result = env->NewLongArray(4);
        if (!result)
            return;
        const jlong values[] = {static_cast<jlong>(info.index), info.pts_us, info.duration_us,
                                std::bit_cast<jlong>(info.lookahead_seconds)};
        env->SetLongArrayRegion(result, 0, 4, values);
    });
    return result;
}
JNIEXPORT jfloatArray JNICALL Java_io_stillflow_StillFlow_nativeFrameMap(JNIEnv* env, jclass, jlong value, jint width,
                                                                         jint height) {
    jfloatArray result = nullptr;
    boundary(env, [&] { result = frame_map(env, handle<sf_frame_plan>(value), width, height); });
    return result;
}
JNIEXPORT jlong JNICALL Java_io_stillflow_StillFlow_nativeRendererCreate(JNIEnv* env, jclass, jint backend) {
    jlong result = 0;
    boundary(env, [&] {
        sf_error error{};
        sf_renderer* renderer = nullptr;
        check(sf_renderer_create(backend, nullptr, &renderer, &error), error);
        result = reinterpret_cast<jlong>(renderer);
    });
    return result;
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeRender(JNIEnv* env, jclass, jlong value, jlong plan,
                                                                jint width, jint height, jint format, jint stride,
                                                                jobject pixels, jobject destination) {
    boundary(env, [&] {
        render(env, handle<sf_renderer>(value), handle<sf_frame_plan>(plan), width, height, format, stride, pixels,
               destination);
    });
}
JNIEXPORT void JNICALL Java_io_stillflow_StillFlow_nativeDestroy(JNIEnv*, jclass, jlong value, jint kind) {
    if (!value)
        return;
    switch (kind) {
    case 0:
        sf_analyzer_destroy(handle<sf_analyzer>(value));
        break;
    case 1:
        sf_plan_destroy(handle<sf_plan>(value));
        break;
    case 2:
        sf_frame_plan_destroy(handle<sf_frame_plan>(value));
        break;
    case 3:
        sf_stream_destroy(handle<sf_stream>(value));
        break;
    case 4:
        sf_renderer_destroy(handle<sf_renderer>(value));
        break;
    case 5:
        sf_gyro_stream_destroy(handle<sf_gyro_stream>(value));
        break;
    default:
        break;
    }
}

JNIEXPORT jobjectArray JNICALL Java_io_stillflow_StillFlow_nativeRendererInfo(JNIEnv* env, jclass, jlong handle) {
    jobjectArray result = nullptr;
    boundary(env, [&] {
        sf_error error{};
        sf_renderer_info info{};
        check(sf_renderer_get_info(::handle<sf_renderer>(handle), &info, &error), error);
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
