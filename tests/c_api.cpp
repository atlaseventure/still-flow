#ifdef STILLFLOW_TEST_DESKTOP
#include "stillflow/desktop.h"
#include "temp_directory.hpp"
#endif
#include "stillflow/stillflow.h"
#include <algorithm>
#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <cstring>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace {
struct Api {
    sf_settings settings = sf_settings_default();
    sf_video video{64, 64, 0, 30};
    sf_image image = sf_image_default();
    sf_error error{};
    std::vector<uint8_t> pixels = std::vector<uint8_t>(4096);
    sf_analyzer* analyzer = nullptr;
    sf_plan* plan = nullptr;
    sf_frame_plan* frame_plan = nullptr;
    sf_renderer* renderer = nullptr;
    sf_frame* frame = nullptr;
    Api() {
        uint32_t seed = 42;
        for (auto& p : pixels) {
            seed = seed * 1664525 + 1013904223;
            p = seed >> 24;
        }
        image.width = 64;
        image.height = 64;
        image.format = SF_GRAY8;
        image.plane_count = 1;
        image.planes[0] = {pixels.data(), pixels.size(), 64};
    }
    ~Api() {
        sf_frame_destroy(frame);
        sf_renderer_destroy(renderer);
        sf_frame_plan_destroy(frame_plan);
        sf_plan_destroy(plan);
        sf_analyzer_destroy(analyzer);
    }
    void create(const sf_runtime* runtime = nullptr) {
        REQUIRE(sf_analyzer_create(&settings, &video, runtime, &analyzer, &error) == SF_OK);
    }
};
} // namespace
TEST_CASE("C API validates arguments and contains exceptions", "[c-api]") {
    Api api;
    auto bad = api.settings;
    bad.analysis_width = 1;
    REQUIRE(sf_analyzer_create(&bad, &api.video, nullptr, &api.analyzer, &api.error) == SF_INVALID_ARGUMENT);
    REQUIRE(api.analyzer == nullptr);
    REQUIRE(std::strlen(api.error.message) > 0);
    api.create();
    auto invalid = api.image;
    invalid.planes[0].row_stride = 1;
    REQUIRE(sf_analyzer_push(api.analyzer, &invalid, 0, 33333, &api.error) == SF_INVALID_ARGUMENT);
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_OK);
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_INVALID_ARGUMENT);
    REQUIRE(sf_analyzer_finish(api.analyzer, &api.plan, &api.error) == SF_OK);
    sf_plan* extra = nullptr;
    REQUIRE(sf_analyzer_finish(api.analyzer, &extra, &api.error) == SF_INVALID_STATE);
    REQUIRE(extra == nullptr);
}
TEST_CASE("C ABI retains input leases and renders independently owned output", "[c-api]") {
    int leases = 0;
    Api api;
    api.image.color.kr = 0.299f;
    api.image.color.kb = 0.114f;
    api.image.color.full_range = 1;
    api.image.user_data = &leases;
    api.image.retain = [](void* p) { ++*static_cast<int*>(p); };
    api.image.release = [](void* p) { --*static_cast<int*>(p); };
    api.create();
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_OK);
    REQUIRE(leases == 1);
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 33333, 33333, &api.error) == SF_OK);
    REQUIRE(leases == 1);
    REQUIRE(sf_analyzer_finish(api.analyzer, &api.plan, &api.error) == SF_OK);
    REQUIRE(leases == 0);
    sf_plan_info info{};
    REQUIRE(sf_plan_get_info(api.plan, &info, &api.error) == SF_OK);
    REQUIRE(info.frames == 2);
    REQUIRE(sf_renderer_create(SF_BACKEND_CPU, nullptr, &api.renderer, &api.error) == SF_OK);
    REQUIRE(sf_plan_get_frame(api.plan, 0, &api.frame_plan, &api.error) == SF_OK);
    sf_plan_destroy(api.plan);
    api.plan = nullptr;
    REQUIRE(sf_renderer_render(api.renderer, api.frame_plan, &api.image, &api.frame, &api.error) == SF_OK);
    sf_image output{};
    REQUIRE(sf_frame_get_image(api.frame, &output, &api.error) == SF_OK);
    REQUIRE(output.color.kr == api.image.color.kr);
    REQUIRE(output.color.kb == api.image.color.kb);
    REQUIRE(output.color.full_range == 1);
    sf_gyro_map gyro_map{};
    REQUIRE(sf_frame_plan_get_gyro_map(api.frame_plan, &gyro_map, &api.error) == SF_INVALID_STATE);
    int32_t width = 0, height = 0;
    REQUIRE(sf_frame_plan_get_output_size(api.frame_plan, &width, &height, &api.error) == SF_OK);
    REQUIRE(width == api.video.width);
    REQUIRE(height == api.video.height);
    auto value = output.planes[0].data[0];
    sf_renderer_destroy(api.renderer);
    api.renderer = nullptr;
    REQUIRE(output.planes[0].data[0] == value);
    REQUIRE(leases == 0);
}
TEST_CASE("C ABI translates cancellation and invalid host scheduler behavior into status codes", "[c-api]") {
    Api api;
    bool stop = false;
    sf_runtime runtime{&stop, [](void* p) -> int32_t { return *static_cast<bool*>(p); }, nullptr, nullptr};
    api.create(&runtime);
    stop = true;
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_CANCELLED);
    stop = false;
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_OK);
    runtime.execute = [](void*, size_t, sf_work_fn, void*) {};
    Api broken;
    broken.create(&runtime);
    REQUIRE(sf_analyzer_push(broken.analyzer, &broken.image, 0, 33333, &broken.error) == SF_OK);
    REQUIRE(sf_analyzer_push(broken.analyzer, &broken.image, 33333, 33333, &broken.error) == SF_INVALID_ARGUMENT);
}

TEST_CASE("C ABI copies borrowed frames and validates YUV plane extents", "[c-api]") {
    Api api;
    api.create();
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 0, 33333, &api.error) == SF_OK);
    std::fill(api.pixels.begin(), api.pixels.end(), 0);
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 33333, 33333, &api.error) == SF_OK);
    api.image.format = SF_NV12;
    api.image.plane_count = 2;
    std::vector<uint8_t> chroma(2048, 128);
    api.image.planes[1] = {chroma.data(), 10, 64};
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 66666, 33333, &api.error) == SF_INVALID_ARGUMENT);
    api.image.planes[1].size_bytes = chroma.size();
    REQUIRE(sf_analyzer_push(api.analyzer, &api.image, 66666, 33333, &api.error) == SF_OK);
    REQUIRE(sf_analyzer_finish(api.analyzer, &api.plan, &api.error) == SF_OK);
}

namespace {
struct GyroApi : Api {
    sf_gyro_settings gyro = sf_gyro_settings_default();
    sf_lens_profile lens = sf_lens_profile_default();
    std::array<sf_frame_timestamp, 2> times{{{9007199254740993LL, 33333}, {9007199254774326LL, 0}}};
    std::array<sf_imu_sample, 2> imu{};

    sf_stream_settings buffering = sf_stream_settings_default();
    sf_gyro_stream* stream = nullptr;
    ~GyroApi() { sf_gyro_stream_destroy(stream); }
    sf_status create_stream(const sf_runtime* runtime = nullptr) {
        return sf_gyro_stream_create(&gyro, &lens, &buffering, image.width, image.height, runtime, &stream, &error);
    }
    GyroApi() {
        image.height = 48;
        lens.fx = 49;
        lens.fy = 50;
        lens.cx = 31.5;
        lens.cy = 23.5;
        imu.back().time_s = 0.1;
    }
    sf_status plan_gyro(const sf_runtime* runtime = nullptr) {
        return sf_gyro_plan_create(&gyro, &lens, image.width, image.height, times.data(), times.size(), imu.data(),
                                   imu.size(), runtime, &plan, &error);
    }
    void take_frame() {
        REQUIRE(sf_plan_get_frame(plan, 1, &frame_plan, &error) == SF_OK);
        sf_plan_destroy(plan);
        plan = nullptr;
    }
    void check_time(sf_frame_info info) const {
        REQUIRE(info.index == 1);
        REQUIRE(info.pts_us == times[1].pts_us);
        REQUIRE(info.duration_us == times[1].duration_us);
        REQUIRE(info.lookahead_seconds == 0);
    }
};
} // namespace

TEST_CASE("Core C API owns rolling-shutter gyro maps and renders resized fisheye frames", "[c-api][gyro]") {
    GyroApi api;
    api.gyro.output_width = 48;
    api.gyro.output_height = 32;
    api.gyro.readout_ms = 12;
    api.lens.model = SF_LENS_OPENCV_FISHEYE;
    api.lens.k[0] = 0.05;
    api.lens.lens_correction_amount = 0.8;
    for (auto& sample : api.imu)
        sample.wy = 0.4;
    std::string log;
    sf_runtime runtime{&log, nullptr, [](void* p, const char* message) { *static_cast<std::string*>(p) = message; },
                       nullptr};
    REQUIRE(api.plan_gyro(&runtime) == SF_OK);
    REQUIRE(log == "Planning gyro stabilization");
    sf_plan_info info{};
    REQUIRE(sf_plan_get_info(api.plan, &info, &api.error) == SF_OK);
    REQUIRE(info.frames == api.times.size());
    REQUIRE(info.origin_us == api.times.front().pts_us);
    REQUIRE(info.analysis_width == 48);
    REQUIRE(info.analysis_height == 32);
    REQUIRE(sf_plan_get_frame(api.plan, 2, &api.frame_plan, &api.error) == SF_INVALID_ARGUMENT);
    api.take_frame();
    sf_frame_info timing{};
    REQUIRE(sf_frame_plan_get_info(api.frame_plan, &timing, &api.error) == SF_OK);
    api.check_time(timing);
    int32_t width = 0, height = 0;
    REQUIRE(sf_frame_plan_get_output_size(api.frame_plan, &width, &height, &api.error) == SF_OK);
    REQUIRE(width == 48);
    REQUIRE(height == 32);
    sf_map visual{};
    REQUIRE(sf_frame_plan_get_map(api.frame_plan, 48, 32, &visual, &api.error) == SF_INVALID_STATE);
    sf_gyro_map map{};
    REQUIRE(sf_frame_plan_get_gyro_map(api.frame_plan, &map, &api.error) == SF_OK);
    REQUIRE(map.input_width == 64);
    REQUIRE(map.input_height == 48);
    REQUIRE(map.output_width == width);
    REQUIRE(map.output_height == height);
    REQUIRE(map.lens_model == api.lens.model);
    REQUIRE(map.in_fx == api.lens.fx);
    REQUIRE(map.in_fy == api.lens.fy);
    REQUIRE(map.in_cx == api.lens.cx);
    REQUIRE(map.in_cy == api.lens.cy);
    REQUIRE(std::ranges::equal(map.k, api.lens.k));
    REQUIRE(map.lens_correction_amount == api.lens.lens_correction_amount);
    REQUIRE(map.out_cx == 23.5);
    REQUIRE(map.out_cy == 15.5);
    REQUIRE(map.fov_scale > 0);
    REQUIRE(map.out_fx == api.lens.fx * 0.75 / map.fov_scale);
    REQUIRE(map.out_fy == api.lens.fy * 0.75 / map.fov_scale);
    REQUIRE(map.zoom_ratio >= 1);
    REQUIRE(map.zoom_ratio <= api.gyro.max_zoom);
    REQUIRE(map.smooth_factor > 0);
    REQUIRE(map.sensor_rows == 1);
    REQUIRE(map.row_count > 1);
    REQUIRE(map.row_count <= SF_GYRO_MAX_ROW_ROTATIONS);
    // Positive yaw during top-to-bottom readout rotates the bottom ray further left.
    REQUIRE(map.row_rotations[2] > map.row_rotations[(map.row_count - 1) * 9 + 2]);
    REQUIRE(map.row_rotations[2] == -map.row_rotations[6]);

    REQUIRE(sf_renderer_create(SF_BACKEND_CPU, nullptr, &api.renderer, &api.error) == SF_OK);
    auto wrong = api.image;
    wrong.width = width;
    wrong.height = height;
    REQUIRE(sf_renderer_render(api.renderer, api.frame_plan, &wrong, &api.frame, &api.error) == SF_INVALID_ARGUMENT);
    std::fill(api.pixels.begin(), api.pixels.end(), 137);
    api.image.color = {0.299f, 0.114f, 0.5f, 0.5f, 1};
    REQUIRE(sf_renderer_render(api.renderer, api.frame_plan, &api.image, &api.frame, &api.error) == SF_OK);
    REQUIRE(sf_frame_get_info(api.frame, &timing, &api.error) == SF_OK);
    api.check_time(timing);
    sf_image output{};
    REQUIRE(sf_frame_get_image(api.frame, &output, &api.error) == SF_OK);
    REQUIRE(output.width == width);
    REQUIRE(output.height == height);
    REQUIRE(output.format == api.image.format);
    REQUIRE(output.color.kr == api.image.color.kr);
    REQUIRE(output.color.kb == api.image.color.kb);
    REQUIRE(output.color.chroma_x == api.image.color.chroma_x);
    REQUIRE(output.color.chroma_y == api.image.color.chroma_y);
    REQUIRE(output.color.full_range == api.image.color.full_range);
    REQUIRE(output.planes[0].size_bytes == size_t(width * height));
    REQUIRE(std::ranges::all_of(std::span(output.planes[0].data, output.planes[0].size_bytes),
                                [](auto v) { return v == 137; }));
}

TEST_CASE("Core C gyro renderer preserves packed channels and NV12 at integer pinhole coordinates", "[c-api][gyro]") {
    const bool streaming = GENERATE(false, true);
    for (auto format : {SF_BGR8, SF_RGBA8, SF_GRAY8, SF_NV12}) {
        CAPTURE(format);
        GyroApi api;
        // Leave a two-pixel guard on each edge for the planner's sensor margin.
        // The even offset preserves the NV12 chroma sample grid as well.
        api.gyro.output_width = 60;
        api.gyro.output_height = 44;
        api.gyro.fov = 60.0 / 64;
        api.gyro.integration_method = SF_IMU_AUTO;
        if (streaming) {
            REQUIRE(api.create_stream() == SF_OK);
            REQUIRE(sf_gyro_stream_push_imu(api.stream, api.imu.data(), api.imu.size(), nullptr) == SF_OK);
            REQUIRE(sf_gyro_stream_push_frame(api.stream, api.times[0].pts_us, api.times[0].duration_us, nullptr) ==
                    SF_OK);
            REQUIRE(sf_gyro_stream_finish(api.stream, nullptr) == SF_OK);
            REQUIRE(sf_gyro_stream_take_ready(api.stream, &api.frame_plan, nullptr) == SF_OK);
            sf_gyro_stream_destroy(api.stream);
            api.stream = nullptr;
        } else {
            REQUIRE(api.plan_gyro() == SF_OK);
            api.take_frame();
        }
        sf_gyro_map map{};
        REQUIRE(sf_frame_plan_get_gyro_map(api.frame_plan, &map, nullptr) == SF_OK);
        REQUIRE(map.row_count == 1);
        REQUIRE(map.sensor_rows == 0);
        REQUIRE(map.fov_scale == 1);
        REQUIRE(map.zoom_ratio == 1);
        REQUIRE(map.out_fx == map.in_fx);
        REQUIRE(map.out_fy == map.in_fy);
        const std::array<double, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
        REQUIRE(std::equal(identity.begin(), identity.end(), map.row_rotations));
        const size_t channels = format == SF_BGR8 ? 3 : format == SF_RGBA8 ? 4 : 1;
        const size_t stride = 64 * channels + 3;
        std::vector<uint8_t> pixels(stride * 48);
        std::vector<uint8_t> chroma(size_t{64} * 24);
        for (size_t i = 0; i < pixels.size(); ++i)
            pixels[i] = api.pixels[i % api.pixels.size()];
        std::copy_n(api.pixels.begin(), chroma.size(), chroma.begin());
        api.image.format = format;
        api.image.planes[0] = {pixels.data(), pixels.size(), stride};
        if (format == SF_NV12) {
            api.image.plane_count = 2;
            api.image.planes[1] = {chroma.data(), chroma.size(), 64};
        }
        REQUIRE(sf_renderer_create(SF_BACKEND_CPU, nullptr, &api.renderer, nullptr) == SF_OK);
        REQUIRE(sf_renderer_render(api.renderer, api.frame_plan, &api.image, &api.frame, nullptr) == SF_OK);
        sf_image output{};
        REQUIRE(sf_frame_get_image(api.frame, &output, nullptr) == SF_OK);
        REQUIRE(output.width == 60);
        REQUIRE(output.height == 44);
        REQUIRE(output.format == format);
        REQUIRE(output.plane_count == api.image.plane_count);
        for (int i = 0; i < output.plane_count; ++i) {
            const auto& in = api.image.planes[i];
            const auto& out = output.planes[i];
            const size_t rows = i == 0 ? 44 : 22;
            const size_t bytes = 60 * channels;
            REQUIRE(out.size_bytes >= (rows - 1) * out.row_stride + bytes);
            for (size_t y = 0; y < rows; ++y) {
                const auto* expected = in.data + (y + (i == 0 ? 2 : 1)) * in.row_stride + 2 * channels;
                REQUIRE(std::memcmp(out.data + y * out.row_stride, expected, bytes) == 0);
            }
        }
    }
}

TEST_CASE("Core C gyro stream retries unconsumed frames and drains exact VFR plans", "[c-api][gyro][stream]") {
    GyroApi api;
    api.gyro.output_width = 48;
    api.gyro.output_height = 32;
    api.buffering.max_buffered_frames = 2;
    REQUIRE(api.create_stream() == SF_OK);
    int32_t width = 0, height = 0;
    REQUIRE(sf_gyro_stream_get_output_size(api.stream, &width, &height, nullptr) == SF_OK);
    REQUIRE(width == 48);
    REQUIRE(height == 32);
    const auto start = api.times[0].pts_us;
    const std::array<int64_t, 3> pts{start, start + 17003, start + 57009};
    const std::array<int64_t, 3> duration{17003, 40006, 22001};
    auto push_imu = [&](size_t i) {
        sf_imu_sample sample{};
        sample.time_s = (pts[i] - start) / 1e6;
        sample.wy = 0.25;
        REQUIRE(sf_gyro_stream_push_imu(api.stream, &sample, 1, nullptr) == SF_OK);
    };
    auto push = [&](size_t i) { return sf_gyro_stream_push_frame(api.stream, pts[i], 22001, nullptr); };
    auto take = [&](sf_status expected) {
        sf_frame_plan_destroy(api.frame_plan);
        api.frame_plan = nullptr;
        REQUIRE(sf_gyro_stream_take_ready(api.stream, &api.frame_plan, nullptr) == expected);
        REQUIRE((api.frame_plan != nullptr) == (expected == SF_OK));
    };
    auto check = [&](size_t i, double ahead) {
        sf_frame_info info{};
        REQUIRE(sf_frame_plan_get_info(api.frame_plan, &info, nullptr) == SF_OK);
        REQUIRE(info.index == i);
        REQUIRE(info.pts_us == pts[i]);
        REQUIRE(info.duration_us == duration[i]);
        REQUIRE(info.lookahead_seconds == Catch::Approx(ahead));
        REQUIRE(sf_frame_plan_get_output_size(api.frame_plan, &width, &height, nullptr) == SF_OK);
        REQUIRE(width == 48);
        REQUIRE(height == 32);
        sf_gyro_map map{};
        REQUIRE(sf_frame_plan_get_gyro_map(api.frame_plan, &map, nullptr) == SF_OK);
        REQUIRE(map.input_width == api.image.width);
        REQUIRE(map.output_height == height);
    };
    REQUIRE(sf_gyro_stream_push_imu(api.stream, nullptr, 0, nullptr) == SF_OK);
    push_imu(0);
    REQUIRE(push(0) == SF_OK);
    take(SF_AGAIN);
    push_imu(1);
    REQUIRE(push(1) == SF_OK);
    REQUIRE(push(2) == SF_AGAIN);
    take(SF_OK);
    check(0, duration[0] / 1e6);
    take(SF_AGAIN);
    push_imu(2);
    REQUIRE(push(2) == SF_OK);
    REQUIRE(sf_gyro_stream_finish(api.stream, nullptr) == SF_OK);
    REQUIRE(sf_gyro_stream_finish(api.stream, nullptr) == SF_OK);
    REQUIRE(push(2) == SF_INVALID_STATE);
    REQUIRE(sf_gyro_stream_push_imu(api.stream, nullptr, 0, nullptr) == SF_INVALID_STATE);
    take(SF_OK);
    check(1, duration[1] / 1e6);
    take(SF_OK);
    check(2, 0);
    take(SF_END);
    uint64_t count = 0;
    auto visitor = [](void* data, const sf_profile_entry* entry) {
        if (std::string_view(entry->stage) == "plan.gyro_stream")
            *static_cast<uint64_t*>(data) = entry->count;
    };
    REQUIRE(sf_gyro_stream_get_profile(api.stream, visitor, &count, nullptr) == SF_OK);
    REQUIRE(count >= pts.size());
}

TEST_CASE("Core C gyro stream validates descriptors, timing and cancellation", "[c-api][gyro][stream]") {
    GyroApi api;
    bool stop = false;
    sf_runtime runtime{&stop, [](void* data) -> int32_t { return *static_cast<bool*>(data); }, nullptr, nullptr};
    SECTION("invalid creation") {
        SECTION("lens") { api.lens.fx = 0; }
        SECTION("gyro settings") { api.gyro.per_axis = 2; }
        SECTION("buffer capacity") { api.buffering.max_buffered_frames = 1; }
        SECTION("input size") { api.image.width = 1; }
        REQUIRE(api.create_stream() == SF_INVALID_ARGUMENT);
        REQUIRE(api.stream == nullptr);
    }
    SECTION("cancelled creation") {
        stop = true;
        REQUIRE(api.create_stream(&runtime) == SF_CANCELLED);
        REQUIRE(api.stream == nullptr);
    }
    SECTION("invalid calls") {
        REQUIRE(api.create_stream(&runtime) == SF_OK);
        int32_t height = 0;
        REQUIRE(sf_gyro_stream_get_output_size(api.stream, nullptr, &height, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_get_profile(api.stream, nullptr, nullptr, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_take_ready(api.stream, nullptr, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_push_imu(api.stream, nullptr, 1, nullptr) == SF_INVALID_ARGUMENT);
        api.imu[1].has_accel = 2;
        REQUIRE(sf_gyro_stream_push_imu(api.stream, api.imu.data(), 2, nullptr) == SF_INVALID_ARGUMENT);
        api.imu[1].has_accel = 0;
        REQUIRE(sf_gyro_stream_push_imu(api.stream, api.imu.data(), 2, nullptr) == SF_OK);
        REQUIRE(sf_gyro_stream_push_frame(api.stream, 100, 1, nullptr) == SF_OK);
        REQUIRE(sf_gyro_stream_push_frame(api.stream, 100, 1, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_push_frame(api.stream, 99, 1, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_push_frame(api.stream, 101, -1, nullptr) == SF_INVALID_ARGUMENT);
        REQUIRE(sf_gyro_stream_push_frame(api.stream, 101, 1, nullptr) == SF_OK);
    }
    SECTION("cancelled processing") {
        REQUIRE(api.create_stream(&runtime) == SF_OK);
        stop = true;
        SECTION("IMU") { REQUIRE(sf_gyro_stream_push_imu(api.stream, nullptr, 0, nullptr) == SF_CANCELLED); }
        SECTION("frame") { REQUIRE(sf_gyro_stream_push_frame(api.stream, 0, 1, nullptr) == SF_CANCELLED); }
        SECTION("finish") { REQUIRE(sf_gyro_stream_finish(api.stream, nullptr) == SF_CANCELLED); }
        SECTION("drain") { REQUIRE(sf_gyro_stream_take_ready(api.stream, &api.frame_plan, nullptr) == SF_CANCELLED); }
    }
}

TEST_CASE("Core C gyro planner rejects invalid descriptors and reports cancellation", "[c-api][gyro]") {
    GyroApi api;
    auto* settings = &api.gyro;
    auto* lens = &api.lens;
    auto* times = api.times.data();
    auto* imu = api.imu.data();
    auto** out = &api.plan;
    size_t frame_count = api.times.size(), imu_count = api.imu.size();
    int width = 64, height = 48;
    sf_status expected = SF_INVALID_ARGUMENT;
    sf_runtime runtime{};
    int remaining_checks = 0;
    SECTION("missing settings") { settings = nullptr; }
    SECTION("missing lens") { lens = nullptr; }
    SECTION("missing timestamps") { times = nullptr; }
    SECTION("missing samples") { imu = nullptr; }
    SECTION("missing plan output") { out = nullptr; }
    SECTION("empty timestamps") { frame_count = 0; }
    SECTION("empty samples") { imu_count = 0; }
    SECTION("input width") { width = 1; }
    SECTION("input height") { height = 1; }
    SECTION("duplicate timestamp") { times[1].pts_us = times[0].pts_us; }
    SECTION("decreasing timestamp") { times[1].pts_us = times[0].pts_us - 1; }
    SECTION("negative duration") { times[1].duration_us = -1; }
    SECTION("lens model") { lens->model = 2; }
    SECTION("lens focal length") { lens->fx = 0; }
    SECTION("lens correction") { lens->lens_correction_amount = 1.01; }
    SECTION("integration method") { settings->integration_method = -1; }
    SECTION("per-axis flag") { settings->per_axis = -1; }
    SECTION("readout flag") { settings->readout_top_to_bottom = 2; }
    SECTION("adaptive zoom flag") { settings->adaptive_zoom = 2; }
    SECTION("zoom envelope flag") { settings->zoom_envelope = 2; }
    SECTION("accelerometer flag") { imu[1].has_accel = 2; }
    SECTION("unrenderable output size") {
        settings->output_width = 48;
        settings->output_height = 1;
    }
    SECTION("smoothness") { settings->smoothness = -0.1; }
    SECTION("sensor bounds exceed max_zoom") {
        settings->fov = 2;
        settings->max_zoom = 1;
    }
    SECTION("cancellation") {
        SECTION("before planning") { remaining_checks = 1; }
        SECTION("after integration") { remaining_checks = 2; }
        SECTION("after planning") { remaining_checks = 3; }
        runtime.user_data = &remaining_checks;
        runtime.cancelled = [](void* p) -> int32_t { return --*static_cast<int*>(p) == 0; };
        expected = SF_CANCELLED;
    }
    REQUIRE(sf_gyro_plan_create(settings, lens, width, height, times, frame_count, imu, imu_count, &runtime, out,
                                &api.error) == expected);
    REQUIRE(api.plan == nullptr);
    REQUIRE(std::strlen(api.error.message) > 0);
}

TEST_CASE("Core C gyro map and output size accessors validate pointers", "[c-api][gyro]") {
    GyroApi api;
    api.gyro.integration_method = SF_IMU_COMPLEMENTARY;
    REQUIRE(api.plan_gyro() == SF_OK);
    api.take_frame();
    sf_gyro_map map{};
    REQUIRE(sf_frame_plan_get_gyro_map(nullptr, &map, nullptr) == SF_INVALID_ARGUMENT);
    REQUIRE(sf_frame_plan_get_gyro_map(api.frame_plan, nullptr, nullptr) == SF_INVALID_ARGUMENT);
    int32_t width = 0, height = 0;
    REQUIRE(sf_frame_plan_get_output_size(nullptr, &width, &height, nullptr) == SF_INVALID_ARGUMENT);
    REQUIRE(sf_frame_plan_get_output_size(api.frame_plan, nullptr, &height, nullptr) == SF_INVALID_ARGUMENT);
    REQUIRE(sf_frame_plan_get_output_size(api.frame_plan, &width, nullptr, nullptr) == SF_INVALID_ARGUMENT);
}

#ifdef STILLFLOW_TEST_DESKTOP
TEST_CASE("Desktop C API accepts streaming gyro and rejects GLES before opening input files", "[c-api][gyro]") {
    auto options = sf_file_options_default();
    stillflow::test::TempDirectory temp;
    const auto missing = (temp.path / "stabilization.json").string();
    options.gyro.config = missing.c_str();
    sf_run_summary summary{};
    sf_error error{};
    options.backend = SF_BACKEND_GLES;
    REQUIRE(sf_stabilize_file(&options, nullptr, &summary, &error) == SF_UNSUPPORTED);
    REQUIRE(std::string_view(error.message).find("GLES") != std::string_view::npos);
    options.backend = SF_BACKEND_CPU;
    REQUIRE(sf_stabilize_file(&options, nullptr, &summary, &error) == SF_INTERNAL_ERROR);
    REQUIRE(std::string_view(error.message).find("Cannot open gyro configuration") != std::string_view::npos);
    options.backend = SF_BACKEND_AUTO;
    options.mode = SF_FILE_STREAM;
    REQUIRE(sf_stabilize_file(&options, nullptr, &summary, &error) == SF_INTERNAL_ERROR);
    REQUIRE(std::string_view(error.message).find("Cannot open gyro configuration") != std::string_view::npos);
}
#endif
