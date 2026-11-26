#ifdef STILLFLOW_TEST_DESKTOP
#include "stillflow/desktop.h"
#include "temp_directory.hpp"
#endif
#include "stillflow/stillflow.h"
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <memory>
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

#ifdef STILLFLOW_TEST_DESKTOP
TEST_CASE("Desktop C API rejects unsupported gyro requests before opening input files", "[c-api][gyro]") {
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
    REQUIRE(sf_stabilize_file(&options, nullptr, &summary, &error) == SF_UNSUPPORTED);
    REQUIRE(std::string_view(error.message).find("Realtime") != std::string_view::npos);
}
#endif
