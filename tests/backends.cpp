#include "stillflow/backends/cpu.hpp"
#ifdef STILLFLOW_TEST_CUDA
#include "stillflow/backends/cuda.hpp"
#endif
#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>

TEST_CASE("CPU identity preserves pixels and downloaded images own their storage", "[backend]") {
    std::vector<uint8_t> pixels(size_t{96} * 64 * 3);
    for (size_t i = 0; i < pixels.size(); ++i)
        pixels[i] = (i * 17) % 256;
    auto input = stillflow::make_host_image({96, 64}, stillflow::PixelFormat::bgr8, pixels);
    auto cpu = stillflow::make_cpu_backend();
    stillflow::Warp warp;
    warp.columns = warp.rows = 2;
    warp.displacement.resize(4);
    const auto map = stillflow::prepare_map(warp, {96, 64}, {96, 64}, 0, false);
    cpu->upload(input);
    REQUIRE(cpu->warp(map) == 0);
    auto output = cpu->download();
    cpu.reset();
    auto view = stillflow::map_host_image(output);
    REQUIRE(std::equal(pixels.begin(), pixels.end(), view.planes[0].bytes.begin()));
}
#ifdef STILLFLOW_TEST_CUDA
TEST_CASE("CUDA cubic reconstruction preserves more subpixel detail than linear sampling", "[backend][cuda]") {
    std::unique_ptr<stillflow::RenderBackend> cuda;
    try {
        cuda = stillflow::make_cuda_backend();
    } catch (const stillflow::BackendError& e) {
        SKIP(e.what());
    }
    const auto signal = [](double x, double y, int channel) {
        return 128 + 55 * std::sin(1.1 * x + channel) + 45 * std::cos(0.9 * y - channel);
    };
    std::vector<uint8_t> pixels(size_t{96} * 64 * 3);
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 96; ++x)
            for (int c = 0; c < 3; ++c)
                pixels[(y * 96 + x) * 3 + c] = static_cast<uint8_t>(std::lround(signal(x, y, c)));
    auto input = stillflow::make_host_image({96, 64}, stillflow::PixelFormat::bgr8, pixels);
    auto cpu = stillflow::make_cpu_backend();
    stillflow::Warp warp;
    warp.columns = 3;
    warp.rows = 4;
    for (int i = 0; i < 12; ++i)
        warp.displacement.push_back({0.7 * std::sin(i), 0.6 * std::cos(i)});
    auto map = stillflow::prepare_map(warp, {96, 64}, {96, 64}, 0.08, false);
    cpu->upload(input);
    cuda->upload(input);
    REQUIRE(cpu->warp(map) == Catch::Approx(cuda->warp(map)).margin(1.0 / (96 * 64)));
    auto a = stillflow::map_host_image(cpu->download());
    auto b = stillflow::map_host_image(cuda->download());
    double linear_error = 0, cubic_error = 0;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 96; ++x) {
            const auto p = stillflow::source_point(map, {map.origin_x + x * map.step_x, map.origin_y + y * map.step_y});
            for (int c = 0; c < 3; ++c) {
                const size_t i = (y * 96 + x) * 3 + c;
                const double expected = signal(p.x, p.y, c);
                linear_error += std::pow(a.planes[0].bytes[i] - expected, 2);
                cubic_error += std::pow(b.planes[0].bytes[i] - expected, 2);
            }
        }
    REQUIRE(cubic_error < linear_error * 0.4);
    REQUIRE(std::sqrt(cubic_error / pixels.size()) < 2);
    // Integer reconstruction also covers the four image borders.
    warp.displacement.assign(12, {});
    map = stillflow::prepare_map(warp, {96, 64}, {96, 64}, 0, false);
    cuda->upload(input);
    REQUIRE(cuda->warp(map) == 0);
    const auto identity = stillflow::map_host_image(cuda->download());
    REQUIRE(std::equal(pixels.begin(), pixels.end(), identity.planes[0].bytes.begin()));
}
#endif
