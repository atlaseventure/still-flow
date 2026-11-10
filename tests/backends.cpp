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
TEST_CASE("CPU and CUDA agree on fractional mesh samples", "[backend][cuda]") {
    std::unique_ptr<stillflow::RenderBackend> cuda;
    try {
        cuda = stillflow::make_cuda_backend();
    } catch (const stillflow::BackendError& e) {
        SKIP(e.what());
    }
    std::vector<uint8_t> pixels(size_t{96} * 64 * 3);
    for (size_t i = 0; i < pixels.size(); ++i)
        pixels[i] = (i * 17) % 256;
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
    double sum = 0;
    int maximum = 0;
    for (size_t i = 0; i < pixels.size(); ++i) {
        const int delta = std::abs(int(a.planes[0].bytes[i]) - int(b.planes[0].bytes[i]));
        sum += delta;
        maximum = std::max(maximum, delta);
    }
    REQUIRE(sum / pixels.size() <= 1.5);
    REQUIRE(maximum <= 8);
}
#endif
