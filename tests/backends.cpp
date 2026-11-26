#include "stillflow/adapters/gyro.hpp"
#include "stillflow/backends/cpu.hpp"
#include "temp_directory.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#ifdef STILLFLOW_TEST_CUDA
#include "stillflow/backends/cuda.hpp"
#endif
#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <numbers>

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

namespace {
using namespace stillflow;

GyroFrameMap pinhole_map(Size size) {
    GyroFrameMap map;
    map.input_size = map.output_size = size;
    map.lens.fx = map.out_fx = size.width * 0.73;
    map.lens.fy = map.out_fy = size.height * 0.87;
    map.lens.cx = map.out_cx = (size.width - 1) * 0.5;
    map.lens.cy = map.out_cy = (size.height - 1) * 0.5;
    map.row_rotations = {Matrix3::eye()};
    return map;
}

Image signal_image(Size size, const auto& signal) {
    std::vector<uint8_t> pixels(static_cast<size_t>(size.width) * size.height * 3);
    for (int y = 0; y < size.height; ++y)
        for (int x = 0; x < size.width; ++x)
            for (int c = 0; c < 3; ++c)
                pixels[(static_cast<size_t>(y) * size.width + x) * 3 + c] =
                    static_cast<uint8_t>(std::lround(signal(x, y, c)));
    return make_host_image(size, PixelFormat::bgr8, std::move(pixels));
}

BackendProvider cpu_provider() {
    return {"cpu", [] { return make_cpu_backend(); }};
}

class VisualOnlyBackend final : public RenderBackend {
  public:
    BackendInfo info() const override { return {"visual", "host", {{PixelFormat::bgr8}, {"host"}}, {}}; }
    void upload(const Image& input) override { image_ = input; }
    double warp(const MapParameters&) override { return 0; }
    Image download() override { return image_; }

  private:
    Image image_;
};

TEST_CASE("Gyro rendering propagates an unsupported backend and retries the next provider", "[backend][gyro]") {
    auto map = pinhole_map({8, 6});
    const auto input = signal_image(map.input_size, [](int x, int y, int c) { return x + y + c; });
    const BackendProvider visual{"visual", [] { return std::make_unique<VisualOnlyBackend>(); }};
    const auto factory = make_renderer_factory({visual});
    Profile profile;
    double invalid = -1;
    for (const auto* request : {"visual", "auto"}) {
        auto renderer = factory->create(request);
        REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), NotImplementedError);
        const auto report = renderer->report();
        REQUIRE(report.failures.size() == 1);
        REQUIRE(report.failures.front().backend == "visual");
        REQUIRE(report.failures.front().stage == "render");
        REQUIRE(report.backends.front().frames == 0);
        REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), NotImplementedError);
        Warp visual;
        visual.columns = visual.rows = 2;
        visual.displacement.resize(4);
        REQUIRE(renderer->render(input, visual, input->size(), 0, false, profile));
    }
    auto renderer = make_renderer_factory({visual, cpu_provider()})->create("auto");
    REQUIRE(renderer->render_gyro(input, map, invalid, profile));
    REQUIRE(renderer->report().backend == "cpu");
    REQUIRE(renderer->report().failures.size() == 1);
    REQUIRE(renderer->report().failures.front().backend == "visual");
    renderer = factory->create("auto");
    REQUIRE_THROWS_AS(renderer->render_gyro({}, map, invalid, profile), std::invalid_argument);
    map.row_rotations.clear();
    REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), std::invalid_argument);
    REQUIRE(renderer->report().failures.empty());
}

Image patterned_image(Size size, PixelFormat format, ColorInfo color) {
    auto storage = std::make_shared<std::array<std::vector<uint8_t>, 3>>();
    HostImageView view{size, format, {}, storage, color};
    const auto add_plane = [&](int width, int height, size_t channels) {
        const size_t index = view.planes.size(), stride = width * channels + 7;
        auto& bytes = (*storage)[index];
        bytes.resize(stride * height);
        for (size_t i = 0; i < bytes.size(); ++i)
            bytes[i] = static_cast<uint8_t>(17 * i + 53 * index);
        view.planes.push_back({stride, bytes});
    };
    if (format == PixelFormat::i420 || format == PixelFormat::nv12 || format == PixelFormat::nv21) {
        add_plane(size.width, size.height, 1);
        add_plane((size.width + 1) / 2, (size.height + 1) / 2, format == PixelFormat::i420 ? 1 : 2);
        if (format == PixelFormat::i420)
            add_plane((size.width + 1) / 2, (size.height + 1) / 2, 1);
    } else
        add_plane(size.width, size.height, pixel_bytes(format));
    return borrow_host_image(std::move(view));
}

TEST_CASE("CPU gyro pinhole identity preserves every channel and color metadata", "[backend][gyro]") {
    auto cpu = make_cpu_backend();
    const auto map = pinhole_map({160, 120});
    for (auto format : {PixelFormat::gray8, PixelFormat::bgr8, PixelFormat::rgba8, PixelFormat::nv12, PixelFormat::nv21,
                        PixelFormat::i420}) {
        CAPTURE(pixel_format_name(format));
        const ColorInfo color{0.299f, 0.114f, true, 0.5f, 0.5f};
        const auto input = patterned_image(map.input_size, format, color);
        cpu->upload(input);
        REQUIRE(cpu->warp_gyro(map) == 0);
        const auto expected = map_host_image(input), actual = map_host_image(cpu->download());
        REQUIRE(actual.format == format);
        REQUIRE(actual.color.kr == color.kr);
        REQUIRE(actual.color.kb == color.kb);
        REQUIRE(actual.color.full_range == color.full_range);
        REQUIRE(actual.color.chroma_x == color.chroma_x);
        REQUIRE(actual.color.chroma_y == color.chroma_y);
        REQUIRE(actual.planes.size() == expected.planes.size());
        for (size_t i = 0; i < actual.planes.size(); ++i) {
            const auto& plane = actual.planes[i];
            const auto& source = expected.planes[i];
            const auto width_bytes = source.row_stride - 7;
            for (int y = 0; y < (i == 0 ? 120 : 60); ++y)
                REQUIRE(std::ranges::equal(plane.bytes.subspan(y * plane.row_stride, width_bytes),
                                           source.bytes.subspan(y * source.row_stride, width_bytes)));
        }
    }
}

TEST_CASE("CPU gyro rejects missing input and invalid map geometry", "[backend][gyro]") {
    auto cpu = make_cpu_backend();
    const auto map = pinhole_map({8, 6});
    REQUIRE_THROWS_AS(cpu->warp_gyro(map), std::invalid_argument);
    cpu->upload(patterned_image(map.input_size, PixelFormat::gray8, {}));
    const auto reject = [&](const auto& change) {
        auto bad = map;
        change(bad);
        REQUIRE_THROWS_AS(cpu->warp_gyro(bad), std::invalid_argument);
    };
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    reject([](auto& m) { --m.input_size.width; });
    reject([](auto& m) { m.output_size.height = 1; });
    reject([](auto& m) { m.row_rotations.clear(); });
    reject([&](auto& m) { m.row_rotations.front()(2, 2) = nan; });
    reject([](auto& m) { m.out_fx = 0; });
    reject([](auto& m) { m.lens.fy = -1; });
    reject([&](auto& m) { m.out_cy = nan; });
    reject([&](auto& m) { m.lens.cx = nan; });
    reject([&](auto& m) { m.lens.k[2] = nan; });
    reject([](auto& m) { m.lens.lens_correction_amount = 1.01; });
    reject([&](auto& m) { m.lens.lens_correction_amount = nan; });
    // Exercise rejection of an unknown lens model supplied by a caller.
    // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
    reject([](auto& m) { m.lens.model = static_cast<LensModel>(2); });
    cpu->upload(patterned_image({1, 6}, PixelFormat::gray8, {}));
    reject([](auto& m) { m.input_size.width = 1; });
}

TEST_CASE("CPU gyro projects YUV chroma independently on an odd-width crop", "[backend][gyro]") {
    auto cpu = make_cpu_backend();
    auto map = pinhole_map({160, 120});
    map.output_size = {127, 90};
    map.out_cy -= 30.75;
    for (auto format : {PixelFormat::nv12, PixelFormat::nv21, PixelFormat::i420})
        for (bool full_range : {false, true}) {
            CAPTURE(pixel_format_name(format), full_range);
            ColorInfo color;
            color.full_range = full_range;
            const auto input = patterned_image(map.input_size, format, color);
            const auto source = map_host_image(input);
            cpu->upload(input);
            REQUIRE(cpu->warp_gyro(map) == (127.0 + 64) / (127 * 90));
            const auto out = map_host_image(cpu->download());
            REQUIRE(out.size.width == 127);
            REQUIRE(out.size.height == 90);
            REQUIRE(out.format == format);
            const auto& luma = out.planes.front();
            // At row 88 only the chroma site is invalid. Its luma must still be sampled.
            cpu->upload(luma_view(input));
            REQUIRE(cpu->warp_gyro(map) == 1.0 / 90);
            const auto gray = map_host_image(cpu->download());
            REQUIRE(std::ranges::equal(luma.bytes.subspan(88 * luma.row_stride, 127),
                                       gray.planes[0].bytes.subspan(88 * gray.planes[0].row_stride, 127)));
            const auto last = luma.bytes.subspan(89 * luma.row_stride, 127);
            REQUIRE(std::ranges::all_of(last, [&](auto code) { return code == (full_range ? 0 : 16); }));
            REQUIRE(out.planes.size() == source.planes.size());
            for (size_t i = 1; i < out.planes.size(); ++i) {
                const auto& chroma = out.planes[i];
                const auto& src = source.planes[i];
                const size_t row_bytes = format == PixelFormat::i420 ? 64 : 128;
                for (size_t y = 0; y < 45; ++y)
                    for (size_t x = 0; x < row_bytes; ++x) {
                        // The 30.75 luma-row shift is 15 + 3/8 chroma rows.
                        const auto a = src.bytes[(y + 15) * src.row_stride + x];
                        const auto b = src.bytes[std::min(y + 16, size_t{59}) * src.row_stride + x];
                        const int expected = y == 44 ? 128 : (5 * a + 3 * b + 4) / 8;
                        REQUIRE(chroma.bytes[y * chroma.row_stride + x] == expected);
                    }
            }
        }
}

TEST_CASE("CPU gyro uses the supplied executor and honors cancellation", "[backend][gyro]") {
    struct Token final : CancellationToken {
        bool stop = false;
        bool cancelled() const noexcept override { return stop; }
    } token;
    struct RowExecutor final : Executor {
        bool called = false;
        void for_each(size_t count, const std::function<void(size_t)>& work) override {
            called = true;
            for (size_t row = 0; row < count; ++row)
                work(row);
        }
    } executor;
    auto cpu = make_cpu_backend({&token, &executor});
    const auto map = pinhole_map({7, 5});
    cpu->upload(patterned_image(map.input_size, PixelFormat::rgba8, {}));
    REQUIRE(cpu->warp_gyro(map) == 0);
    REQUIRE(executor.called);
    token.stop = true;
    REQUIRE_THROWS_AS(cpu->warp_gyro(map), Cancelled);
}

#ifdef STILLFLOW_TEST_CUDA
std::unique_ptr<RenderBackend> cuda_or_skip() {
    try {
        return make_cuda_backend();
    } catch (const BackendError& error) {
        SKIP(error.what());
    }
}

#endif

GyroFrameMap rolling_fisheye_map() {
    // Gyroflow HERO6 Black Linear NO-EIS 2.7k 4:3 50 fps calibration.
    const LensProfile hero6{LensModel::opencv_fisheye,
                            {2704, 2028},
                            1504.455532,
                            1501.155173,
                            1355.710645,
                            1029.277298,
                            {0.3504237415, 0.0451971607, 0.3189255905, -0.2164699297},
                            0.8};
    auto map = pinhole_map({160, 120});
    map.lens = scale_lens_profile(hero6, map.input_size);
    map.output_size = {160, 90};
    map.out_cy = 44.5;
    map.row_rotations.clear();
    for (int i = 0; i < 64; ++i) {
        const double t = i / 63.0;
        map.row_rotations.push_back(to_matrix3(exp_so3(0.06 - 0.11 * t, -0.08 + 0.13 * t, 0.04 + 0.05 * t)));
    }
    return map;
}

double gyro_signal(double x, double y, int c) {
    return 128 + 60 * std::sin(0.21 * x + c) + 47 * std::cos(0.17 * y - c);
}

#ifdef STILLFLOW_TEST_CUDA
TEST_CASE("CUDA pinhole identity preserves every BGR byte", "[backend][cuda][gyro]") {
    auto cuda = cuda_or_skip();
    const auto map = pinhole_map({160, 120});
    const auto input = signal_image(map.input_size, gyro_signal);
    REQUIRE_THROWS_AS(cuda->warp_gyro(map), std::invalid_argument);
    cuda->upload(input);
    REQUIRE(cuda->warp_gyro(map) == 0);
    const auto actual = map_host_image(cuda->download());
    const auto expected = map_host_image(input);
    REQUIRE(std::ranges::equal(actual.planes[0].bytes, expected.planes[0].bytes));

    auto bad = map;
    bad.input_size.height--;
    REQUIRE_THROWS_AS(cuda->warp_gyro(bad), std::invalid_argument);
    bad = map;
    bad.output_size.width = 1;
    REQUIRE_THROWS_AS(cuda->warp_gyro(bad), std::invalid_argument);
    bad = map;
    bad.lens.k[2] = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(cuda->warp_gyro(bad), std::invalid_argument);
    bad = map;
    bad.out_fx = 0;
    REQUIRE_THROWS_AS(cuda->warp_gyro(bad), std::invalid_argument);
    bad = map;
    bad.lens.lens_correction_amount = 1.01;
    REQUIRE_THROWS_AS(cuda->warp_gyro(bad), std::invalid_argument);
}

#endif

void check_rolling_crop(RenderBackend& backend) {
    const auto map = rolling_fisheye_map();
    const auto render = [&](const auto& signal) {
        backend.upload(signal_image(map.input_size, signal));
        REQUIRE(backend.warp_gyro(map) == 0);
        return map_host_image(backend.download());
    };
    const auto output = render(gyro_signal);
    REQUIRE(output.size.width == 160);
    REQUIRE(output.size.height == 90);

    const std::array<uint8_t, 3> color{7, 128, 249};
    const auto solid = [&](int, int, int c) { return color[c]; };
    const auto flat = render(solid);
    const auto expected_flat = map_host_image(signal_image(map.output_size, solid));
    REQUIRE(std::ranges::equal(flat.planes[0].bytes, expected_flat.planes[0].bytes));

    // Evaluate the eight-tap reconstruction of phase-shifted ramps independently in double precision.
    const auto ramp = [](int coordinate, int c) {
        const int phase = (coordinate + c) % 6;
        return 85 * std::min(phase, 6 - phase);
    };
    const auto horizontal = render([&](int x, int, int c) { return ramp(x, c); });
    const auto vertical = render([&](int, int y, int c) { return ramp(y, c); });
    const auto axis_error = [&](double position, const uint8_t* codes) {
        const int base = static_cast<int>(position);
        const int channel = (4 - base % 3) % 3;
        const double t = position - base;
        const auto sinc = [](double x) {
            const double a = std::numbers::pi * x;
            return a == 0 ? 1.0 : std::sin(a) / a;
        };
        double sum = 0, weights = 0;
        for (int k = -3; k <= 4; ++k) {
            const double w = sinc(t - k) * sinc((t - k) / 4);
            sum += w * ramp(base + k, channel);
            weights += w;
        }
        const double expected = sum / weights;
        return (codes[channel] - expected) / 85;
    };
    double squared_error = 0, max_position_error = 0;
    for (int y = 0; y < output.size.height; ++y)
        for (int x = 0; x < output.size.width; ++x) {
            Point src;
            REQUIRE(project_output_to_input(map, {double(x), double(y)}, src));
            const size_t i = (static_cast<size_t>(y) * output.size.width + x) * 3;
            for (int c = 0; c < 3; ++c)
                squared_error += std::pow(output.planes[0].bytes[i + c] - gyro_signal(src.x, src.y, c), 2);
            max_position_error =
                std::max(max_position_error, std::hypot(axis_error(src.x, horizontal.planes[0].bytes.data() + i),
                                                        axis_error(src.y, vertical.planes[0].bytes.data() + i)));
        }
    REQUIRE(std::sqrt(squared_error / (output.size.width * output.size.height * 3)) < 0.41);
    REQUIRE(max_position_error < 0.01);
}

TEST_CASE("CPU rolling fisheye crop agrees with the double precision projection", "[backend][gyro]") {
    check_rolling_crop(*make_cpu_backend());
}
#ifdef STILLFLOW_TEST_CUDA
TEST_CASE("CUDA rolling fisheye crop agrees with the double precision projection", "[backend][cuda][gyro]") {
    check_rolling_crop(*cuda_or_skip());
}
#endif

void check_invalid_rays(RenderBackend& backend) {
    auto map = pinhole_map({80, 60});
    const auto input = signal_image(map.input_size, [](int, int, int) { return 219; });
    // Seed the pool with a bright frame so skipped writes cannot pass as black.
    backend.upload(input);
    REQUIRE(backend.warp_gyro(map) == 0);
    backend.download();
    for (double yaw : {0.32, 3.14}) {
        map.row_rotations = {to_matrix3(exp_so3(0.05, yaw, -0.04))};
        backend.upload(input);
        const double fraction = backend.warp_gyro(map);
        const auto view = map_host_image(backend.download());
        int invalid = 0;
        for (int y = 0; y < view.size.height; ++y)
            for (int x = 0; x < view.size.width; ++x) {
                Point src;
                const bool valid = project_output_to_input(map, {double(x), double(y)}, src);
                invalid += !valid;
                const auto pixel = view.planes[0].bytes.subspan((static_cast<size_t>(y) * view.size.width + x) * 3, 3);
                REQUIRE(std::ranges::all_of(pixel, [valid](auto code) { return code == (valid ? 219 : 0); }));
            }
        REQUIRE(fraction == double(invalid) / (view.size.width * view.size.height));
        REQUIRE(fraction > 0);
        if (yaw < 1)
            REQUIRE(fraction < 1);
        else
            REQUIRE(fraction == 1);
    }
}

TEST_CASE("CPU reports off-sensor and behind-camera rays and clears invalid pixels", "[backend][gyro]") {
    check_invalid_rays(*make_cpu_backend());
}
#ifdef STILLFLOW_TEST_CUDA
TEST_CASE("CUDA reports off-sensor and behind-camera rays and clears invalid pixels", "[backend][cuda][gyro]") {
    check_invalid_rays(*cuda_or_skip());
}
#endif

#ifdef STILLFLOW_TEST_CUDA
TEST_CASE("CUDA gyro resamples native NV12 with independently projected chroma", "[backend][cuda][gyro]") {
    auto cuda = cuda_or_skip();
    auto map = pinhole_map({160, 120});
    const auto input = signal_image(map.input_size, [](int, int, int c) { return c == 2 ? 210 : 40; });
    cuda->upload(input);
    REQUIRE(cuda->warp_gyro(map) == 0);
    const auto native = cuda->download();
    cuda->upload(native);
    map.output_size = {127, 90};
    map.out_cy -= 30.75;
    const double fraction = cuda->warp_gyro(map);
    REQUIRE(fraction == Catch::Approx((127.0 + 64) / (127 * 90)));
    const auto output = map_host_image(cuda->download());
    REQUIRE(output.size.width == 127);
    REQUIRE(output.size.height == 90);
    const auto& plane = output.planes.front();
    REQUIRE(plane.bytes[2] == Catch::Approx(210).margin(2));
    // At row 88 luma is valid; the last chroma site (88.5) and luma row 89 are outside.
    const auto last = plane.bytes.subspan(89 * plane.row_stride, size_t{127} * 3);
    REQUIRE(std::ranges::all_of(last, [](auto code) { return code == 0; }));
}

TEST_CASE("Gyro renderer selects CUDA, falls back on unsupported gyro and honors explicit CPU",
          "[backend][cuda][gyro]") {
    cuda_or_skip();
    const BackendProvider gpu{"cuda", make_cuda_backend, true};
    const auto map = rolling_fisheye_map();
    const auto input = signal_image(map.input_size, gyro_signal);
    double invalid = -1;
    Profile profile;
    const auto factory = make_renderer_factory({gpu, cpu_provider()});
    auto renderer = factory->create("auto");
    const auto output = renderer->render_gyro(input, map, invalid, profile);
    REQUIRE(output->size().width == 160);
    REQUIRE(output->size().height == 90);
    REQUIRE(invalid == 0);
    REQUIRE(renderer->report().backend == "cuda");
    REQUIRE(renderer->report().backends.front().frames == 1);
    for (const auto* stage : {"render.cuda", "render.cuda.upload", "render.cuda.warp", "render.cuda.download"})
        REQUIRE(profile.stages().at(stage).count == 1);
    auto cpu = factory->create("cpu");
    REQUIRE(cpu->render_gyro(input, map, invalid, profile));

    const auto gray =
        make_host_image(map.input_size, PixelFormat::gray8,
                        std::vector<uint8_t>(static_cast<size_t>(map.input_size.width) * map.input_size.height));
    REQUIRE(cpu->render_gyro(gray, map, invalid, profile));
    REQUIRE(cpu->report().backends.front().frames == 2);
    REQUIRE(renderer->render_gyro(gray, map, invalid, profile));
    REQUIRE(invalid == 0);
    const auto report = renderer->report();
    REQUIRE(report.backend == "mixed");
    REQUIRE(report.backends.back().info.id == "cpu");
    REQUIRE(report.backends.back().frames == 1);
    REQUIRE(report.failures.size() == 1);
    REQUIRE(report.failures.front().backend == "cuda");
}

TEST_CASE("CPU and CUDA rolling fisheye pixels agree within one code value", "[backend][cuda][gyro]") {
    auto cuda = cuda_or_skip();
    auto cpu = make_cpu_backend();
    auto map = rolling_fisheye_map();
    const auto input = signal_image(map.input_size, gyro_signal);
    cpu->upload(input);
    cuda->upload(input);
    for (bool sensor_rows : {false, true}) {
        map.sensor_rows = sensor_rows;
        REQUIRE(cpu->warp_gyro(map) == cuda->warp_gyro(map));
        const auto host = map_host_image(cpu->download()), device = map_host_image(cuda->download());
        int max_difference = 0;
        for (int y = 0; y < map.output_size.height; ++y)
            for (int i = 0; i < 3 * map.output_size.width; ++i)
                max_difference =
                    std::max(max_difference, std::abs(host.planes[0].bytes[y * host.planes[0].row_stride + i] -
                                                      device.planes[0].bytes[y * device.planes[0].row_stride + i]));
        REQUIRE(max_difference <= 1);
    }
}
#endif
} // namespace

TEST_CASE("Gyro adapters resolve projects and camera-frame IMU samples", "[gyro][adapter]") {
    using namespace stillflow;
    using Json = nlohmann::json;
    test::TempDirectory temp;
    const auto csv = temp.path / "camera.imu_camera.csv";
    std::ofstream(csv, std::ios::binary)
        << "t_video_s,wx,wy,wz,ax,ay,az\r\n0,1,2,3,4,5,6\r\n0.12,7,8,9,10,11,12\r\n\r\n";
    Json intrinsics = {{"camera_matrix", {{20, 0, 15}, {0, 21, 7}, {0, 0, 1}}},
                       {"distortion_coeffs", {0.2, 0.1, 0, 0}}};
    Json lens = intrinsics;
    lens["calib_dimension"] = {{"w", 32}, {"h", 16}};
    lens["model"] = "opencv_fisheye (k1..k4)";
    const Json size = {{"w", 24}, {"h", 12}};
    Json defaults = {{"output_dimension", size},
                     {"max_zoom", 129},
                     {"adaptive_zoom_window_s", 0},
                     {"smoothing_params", {{"smoothness", 0.3}, {"smoothness_yaw", 0.7}, {"per_axis", 1}}}};
    const auto config = temp.path / "stabilization.json";
    std::ofstream(config) << Json{{"lens", lens},
                                  {"imu", {{"file", csv.filename().string()}}},
                                  {"rolling_shutter", {{"readout_ms", 8}, {"direction", "BottomToTop"}}},
                                  {"gyroflow_reference", defaults}};
    const auto project = temp.path / "camera.gyroflow";
    std::ofstream(project) << Json{{"calibration_data",
                                    {{"calib_dimension", lens["calib_dimension"]},
                                     {"output_dimension", size},
                                     {"frame_readout_time", 8},
                                     {"fisheye_params", intrinsics}}},
                                   {"stabilization",
                                    {{"frame_readout_direction", "BottomToTop"},
                                     {"max_zoom", 129},
                                     {"adaptive_zoom_window", 0},
                                     {"smoothing_params",
                                      {{{"name", "smoothness"}, {"value", 0.3}},
                                       {{"name", "smoothness_yaw"}, {"value", 0.7}},
                                       {{"name", "per_axis"}, {"value", 1}}}}}}};
    for (const auto& path : {config, project}) {
        auto input = load_gyro_input(path);
        REQUIRE(input.config_path == path);
        REQUIRE(input.imu_path == csv);
        REQUIRE(input.lens.model == LensModel::opencv_fisheye);
        REQUIRE(input.lens.fy == 21);
        REQUIRE(input.lens.k[1] == 0.1);
        REQUIRE(input.settings.smoothness == 0.3);
        REQUIRE(input.settings.per_axis);
        REQUIRE(input.settings.smoothness_yaw == 0.7);
        REQUIRE(input.settings.readout_ms == 8);
        REQUIRE(!input.settings.readout_top_to_bottom);
        REQUIRE(input.settings.max_zoom == Catch::Approx(1.29));
        REQUIRE(!input.settings.adaptive_zoom);
        REQUIRE(input.settings.output_size.width == 24);
        REQUIRE(input.imu.size() == 2);
        REQUIRE(input.imu.back().has_accel);
        REQUIRE(input.imu.back().az == 12);
        REQUIRE(input.imu.back().time_s == 0.12);
        const auto track = integrate_imu(input.imu, input.settings);
        const auto maps = plan_gyro_frames(std::array{0.0, 0.06, 0.12}, track, input.lens, {32, 16}, input.settings);
        REQUIRE(maps.size() == 3);
        REQUIRE(maps.back().raw_orientation.w == Catch::Approx(track.samples.back().q.w));
        REQUIRE(maps.back().raw_orientation.w < 1);
        for (const auto& map : maps) {
            Point src;
            REQUIRE(project_output_to_input(map, {map.out_cx, map.out_cy}, src));
        }
        const auto override = temp.path / "rates.csv";
        std::ofstream(override) << "t_video_s,wx,wy,wz\n0,4,5,6\n";
        auto rates = load_gyro_input(path, override);
        REQUIRE(rates.imu.size() == 1);
        REQUIRE(!rates.imu.front().has_accel);
        REQUIRE(rates.imu.front().wy == 5);
    }
    for (const auto* row : {"0,1,nan,3", "0,1,2", "0,1,2,3x", "-1,1,2,3"}) {
        std::ofstream(csv) << "t_video_s,wx,wy,wz\n0,1,2,3\n" << row << '\n';
        REQUIRE_THROWS_AS(load_gyro_input(project), std::runtime_error);
    }
    REQUIRE_THROWS_AS(load_gyro_input(project, temp.path / "missing.csv"), std::runtime_error);
}

TEST_CASE("Packet-timed IMU uses continuous camera ticks and project sync when they agree", "[adapter][gyro]") {
    using namespace stillflow;
    test::TempDirectory temp;
    const auto csv = temp.path / "imu.csv";
    std::array times{0.0275, 0.033, 0.0385, 0.044, 0.049, 0.054, 0.059, 0.0645, 0.070};
    bool microsecond_clock = false;
    bool packet_timed = false;
    bool truncate = false;
    SECTION("Packet-interpolated CSV") { packet_timed = true; }
    SECTION("Truncated GPMF") { truncate = true; }
    SECTION("Microsecond stream clock") { microsecond_clock = true; }
    SECTION("Irregular CSV in first packet") { times[1] += 0.001; }
    SECTION("Irregular CSV in final packet") { times[7] -= 0.001; }

    std::ofstream samples(csv);
    samples << "t_video_s,wx,wy,wz\n";
    for (double t : times)
        samples << t << ",1,2,3\n";
    samples.close();
    const auto config = temp.path / "stabilization.json";
    std::ofstream(config) << R"({
        "imu":{"file":"imu.csv","time_offset_s":-0.0275},
        "lens":{"model":"none","calib_dimension":{"w":32,"h":16},
                "camera_matrix":[[16,0,16],[0,16,8],[0,0,1]],"distortion_coeffs":[0,0,0,0]},
        "other_files":{"raw_gpmf":"gyro.bin"},
        "gyroflow_reference":{"integration_method":2,"sync_offsets_ms":{"0s":50,"1s":51}}
    })";
    const auto record = [](const std::string& key, char type, char stride, const std::string& data) {
        const auto count = data.size() / stride;
        return key + type + stride + static_cast<char>(count >> 8) + static_cast<char>(count) + data +
               std::string((4 - data.size() % 4) % 4, '\0');
    };
    const auto scalar = [&](const std::string& key, int value) {
        return record(key, 'L', 4, {0, 0, static_cast<char>(value >> 8), static_cast<char>(value)});
    };
    const auto bin = temp.path / "gyro.bin";
    std::ofstream packets(bin, std::ios::binary);
    for (int i = 0; i < 3; ++i) {
        auto stream =
            scalar("TICK", 1000 + 15 * i) + scalar("TSMP", 3 * (i + 1)) + record("GYRO", 's', 6, std::string(18, '\0'));
        if (i == 2 && microsecond_clock)
            stream += record("STMP", 'J', 8, std::string(8, '\0'));
        packets << record("DEVC", 0, 1, record("STRM", 0, 1, stream));
    }
    packets.close();
    if (truncate)
        std::filesystem::resize_file(bin, std::filesystem::file_size(bin) - 1);

    const auto explicit_csv = load_gyro_input(config, csv);
    REQUIRE(!explicit_csv.camera_clock);
    const auto loaded = load_gyro_input(config);
    REQUIRE(loaded.imu.size() == times.size());
    REQUIRE(explicit_csv.imu.size() == times.size());
    REQUIRE(loaded.camera_clock == packet_timed);
    for (size_t i = 0; i < times.size(); ++i) {
        REQUIRE(explicit_csv.imu[i].time_s == Catch::Approx(times[i]));
        REQUIRE(loaded.imu[i].time_s == Catch::Approx(packet_timed ? 0.0475 + i * 0.005005 : times[i]));
    }
}
