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

TEST_CASE("CPU reports unsupported gyro rendering through both rendering interfaces", "[backend][gyro]") {
    auto map = pinhole_map({8, 6});
    auto input = signal_image(map.input_size, [](int x, int y, int c) { return x + y + c; });
    auto cpu = make_cpu_backend();
    cpu->upload(input);
    REQUIRE_THROWS_AS(cpu->warp_gyro(map), NotImplementedError);
    const auto factory = make_renderer_factory({cpu_provider()});
    Profile profile;
    double invalid = -1;
    for (const auto* request : {"cpu", "auto"}) {
        auto renderer = factory->create(request);
        REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), NotImplementedError);
        const auto report = renderer->report();
        REQUIRE(report.failures.size() == 1);
        REQUIRE(report.failures.front().backend == "cpu");
        REQUIRE(report.failures.front().stage == "render");
        REQUIRE(report.backends.front().frames == 0);
        REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), NotImplementedError);
        Warp visual;
        visual.columns = visual.rows = 2;
        visual.displacement.resize(4);
        REQUIRE(renderer->render(input, visual, input->size(), 0, false, profile));
    }
    auto renderer = factory->create("auto");
    REQUIRE_THROWS_AS(renderer->render_gyro({}, map, invalid, profile), std::invalid_argument);
    map.row_rotations.clear();
    REQUIRE_THROWS_AS(renderer->render_gyro(input, map, invalid, profile), std::invalid_argument);
    REQUIRE(renderer->report().failures.empty());
}

#ifdef STILLFLOW_TEST_CUDA
std::unique_ptr<RenderBackend> cuda_or_skip() {
    try {
        return make_cuda_backend();
    } catch (const BackendError& error) {
        SKIP(error.what());
    }
}

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

TEST_CASE("CUDA rolling fisheye crop agrees with the double precision projection", "[backend][cuda][gyro]") {
    auto cuda = cuda_or_skip();
    const auto map = rolling_fisheye_map();
    const auto render = [&](const auto& signal) {
        cuda->upload(signal_image(map.input_size, signal));
        REQUIRE(cuda->warp_gyro(map) == 0);
        return map_host_image(cuda->download());
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

TEST_CASE("CUDA reports off-sensor and behind-camera rays and clears invalid pixels", "[backend][cuda][gyro]") {
    auto cuda = cuda_or_skip();
    auto map = pinhole_map({80, 60});
    const auto input = signal_image(map.input_size, [](int, int, int) { return 219; });
    // Seed the pool with a bright frame so skipped writes cannot pass as black.
    cuda->upload(input);
    REQUIRE(cuda->warp_gyro(map) == 0);
    cuda->download();
    for (double yaw : {0.32, 3.14}) {
        map.row_rotations = {to_matrix3(exp_so3(0.05, yaw, -0.04))};
        cuda->upload(input);
        const double fraction = cuda->warp_gyro(map);
        const auto view = map_host_image(cuda->download());
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
    REQUIRE_THROWS_AS(factory->create("cpu")->render_gyro(input, map, invalid, profile), NotImplementedError);

    renderer = make_renderer_factory({cpu_provider(), gpu})->create("auto");
    REQUIRE(renderer->render_gyro(input, map, invalid, profile));
    REQUIRE(renderer->report().backend == "cuda");
    REQUIRE(renderer->report().failures.front().backend == "cpu");

    renderer = factory->create("auto");
    const auto gray =
        make_host_image(map.input_size, PixelFormat::gray8,
                        std::vector<uint8_t>(static_cast<size_t>(map.input_size.width) * map.input_size.height));
    try {
        renderer->render_gyro(gray, map, invalid, profile);
        FAIL("Both backends should fail on gray gyro input");
    } catch (const NotImplementedError&) {
        FAIL("CUDA format failure must propagate as a backend failure");
    } catch (const BackendError& error) {
        REQUIRE(std::string(error.what()).find("cuda:") != std::string::npos);
        REQUIRE(std::string(error.what()).find("cpu:") != std::string::npos);
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
