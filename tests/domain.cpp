#include "fixture.hpp"
#include "stillflow/domain/gyro.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <limits>
#include <numbers>

TEST_CASE("Trajectory and crop plans match the validated golden sample", "[domain][golden]") {
    std::ifstream input(STILLFLOW_GOLDEN_FILE);
    REQUIRE(input.good());
    const auto actual = golden_values();
    size_t index = 0;
    for (double expected; input >> expected; ++index) {
        REQUIRE(index < actual.size());
        CAPTURE(index);
        REQUIRE(actual[index] == Catch::Approx(expected).margin(1e-8).epsilon(1e-8));
    }
    REQUIRE(index == actual.size());
}
TEST_CASE("Identity and translation invert with the documented crop", "[domain]") {
    stillflow::Warp warp;
    warp.columns = warp.rows = 2;
    warp.displacement.assign(4, {3, -2});
    auto map = stillflow::prepare_map(warp, {100, 100}, {100, 100}, 0.1, false);
    auto source = stillflow::source_point(map, {50, 50});
    REQUIRE(source.x == Catch::Approx(47));
    REQUIRE(source.y == Catch::Approx(52));
    REQUIRE(map.origin_x == Catch::Approx(9.9));
    REQUIRE(map.step_x == Catch::Approx(0.8));
    warp.displacement.clear();
    REQUIRE_THROWS_AS(stillflow::prepare_map(warp, {100, 100}, {100, 100}, 0, false), std::invalid_argument);
}

namespace {
using namespace stillflow;

void same_rotation(Quaternion actual, Quaternion expected, double tolerance = 1e-10) {
    const auto d = log_so3(multiply(conjugate(expected), actual));
    REQUIRE(std::hypot(d[0], d[1], d[2]) < tolerance);
}

LensProfile centered_lens(Size size) {
    LensProfile lens;
    lens.fx = lens.fy = 0.8 * size.width;
    lens.cx = (size.width - 1) * 0.5;
    lens.cy = (size.height - 1) * 0.5;
    return lens;
}

void require_containment(const GyroFrameMap& map) {
    for (int i = 0; i <= 128; ++i) {
        const double u = i / 128.0, w = map.output_size.width - 1, h = map.output_size.height - 1;
        for (Point p : {Point{u * w, 0}, Point{u * w, h}, Point{0, u * h}, Point{w, u * h}}) {
            Point src;
            REQUIRE(project_output_to_input(map, p, src));
            REQUIRE(src.x >= 0.5 - 1e-7);
            REQUIRE(src.y >= 0.5 - 1e-7);
            REQUIRE(src.x <= map.input_size.width - 1.5 + 1e-7);
            REQUIRE(src.y <= map.input_size.height - 1.5 + 1e-7);
        }
    }
}
} // namespace

TEST_CASE("SO(3) maps use shortest arcs and camera-to-world composition", "[domain][gyro]") {
    using namespace stillflow;
    const auto x = exp_so3(std::numbers::pi / 2, 0, 0), y = exp_so3(0, std::numbers::pi / 2, 0);
    const auto composed = to_matrix3(multiply(x, y));
    REQUIRE(composed(0, 2) == Catch::Approx(1));
    REQUIRE(composed(2, 0) == Catch::Approx(0).margin(1e-14));
    const auto m = to_matrix3(multiply(y, x));
    REQUIRE(m(1, 2) == Catch::Approx(-1));
    REQUIRE(log_so3(exp_so3(0, 0, 1.5 * std::numbers::pi))[2] == Catch::Approx(-std::numbers::pi / 2));
    const auto small = log_so3(exp_so3(1e-12, -3e-12, 2e-12));
    REQUIRE(small[1] == Catch::Approx(-3e-12).margin(1e-24));
    same_rotation(slerp(x, {-x.w, -x.x, 0, 0}, 0.3), x);
    same_rotation(slerp({}, exp_so3(0, 0, 1.5 * std::numbers::pi), 0.5), exp_so3(0, 0, -std::numbers::pi / 4));
    REQUIRE(normalize({0, 0, 3, 4}).y == Catch::Approx(0.6));
    REQUIRE_THROWS_AS(normalize({0, 0, 0, 0}), std::invalid_argument);
}

TEST_CASE("Gyro integration respects body rates, irregular intervals and video time offset", "[domain][gyro]") {
    using namespace stillflow;
    std::vector<ImuSample> imu{{.time_s = 1, .wx = 2}, {.time_s = 1.25}, {.time_s = 1.25}, {.time_s = 2, .wy = 4}};
    GyroSettings s;
    s.time_offset_s = 0.001;
    const auto track = integrate_imu(imu, s);
    REQUIRE(track.samples.size() == 3);
    REQUIRE(track.samples[1].time_s == Catch::Approx(1.251));
    const auto pitch = exp_so3(0.25, 0, 0);
    same_rotation(track.sample(-1), {});
    same_rotation(track.sample(1.126), exp_so3(0.125, 0, 0));
    same_rotation(track.sample(1.626), multiply(pitch, exp_so3(0, 0.75, 0)));
    same_rotation(track.sample(4), multiply(pitch, exp_so3(0, 1.5, 0)));
    same_rotation(integrate_imu({}, s).sample(2), {});
    imu.back().time_s = 0.5;
    REQUIRE_THROWS_AS(integrate_imu(imu, s), std::invalid_argument);
    imu.back().time_s = std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(integrate_imu(imu, s), std::invalid_argument);
    imu.back().time_s = 2;
    imu.front().wy = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(integrate_imu(imu, s), std::invalid_argument);
}

TEST_CASE("Complementary integration estimates upright tilt, rejects acceleration and preserves yaw",
          "[domain][gyro]") {
    using namespace stillflow;
    GyroSettings s;
    s.integration_method = ImuIntegrationMethod::complementary;
    const double roll = 0.4;
    // Body-frame proper acceleration for a camera rolled about the optical axis.
    std::vector<ImuSample> imu{{.ax = -9.81 * std::sin(roll), .ay = -9.81 * std::cos(roll), .has_accel = true}};
    same_rotation(integrate_imu(imu, s).sample(0), exp_so3(0, 0, roll));
    imu.clear();
    for (int i = 0; i <= 400; ++i)
        imu.push_back({.time_s = i * 0.01, .wy = 0.5, .ay = -9.81, .has_accel = true});
    same_rotation(integrate_imu(imu, s).sample(4), exp_so3(0, 2, 0));
    for (auto& p : imu) {
        p.wy = 0;
        p.wx = 0.1;
        p.ay = -30;
    }
    same_rotation(integrate_imu(imu, s).sample(4), exp_so3(0.4, 0, 0));
    for (auto& p : imu)
        p.ay = -9.81;
    const double corrected = log_so3(integrate_imu(imu, s).sample(4))[0];
    REQUIRE(corrected > 0);
    REQUIRE(corrected < 0.25);
    for (auto& p : imu)
        p.has_accel = false;
    same_rotation(integrate_imu(imu, s).sample(4), exp_so3(0.4, 0, 0));
}

TEST_CASE("Blended Kannala-Brandt optics invert the forward radius", "[domain][gyro]") {
    using namespace stillflow;
    auto lens = centered_lens({641, 481});
    lens.model = LensModel::opencv_fisheye;
    lens.k = {0.04, -0.015, 0.004, 0.001};
    lens.calib_size = {641, 481};
    const auto scaled = scale_lens_profile(lens, {1282, 1443});
    REQUIRE(scaled.fx == Catch::Approx(2 * lens.fx));
    REQUIRE(scaled.fy == Catch::Approx(3 * lens.fy));
    REQUIRE(scaled.cx == Catch::Approx(2 * lens.cx));
    REQUIRE(scaled.cy == Catch::Approx(3 * lens.cy));
    REQUIRE(distort_point(lens, {}).x == 0);
    for (double amount : {0.0, 0.25, 0.65, 1.0}) {
        lens.lens_correction_amount = amount;
        for (double r : {1e-12, 0.02, 1.0, 3.0, 15.0}) {
            const Point p = {0.6 * r, -0.8 * r};
            const double t = std::atan(r);
            const double kb =
                t + 0.04 * std::pow(t, 3) - 0.015 * std::pow(t, 5) + 0.004 * std::pow(t, 7) + 0.001 * std::pow(t, 9);
            const auto d = distort_point(lens, p);
            REQUIRE(d.x == Catch::Approx(0.6 * std::lerp(r, kb, amount)).margin(1e-14));
            const auto back = undistort_point(lens, d);
            REQUIRE(back.x == Catch::Approx(p.x).margin(1e-12));
            REQUIRE(std::abs(std::atan(std::hypot(back.x, back.y)) - t) < 1e-10);
            REQUIRE(back.y == Catch::Approx(p.y).margin(1e-12));
        }
    }
    lens.k = {-0.4, 0, 0, 0};
    const Point inside{std::tan(0.5), 0};
    REQUIRE(undistort_point(lens, distort_point(lens, inside)).x == Catch::Approx(inside.x));
    lens.k = {};
    REQUIRE_THROWS_AS(undistort_point(lens, {2, 0}), std::domain_error);
    lens.fx = 0;
    REQUIRE_THROWS_AS(scale_lens_profile(lens, {640, 480}), std::invalid_argument);
}

TEST_CASE("Bidirectional smoothing attenuates jitter without shifting its peak", "[domain][gyro]") {
    using namespace stillflow;
    OrientationTrack raw;
    for (int i = 0; i <= 400; ++i)
        raw.samples.push_back({i * 0.01, exp_so3(0, i == 200 ? 0.2 : 0, 0)});
    GyroSettings s;
    s.smoothness = 0.2;
    const auto smooth = smooth_orientations(raw, s);
    const double peak = log_so3(smooth.sample(2))[1];
    REQUIRE(peak > 0);
    REQUIRE(peak < 0.05);
    for (double d : {0.01, 0.05, 0.1}) {
        const double before = log_so3(smooth.sample(2 - d))[1], after = log_so3(smooth.sample(2 + d))[1];
        REQUIRE(before > 0);
        REQUIRE(before < peak);
        REQUIRE(after == Catch::Approx(before).margin(5e-4));
    }
    s.alpha_0_1s = 0.001;
    REQUIRE(log_so3(smooth_orientations(raw, s).sample(2))[1] > peak);
    s.per_axis = true;
    s.smoothness_yaw = 0;
    same_rotation(smooth_orientations(raw, s).sample(2), raw.sample(2));
    s.smoothness_yaw = 1;
    REQUIRE(log_so3(smooth_orientations(raw, s).sample(2))[1] < 0.05);
    std::vector<double> local(raw.samples.size(), 1);
    local[200] = 0;
    same_rotation(smooth_orientations(raw, s, local).sample(2), raw.sample(2));
    s.smoothness = 0;
    same_rotation(smooth_orientations(raw, s).sample(2), raw.sample(2));
    local.pop_back();
    REQUIRE_THROWS_AS(smooth_orientations(raw, s, local), std::invalid_argument);
}

TEST_CASE("Per-axis smoothing uses each camera axis and the actual interval in both passes", "[domain][gyro]") {
    using namespace stillflow;
    GyroSettings s;
    s.per_axis = true;
    s.smoothness_pitch = 0.2;
    s.smoothness_yaw = 0.5;
    s.smoothness_roll = 0.8;
    s.alpha_0_1s = 1;
    const std::array constants{s.smoothness_pitch, s.smoothness_yaw, s.smoothness_roll};
    for (size_t axis = 0; axis < 3; ++axis) {
        std::array<double, 3> pulse{};
        pulse[axis] = 0.25;
        const OrientationTrack raw{{{0, {}}, {0.1, exp_so3(pulse[0], pulse[1], pulse[2])}, {0.4, {}}}};
        const auto smooth = smooth_orientations(raw, s);
        const double decay1 = std::exp(-0.1 / constants[axis]), decay2 = std::exp(-0.3 / constants[axis]);
        const double forward1 = 0.25 * (1 - decay1), last = forward1 * decay2;
        const double middle = last * decay2 + forward1 * (1 - decay2);
        REQUIRE(log_so3(smooth.sample(0))[axis] == Catch::Approx(middle * decay1));
        REQUIRE(log_so3(smooth.sample(0.1))[axis] == Catch::Approx(middle));
        REQUIRE(log_so3(smooth.sample(0.4))[axis] == Catch::Approx(last));
    }
}

TEST_CASE("Fisheye planning preserves sensor pixel aspect and inverts the output ray", "[domain][gyro]") {
    using namespace stillflow;
    const Size input{2704, 2028};
    auto lens = centered_lens(input);
    lens.model = LensModel::opencv_fisheye;
    lens.fx = 1186.54;
    lens.fy = 1186.47;
    lens.cx += 10;
    GyroSettings s;
    s.output_size = {2704, 1520};
    const auto maps = plan_gyro_frames(std::array{1.0}, {}, lens, input, s);
    const auto& m = maps.at(0);
    REQUIRE(m.row_rotations.size() == 1);
    REQUIRE(m.fov_scale == 1);
    REQUIRE(m.out_fx == lens.fx);
    REQUIRE(m.out_fy == lens.fy);
    REQUIRE(m.out_cy == 759.5);
    Point src;
    REQUIRE(project_output_to_input(m, {m.out_cx + 1000, m.out_cy}, src));
    REQUIRE(src.x == Catch::Approx(lens.cx + lens.fx * std::atan(1000 / lens.fx)));
    REQUIRE(src.y == Catch::Approx(lens.cy));
    require_containment(m);
    auto behind = m;
    behind.row_rotations.front() = to_matrix3(exp_so3(0, std::numbers::pi, 0));
    REQUIRE_FALSE(project_output_to_input(behind, {m.out_cx, m.out_cy}, src));
    REQUIRE_FALSE(project_output_to_input(m, {std::numeric_limits<double>::quiet_NaN(), 0}, src));
}

TEST_CASE("Rolling shutter samples the cropped sensor rows in either readout direction", "[domain][gyro]") {
    using namespace stillflow;
    const Size input{401, 301};
    const auto lens = centered_lens(input);
    const OrientationTrack raw{{{0, {}}, {1, exp_so3(0, 2, 0)}}};
    GyroSettings s;
    s.smoothness = 0;
    s.output_size = {401, 151};
    s.readout_ms = 20;
    s.fov = 0.9;
    for (bool down : {false, true}) {
        s.readout_top_to_bottom = down;
        const auto maps = plan_gyro_frames(std::array{0.5}, raw, lens, input, s);
        const auto& map = maps.at(0);
        REQUIRE(map.row_rotations.size() == 64);
        same_rotation(map.raw_orientation, exp_so3(0, 1, 0));
        for (int y : {0, 75, 150}) {
            Point src;
            REQUIRE(project_output_to_input(map, {map.out_cx, double(y)}, src));
            const double row_offset = (src.y / (input.height - 1) - 0.5) * 0.02 * (down ? 1 : -1);
            REQUIRE(src.x == Catch::Approx(lens.cx + lens.fx * std::tan(-2 * row_offset)).margin(1e-7));
        }
        require_containment(map);
    }
}

TEST_CASE("Rolling shutter containment uses row times at the final zoom", "[domain][gyro]") {
    using namespace stillflow;
    const Size input{401, 301};
    auto lens = centered_lens(input);
    const OrientationTrack raw{{{0, {}}, {1, exp_so3(0, 2, 0)}}};
    GyroSettings s;
    s.smoothness = 0;
    s.readout_ms = 100;
    const auto maps = plan_gyro_frames(std::array{0.5}, raw, lens, input, s);
    const auto& map = maps.at(0);
    REQUIRE(map.zoom_ratio > 1.1);
    REQUIRE(map.zoom_ratio < s.max_zoom);
    require_containment(map);
    Point src;
    REQUIRE(project_output_to_input(map, {map.out_cx, 0}, src));
    const double dt = (src.y / (input.height - 1) - 0.5) * 0.1;
    REQUIRE(src.x == Catch::Approx(lens.cx + lens.fx * std::tan(-2 * dt)).margin(1e-5));
}

TEST_CASE("Zoom relaxation retains containment and dynamic zoom anticipates a camera impulse", "[domain][gyro]") {
    using namespace stillflow;
    const Size size{321, 181};
    auto lens = centered_lens(size);
    OrientationTrack raw;
    std::vector<double> times;
    for (int i = 0; i <= 40; ++i) {
        times.push_back(i * 0.1);
        raw.samples.push_back({times.back(), exp_so3(0, 0, i == 20 ? 0.65 : 0)});
    }
    GyroSettings s;
    s.adaptive_zoom_window_s = 0.6;
    s.max_zoom = 1.1;
    for (int iterations : {0, 2}) {
        s.max_zoom_iterations = iterations;
        const auto maps = plan_gyro_frames(times, raw, lens, size, s);
        REQUIRE(maps.size() == times.size());
        REQUIRE(maps[20].smooth_factor < 1);
        REQUIRE(maps[20].zoom_ratio == Catch::Approx(1.1).margin(0.001));
        REQUIRE(maps[18].zoom_ratio > maps.front().zoom_ratio + 0.02);
        REQUIRE(maps.back().zoom_ratio < maps[20].zoom_ratio - 0.05);
        for (const auto& map : maps) {
            REQUIRE(map.zoom_ratio <= s.max_zoom + 1e-12);
            REQUIRE(map.zoom_ratio == Catch::Approx(1 / map.fov_scale));
            require_containment(map);
        }
    }
    s.adaptive_zoom = false;
    const auto fixed = plan_gyro_frames(times, raw, lens, size, s);
    for (const auto& map : fixed)
        REQUIRE(map.fov_scale == fixed.front().fov_scale);
    s.adaptive_zoom = true;
    s.adaptive_zoom_window_s = 0;
    REQUIRE(plan_gyro_frames(times, raw, lens, size, s).front().fov_scale == fixed.front().fov_scale);
    s.max_zoom = 1; // Even a centered pinhole identity requires the half-pixel margin.
    REQUIRE_THROWS_AS(plan_gyro_frames(times, raw, lens, size, s), std::domain_error);
}

TEST_CASE("Gyro planner validates settings, sizes and timelines", "[domain][gyro]") {
    using namespace stillflow;
    const Size size{640, 480};
    auto lens = centered_lens(size);
    REQUIRE(plan_gyro_frames({}, {}, lens, size, {}).empty());
    REQUIRE_THROWS_AS(plan_gyro_frames(std::array{1.0, 0.0}, {}, lens, size, {}), std::invalid_argument);
    REQUIRE_THROWS_AS(plan_gyro_frames({}, {}, lens, {}, {}), std::invalid_argument);
    lens.calib_size = {0, 1};
    REQUIRE_THROWS_AS(scale_lens_profile(lens, size), std::invalid_argument);
    GyroSettings s;
    s.output_size = {100, 0};
    REQUIRE_THROWS_AS(validate_gyro_settings(s), std::invalid_argument);
    s.output_size = {};
    s.max_zoom_iterations = 31;
    REQUIRE_THROWS_AS(validate_gyro_settings(s), std::invalid_argument);
    s.max_zoom_iterations = 0;
    s.smoothness = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(validate_gyro_settings(s), std::invalid_argument);
}
