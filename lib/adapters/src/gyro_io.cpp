#include "gpmf_clock.hpp"
#include "stillflow/adapters/gyro.hpp"
#include <algorithm>
#include <charconv>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>

namespace stillflow {
namespace {
using Json = nlohmann::json;
using Path = std::filesystem::path;

std::vector<std::string_view> fields(std::string_view line) {
    if (line.ends_with('\r'))
        line.remove_suffix(1);
    std::vector<std::string_view> values;
    for (;;) {
        const auto comma = line.find(',');
        values.push_back(line.substr(0, comma));
        if (comma == line.npos)
            return values;
        line.remove_prefix(comma + 1);
    }
}

std::vector<ImuSample> load_imu_csv(const Path& path) {
    std::ifstream input(path);
    std::string line;
    if (!std::getline(input, line))
        throw std::runtime_error("Cannot read IMU CSV header: " + path.string());
    constexpr std::array names{"t_video_s", "wx", "wy", "wz", "ax", "ay", "az"};
    std::array<size_t, names.size()> columns{};
    const auto header = fields(line);
    const size_t width = header.size();
    size_t accel = 0;
    for (size_t i = 0; i < names.size(); ++i) {
        auto it = std::ranges::find(header, names[i]);
        if (it == header.end() && i < 4)
            throw std::runtime_error("Missing IMU CSV column " + std::string(names[i]) + ": " + path.string());
        columns[i] = it - header.begin();
        if (i >= 4 && it != header.end())
            ++accel;
    }
    if (accel != 0 && accel != 3)
        throw std::runtime_error("IMU CSV requires all three accelerometer columns: " + path.string());
    std::vector<ImuSample> samples;
    for (size_t row = 2; std::getline(input, line); ++row) {
        if (line.empty() || line == "\r")
            continue;
        const auto values = fields(line);
        const auto invalid = [&] {
            return std::runtime_error("Invalid IMU CSV at " + path.string() + ":" + std::to_string(row));
        };
        if (values.size() != width)
            throw invalid();
        std::array<double, names.size()> numbers{};
        for (size_t i = 0; i < 4 + accel; ++i) {
            const auto value = values[columns[i]];
            const auto* end = std::to_address(value.end());
            const auto result = std::from_chars(std::to_address(value.begin()), end, numbers[i]);
            if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(numbers[i]))
                throw invalid();
        }
        if (!samples.empty() && numbers[0] < samples.back().time_s)
            throw std::runtime_error("Decreasing IMU timestamp at " + path.string() + ":" + std::to_string(row));
        samples.push_back(
            {numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6], accel != 0});
    }
    if (input.bad() || samples.empty())
        throw std::runtime_error("Cannot read IMU samples: " + path.string());
    return samples;
}

Size dimensions(const Json& object) { return {object.at("w"), object.at("h")}; }

void read_lens(LensProfile& lens, const Json& calibration, const Json& intrinsics) {
    lens.calib_size = dimensions(calibration.at("calib_dimension"));
    const auto matrix = intrinsics.at("camera_matrix").get<std::array<std::array<double, 3>, 3>>();
    lens.fx = matrix[0][0];
    lens.fy = matrix[1][1];
    lens.cx = matrix[0][2];
    lens.cy = matrix[1][2];
    lens.k = intrinsics.at("distortion_coeffs").get<std::array<double, 4>>();
    if (!calibration.value("asymmetrical", true)) {
        lens.cx = lens.calib_size.width / 2.0;
        lens.cy = lens.calib_size.height / 2.0;
    }
}

// A stabilization package can carry the calibrated matrix without the
// Gyroflow profile's symmetry flag. Only inherit it from the same calibration.
void read_packaged_lens(GyroInput& gyro, const Json& config) {
    const auto& lens = config.at("lens");
    read_lens(gyro.lens, lens, lens);
    const auto source = config.value("source", Json::object());
    if (lens.contains("asymmetrical") || !source.contains("gyroflow_project") || !lens.contains("profile"))
        return;
    std::ifstream project(gyro.config_path.parent_path() / source.at("gyroflow_project").get<std::string>());
    if (!project)
        return;
    const auto data = Json::parse(project, nullptr, false);
    if (!data.contains("calibration_data"))
        return;
    const auto& calibration = data.at("calibration_data");
    if (!calibration.contains("fisheye_params") || !calibration.contains("calib_dimension") ||
        calibration.value("identifier", std::string{}) != lens.at("profile").get<std::string>() ||
        calibration.at("calib_dimension") != lens.at("calib_dimension"))
        return;
    const auto& intrinsics = calibration.at("fisheye_params");
    if (intrinsics.contains("camera_matrix") && intrinsics.contains("distortion_coeffs") &&
        intrinsics.at("camera_matrix") == lens.at("camera_matrix") &&
        intrinsics.at("distortion_coeffs") == lens.at("distortion_coeffs"))
        read_lens(gyro.lens, calibration, intrinsics);
}

void read_direction(GyroSettings& settings, const std::string& direction) {
    if (direction != "TopToBottom" && direction != "BottomToTop")
        throw std::runtime_error("Unsupported gyro readout direction: " + direction);
    settings.readout_top_to_bottom = direction == "TopToBottom";
}

void read_reference(GyroInput& gyro, const Json& ref, int integration) {
    auto& s = gyro.settings;
    auto params = ref.value("smoothing_params", Json::object());
    if (params.is_array()) {
        auto named = Json::object();
        for (const auto& entry : params)
            named[entry.at("name").get<std::string>()] = entry.at("value");
        params = std::move(named);
    }
    s.smoothness = params.value("smoothness", s.smoothness);
    s.smoothness_pitch = params.value("smoothness_pitch", s.smoothness_pitch);
    s.smoothness_yaw = params.value("smoothness_yaw", s.smoothness_yaw);
    s.smoothness_roll = params.value("smoothness_roll", s.smoothness_roll);
    if (params.contains("per_axis")) {
        const auto& per_axis = params.at("per_axis");
        s.per_axis = per_axis.is_boolean() ? per_axis.get<bool>() : per_axis.get<double>() != 0;
    }
    s.max_smoothness = params.value("max_smoothness", s.max_smoothness);
    s.alpha_0_1s = params.value("alpha_0_1s", s.alpha_0_1s);
    s.fov = ref.value("fov", s.fov);
    s.max_zoom = ref.value("max_zoom", s.max_zoom);
    if (s.max_zoom > 10)
        s.max_zoom /= 100;
    s.max_zoom_iterations = ref.value("max_zoom_iterations", s.max_zoom_iterations);
    s.adaptive_zoom_window_s =
        ref.value("adaptive_zoom_window_s", ref.value("adaptive_zoom_window", s.adaptive_zoom_window_s));
    s.adaptive_zoom = s.adaptive_zoom_window_s > 0;
    s.zoom_envelope = ref.value("adaptive_zoom_method", 1) == 1;
    if (ref.contains("adaptive_zoom_method"))
        gyro.reference_zoom_method = ref.at("adaptive_zoom_method").get<int>();
    gyro.lens.lens_correction_amount = ref.value("lens_correction_amount", gyro.lens.lens_correction_amount);
    if (ref.contains("output_dimension"))
        s.output_size = dimensions(ref.at("output_dimension"));
    if ((integration == 1 || integration == 2) && ref.value("horizon_lock_amount", 0.0) > 0)
        s.integration_method = ImuIntegrationMethod::complementary;
}
void align_packet_clock(GyroInput& gyro, const Json& config) {
    const auto files = config.value("other_files", Json::object());
    const auto ref = config.value("gyroflow_reference", Json::object());
    const auto& imu = config.at("imu");
    if (!files.contains("raw_gpmf") || !ref.contains("sync_offsets_ms") || ref.value("integration_method", 0) != 2 ||
        !imu.contains("time_offset_s") ||
        std::abs(gyro.imu.front().time_s + imu.at("time_offset_s").get<double>()) > 1e-6)
        return;
    std::map<double, double> sync;
    for (const auto& [name, offset] : ref.at("sync_offsets_ms").items()) {
        if (!name.ends_with('s') || !offset.is_number())
            return;
        double seconds = 0;
        const auto end = name.data() + name.size() - 1;
        const auto parsed = std::from_chars(name.data(), end, seconds);
        if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(seconds))
            return;
        sync.emplace(seconds, offset.get<double>() / 1000);
    }
    for (auto hi = sync.begin(); hi != sync.end(); ++hi) {
        if (hi != sync.begin() && hi->first + hi->second <= std::prev(hi)->first + std::prev(hi)->second)
            return;
    }
    if (sync.empty())
        return;
    const auto period =
        gpmf_sample_period(gyro.config_path.parent_path() / files.at("raw_gpmf").get<std::string>(), gyro.imu);
    if (!period)
        return;
    // VQF associates each integrated interval with its last rate sample.
    // Move instantaneous measurements half a sample earlier for trapezoidal integration.
    for (size_t i = 0; i < gyro.imu.size(); ++i) {
        const double t = i * *period;
        const auto hi = sync.lower_bound(t);
        double offset = hi == sync.end() ? sync.rbegin()->second : hi->second;
        if (hi != sync.begin() && hi != sync.end()) {
            const auto lo = std::prev(hi);
            offset = std::lerp(lo->second, hi->second, (t - lo->first) / (hi->first - lo->first));
        }
        gyro.imu[i].time_s = t + offset - *period * 0.5;
    }
    gyro.camera_clock = true;
}
} // namespace

GyroInput load_gyro_input(const Path& config_path, const Path& imu_override) {
    std::ifstream input(config_path);
    if (!input)
        throw std::runtime_error("Cannot open gyro configuration: " + config_path.string());
    GyroInput gyro;
    gyro.config_path = config_path;
    gyro.imu_path = imu_override;
    try {
        const auto config = Json::parse(input);
        auto& s = gyro.settings;
        if (config.contains("calibration_data") && config.contains("stabilization")) {
            const auto& calibration = config.at("calibration_data");
            const auto& ref = config.at("stabilization");
            gyro.lens.model = LensModel::opencv_fisheye;
            read_lens(gyro.lens, calibration, calibration.at("fisheye_params"));
            if (calibration.contains("output_dimension") && !calibration.at("output_dimension").is_null())
                s.output_size = dimensions(calibration.at("output_dimension"));
            s.readout_ms = ref.contains("frame_readout_time") ? ref.at("frame_readout_time").get<double>()
                                                              : calibration.value("frame_readout_time", 0.0);
            read_direction(s, ref.value("frame_readout_direction", std::string("TopToBottom")));
            read_reference(gyro, ref, config.value("gyro_source", Json::object()).value("integration_method", 0));
            if (gyro.imu_path.empty())
                gyro.imu_path = config_path.parent_path() / (config_path.stem().string() + ".imu_camera.csv");
        } else {
            const auto& lens = config.at("lens");
            const auto model = lens.at("model").get<std::string>();
            if (model.starts_with("opencv_fisheye"))
                gyro.lens.model = LensModel::opencv_fisheye;
            else if (model != "none")
                throw std::runtime_error("Unsupported gyro lens model: " + model);
            read_packaged_lens(gyro, config);
            if (config.contains("rolling_shutter")) {
                const auto& rs = config.at("rolling_shutter");
                s.readout_ms = rs.value("readout_ms", 0.0);
                read_direction(s, rs.value("direction", std::string("TopToBottom")));
            }
            if (config.contains("gyroflow_reference")) {
                const auto& ref = config.at("gyroflow_reference");
                read_reference(gyro, ref, ref.value("integration_method", 0));
            }
            if (gyro.imu_path.empty())
                gyro.imu_path = config_path.parent_path() / config.at("imu").at("file").get<std::string>();
        }
        gyro.imu = load_imu_csv(gyro.imu_path);
        if (imu_override.empty() && config.contains("imu"))
            align_packet_clock(gyro, config);
    } catch (const Json::exception& e) {
        throw std::runtime_error("Invalid gyro configuration " + config_path.string() + ": " + e.what());
    }
    validate_gyro_settings(gyro.settings);
    return gyro;
}
} // namespace stillflow
