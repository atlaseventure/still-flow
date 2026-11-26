#pragma once
#include "stillflow/domain/geometry.hpp"
#include <deque>
#include <optional>
#include <span>
#include <vector>

namespace stillflow {
// Camera to world in the right-handed OpenCV frame (+X right, +Y down, +Z forward).
struct Quaternion {
    double w = 1, x = 0, y = 0, z = 0;
};
Quaternion normalize(Quaternion q);
Quaternion conjugate(Quaternion q);
Quaternion multiply(Quaternion a, Quaternion b);
Quaternion exp_so3(double rx, double ry, double rz);
std::array<double, 3> log_so3(Quaternion q);
Quaternion slerp(Quaternion a, Quaternion b, double t);
Matrix3 to_matrix3(Quaternion q);

enum class LensModel { none, opencv_fisheye };
struct LensProfile {
    LensModel model = LensModel::none;
    Size calib_size{};
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::array<double, 4> k{};
    double lens_correction_amount = 1.0;
};
LensProfile scale_lens_profile(const LensProfile& lens, Size target_size);
// Both directions operate in normalized coordinates and use the same radial blend.
Point distort_point(const LensProfile& lens, Point normalized);
Point undistort_point(const LensProfile& lens, Point distorted);

struct ImuSample {
    double time_s = 0;
    double wx = 0, wy = 0, wz = 0; // rad/s
    double ax = 0, ay = 0, az = 0; // proper acceleration, m/s^2
    bool has_accel = false;
};
enum class ImuIntegrationMethod { gyro_only, complementary };
struct GyroSettings {
    double smoothness = 0.5;
    bool per_axis = false;
    double smoothness_pitch = 0.5, smoothness_yaw = 0.5, smoothness_roll = 0.5;
    double max_smoothness = 1.0, alpha_0_1s = 0.1;
    double readout_ms = 0.0;
    bool readout_top_to_bottom = true;
    double fov = 1.0, max_zoom = 1.30;
    int max_zoom_iterations = 10;
    double adaptive_zoom_window_s = 4.0;
    bool adaptive_zoom = true;
    double time_offset_s = 0.0; // Effective video time = IMU time + offset.
    ImuIntegrationMethod integration_method = ImuIntegrationMethod::gyro_only;
    Size output_size{};
    bool zoom_envelope = false;
};
void validate_gyro_settings(const GyroSettings& settings);

struct OrientationSample {
    double time_s = 0;
    Quaternion q;
};
struct OrientationTrack {
    std::vector<OrientationSample> samples; // Increasing effective video timestamps.
    Quaternion sample(double time_s) const;
};
OrientationTrack integrate_imu(std::span<const ImuSample> samples, const GyroSettings& settings);
OrientationTrack smooth_orientations(const OrientationTrack& raw, const GyroSettings& settings,
                                     std::span<const double> local_smoothness = {});

class ImuIntegrator {
  public:
    explicit ImuIntegrator(GyroSettings settings);
    void push(std::span<const ImuSample> samples);
    void prune_before(double time_s);
    const OrientationTrack& track() const { return track_; }
    size_t size() const { return track_.samples.size(); }

  private:
    GyroSettings settings_;
    OrientationTrack track_;
    std::optional<ImuSample> previous_;
    std::vector<ImuSample> initial_samples_;
};

struct GyroFrameMap {
    Size input_size{}, output_size{};
    LensProfile lens;
    double out_fx = 0, out_fy = 0, out_cx = 0, out_cy = 0;
    double fov_scale = 1.0, zoom_ratio = 1.0, smooth_factor = 1.0;
    Quaternion raw_orientation, smooth_orientation;
    // Uniform row knots mapping smooth camera rays into the raw row camera.
    std::vector<Matrix3> row_rotations;
    bool sensor_rows = false; // Index by projected input row for physical readout.
};
bool project_output_to_input(const GyroFrameMap& map, Point output_px, Point& input_px);
// Throws std::domain_error when the zoom limit and sensor margin are incompatible.
std::vector<GyroFrameMap> plan_gyro_frames(std::span<const double> frame_times_s, const OrientationTrack& raw,
                                           const LensProfile& lens, Size input_size, const GyroSettings& settings);

struct GyroStreamConfig {
    double lookahead_seconds = 1.35;
    size_t max_buffered_frames = 120;
};
// IMU timestamps (after time_offset_s) and frame timestamps share a time base.
// Supply IMU through the preview and row readout before draining ready frames.
class GyroStreamPlanner {
  public:
    GyroStreamPlanner(GyroSettings settings, LensProfile lens, Size input_size, GyroStreamConfig stream = {});
    Size output_size() const { return identity_.output_size; }
    void push_imu(std::span<const ImuSample> samples);
    bool push_frame(double time_s);
    void finish();
    bool take_ready(GyroFrameMap& out, double* lookahead_seconds = nullptr);
    size_t buffered() const { return pending_times_.size(); }
    bool full() const { return buffered() >= stream_.max_buffered_frames; }
    size_t imu_buffered() const { return integrator_.size(); }

  private:
    GyroSettings settings_;
    GyroStreamConfig stream_;
    ImuIntegrator integrator_;
    GyroFrameMap identity_;
    double dim_, readout_, output_limit_, identity_scale_, min_scale_, velocity_scale_;
    std::deque<double> pending_times_;
    std::array<double, 3> running_peak_{};
    double prev_time_ = 0, prev_scale_ = 1, prev_retained_ = 1;
    bool has_prev_ = false, finished_ = false;
};
} // namespace stillflow
