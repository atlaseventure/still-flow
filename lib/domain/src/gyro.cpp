#include "stillflow/domain/gyro.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <numbers>
#include <numeric>
#include <ranges>
#include <stdexcept>

namespace stillflow {
namespace {
using Vec = Eigen::Vector3d;
Eigen::Quaterniond eigen(Quaternion q) { return {q.w, q.x, q.y, q.z}; }
Quaternion quaternion(const Eigen::Quaterniond& q) { return {q.w(), q.x(), q.y(), q.z()}; }
bool finite(double v) { return std::isfinite(v); }
bool unit_interval(double v) { return v >= 0 && v <= 1; }
bool positive_size(Size s) { return s.width > 0 && s.height > 0; }
bool default_size(Size s) { return s.width == 0 && s.height == 0; }

double fisheye_radius(const std::array<double, 4>& k, double theta) {
    const double u = theta * theta;
    return theta * (1 + u * (k[0] + u * (k[1] + u * (k[2] + u * k[3]))));
}
double radial_derivative(const std::array<double, 4>& k, double theta) {
    const double u = theta * theta;
    return 1 + u * (3 * k[0] + u * (5 * k[1] + u * (7 * k[2] + u * 9 * k[3])));
}
} // namespace

Quaternion normalize(Quaternion q) {
    const auto v = eigen(q);
    const double n = v.coeffs().stableNorm();
    if (!(n > 0 && finite(n)))
        throw std::invalid_argument("Invalid quaternion");
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}
Quaternion conjugate(Quaternion q) { return quaternion(eigen(q).conjugate()); }
Quaternion multiply(Quaternion a, Quaternion b) { return quaternion(eigen(a) * eigen(b)); }
Quaternion exp_so3(double rx, double ry, double rz) {
    const double angle = std::hypot(rx, ry, rz);
    const double s = angle < 1e-8 ? 0.5 : std::sin(angle / 2) / angle;
    return {std::cos(angle / 2), rx * s, ry * s, rz * s};
}
std::array<double, 3> log_so3(Quaternion q) {
    q = normalize(q);
    const double n = std::hypot(q.x, q.y, q.z);
    const double s = std::copysign(n < 1e-12 ? 2 : 2 * std::atan2(n, std::abs(q.w)) / n, q.w);
    return {q.x * s, q.y * s, q.z * s};
}
Quaternion slerp(Quaternion a, Quaternion b, double t) {
    return normalize(quaternion(eigen(normalize(a)).slerp(t, eigen(normalize(b)))));
}
Matrix3 to_matrix3(Quaternion q) {
    Matrix3 m;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>{m.values.data()} = eigen(normalize(q)).toRotationMatrix();
    return m;
}

LensProfile scale_lens_profile(const LensProfile& lens, Size target_size) {
    LensProfile out = lens;
    if (!positive_size(target_size) || (!default_size(lens.calib_size) && !positive_size(lens.calib_size)) ||
        !unit_interval(lens.lens_correction_amount) || !std::ranges::all_of(lens.k, finite) ||
        (lens.model != LensModel::none && lens.model != LensModel::opencv_fisheye))
        throw std::invalid_argument("Invalid lens profile");
    if (positive_size(lens.calib_size)) {
        const double sx = double(target_size.width) / lens.calib_size.width;
        const double sy = double(target_size.height) / lens.calib_size.height;
        out.fx *= sx;
        out.cx *= sx;
        out.fy *= sy;
        out.cy *= sy;
    }
    out.calib_size = target_size;
    if (!(finite(out.fx) && out.fx > 0 && finite(out.fy) && out.fy > 0 && finite(out.cx) && finite(out.cy)))
        throw std::invalid_argument("Invalid lens intrinsics");
    return out;
}
Point distort_point(const LensProfile& lens, Point p) {
    if (lens.model == LensModel::none || lens.lens_correction_amount == 0)
        return p;
    const double r = std::hypot(p.x, p.y);
    if (r == 0)
        return p;
    return p * (std::lerp(r, fisheye_radius(lens.k, std::atan(r)), lens.lens_correction_amount) / r);
}
Point undistort_point(const LensProfile& lens, Point p) {
    const double amount = lens.lens_correction_amount;
    const double rd = std::hypot(p.x, p.y);
    if (rd == 0 || amount == 0 || lens.model == LensModel::none)
        return p;
    double lo = 0, hi = std::numbers::pi / 2;
    if (!finite(rd))
        throw std::domain_error("Fisheye ray outside forward hemisphere");
    double theta = std::atan(rd);
    for (int i = 0; i < 64; ++i) {
        const double r = std::tan(theta);
        const double error = std::lerp(r, fisheye_radius(lens.k, theta), amount) - rd;
        const double derivative = std::lerp(1 + r * r, radial_derivative(lens.k, theta), amount);
        // Calibrations can fold outside their usable FOV. Invert the increasing branch.
        if (derivative > 0) {
            const double step = error / derivative;
            if (std::abs(step) < 1e-14 && theta < std::numbers::pi / 2)
                return p * (r / rd);
            if (error < 0)
                lo = theta;
            else
                hi = theta;
            const double next = theta - step;
            if (next > lo && next < hi) {
                theta = next;
                continue;
            }
        } else {
            hi = theta;
        }
        theta = std::midpoint(lo, hi);
    }
    throw std::domain_error("Fisheye ray outside invertible field of view");
}
void validate_gyro_settings(const GyroSettings& s) {
    if (!unit_interval(s.smoothness) || !unit_interval(s.smoothness_pitch) || !unit_interval(s.smoothness_yaw) ||
        !unit_interval(s.smoothness_roll) || !finite(s.max_smoothness) || s.max_smoothness < 0 ||
        !finite(s.alpha_0_1s) || s.alpha_0_1s < 0 || !finite(s.readout_ms) || s.readout_ms < 0 || !finite(s.fov) ||
        s.fov <= 0 || !finite(s.max_zoom) || s.max_zoom < 1 || s.max_zoom_iterations < 0 ||
        s.max_zoom_iterations > 30 || !finite(s.adaptive_zoom_window_s) || s.adaptive_zoom_window_s < 0 ||
        !finite(s.time_offset_s) || (!default_size(s.output_size) && !positive_size(s.output_size)) ||
        (s.integration_method != ImuIntegrationMethod::gyro_only &&
         s.integration_method != ImuIntegrationMethod::complementary))
        throw std::invalid_argument("Invalid gyro settings");
}
Quaternion OrientationTrack::sample(double time_s) const {
    if (!finite(time_s))
        throw std::invalid_argument("Invalid orientation evaluation time");
    if (samples.empty())
        return {};
    const auto it = std::ranges::lower_bound(samples, time_s, {}, &OrientationSample::time_s);
    if (it == samples.end())
        return samples.back().q;
    if (it == samples.begin() || it->time_s == time_s)
        return it->q;
    const auto& prev = *std::prev(it);
    return slerp(prev.q, it->q, (time_s - prev.time_s) / (it->time_s - prev.time_s));
}
namespace {
Quaternion gyro_step(const ImuSample& a, const ImuSample& b, double dt) {
    return exp_so3(std::midpoint(a.wx, b.wx) * dt, std::midpoint(a.wy, b.wy) * dt, std::midpoint(a.wz, b.wz) * dt);
}
Vec up_measurement(const ImuSample& s) {
    const Vec a{s.ax, s.ay, s.az};
    const double n = a.norm();
    if (s.has_accel && std::abs(n - 9.81) < 2.5)
        return a / n;
    return Vec::Zero();
}
// The shortest world-frame tilt has its axis perpendicular to world up.
Quaternion level(Quaternion q, const Vec& measured, double gain) {
    const Vec up = eigen(q) * measured;
    const Vec world_up{0, -1, 0};
    Vec axis = up.cross(world_up);
    const double sine = axis.norm();
    if (sine == 0)
        axis = Vec::UnitX();
    else
        axis /= sine;
    const Vec delta = axis * (gain * std::atan2(sine, up.dot(world_up)));
    return normalize(multiply(exp_so3(delta.x(), delta.y(), delta.z()), q));
}
Quaternion initial_tilt(std::span<const ImuSample> samples) {
    Quaternion relative;
    Vec sum = Vec::Zero();
    for (size_t i = 0; i < samples.size() && samples[i].time_s - samples.front().time_s <= 0.1; ++i) {
        if (i)
            relative = normalize(
                multiply(relative, gyro_step(samples[i - 1], samples[i], samples[i].time_s - samples[i - 1].time_s)));
        sum += eigen(relative) * up_measurement(samples[i]);
    }
    return sum.norm() > 1e-8 ? level({}, sum.normalized(), 1) : Quaternion{};
}
void validate_samples(std::span<const ImuSample> samples, const GyroSettings& settings, const ImuSample* previous) {
    for (const auto& s : samples) {
        if (!finite(s.time_s + settings.time_offset_s) || (previous && s.time_s < previous->time_s) || !finite(s.wx) ||
            !finite(s.wy) || !finite(s.wz) ||
            (settings.integration_method == ImuIntegrationMethod::complementary && s.has_accel &&
             (!finite(s.ax) || !finite(s.ay) || !finite(s.az))))
            throw std::invalid_argument("Invalid IMU sample");
        previous = &s;
    }
}
void append_imu(OrientationTrack& track, std::span<const ImuSample> samples, const GyroSettings& settings,
                const ImuSample* previous, Quaternion q) {
    for (const auto& s : samples) {
        if (previous) {
            const double dt = s.time_s - previous->time_s;
            q = normalize(multiply(q, gyro_step(*previous, s, dt)));
            if (settings.integration_method == ImuIntegrationMethod::complementary) {
                const Vec up = up_measurement(s);
                if (!up.isZero())
                    q = level(q, up, -std::expm1(-dt / 3));
            }
        }
        const double t = s.time_s + settings.time_offset_s;
        if (!track.samples.empty() && track.samples.back().time_s == t)
            track.samples.back().q = q;
        else
            track.samples.push_back({t, q});
        previous = &s;
    }
}
} // namespace
OrientationTrack integrate_imu(std::span<const ImuSample> samples, const GyroSettings& settings) {
    validate_gyro_settings(settings);
    validate_samples(samples, settings, nullptr);
    OrientationTrack track;
    track.samples.reserve(samples.size());
    const auto tilt =
        settings.integration_method == ImuIntegrationMethod::complementary ? initial_tilt(samples) : Quaternion{};
    append_imu(track, samples, settings, nullptr, tilt);
    return track;
}

ImuIntegrator::ImuIntegrator(GyroSettings settings) : settings_(settings) { validate_gyro_settings(settings_); }
void ImuIntegrator::push(std::span<const ImuSample> samples) {
    if (samples.empty())
        return;
    validate_samples(samples, settings_, previous_ ? &*previous_ : nullptr);
    if (settings_.integration_method == ImuIntegrationMethod::complementary &&
        (!previous_ || !initial_samples_.empty())) {
        const double start = initial_samples_.empty() ? samples.front().time_s : initial_samples_.front().time_s;
        const auto end =
            std::ranges::partition_point(samples, [start](const auto& s) { return s.time_s - start <= 0.1; });
        if (end != samples.begin()) {
            // Only the initial tilt depends on later measurements. Rebuild this
            // short prefix before committing the new orientation history.
            const auto old_count = initial_samples_.size();
            initial_samples_.insert(initial_samples_.end(), samples.begin(), end);
            OrientationTrack updated;
            try {
                append_imu(updated, initial_samples_, settings_, nullptr, initial_tilt(initial_samples_));
                append_imu(updated, {end, samples.end()}, settings_, &initial_samples_.back(),
                           updated.samples.back().q);
            } catch (...) {
                initial_samples_.resize(old_count);
                throw;
            }
            if (end != samples.end())
                initial_samples_.clear();
            track_ = std::move(updated);
            previous_ = samples.back();
            return;
        }
    }
    const auto old_count = size();
    const auto q = previous_ ? track_.samples.back().q : Quaternion{};
    try {
        append_imu(track_, samples, settings_, previous_ ? &*previous_ : nullptr, q);
    } catch (...) {
        track_.samples.resize(old_count);
        if (old_count)
            track_.samples.back().q = q;
        throw;
    }
    initial_samples_.clear();
    previous_ = samples.back();
}
void ImuIntegrator::prune_before(double time_s) {
    // Once a frame uses the initial tilt its history must stay fixed.
    initial_samples_.clear();
    const auto after = std::ranges::upper_bound(track_.samples, time_s, {}, &OrientationSample::time_s);
    if (after != track_.samples.begin())
        track_.samples.erase(track_.samples.begin(), std::prev(after));
}
namespace {
double track_value(const OrientationTrack& raw, std::span<const double> values, double t) {
    const auto it = std::ranges::lower_bound(raw.samples, t, {}, &OrientationSample::time_s);
    if (it == raw.samples.end())
        return values.back();
    if (it == raw.samples.begin())
        return values.front();
    const size_t i = it - raw.samples.begin();
    const double prev = raw.samples[i - 1].time_s;
    return std::lerp(values[i - 1], values[i], (t - prev) / (it->time_s - prev));
}
void filter_rates(const OrientationTrack& raw, std::vector<Vec>& values) {
    const auto filter = [&](size_t i, size_t from) {
        const double dt = std::abs(raw.samples[i].time_s - raw.samples[from].time_s);
        values[i] += std::exp(-dt / 0.1) * (values[from] - values[i]);
    };
    for (size_t i = 1; i < values.size(); ++i)
        filter(i, i - 1);
    for (size_t i = values.size() - 1; i > 0; --i)
        filter(i - 1, i);
}

// Zero-phase angular speed in camera axes (or the rotation magnitude on all axes).
std::vector<Vec> angular_speeds(const OrientationTrack& raw, bool per_axis) {
    const auto& samples = raw.samples;
    std::vector<Vec> speed(samples.size(), Vec::Zero());
    for (size_t i = 0; i < samples.size(); ++i) {
        const auto& before = samples[i == 0 ? 0 : i - 1];
        const auto& after = samples[std::min(i + 1, samples.size() - 1)];
        const auto delta = log_so3(multiply(conjugate(before.q), after.q));
        const Vec rate = Eigen::Map<const Vec>(delta.data()) / (after.time_s - before.time_s);
        speed[i] = per_axis ? Vec(rate.cwiseAbs()) : Vec::Constant(rate.norm());
    }
    filter_rates(raw, speed);
    return speed;
}
OrientationTrack smooth_track(const OrientationTrack& raw, const GyroSettings& s, std::span<const double> local,
                              double velocity_scale, Vec* decayed_peak = nullptr) {
    validate_gyro_settings(s);
    const auto n = raw.samples.size();
    if (!local.empty() && (local.size() != n || !std::ranges::all_of(local, unit_interval)))
        throw std::invalid_argument("Invalid local smoothness");
    for (size_t i = 0; i < n; ++i) {
        const auto& p = raw.samples[i];
        if (!finite(p.time_s) || (i && p.time_s <= raw.samples[i - 1].time_s))
            throw std::invalid_argument("Invalid orientation time sequence");
        normalize(p.q);
    }
    if (n < 2 || s.smoothness == 0)
        return raw;
    const auto speed = angular_speeds(raw, s.per_axis);
    OrientationTrack out = raw;
    std::vector<Vec> adaptation(n, Vec::Ones());
    const auto step = [&](size_t i, size_t from) {
        const double dt = std::abs(raw.samples[i].time_s - raw.samples[from].time_s);
        const auto gain = [&](double strength, int axis) {
            const double local_factor = local.empty() ? 1 : local[i];
            if (strength * local_factor == 0 || s.max_smoothness == 0)
                return 1.0;
            const double limit = (250 * std::numbers::pi / 180) * velocity_scale * strength * local_factor;
            const double fraction = speed[i][axis] / limit * adaptation[i][axis];
            const double slow = -std::expm1(-dt / s.max_smoothness);
            const double fast = s.alpha_0_1s > 0 ? -std::expm1(-dt / s.alpha_0_1s) : 1.0;
            return std::min(1.0, std::lerp(slow, std::max(slow, fast), fraction));
        };
        const Quaternion a = out.samples[from].q, b = out.samples[i].q;
        if (!s.per_axis)
            return slerp(a, b, gain(s.smoothness, 0));
        const auto delta = log_so3(multiply(conjugate(a), b));
        return normalize(
            multiply(a, exp_so3(delta[0] * gain(s.smoothness_pitch, 0), delta[1] * gain(s.smoothness_yaw, 1),
                                delta[2] * gain(s.smoothness_roll, 2))));
    };
    for (int pass = 0; pass < 2; ++pass) {
        if (pass == 1) {
            Vec peak = Vec::Zero();
            for (size_t i = 0; i < n; ++i) {
                const auto d = log_so3(multiply(conjugate(raw.samples[i].q), out.samples[i].q));
                const Eigen::Map<const Vec> angle(d.data());
                adaptation[i] = s.per_axis ? Vec(angle.cwiseAbs()) : Vec::Constant(angle.norm());
                peak = peak.cwiseMax(adaptation[i]);
            }
            if (decayed_peak) {
                peak = peak.cwiseMax(*decayed_peak);
                *decayed_peak = peak;
            }
            for (auto& distance : adaptation) {
                distance = distance.cwiseQuotient(peak.cwiseMax(1e-12));
                distance = (distance.array() >= 0.5).select(distance, 0);
            }
            filter_rates(raw, adaptation);
            peak.setZero();
            for (const auto& distance : adaptation)
                peak = peak.cwiseMax(distance);
            for (auto& distance : adaptation)
                distance = Vec::Constant(0.5) + 0.5 * distance.cwiseQuotient(peak.cwiseMax(1e-12));
        }
        for (size_t i = 1; i < n; ++i)
            out.samples[i].q = step(i, i - 1);
        // Start the reverse filter at the last observed forward-filtered orientation.
        for (size_t i = n - 1; i > 0; --i)
            out.samples[i - 1].q = step(i - 1, i);
    }
    return out;
}
} // namespace

OrientationTrack smooth_orientations(const OrientationTrack& raw, const GyroSettings& settings,
                                     std::span<const double> local) {
    return smooth_track(raw, settings, local, 1);
}

namespace {
// Unclipped sensor coordinates are also needed to estimate the row exposure time.
bool project_ray(const GyroFrameMap& m, Point p, const Matrix3& a, const Matrix3& b, double blend, double min_z,
                 Point& src) {
    const double x = (p.x - m.out_cx) / m.out_fx, y = (p.y - m.out_cy) / m.out_fy;
    const auto component = [&](int row) {
        const double v = a(row, 0) * x + a(row, 1) * y + a(row, 2);
        if (blend == 0)
            return v;
        return std::lerp(v, b(row, 0) * x + b(row, 1) * y + b(row, 2), blend);
    };
    const double z = component(2);
    if (!(z > min_z && finite(z)))
        return false;
    const Point d = distort_point(m.lens, {component(0) / z, component(1) / z});
    // Keep exact sensor borders when an unchanged ray uses the same intrinsics.
    src = {d.x == x && m.lens.fx == m.out_fx && m.lens.cx == m.out_cx ? p.x : d.x * m.lens.fx + m.lens.cx,
           d.y == y && m.lens.fy == m.out_fy && m.lens.cy == m.out_cy ? p.y : d.y * m.lens.fy + m.lens.cy};
    return finite(src.x) && finite(src.y);
}
bool project(const GyroFrameMap& m, Point p, double margin, double min_z, Point& src) {
    if (m.row_rotations.empty() || !finite(p.x) || !finite(p.y))
        return false;
    const auto last = m.row_rotations.size() - 1;
    double fraction = p.y / std::max(1, m.output_size.height - 1);
    Point point;
    for (int i = 0; i < (m.sensor_rows ? 3 : 1); ++i) {
        const double row = std::clamp(fraction, 0.0, 1.0) * last;
        const auto index = static_cast<size_t>(row);
        if (!project_ray(m, p, m.row_rotations[index], m.row_rotations[std::min(index + 1, last)], row - index, min_z,
                         point))
            return false;
        fraction = point.y / std::max(1, m.input_size.height - 1);
    }
    if (point.x < margin || point.y < margin || point.x > m.input_size.width - 1 - margin ||
        point.y > m.input_size.height - 1 - margin)
        return false;
    src = point;
    return true;
}

struct FrameGeometry {
    const OrientationTrack& raw;
    double time, base_fx, base_fy, signed_readout, identity_scale;
    size_t row_knots = 64;

    void set_scale(GyroFrameMap& map, double scale) const {
        map.fov_scale = scale;
        // Extra zoom relative to the full sensor. A wider delivery crop consumes no zoom budget.
        map.zoom_ratio = std::max(1.0, identity_scale / scale);
        map.out_fx = base_fx / scale;
        map.out_fy = base_fy / scale;
        if (!(finite(map.out_fx) && map.out_fx > 0 && finite(map.out_fy) && map.out_fy > 0))
            throw std::invalid_argument("Invalid output focal lengths");
    }
    void set_orientation(GyroFrameMap& map, Quaternion smooth) const {
        map.smooth_orientation = smooth;
        const auto rotation = [&](Quaternion q) { return to_matrix3(multiply(conjugate(q), smooth)); };
        map.sensor_rows = signed_readout != 0;
        if (!map.sensor_rows) {
            map.row_rotations.assign(1, rotation(map.raw_orientation));
            return;
        }
        map.row_rotations.resize(row_knots);
        for (size_t i = 0; i < map.row_rotations.size(); ++i) {
            const double fraction = double(i) / (map.row_rotations.size() - 1);
            map.row_rotations[i] = rotation(raw.sample(time + (fraction - 0.5) * signed_readout));
        }
    }
    bool fits(GyroFrameMap& map, double scale, bool full_perimeter = false, Point* limiting = nullptr) const {
        set_scale(map, scale);
        const auto inside = [&](Point p) {
            Point src;
            if (project(map, p, 0.5, 1e-4, src))
                return true;
            if (limiting)
                *limiting = p;
            return false;
        };
        const double w = map.output_size.width - 1, h = map.output_size.height - 1;
        const auto edges_fit = [&](double u) {
            return inside({u * w, 0}) && inside({u * w, h}) && inside({0, u * h}) && inside({w, u * h});
        };
        if (!edges_fit(0) || !edges_fit(1))
            return false;
        // Probe the center and progressively bisect the edges to find a limiting ray early.
        for (int divisions = 2; divisions <= 64; divisions *= 2)
            for (int i = 1; i < divisions; i += 2)
                if (!edges_fit(double(i) / divisions))
                    return false;
        if (full_perimeter) {
            for (int x = 0; x < map.output_size.width; ++x)
                if (!inside({double(x), 0}) || !inside({double(x), h}))
                    return false;
            for (int y = 1; y < map.output_size.height - 1; ++y)
                if (!inside({0, double(y)}) || !inside({w, double(y)}))
                    return false;
        }
        return true;
    }
    // Bisect a failing ray first. Its limit bounds the whole perimeter, so most
    // previews need only one complete scan to certify the resulting scale.
    double preview_scale(GyroFrameMap& map, double ceiling, double floor = 0.05) const {
        Point limiting, src;
        constexpr int steps = 1 << 24;
        const auto scale = [&](int index) { return std::lerp(floor, ceiling, double(index) / steps); };
        int upper = steps;
        while (!fits(map, scale(upper), false, &limiting)) {
            set_scale(map, floor);
            if (upper == 0 || !project(map, limiting, 0.5, 1e-4, src))
                return 0;
            int lo = 0, hi = upper;
            while (hi - lo > 1) {
                const int mid = std::midpoint(lo, hi);
                set_scale(map, scale(mid));
                if (project(map, limiting, 0.5, 1e-4, src))
                    lo = mid;
                else
                    hi = mid;
            }
            upper = lo;
        }
        return scale(upper);
    }
    double largest_scale(GyroFrameMap& map, double ceiling, double floor = 0.05, bool full_perimeter = false) const {
        if (fits(map, ceiling, full_perimeter))
            return ceiling;
        if (!fits(map, floor, full_perimeter))
            return 0;
        double lo = floor, hi = ceiling;
        for (int i = 0; i < 24; ++i) {
            const double mid = std::midpoint(lo, hi);
            if (fits(map, mid, full_perimeter))
                lo = mid;
            else
                hi = mid;
        }
        return lo;
    }
};

// Spread the strongest required reduction at each timestamp. Overlapping frame
// neighborhoods should not compound more strongly at higher frame rates.
template <class Times>
void attenuate(const Times& times, std::vector<double>& factors, double t, double ratio, double sigma) {
    const auto start = std::ranges::lower_bound(times, t - 4 * sigma);
    for (size_t i = start - times.begin(); i < times.size() && times[i] <= t + 4 * sigma; ++i) {
        const double u = (times[i] - t) / sigma;
        factors[i] = std::min(factors[i], 1 - (1 - ratio) * std::exp(-0.5 * u * u));
    }
}

std::vector<double> gaussian_zoom(std::span<const double> times, const std::vector<double>& scales,
                                  const GyroSettings& s) {
    const double half = s.adaptive_zoom_window_s / 2;
    std::deque<size_t> minima;
    std::vector<double> envelope(times.size()), result(scales);
    size_t next = 0;
    for (size_t i = 0; i < times.size(); ++i) {
        while (next < times.size() && times[next] <= times[i] + half) {
            while (!minima.empty() && scales[minima.back()] >= scales[next])
                minima.pop_back();
            minima.push_back(next++);
        }
        while (times[minima.front()] < times[i] - half)
            minima.pop_front();
        envelope[i] = scales[minima.front()];
    }
    const double sigma = s.adaptive_zoom_window_s / 4;
    if (sigma == 0)
        return envelope;
    for (size_t i = 0; i < times.size(); ++i) {
        double sum = 0, weights = 0;
        const auto first = std::ranges::lower_bound(times, times[i] - 4 * sigma);
        for (size_t j = first - times.begin(); j < times.size() && times[j] <= times[i] + 4 * sigma; ++j) {
            const double d = (times[j] - times[i]) / sigma;
            const double weight = std::exp(-0.5 * d * d);
            sum += envelope[j] * weight;
            weights += weight;
        }
        result[i] = std::min(scales[i], sum / weights);
    }
    return result;
}

std::vector<double> zoom_curve(std::span<const double> times, const std::vector<double>& scales,
                               const GyroSettings& s) {
    if (!s.adaptive_zoom || s.adaptive_zoom_window_s == 0)
        return std::vector<double>(times.size(), std::ranges::min(scales));
    if (!s.zoom_envelope)
        return gaussian_zoom(times, scales, s);
    std::vector<double> result(scales);
    for (double tau : {s.adaptive_zoom_window_s, 0.2}) {
        const auto advance = [&](size_t i, size_t from) {
            const double dt = std::abs(times[i] - times[from]);
            result[i] = std::min(result[i], std::lerp(result[i], result[from], std::exp(-dt / tau)));
        };
        for (size_t i = times.size() - 1; i > 0; --i)
            advance(i - 1, i);
        for (size_t i = 1; i < times.size(); ++i)
            advance(i, i - 1);
    }
    return result;
}

double retained_correction(const FrameGeometry& g, GyroFrameMap& map, double min_scale) {
    const auto target = map.smooth_orientation;
    g.set_orientation(map, map.raw_orientation);
    if (!g.fits(map, min_scale))
        throw std::domain_error("Sensor bounds require more than max_zoom");
    double lo = 0, hi = 1;
    for (int j = 0; j < 24; ++j) {
        const double mid = std::midpoint(lo, hi);
        g.set_orientation(map, slerp(map.raw_orientation, target, mid));
        if (g.fits(map, min_scale))
            lo = mid;
        else
            hi = mid;
    }
    g.set_orientation(map, target);
    return lo;
}

struct ProjectionSetup {
    GyroFrameMap identity;
    double dim, readout, output_limit, identity_scale, min_scale, velocity_scale;
};
ProjectionSetup projection_setup(const LensProfile& lens, Size input_size, const GyroSettings& s) {
    const OrientationTrack raw;
    const auto scaled = scale_lens_profile(lens, input_size);
    const Size output = default_size(s.output_size) ? input_size : s.output_size;
    const double dim = double(output.width) / input_size.width / s.fov;
    const double readout = (s.readout_top_to_bottom ? 1 : -1) * s.readout_ms / 1000;
    GyroFrameMap identity;
    identity.input_size = input_size;
    identity.output_size = output;
    identity.lens = scaled;
    identity.out_cx = (output.width - 1) * 0.5;
    identity.out_cy = (output.height - 1) * 0.5;
    // The output aspect sets the geometric search ceiling. Measure extra zoom
    // against the uncropped sensor aspect: a 16:9 delivery crop of a 4:3 camera
    // should not consume the stabilization budget before the camera has moved.
    const double nominal = double(output.width) / input_size.width;
    const FrameGeometry unrotated{raw, 0, scaled.fx * nominal, scaled.fy * nominal, 0, 4};
    unrotated.set_orientation(identity, {});
    const bool pinhole = scaled.model == LensModel::none || scaled.lens_correction_amount == 0;
    const double output_limit = pinhole ? 1.0 : unrotated.largest_scale(identity, 4);
    auto sensor = identity;
    sensor.output_size = input_size;
    sensor.out_cx = (input_size.width - 1) * 0.5;
    sensor.out_cy = (input_size.height - 1) * 0.5;
    const FrameGeometry full_sensor{raw, 0, scaled.fx, scaled.fy, 0, 4};
    const double identity_scale = pinhole ? 1.0 : std::min(output_limit, full_sensor.largest_scale(sensor, 4));
    if (identity_scale <= 0)
        throw std::domain_error("Unrotated field of view exceeds sensor bounds");
    const double min_scale = std::max(0.05, identity_scale / s.max_zoom);
    // Reference velocity thresholds are specified at a 120 degree diagonal FOV.
    const double velocity_scale = 2 * std::atan(std::hypot(input_size.width, input_size.height) / (2 * scaled.fy)) /
                                  (120 * std::numbers::pi / 180);
    return {std::move(identity), dim, readout, output_limit, identity_scale, min_scale, velocity_scale};
}

} // namespace

bool project_output_to_input(const GyroFrameMap& map, Point output_px, Point& input_px) {
    return project(map, output_px, 0, 1e-6, input_px);
}

std::vector<GyroFrameMap> plan_gyro_frames(std::span<const double> times, const OrientationTrack& raw,
                                           const LensProfile& lens, Size input_size, const GyroSettings& s) {
    validate_gyro_settings(s);
    if (!std::ranges::all_of(times, finite) || !std::ranges::is_sorted(times))
        throw std::invalid_argument("Invalid frame times");
    const auto [identity, dim, readout, output_limit, identity_scale, min_scale, velocity_scale] =
        projection_setup(lens, input_size, s);
    const auto& scaled = identity.lens;
    const double sigma = 0.5;
    const auto sample_times = raw.samples | std::views::transform(&OrientationSample::time_s);
    std::vector<double> local(raw.samples.size(), 1), scales(times.size());
    std::vector<GyroFrameMap> maps(times.size(), identity);
    const auto geometry = [&](size_t i) {
        return FrameGeometry{raw, times[i], scaled.fx * dim, scaled.fy * dim, readout, identity_scale};
    };
    for (int iteration = 0;; ++iteration) {
        const auto smooth = smooth_track(raw, s, local, velocity_scale);
        if (times.empty())
            return {};
        std::vector<double> reduction(local.size(), 1);
        bool relax = false;
        for (size_t i = 0; i < times.size(); ++i) {
            auto& map = maps[i];
            map.raw_orientation = raw.sample(times[i]);
            geometry(i).set_orientation(map, smooth.sample(times[i]));
            map.smooth_factor = local.empty() ? 1 : track_value(raw, local, times[i]);
            scales[i] = geometry(i).largest_scale(map, output_limit);
            if (scales[i] < min_scale - 1e-4 && iteration < s.max_zoom_iterations && !local.empty()) {
                attenuate(sample_times, reduction, times[i], scales[i] / min_scale, sigma);
                relax = true;
            }
        }
        if (!relax)
            break;
        for (size_t i = 0; i < local.size(); ++i)
            local[i] *= reduction[i];
    }

    // Find the maximum retained correction that fits at the zoom limit, then
    // spread that reduction in video time before certifying the final maps.
    std::vector<double> retained(times.size(), 1);
    for (size_t i = 0; i < times.size(); ++i) {
        if (scales[i] >= min_scale)
            continue;
        auto& map = maps[i];
        const auto g = geometry(i);
        if (g.fits(map, min_scale)) {
            scales[i] = min_scale;
            continue;
        }
        const double lo = retained_correction(g, map, min_scale);
        attenuate(times, retained, times[i], lo, sigma);
        retained[i] = std::min(retained[i], lo);
    }
    for (size_t i = 0; i < times.size(); ++i) {
        if (retained[i] == 1)
            continue;
        auto& map = maps[i];
        geometry(i).set_orientation(map, slerp(map.raw_orientation, map.smooth_orientation, retained[i]));
        map.smooth_factor *= retained[i];
        scales[i] = geometry(i).largest_scale(map, output_limit, min_scale);
        if (scales[i] < min_scale)
            throw std::domain_error("Relaxed orientation exceeds sensor bounds");
    }
    const auto final_scales = zoom_curve(times, scales, s);
    for (size_t i = 0; i < times.size(); ++i) {
        const double ceiling = std::clamp(final_scales[i], min_scale, output_limit);
        const auto g = geometry(i);
        const double scale = g.largest_scale(maps[i], ceiling, min_scale, true);
        if (scale == 0)
            throw std::domain_error("Final rolling shutter map exceeds sensor bounds");
        g.set_scale(maps[i], scale);
    }
    return maps;
}

GyroStreamPlanner::GyroStreamPlanner(GyroSettings settings, LensProfile lens, Size input_size, GyroStreamConfig stream)
    : settings_(settings), stream_(stream), integrator_(settings) {
    if (input_size.width < 2 || input_size.height < 2 || !finite(stream.lookahead_seconds) ||
        stream.lookahead_seconds < 0 || stream.lookahead_seconds > 10 || stream.max_buffered_frames < 2 ||
        stream.max_buffered_frames > 600)
        throw std::invalid_argument("Invalid gyro streaming size, lookahead or buffer capacity");
    auto setup = projection_setup(lens, input_size, settings);
    identity_ = std::move(setup.identity);
    dim_ = setup.dim;
    readout_ = setup.readout;
    output_limit_ = setup.output_limit;
    identity_scale_ = setup.identity_scale;
    min_scale_ = setup.min_scale;
    velocity_scale_ = setup.velocity_scale;
}

void GyroStreamPlanner::push_imu(std::span<const ImuSample> samples) {
    if (finished_)
        throw std::logic_error("Stream is already finished");
    integrator_.push(samples);
}
bool GyroStreamPlanner::push_frame(double time_s) {
    if (finished_)
        throw std::logic_error("Stream is already finished");
    if (!finite(time_s) || (!pending_times_.empty() && time_s <= pending_times_.back()))
        throw std::invalid_argument("Gyro frame times must increase");
    if (full())
        return false;
    pending_times_.push_back(time_s);
    return true;
}
void GyroStreamPlanner::finish() {
    if (finished_)
        throw std::logic_error("Stream is already finished");
    finished_ = true;
}

bool GyroStreamPlanner::take_ready(GyroFrameMap& out, double* lookahead_seconds) {
    if (pending_times_.empty())
        return false;
    const double time = pending_times_.front(), ahead = pending_times_.back() - time;
    if (!finished_ && !full() && (buffered() < 2 || ahead + 2e-6 < stream_.lookahead_seconds))
        return false;

    const double dt = has_prev_ ? std::max(1e-6, time - prev_time_) : 0;
    const double preview_end = std::min(pending_times_.back(), time + stream_.lookahead_seconds);
    const double half_readout = 0.5 * std::abs(readout_);
    const double past_support = std::max(4.0, 4 * settings_.max_smoothness) + 1;
    const auto& samples = integrator_.track().samples;
    const auto first = finished_ ? samples.begin()
                                 : std::ranges::upper_bound(samples, time - past_support - half_readout, {},
                                                            &OrientationSample::time_s);
    const auto end =
        finished_ ? samples.end()
                  : std::ranges::upper_bound(samples, preview_end + half_readout, {}, &OrientationSample::time_s);
    const OrientationTrack raw{{first == samples.begin() ? first : std::prev(first), end}};
    Vec peak = Eigen::Map<const Vec>(running_peak_.data()) * std::exp(-dt / 2);
    const auto smooth = smooth_track(raw, settings_, {}, velocity_scale_, &peak);

    std::vector<double> times{time};
    const double spacing = std::max(0.1, (preview_end - time) / 14);
    const auto last = std::prev(std::ranges::upper_bound(pending_times_, preview_end));
    for (auto it = std::next(pending_times_.begin()); it <= last; ++it)
        if (it == last || *it - times.back() >= spacing - 1e-9)
            times.push_back(*it);

    const auto geometry = [&](size_t i) {
        return FrameGeometry{raw,      times[i],        identity_.lens.fx * dim_,        identity_.lens.fy * dim_,
                             readout_, identity_scale_, i == 0 ? size_t{64} : size_t{16}};
    };
    std::vector<double> scales(times.size()), retained(times.size(), 1);
    GyroFrameMap current;
    double current_limit = 1;
    for (size_t i = 0; i < times.size(); ++i) {
        auto map = identity_;
        const auto g = geometry(i);
        map.raw_orientation = raw.sample(times[i]);
        g.set_orientation(map, smooth.sample(times[i]));
        scales[i] = g.preview_scale(map, output_limit_);
        if (scales[i] < min_scale_) {
            if (!g.fits(map, min_scale_)) {
                const double limit = retained_correction(g, map, min_scale_);
                attenuate(times, retained, times[i], limit, 0.5);
                retained[i] = std::min(retained[i], limit);
                if (i == 0)
                    current_limit = limit;
            }
            scales[i] = min_scale_;
        }
        if (i == 0)
            current = std::move(map);
    }
    const auto g = geometry(0);
    const double factor =
        has_prev_ ? std::min(current_limit, std::lerp(retained.front(), prev_retained_, std::exp(-dt / 0.2)))
                  : retained.front();
    if (factor < 1) {
        g.set_orientation(current, slerp(current.raw_orientation, current.smooth_orientation, factor));
        current.smooth_factor = factor;
        scales.front() = g.preview_scale(current, output_limit_, min_scale_);
    }
    const bool adaptive = settings_.adaptive_zoom && settings_.adaptive_zoom_window_s > 0;
    const double tau = adaptive ? std::max(0.2, settings_.adaptive_zoom_window_s) : 0.2;
    double anticipated = scales.back();
    for (size_t i = times.size() - 1; i > 0; --i) {
        const double weight = std::exp(-(times[i] - times[i - 1]) / tau);
        anticipated = std::min(scales[i - 1], std::lerp(scales[i - 1], anticipated, weight));
    }
    double target = anticipated;
    if (has_prev_) {
        if (!adaptive)
            target = std::min(prev_scale_, anticipated);
        else if (anticipated > prev_scale_)
            target = std::lerp(anticipated, prev_scale_, std::exp(-dt / tau));
        else
            target = std::min(scales.front(), std::lerp(anticipated, prev_scale_, std::exp(-dt / 0.2)));
    }
    const double scale = g.largest_scale(current, std::clamp(target, min_scale_, output_limit_), min_scale_, true);
    if (scale == 0)
        throw std::domain_error("Final rolling shutter map exceeds sensor bounds");
    g.set_scale(current, scale);
    out = std::move(current);
    if (lookahead_seconds)
        *lookahead_seconds = ahead;
    prev_scale_ = scale;
    prev_retained_ = factor;
    prev_time_ = time;
    has_prev_ = true;
    Eigen::Map<Vec>{running_peak_.data()} = peak;
    pending_times_.pop_front();
    integrator_.prune_before(time - past_support - half_readout);
    return true;
}
} // namespace stillflow
