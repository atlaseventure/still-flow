#include "stillflow/domain/motion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>

namespace stillflow {
namespace {
bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y); }

Point row_motion(const Motion& motion, double position) {
    if (motion.row_delta.empty())
        return {};
    const double coordinate = position * (motion.row_delta.size() - 1);
    const auto low = static_cast<size_t>(coordinate);
    const auto high = std::min(low + 1, motion.row_delta.size() - 1);
    return motion.row_delta[low] * (1 - (coordinate - low)) + motion.row_delta[high] * (coordinate - low);
}

// https://github.com/liutao23/LightStab/blob/main/model/utils.py
// LightStab demo: LightOnlineStab.inference(repeat=50) and
// utils.generateSmooth_online/_causal7_once. Each pass averages its INPUT at
// t and t-1..t-6; it does not recursively average its own output. The current
// weight is one and available taps are normalized locally at shot boundaries.
// We use the demo's Gaussian fallback, gated by measured motion and confidence
// instead of pretrained kernels. Positions are rebased to O_t after each input
// displacement, so the result is directly M_t = S_t - O_t without large sums.
class CausalPath {
  public:
    Point advance(Point delta, double time, double confidence, double seconds, double pixel_scale) {
        if (seconds <= 0)
            return {};
        constexpr double reference_dt = 1.0 / 30;
        const double window = 6 * reference_dt * seconds / 0.45;
        Point trend;
        double trend_weight = 0;
        for (auto& state : observations_) {
            state.position = state.position - delta;
            trend = trend - state.position * (state.confidence * reference_dt / (time - state.time));
            trend_weight += state.confidence;
        }
        trend = trend * (1 / std::max(1e-6, trend_weight));
        const double attenuation = std::exp(-0.02 * trend.dot(trend) / (pixel_scale * pixel_scale));
        std::array<double, 6> kernel{};
        double denominator = 1;
        for (size_t lag = 0; lag < observations_.size(); ++lag) {
            const auto& state = observations_[observations_.size() - 1 - lag];
            const double age = time - state.time;
            kernel[lag] = std::exp(-9 * age * age / (window * window)) * attenuation * state.confidence;
            denominator += kernel[lag];
        }
        Point correction;
        for (auto& pass : passes_) {
            const Point input = correction;
            for (auto& position : pass)
                position = position - delta;
            for (size_t lag = 0; lag < pass.size(); ++lag)
                correction = correction + pass[pass.size() - 1 - lag] * kernel[lag];
            correction = correction * (1 / denominator);
            pass.push_back(input);
            if (pass.size() > 6)
                pass.pop_front();
        }
        observations_.push_back({time, {}, std::clamp(confidence, 0.0, 1.0)});
        if (observations_.size() > 6)
            observations_.pop_front();
        if (!finite(correction))
            throw std::runtime_error("Nonfinite mesh trajectory");
        return correction;
    }

  private:
    struct Observation {
        double time;
        Point position;
        double confidence;
    };
    std::deque<Observation> observations_;
    std::array<std::deque<Point>, trajectory_smoothing_passes> passes_;
};

std::vector<Point> initialize_rows(std::vector<CausalPath>& paths, const std::vector<FrameMotion>& frames, size_t start,
                                   int columns, int rows, bool horizontal, double seconds, double pixel_scale) {
    if (seconds <= 0 || frames[start].motion.state == MotionState::missing)
        return {};
    // Reflect a reliable prefix around the shot boundary. The original causal
    // cascade then starts with real local-motion history instead of retaining
    // the first frame's rolling-shutter shape. Never cross an unreliable link.
    size_t end = start;
    const double origin = frames[start].seconds;
    // Five kernel widths cover the cascade's dominant history; its exact
    // maximum support is six observations per pass.
    for (size_t i = start + 1; i < frames.size() && i - start <= size_t{6} * trajectory_smoothing_passes; ++i) {
        const auto& frame = frames[i];
        if (frame.segment != frames[start].segment || frame.motion.reset)
            break;
        if (!std::isfinite(frame.seconds) || frame.seconds <= frames[i - 1].seconds)
            throw std::invalid_argument("Trajectory timestamps must increase within a shot");
        if (frame.seconds - origin > 5 * seconds || frame.motion.state != MotionState::tracked ||
            frame.motion.rs_confidence < 0.5 || frame.motion.row_delta.empty())
            break;
        end = i;
    }
    if (end - start < 6)
        return {};
    std::vector<Point> correction(paths.size());
    for (int y = 0; y < rows; ++y)
        for (int x = 0; x < columns; ++x) {
            const int vertex = y * columns + x;
            const double position =
                horizontal ? static_cast<double>(x) / (columns - 1) : static_cast<double>(y) / (rows - 1);
            auto& path = paths[vertex];
            path.advance({}, 2 * origin - frames[end].seconds, frames[end].motion.rs_confidence, seconds, pixel_scale);
            for (size_t i = end; i > start; --i) {
                const Point delta = row_motion(frames[i].motion, position) * -1;
                if (!finite(delta))
                    throw std::runtime_error("Nonfinite observed mesh motion");
                correction[vertex] = path.advance(delta, 2 * origin - frames[i - 1].seconds,
                                                  frames[i].motion.rs_confidence, seconds, pixel_scale);
            }
        }
    return correction;
}

double gradient_bound(const Warp& warp, Size size) {
    const double dx = static_cast<double>(size.width - 1) / (warp.columns - 1);
    const double dy = static_cast<double>(size.height - 1) / (warp.rows - 1);
    double bound = 0;
    for (int y = 0; y + 1 < warp.rows; ++y)
        for (int x = 0; x + 1 < warp.columns; ++x) {
            const auto a = warp.displacement[y * warp.columns + x];
            const auto b = warp.displacement[y * warp.columns + x + 1];
            const auto c = warp.displacement[(y + 1) * warp.columns + x];
            const auto d = warp.displacement[(y + 1) * warp.columns + x + 1];
            const double horizontal = std::max(norm(b - a), norm(d - c)) / dx;
            const double vertical = std::max(norm(c - a), norm(d - b)) / dy;
            bound = std::max(bound, std::hypot(horizontal, vertical));
        }
    return bound;
}

bool fits(const Warp& warp, Size size, double crop, double strength) {
    Warp candidate = warp;
    candidate.strength = strength;
    const auto map = prepare_map(candidate, size, size, crop, false);
    const auto valid = [&](double x, double y) {
        const Point target(map.origin_x + x * (size.width - 1) * map.step_x,
                           map.origin_y + y * (size.height - 1) * map.step_y);
        const auto p = source_point(map, target);
        return finite(p) && p.x >= 0 && p.x <= size.width - 1 && p.y >= 0 && p.y <= size.height - 1;
    };
    // A contractive displacement has no folded cells. Check the crop boundary;
    // the rendering backend additionally checks every output pixel.
    for (int i = 0; i <= 64; ++i) {
        const double t = i / 64.0;
        if (!valid(t, 0) || !valid(t, 1) || !valid(0, t) || !valid(1, t))
            return false;
    }
    return true;
}
} // namespace

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size size, const StabilizationSettings& options,
                             bool horizontal) {
    if (size.width < 2 || size.height < 2 || options.bands < 4 || options.bands > 32)
        throw std::invalid_argument("Invalid trajectory geometry");
    std::vector<Warp> output(frames.size());
    const int columns = horizontal ? options.bands : 9, rows = horizontal ? 9 : options.bands;
    const int count = columns * rows;
    const double pixel_scale = std::max(size.width, size.height) / 960.0;
    std::vector<CausalPath> global(count), local(count);
    double missing_seconds = 0, row_strength = 0, previous_crop = 1, previous_tracking = 1;
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const auto& motion = frame.motion;
        const bool start = i == 0 || frame.segment != frames[i - 1].segment || motion.reset;
        if (!std::isfinite(frame.seconds) || (!start && frame.seconds <= frames[i - 1].seconds))
            throw std::invalid_argument("Trajectory timestamps must increase within a shot");
        std::vector<Point> initial_rows;
        if (start) {
            global.assign(count, CausalPath{});
            local.assign(count, CausalPath{});
            missing_seconds = 0;
            row_strength = 0;
            previous_crop = previous_tracking = 1;
            if (options.rolling_shutter && options.rs_strength > 0)
                initial_rows =
                    initialize_rows(local, frames, i, columns, rows, horizontal, options.rs_seconds, pixel_scale);
        }
        const double dt = start ? std::max(0.0, frame.duration) : frame.seconds - frames[i - 1].seconds;
        const bool missing = motion.state == MotionState::missing;
        const bool has_rows = !start && motion.state == MotionState::tracked && options.rolling_shutter;
        row_strength = !initial_rows.empty() ? 1 : has_rows ? std::min(1.0, row_strength + dt / 0.15) : 0;
        auto& warp = output[i];
        warp.columns = columns;
        warp.rows = rows;
        warp.displacement.resize(count);
        double row_energy = 0;
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < columns; ++x) {
                const int vertex = y * columns + x;
                const Point p(static_cast<double>(x) * (size.width - 1) / (columns - 1),
                              static_cast<double>(y) * (size.height - 1) / (rows - 1));
                const Point delta = start || missing ? Point{} : transform(motion.step, p) - p;
                const Point residual = has_rows ? row_motion(motion, horizontal ? static_cast<double>(x) / (columns - 1)
                                                                                : static_cast<double>(y) / (rows - 1))
                                                : Point{};
                if (!finite(delta) || !finite(residual))
                    throw std::runtime_error("Nonfinite observed mesh motion");
                const auto g = global[vertex].advance(delta, frame.seconds, start ? 1 : motion.confidence,
                                                      options.smooth_seconds, pixel_scale);
                const auto local_correction =
                    initial_rows.empty()
                        ? local[vertex].advance(residual, frame.seconds, has_rows ? motion.rs_confidence : 0,
                                                options.rs_seconds, pixel_scale)
                        : initial_rows[vertex];
                const auto r = local_correction * (options.rs_strength * row_strength);
                warp.displacement[vertex] = g + r;
                warp.global_offset = warp.global_offset + g * (1.0 / count);
                row_energy += r.dot(r);
            }
        warp.row_correction_rms = std::sqrt(row_energy / count);
        // Bound the displacement Jacobian so backward fixed-point sampling is
        // contractive and cannot fold. Eight iterations leave < 0.25^8 error.
        const double bound = gradient_bound(warp, size);
        double high = bound > 0.25 ? 0.25 / bound : 1;
        if (!fits(warp, size, options.crop, high)) {
            double low = 0;
            for (int iteration = 0; iteration < 12; ++iteration) {
                const double middle = (low + high) / 2;
                if (fits(warp, size, options.crop, middle))
                    low = middle;
                else
                    high = middle;
            }
            high = low;
        }
        warp.crop_strength = std::min(high, previous_crop + dt / 0.25);
        missing_seconds = missing ? missing_seconds + dt : 0;
        warp.tracking_strength = std::min(std::max(0.0, 1 - missing_seconds / 0.25), previous_tracking + dt / 0.25);
        warp.strength = std::min(warp.crop_strength, warp.tracking_strength);
        previous_crop = warp.crop_strength;
        previous_tracking = warp.tracking_strength;
    }
    return output;
}

Point source_point(const MapParameters& map, Point target) {
    Point source = target;
    for (int iteration = 0; iteration < mesh_inverse_iterations; ++iteration) {
        const double gx = std::clamp(source.x * map.grid_x, 0.0, static_cast<double>(map.columns - 1));
        const double gy = std::clamp(source.y * map.grid_y, 0.0, static_cast<double>(map.rows - 1));
        const int x = std::min(static_cast<int>(gx), map.columns - 2);
        const int y = std::min(static_cast<int>(gy), map.rows - 2);
        const double fx = gx - x, fy = gy - y;
        const int a = y * map.columns + x, b = a + map.columns;
        const auto sample = [&](const double* values) {
            return (values[a] * (1 - fx) + values[a + 1] * fx) * (1 - fy) +
                   (values[b] * (1 - fx) + values[b + 1] * fx) * fy;
        };
        source = target - Point(sample(map.offset_x), sample(map.offset_y));
    }
    return source;
}

MapParameters prepare_map(const Warp& warp, Size size, Size output, double crop, bool /*horizontal*/) {
    const int count = warp.columns * warp.rows;
    if (warp.columns < 2 || warp.rows < 2 || warp.columns > 32 || warp.rows > 32 || count > max_mesh_vertices ||
        warp.displacement.size() != static_cast<size_t>(count) || size.width < 2 || size.height < 2 ||
        output.width < 2 || output.height < 2 || !std::isfinite(warp.strength) || warp.strength < 0 ||
        warp.strength > 1 || !std::isfinite(crop) || crop < 0 || crop > 0.25)
        throw std::invalid_argument("Invalid displacement mesh");
    MapParameters map;
    map.columns = warp.columns;
    map.rows = warp.rows;
    map.origin_x = crop * (output.width - 1);
    map.origin_y = crop * (output.height - 1);
    map.step_x = map.step_y = 1 - 2 * crop;
    map.grid_x = static_cast<double>(warp.columns - 1) / (output.width - 1);
    map.grid_y = static_cast<double>(warp.rows - 1) / (output.height - 1);
    const double sx = static_cast<double>(output.width - 1) / (size.width - 1);
    const double sy = static_cast<double>(output.height - 1) / (size.height - 1);
    for (int i = 0; i < count; ++i) {
        if (!finite(warp.displacement[i]))
            throw std::runtime_error("Nonfinite mesh displacement");
        map.offset_x[i] = warp.displacement[i].x * warp.strength * sx;
        map.offset_y[i] = warp.displacement[i].y * warp.strength * sy;
    }
    return map;
}
} // namespace stillflow
