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
// Confidence weights are shared by the mesh rather than adapting separately
// to each vertex's speed. Positions are rebased after each input displacement,
// so the result is directly M_t = S_t - O_t without large sums.
template <size_t Passes> class CausalPath {
  public:
    Point advance(Point delta, double time, double confidence, double seconds) {
        if (seconds <= 0)
            return {};
        constexpr double reference_dt = 1.0 / 30;
        const double window = 6 * reference_dt * seconds / 0.45;
        std::array<double, 6> kernel{};
        double denominator = 1;
        for (size_t lag = 0; lag < observations_.size(); ++lag) {
            const auto& state = observations_[observations_.size() - 1 - lag];
            const double age = time - state.time;
            kernel[lag] = std::exp(-9 * age * age / (window * window)) * state.confidence;
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
        observations_.push_back({time, std::clamp(confidence, 0.0, 1.0)});
        if (observations_.size() > 6)
            observations_.pop_front();
        if (!finite(correction))
            throw std::runtime_error("Nonfinite mesh trajectory");
        return correction;
    }

  private:
    struct Observation {
        double time;
        double confidence;
    };
    std::deque<Observation> observations_;
    std::array<std::deque<Point>, Passes> passes_;
};

// Half of the cascade runs forward and half backward. Each pass applies the
// same kernel, so |H_forward * H_backward| equals the causal |H_50| while the
// opposite group delays cancel: offline planning keeps the full smoothing
// strength without lagging behind intentional camera motion.
// A single temporal kernel preserves the global similarity model. Adapting
// independently to each vertex's speed bends a rigid rotation into a mesh warp.
using SymmetricPath = CausalPath<trajectory_smoothing_passes / 2>;
static_assert(trajectory_smoothing_passes % 2 == 0);

// Confidence-weighted linear fit of one vertex path near a shot end, evaluated
// at that end. Pivoting the reflection on the raw end sample would mirror its
// shake into the padding and pull the boundary frames by twice that amount.
Point fit_endpoint(const std::vector<double>& times, const std::vector<double>& weights,
                   const std::vector<Point>& positions, size_t first, size_t last, size_t end) {
    const double at = times[end];
    double w = 0, t = 0, tt = 0;
    Point x, tx;
    for (size_t k = first; k < last; ++k) {
        const double dt = times[k] - at;
        w += weights[k];
        t += weights[k] * dt;
        tt += weights[k] * dt * dt;
        x = x + positions[k] * weights[k];
        tx = tx + positions[k] * (weights[k] * dt);
    }
    const double determinant = w * tt - t * t;
    if (w <= 1e-9)
        return positions[end];
    if (std::abs(determinant) <= 1e-12 * std::max(1.0, w * tt))
        return x * (1 / w);
    // Intercept of x = a + b * (time - at).
    return (x * tt - tx * t) * (1 / determinant);
}

// Zero-phase trajectory for frames [first, last). Point reflection about
// fitted shot ends extends constant-velocity motion linearly, so the symmetric
// response neither lags nor bends a steady pan at the boundaries. Applying the
// same construction to row motion also avoids accumulating steady parallax
// into a persistent scanline warp that consumes the global correction budget.
void plan_paths(std::vector<std::vector<Point>>& output, const std::vector<FrameMotion>& frames, size_t first,
                size_t last, Size size, int columns, int rows, double seconds, bool scanline, bool horizontal) {
    const size_t n = last - first, count = static_cast<size_t>(columns) * rows;
    for (size_t i = first; i < last; ++i)
        output[i].assign(count, Point{});
    if (seconds <= 0 || n < 2)
        return;
    // Observation into each frame; the shot's first frame has none.
    std::vector<double> times(n), confidence(n);
    // The temporal kernel is shared: affine global motion needs only three
    // anchors, and scanline motion has one independent path per band.
    const bool affine = std::all_of(frames.begin() + first, frames.begin() + last, [](const auto& f) {
        return f.motion.step(2, 0) == 0 && f.motion.step(2, 1) == 0 && f.motion.step(2, 2) == 1;
    });
    const size_t paths = scanline ? static_cast<size_t>(horizontal ? columns : rows) : (affine ? 3 : count);
    std::vector<std::vector<Point>> positions(paths, std::vector<Point>(n));
    for (size_t k = 0; k < n; ++k) {
        const auto& frame = frames[first + k];
        times[k] = frame.seconds;
        confidence[k] =
            k == 0 ? 1 : std::clamp(scanline ? frame.motion.rs_confidence : frame.motion.confidence, 0.0, 1.0);
        const bool observed = k > 0 && (scanline ? frame.motion.state == MotionState::tracked
                                                 : frame.motion.state != MotionState::missing);
        for (size_t vertex = 0; vertex < paths; ++vertex) {
            Point delta;
            if (observed) {
                const size_t grid_row = vertex / columns;
                const double x = affine ? (vertex == 1 ? 1.0 : 0.0) : double(vertex % columns) / (columns - 1);
                const double y = affine ? (vertex == 2 ? 1.0 : 0.0) : double(grid_row) / (rows - 1);
                const Point p(x * (size.width - 1), y * (size.height - 1));
                delta = scanline ? row_motion(frame.motion, double(vertex) / (paths - 1))
                                 : transform(frame.motion.step, p) - p;
                if (!finite(delta))
                    throw std::runtime_error("Nonfinite observed mesh motion");
            }
            positions[vertex][k] = k ? positions[vertex][k - 1] + delta : Point{};
        }
    }
    // The cascade's exact support is six observations per pass.
    const size_t pad = std::min(n - 1, size_t{6} * (trajectory_smoothing_passes / 2));
    // About three times the cascade's effective half width.
    const double fit_seconds = 3 * seconds;
    size_t head = 1, tail = n - 1;
    while (head < n && times[head] - times[0] <= fit_seconds)
        ++head;
    while (tail > 0 && times[n - 1] - times[tail - 1] <= fit_seconds)
        --tail;
    // Extended sample k maps to original index source[k], mirrored when outside
    // the shot. The confidence of a step is that of the observation it spans.
    const size_t total = n + 2 * pad;
    std::vector<double> time(total), step_confidence(total);
    for (size_t k = 0; k < total; ++k) {
        if (k < pad) {
            const size_t j = pad - k;
            time[k] = 2 * times[0] - times[j];
            step_confidence[k] = k == 0 ? 1 : confidence[j + 1];
        } else if (k < pad + n) {
            time[k] = times[k - pad];
            step_confidence[k] = k == pad ? (pad ? confidence[1] : 1) : confidence[k - pad];
        } else {
            const size_t j = n - 1 - (k - pad - n + 1);
            time[k] = 2 * times[n - 1] - times[j];
            step_confidence[k] = confidence[j + 1];
        }
    }
    std::vector<Point> extended(total), forward(total);
    for (size_t vertex = 0; vertex < paths; ++vertex) {
        const auto& x = positions[vertex];
        const Point begin = fit_endpoint(times, confidence, x, 0, head, 0);
        const Point end = fit_endpoint(times, confidence, x, tail, n, n - 1);
        for (size_t k = 0; k < total; ++k)
            extended[k] = k < pad       ? begin * 2 - x[pad - k]
                          : k < pad + n ? x[k - pad]
                                        : end * 2 - x[n - 1 - (k - pad - n + 1)];
        SymmetricPath path;
        for (size_t k = 0; k < total; ++k)
            forward[k] =
                path.advance(k ? extended[k] - extended[k - 1] : Point{}, time[k], step_confidence[k], seconds);
        // Smooth S_f = X + forward in reversed time. The step from k+1 to k is
        // S_f[k] - S_f[k+1]; the result is relative to S_f, so both add up.
        SymmetricPath reverse;
        for (size_t k = total; k-- > 0;) {
            const bool last_sample = k + 1 == total;
            const Point delta = last_sample ? Point{} : extended[k] + forward[k] - extended[k + 1] - forward[k + 1];
            const Point correction =
                forward[k] + reverse.advance(delta, -time[k], last_sample ? 1 : step_confidence[k + 1], seconds);
            if (k >= pad && k < pad + n)
                output[first + k - pad][vertex] = correction;
        }
    }
    for (size_t i = first; i < last; ++i) {
        auto& mesh = output[i];
        const auto anchors = mesh;
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < columns; ++x) {
                const size_t vertex = static_cast<size_t>(y) * columns + x;
                if (scanline)
                    mesh[vertex] = anchors[horizontal ? x : y];
                else if (affine)
                    mesh[vertex] = anchors[0] + (anchors[1] - anchors[0]) * (double(x) / (columns - 1)) +
                                   (anchors[2] - anchors[0]) * (double(y) / (rows - 1));
            }
    }
}

} // namespace

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size size, const StabilizationSettings& options,
                             bool horizontal) {
    if (size.width < 2 || size.height < 2 || options.bands < 4 || options.bands > 32)
        throw std::invalid_argument("Invalid trajectory geometry");
    std::vector<Warp> output(frames.size());
    const int columns = horizontal ? options.bands : 9, rows = horizontal ? 9 : options.bands;
    const int count = columns * rows;
    std::vector<std::vector<Point>> global(frames.size()), local(frames.size());
    for (size_t first = 0; first < frames.size();) {
        size_t last = first + 1;
        if (!std::isfinite(frames[first].seconds))
            throw std::invalid_argument("Trajectory timestamps must increase within a shot");
        while (last < frames.size() && frames[last].segment == frames[last - 1].segment && !frames[last].motion.reset) {
            if (!std::isfinite(frames[last].seconds) || frames[last].seconds <= frames[last - 1].seconds)
                throw std::invalid_argument("Trajectory timestamps must increase within a shot");
            ++last;
        }
        plan_paths(global, frames, first, last, size, columns, rows, options.smooth_seconds, false, horizontal);
        plan_paths(local, frames, first, last, size, columns, rows, options.rolling_shutter ? options.rs_seconds : 0,
                   true, horizontal);
        first = last;
    }
    double missing_seconds = 0, previous_tracking = 1;
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        const auto& motion = frame.motion;
        const bool start = i == 0 || frame.segment != frames[i - 1].segment || motion.reset;
        if (!std::isfinite(frame.seconds) || (!start && frame.seconds <= frames[i - 1].seconds))
            throw std::invalid_argument("Trajectory timestamps must increase within a shot");
        if (start) {
            missing_seconds = 0;
            previous_tracking = 1;
        }
        const double dt = start ? std::max(0.0, frame.duration) : frame.seconds - frames[i - 1].seconds;
        const bool missing = motion.state == MotionState::missing;
        auto& warp = output[i];
        warp.columns = columns;
        warp.rows = rows;
        warp.displacement.resize(count);
        double row_energy = 0;
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < columns; ++x) {
                const int vertex = y * columns + x;
                const auto g = global[i][vertex];
                const auto r = local[i][vertex] * options.rs_strength;
                warp.displacement[vertex] = g + r;
                warp.global_offset = warp.global_offset + g * (1.0 / count);
                row_energy += r.dot(r);
            }
        warp.row_correction_rms = std::sqrt(row_energy / count);
        missing_seconds = missing ? missing_seconds + dt : 0;
        warp.tracking_strength = std::min(std::max(0.0, 1 - missing_seconds / 0.25), previous_tracking + dt / 0.25);
        previous_tracking = warp.tracking_strength;
    }
    plan_crops(frames, output, size, options);
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
    const auto window = warp.window.scale == 0 ? CropWindow{crop, crop, 1 - 2 * crop} : warp.window;
    if (!std::isfinite(window.left) || !std::isfinite(window.top) || !std::isfinite(window.scale) ||
        window.scale <= 0 || window.scale > 1 || window.left < 0 || window.top < 0 ||
        window.left + window.scale > 1 + 1e-12 || window.top + window.scale > 1 + 1e-12)
        throw std::invalid_argument("Invalid crop window");
    map.origin_x = window.left * (output.width - 1);
    map.origin_y = window.top * (output.height - 1);
    map.step_x = map.step_y = window.scale;
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
