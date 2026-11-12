#include "stillflow/domain/motion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
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
struct PathKernel {
    std::array<double, 6> weights{};
    double denominator = 1;
    size_t count = 0;
};
std::vector<PathKernel> kernels(const std::vector<double>& time, const std::vector<double>& confidence, double seconds,
                                bool reverse) {
    std::vector<PathKernel> result(time.size());
    const double window = 0.2 * seconds / 0.45;
    for (size_t k = 0; k < time.size(); ++k) {
        auto& kernel = result[k];
        kernel.count = std::min(size_t{6}, reverse ? time.size() - 1 - k : k);
        for (size_t lag = 0; lag < kernel.count; ++lag) {
            const size_t j = reverse ? k + lag + 1 : k - lag - 1;
            const double age = time[k] - time[j];
            const double weight = reverse ? (j + 1 == time.size() ? 1 : confidence[j + 1]) : confidence[j];
            kernel.weights[lag] = std::exp(-9 * age * age / (window * window)) * std::clamp(weight, 0.0, 1.0);
            kernel.denominator += kernel.weights[lag];
        }
    }
    return result;
}
template <size_t Passes> class CausalPath {
  public:
    Point advance(Point delta, const PathKernel& kernel) {
        Point correction;
        std::array<size_t, 6> indices{};
        for (size_t lag = 0; lag < kernel.count; ++lag)
            indices[lag] = (cursor_ + 5 - lag) % 6;
        const double reciprocal = 1 / kernel.denominator;
        for (auto& pass : passes_) {
            const Point input = correction;
            for (size_t lag = 0; lag < kernel.count; ++lag) {
                auto& position = pass[indices[lag]];
                position = position - delta;
                correction = correction + position * kernel.weights[lag];
            }
            correction = correction * reciprocal;
            pass[cursor_] = input;
        }
        cursor_ = (cursor_ + 1) % 6;
        if (!finite(correction))
            throw std::runtime_error("Nonfinite mesh trajectory");
        return correction;
    }

  private:
    size_t cursor_ = 0;
    std::array<std::array<Point, 6>, Passes> passes_{};
};

// Half of the cascade runs forward and half backward. Each pass applies the
// same kernel, so |H_forward * H_backward| equals the causal |H_50| while the
// opposite group delays cancel: offline planning keeps the full smoothing
// strength without lagging behind intentional camera motion.
// A single temporal kernel preserves the global similarity model. Adapting
// independently to each vertex's speed bends a rigid rotation into a mesh warp.
using SymmetricPath = CausalPath<trajectory_smoothing_passes / 2>;
static_assert(trajectory_smoothing_passes % 2 == 0);
struct PathCache {
    double seconds = 0;
    int segment = -1;
    bool scanline = false;
    std::vector<double> time, confidence;
    std::vector<std::vector<Point>> delta, forward;
    std::vector<SymmetricPath> snapshot;
    size_t at = 0;
};
} // namespace
struct TrajectoryCache::Impl {
    std::vector<PathCache> paths;
    size_t cursor = 0;
};
TrajectoryCache::TrajectoryCache() : impl(std::make_unique<Impl>()) {}
TrajectoryCache::~TrajectoryCache() = default;
namespace {
PathCache* cache_slot(TrajectoryCache* cache) {
    if (!cache)
        return nullptr;
    auto& state = *cache->impl;
    if (state.cursor == state.paths.size())
        state.paths.emplace_back();
    return &state.paths[state.cursor++];
}

// Confidence-weighted linear fit of one vertex path near a shot end, evaluated
// at that end. Pivoting the reflection on the raw end sample would mirror its
// shake into the padding and pull the boundary frames by twice that amount.
Point fit_endpoint(const std::vector<double>& times, const std::vector<double>& weights,
                   const std::vector<Point>& positions, size_t first, size_t last, size_t end, bool scanline) {
    const double at = times[end];
    double w = 0, t = 0, tt = 0;
    Point x, tx;
    for (size_t k = first; k < last; ++k) {
        const double dt = times[k] - at;
        double weight = weights[k];
        // A rectangular fit leaks the phase of boundary wobble into its
        // intercept. Taper scanline observations at both ends of the fit so
        // oscillation cancels without anchoring the target to the first row
        // shape. The linear model still preserves steady parallax drift.
        if (scanline && last - first >= 6) {
            const double phase = (times[k] - times[first]) / (times[last - 1] - times[first]);
            const double taper = std::sin(std::numbers::pi * phase);
            weight *= taper * taper;
        }
        w += weight;
        t += weight * dt;
        tt += weight * dt * dt;
        x = x + positions[k] * weight;
        tx = tx + positions[k] * (weight * dt);
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
void plan_paths(std::vector<std::vector<Point>>& output, std::span<const FrameMotion> frames, size_t first, size_t last,
                Size size, int columns, int rows, double seconds, bool scanline, bool horizontal, PathCache* cache,
                size_t output_first, size_t output_last) {
    const size_t n = last - first, count = static_cast<size_t>(columns) * rows;
    for (size_t i = std::max(first, output_first); i < std::min(last, output_last); ++i)
        output[i].assign(count, Point{});
    if (seconds <= 0 || n < 2)
        return;
    // Observation into each frame; the shot's first frame has none.
    std::vector<double> times(n), confidence(n);
    // The temporal kernel is shared: affine global motion needs only three
    // anchors, and scanline motion has one independent path per band.
    const bool affine = std::all_of(frames.begin() + first, frames.begin() + last, [](const auto& f) {
        return f.motion.step(2, 0) == 0 && f.motion.step(2, 1) == 0 && std::isfinite(f.motion.step(2, 2)) &&
               f.motion.step(2, 2) != 0;
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
    const auto forward_kernels = kernels(time, step_confidence, seconds, false);
    const auto reverse_kernels = kernels(time, step_confidence, seconds, true);
    PathCache next;
    if (cache) {
        next.seconds = seconds;
        next.segment = frames[first].segment;
        next.scanline = scanline;
        next.time = time;
        next.confidence = step_confidence;
        next.at = pad + n - 2;
        next.delta.resize(paths);
        next.forward.resize(paths);
        next.snapshot.resize(paths);
    }
    std::vector<Point> extended(total), forward(total), delta(total);
    for (size_t vertex = 0; vertex < paths; ++vertex) {
        const auto& x = positions[vertex];
        const Point begin = fit_endpoint(times, confidence, x, 0, head, 0, scanline);
        const Point end = fit_endpoint(times, confidence, x, tail, n, n - 1, scanline);
        for (size_t k = 0; k < total; ++k)
            extended[k] = k < pad       ? begin * 2 - x[pad - k]
                          : k < pad + n ? x[k - pad]
                                        : end * 2 - x[n - 1 - (k - pad - n + 1)];
        if (scanline && n >= 6) {
            // Reflected pairs cancel their wobble around the fitted endpoint;
            // the unpaired raw endpoint would leave an impulse in the target.
            // Filter the fitted value here, then express the correction relative
            // to the actual observation below so the first frame is corrected too.
            extended[pad] = begin;
            extended[pad + n - 1] = end;
        }
        for (size_t k = 0; k < total; ++k)
            delta[k] = k ? extended[k] - extended[k - 1] : Point{};
        size_t reuse_begin = total, reuse_end = total, old_begin = 0;
        if (cache && cache->seconds == seconds && cache->segment == frames[first].segment &&
            cache->scanline == scanline && cache->delta.size() == paths && !cache->time.empty()) {
            size_t streak = 0, previous = 0;
            for (size_t k = 0; k < total; ++k) {
                const auto found = std::lower_bound(cache->time.begin(), cache->time.end(), time[k]);
                const size_t j = static_cast<size_t>(found - cache->time.begin());
                const bool same = found != cache->time.end() && *found == time[k] &&
                                  cache->confidence[j] == step_confidence[k] &&
                                  norm(cache->delta[vertex][j] - delta[k]) < 1e-10;
                streak = same ? (streak && j == previous + 1 ? streak + 1 : 1) : 0;
                previous = j;
                // The state retains six samples per pass, plus the six kernel observations.
                if (streak >= size_t{6} * (trajectory_smoothing_passes / 2 + 1) && j <= cache->at) {
                    if (reuse_begin == total) {
                        reuse_begin = k;
                        old_begin = j;
                    }
                    if (j == cache->at && k <= next.at) {
                        reuse_end = k;
                        break;
                    }
                } else {
                    reuse_begin = total;
                }
            }
            if (reuse_end == total)
                reuse_begin = total;
        }
        SymmetricPath path;
        for (size_t k = 0; k < total; ++k) {
            if (k == reuse_begin) {
                for (size_t i = k; i <= reuse_end; ++i)
                    forward[i] = cache->forward[vertex][old_begin + i - k];
                path = cache->snapshot[vertex];
                k = reuse_end;
            } else
                forward[k] = path.advance(delta[k], forward_kernels[k]);
            if (cache && k == next.at)
                next.snapshot[vertex] = path;
        }
        if (cache) {
            next.delta[vertex] = delta;
            next.forward[vertex] = forward;
        }
        // Smooth S_f = X + forward in reversed time. The step from k+1 to k is
        // S_f[k] - S_f[k+1]; the result is relative to S_f, so both add up.
        SymmetricPath reverse;
        const size_t reverse_end = pad + (std::max(first, output_first) - first);
        for (size_t k = total; k-- > reverse_end;) {
            const bool last_sample = k + 1 == total;
            const Point delta = last_sample ? Point{} : extended[k] + forward[k] - extended[k + 1] - forward[k + 1];
            const Point correction = forward[k] + reverse.advance(delta, reverse_kernels[k]);
            if (k >= pad && k < pad + n && first + k - pad >= output_first && first + k - pad < output_last)
                output[first + k - pad][vertex] = correction + (extended[k] - x[k - pad]);
        }
    }
    if (cache)
        *cache = std::move(next);
    for (size_t i = std::max(first, output_first); i < std::min(last, output_last); ++i) {
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

std::vector<Warp> plan_warps(std::span<const FrameMotion> frames, Size size, const StabilizationSettings& options,
                             bool horizontal, bool first_shot_start, TrajectoryCache* cache, size_t output_first,
                             size_t output_last, bool plan_viewports) {
    if (size.width < 2 || size.height < 2 || options.bands < 4 || options.bands > 32)
        throw std::invalid_argument("Invalid trajectory geometry");
    output_last = std::min(output_last, frames.size());
    if (output_first > output_last)
        throw std::invalid_argument("Invalid planning range");
    if (cache)
        cache->impl->cursor = 0;
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
        plan_paths(global, frames, first, last, size, columns, rows, options.smooth_seconds, false, horizontal,
                   cache_slot(cache), output_first, output_last);
        plan_paths(local, frames, first, last, size, columns, rows, options.rolling_shutter ? options.rs_seconds : 0,
                   true, horizontal, cache_slot(cache), output_first, output_last);
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
        missing_seconds = missing ? missing_seconds + dt : 0;
        const double tracking = std::min(std::max(0.0, 1 - missing_seconds / 0.25), previous_tracking + dt / 0.25);
        previous_tracking = tracking;
        if (i < output_first || i >= output_last)
            continue;
        auto& warp = output[i];
        warp.tracking_strength = tracking;
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
    }
    if (!plan_viewports) {
        if (cache)
            cache->impl->paths.resize(cache->impl->cursor);
        return output;
    }
    plan_crops(frames.subspan(output_first, output_last - output_first),
               std::span<Warp>(output).subspan(output_first, output_last - output_first), size, options);
    // A fast opening pan can exceed the crop budget of the extrapolated
    // target. Scaling the whole correction then reintroduces frame-to-frame
    // shake throughout the eroded crop-cap neighborhood. During endpoint
    // initialization, relax the target's temporal scale instead: follow the
    // fast trend while still filtering shake, then recheck every crop bound.
    std::vector<double> startup_blend(frames.size());
    double shot_start = frames.empty() ? 0 : frames.front().seconds;
    bool initialize = first_shot_start;
    for (size_t i = 0; i < frames.size() && options.smooth_seconds > 0 && options.crop > 0; ++i) {
        if (frames[i].motion.reset || (i > 0 && frames[i].segment != frames[i - 1].segment)) {
            shot_start = frames[i].seconds;
            initialize = true;
        }
        if (!initialize || i < output_first || i >= output_last)
            continue;
        const auto& w = output[i];
        // Tracking loss keeps its existing fade; this only responds to crop pressure.
        if (w.tracking_strength < 0.99 || w.window.scale > 1 - 2 * options.crop + 1e-6)
            continue;
        const double pressure = std::clamp(2 * (1 - w.smoothed_crop_strength), 0.0, 1.0);
        const double age = frames[i].seconds - shot_start;
        // Match the endpoint fit horizon, fading over its final smoothing scale.
        const double startup = std::clamp((3 * options.smooth_seconds - age) / options.smooth_seconds, 0.0, 1.0);
        startup_blend[i] = pressure * startup;
    }
    if (std::any_of(startup_blend.begin(), startup_blend.end(), [](double blend) { return blend > 0.02; })) {
        std::vector<std::vector<Point>> responsive(frames.size());
        for (size_t first = 0; first < frames.size();) {
            size_t last = first + 1;
            while (last < frames.size() && frames[last].segment == frames[last - 1].segment &&
                   !frames[last].motion.reset)
                ++last;
            if (std::any_of(startup_blend.begin() + first, startup_blend.begin() + last,
                            [](double blend) { return blend > 0; }))
                plan_paths(responsive, frames, first, last, size, columns, rows, options.smooth_seconds * 0.4, false,
                           horizontal, cache_slot(cache), output_first, output_last);
            first = last;
        }
        for (size_t i = 0; i < output.size(); ++i) {
            if (startup_blend[i] == 0)
                continue;
            auto& w = output[i];
            for (int vertex = 0; vertex < count; ++vertex) {
                const auto adjustment = (responsive[i][vertex] - global[i][vertex]) * startup_blend[i];
                w.displacement[vertex] = w.displacement[vertex] + adjustment;
                w.global_offset = w.global_offset + adjustment * (1.0 / count);
            }
        }
        plan_crops(frames.subspan(output_first, output_last - output_first),
                   std::span<Warp>(output).subspan(output_first, output_last - output_first), size, options);
    }
    if (cache)
        cache->impl->paths.resize(cache->impl->cursor);
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
