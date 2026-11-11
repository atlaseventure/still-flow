#include "stillflow/domain/motion.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace stillflow {
namespace {
struct Bounds {
    double left = 0, top = 0, right = 1, bottom = 1;
    void intersect(const Bounds& b) {
        left = std::max(left, b.left);
        top = std::max(top, b.top);
        right = std::min(right, b.right);
        bottom = std::min(bottom, b.bottom);
    }
    double scale() const { return std::min(right - left, bottom - top); }
    bool contains(CropWindow w) const {
        return w.left >= left && w.top >= top && w.left + w.scale <= right && w.top + w.scale <= bottom;
    }
};

// The inset source rectangle has a simple, unfolded image under a contractive
// bilinear mesh. Each boundary is piecewise linear, so its extrema occur at
// mesh intersections. Keeping the viewport beyond all four inward extrema is
// a conservative containment certificate, without image-color heuristics.
struct Edges {
    double left = 0, top = 0, right = 0, bottom = 0, pad_x = 0, pad_y = 0;
    bool identity = true;
    Bounds at(double strength) const {
        if (identity || strength == 0)
            return {};
        return {std::max(0.0, pad_x + strength * left), std::max(0.0, pad_y + strength * top),
                std::min(1.0, 1 - pad_x + strength * right), std::min(1.0, 1 - pad_y + strength * bottom)};
    }
};

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

Edges measure_edges(const Warp& warp, Size size) {
    Edges e;
    const double width = size.width - 1, height = size.height - 1;
    // Half an analysis pixel covers interpolation/float roundoff at native
    // resolution, including the bounded error of eight inverse iterations.
    constexpr double pad = 0.5;
    e.pad_x = pad / width;
    e.pad_y = pad / height;
    for (const auto p : warp.displacement)
        e.identity = e.identity && p.x == 0 && p.y == 0;
    const auto displacement = [&](double x, double y) {
        const double gx = x * (warp.columns - 1) / width, gy = y * (warp.rows - 1) / height;
        const int ix = std::min(static_cast<int>(gx), warp.columns - 2);
        const int iy = std::min(static_cast<int>(gy), warp.rows - 2);
        const double fx = gx - ix, fy = gy - iy;
        const int a = iy * warp.columns + ix, b = a + warp.columns;
        return (warp.displacement[a] * (1 - fx) + warp.displacement[a + 1] * fx) * (1 - fy) +
               (warp.displacement[b] * (1 - fx) + warp.displacement[b + 1] * fx) * fy;
    };
    e.left = e.top = -1e100;
    e.right = e.bottom = 1e100;
    for (int y = 0; y < warp.rows; ++y) {
        const double py = std::clamp(y * height / (warp.rows - 1), pad, height - pad);
        e.left = std::max(e.left, displacement(pad, py).x / width);
        e.right = std::min(e.right, displacement(width - pad, py).x / width);
    }
    for (int x = 0; x < warp.columns; ++x) {
        const double px = std::clamp(x * width / (warp.columns - 1), pad, width - pad);
        e.top = std::max(e.top, displacement(px, pad).y / height);
        e.bottom = std::min(e.bottom, displacement(px, height - pad).y / height);
    }
    return e;
}

std::pair<size_t, size_t> neighborhood(std::span<const FrameMotion> frames, size_t i, double radius) {
    const auto lower = std::lower_bound(frames.begin(), frames.end(), frames[i].seconds - radius,
                                        [](const auto& f, double t) { return f.seconds < t; });
    const auto upper = std::upper_bound(frames.begin(), frames.end(), frames[i].seconds + radius,
                                        [](double t, const auto& f) { return t < f.seconds; });
    return {static_cast<size_t>(lower - frames.begin()), static_cast<size_t>(upper - frames.begin())};
}

double weight(const FrameMotion& f, double time, double radius) {
    return (1 + std::cos(std::numbers::pi * std::clamp((f.seconds - time) / radius, -1.0, 1.0))) *
           std::max(1e-6, f.duration);
}

std::vector<double> smooth_caps(std::span<const FrameMotion> frames, const std::vector<double>& caps) {
    // Erode before averaging: every value in the averaging window is <= the
    // current cap. This anticipates both decreases and recoveries without
    // exceeding geometric/tracking limits, including at shot boundaries.
    constexpr double radius = 0.25;
    std::vector<double> floor(caps.size()), result(caps.size());
    for (size_t i = 0; i < caps.size(); ++i) {
        const auto [first, last] = neighborhood(frames, i, radius);
        floor[i] = *std::min_element(caps.begin() + first, caps.begin() + last);
    }
    for (size_t i = 0; i < caps.size(); ++i) {
        const auto [first, last] = neighborhood(frames, i, radius);
        double sum = 0, total = 0;
        for (size_t j = first; j < last; ++j) {
            const double w = weight(frames[j], frames[i].seconds, radius);
            sum += floor[j] * w;
            total += w;
        }
        result[i] = std::min(caps[i], sum / total);
    }
    return result;
}

Bounds common_bounds(std::span<const Edges> edges, std::span<const double> strengths, double fraction) {
    Bounds common;
    for (size_t i = 0; i < edges.size(); ++i)
        common.intersect(edges[i].at(strengths[i] * fraction));
    return common;
}

CropWindow choose_window(std::span<const Edges> edges, std::span<const double> strengths, double minimum_scale) {
    auto common = common_bounds(edges, strengths, 1);
    if (common.scale() < minimum_scale) {
        double low = 0, high = 1;
        for (int iteration = 0; iteration < 20; ++iteration) {
            const double middle = (low + high) * 0.5;
            if (common_bounds(edges, strengths, middle).scale() >= minimum_scale)
                low = middle;
            else
                high = middle;
        }
        common = common_bounds(edges, strengths, low);
    }
    const double scale = std::clamp(common.scale(), minimum_scale, 1.0);
    const double centered = (1 - scale) * 0.5;
    return {std::clamp(centered, common.left, std::max(common.left, common.right - scale)),
            std::clamp(centered, common.top, std::max(common.top, common.bottom - scale)), scale};
}

std::vector<CropWindow> adaptive_windows(std::span<const FrameMotion> frames, std::span<const Edges> edges,
                                         std::span<const double> strengths, CropWindow fixed, double minimum_scale) {
    std::vector<CropWindow> target(frames.size()), result(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto [first, last] = neighborhood(frames, i, 0.5);
        target[i] =
            choose_window(edges.subspan(first, last - first), strengths.subspan(first, last - first), minimum_scale);
    }
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto [first, last] = neighborhood(frames, i, 1.0);
        double total = 0;
        for (size_t j = first; j < last; ++j) {
            const double w = weight(frames[j], frames[i].seconds, 1.0);
            result[i].left += target[j].left * w;
            result[i].top += target[j].top * w;
            result[i].scale += target[j].scale * w;
            total += w;
        }
        result[i].left /= total;
        result[i].top /= total;
        result[i].scale /= total;
    }
    // A shared blend toward a fixed viewport bounds every velocity and
    // acceleration, preserving convex viewport/zoom-budget constraints.
    double blend = 1;
    std::array<double, 3> previous{};
    double previous_dt = 0;
    const auto coordinates = [](CropWindow w) {
        return std::array{w.left + w.scale * 0.5, w.top + w.scale * 0.5, w.scale};
    };
    for (size_t i = 1; i < result.size(); ++i) {
        const double dt = frames[i].seconds - frames[i - 1].seconds;
        const auto a = coordinates(result[i - 1]), b = coordinates(result[i]);
        for (size_t axis = 0; axis < 3; ++axis) {
            const double speed_limit = axis == 2 ? 0.04 : 0.03;
            const double speed = (b[axis] - a[axis]) / dt;
            blend = std::min(blend, speed_limit / std::max(1e-12, std::abs(speed)));
            if (i > 1) {
                const double acceleration = 2 * (speed - previous[axis]) / (dt + previous_dt);
                blend = std::min(blend, 2 * speed_limit / std::max(1e-12, std::abs(acceleration)));
            }
            previous[axis] = speed;
        }
        previous_dt = dt;
    }
    for (auto& w : result) {
        w.scale = std::clamp(fixed.scale + blend * (w.scale - fixed.scale), minimum_scale, 1.0);
        w.left = std::clamp(fixed.left + blend * (w.left - fixed.left), 0.0, 1 - w.scale);
        w.top = std::clamp(fixed.top + blend * (w.top - fixed.top), 0.0, 1 - w.scale);
    }
    return result;
}

void plan_shot(std::span<const FrameMotion> frames, std::span<Warp> warps, Size size,
               const StabilizationSettings& options) {
    std::vector<Edges> edges;
    std::vector<double> base;
    for (auto& w : warps) {
        // Validate before indexing the mesh; the returned map is unused.
        w.window = {};
        prepare_map(w, size, size, 0, false);
        const double bound = gradient_bound(w, size);
        w.geometry_strength = bound > 0.25 ? 0.25 / bound : 1;
        edges.push_back(measure_edges(w, size));
        base.push_back(std::min(w.geometry_strength, w.tracking_strength));
    }
    base = smooth_caps(frames, base);
    const double minimum_scale = 1 - 2 * options.crop;
    const auto fixed = options.crop_mode == CropMode::fixed ? CropWindow{options.crop, options.crop, minimum_scale}
                                                            : choose_window(edges, base, minimum_scale);
    const auto windows = options.crop_mode == CropMode::adaptive
                             ? adaptive_windows(frames, edges, base, fixed, minimum_scale)
                             : std::vector<CropWindow>(warps.size(), fixed);
    std::vector<double> crop_caps(base.size());
    for (size_t i = 0; i < warps.size(); ++i) {
        auto& w = warps[i];
        w.window = windows[i];
        double low = 1;
        if (!edges[i].at(base[i]).contains(w.window)) {
            low = 0;
            double high = 1;
            for (int iteration = 0; iteration < 20; ++iteration) {
                const double middle = (low + high) * 0.5;
                if (edges[i].at(base[i] * middle).contains(w.window))
                    low = middle;
                else
                    high = middle;
            }
        }
        w.crop_strength = low;
        crop_caps[i] = low;
    }
    // Smooth each independent constraint once. Smoothing the already-smoothed
    // base again would unnecessarily delay recovery even when cropping fits.
    const auto fractions = smooth_caps(frames, crop_caps);
    for (size_t i = 0; i < warps.size(); ++i) {
        warps[i].smoothed_crop_strength = fractions[i];
        warps[i].strength = warps[i].planned_strength = base[i] * fractions[i];
        warps[i].render_strength = 1;
        warps[i].boundary_retries = 0;
        warps[i].boundary_invalid_fraction = 0;
    }
}
} // namespace

void StreamCrop::apply(Warp& warp, std::span<const Warp> future, Size size, const StabilizationSettings& settings,
                       double dt, bool reset) {
    const double minimum_scale = 1 - 2 * settings.crop;
    CropWindow desired{settings.crop, settings.crop, minimum_scale};
    if (settings.crop_mode != CropMode::fixed) {
        std::vector<Edges> edges;
        std::vector<double> strengths;
        for (const auto& item : future) {
            edges.push_back(measure_edges(item, size));
            strengths.push_back(std::min(item.geometry_strength, item.tracking_strength));
        }
        desired = choose_window(edges, strengths, minimum_scale);
    }
    if (reset || !initialized_) {
        target_ = window_ = desired;
        strength_ = 1;
        initialized_ = true;
    }
    // Range <= 2*crop on each coordinate: tau >= range / velocity,
    // tau >= sqrt(2*range / acceleration). Convex updates cannot overshoot.
    const double tau = std::max({0.5, 2 * settings.crop / 0.02, std::sqrt(4 * settings.crop / 0.04)});
    const double alpha = dt / (tau + dt);
    const auto blend = [alpha](CropWindow a, CropWindow b) {
        return CropWindow{a.left + alpha * (b.left - a.left), a.top + alpha * (b.top - a.top),
                          a.scale + alpha * (b.scale - a.scale)};
    };
    target_ = blend(target_, desired);
    window_ = blend(window_, target_);
    warp.window = window_;
    const auto edges = measure_edges(warp, size);
    const double base = std::min(warp.geometry_strength, warp.tracking_strength);
    double low = 0, high = 1;
    if (edges.at(base).contains(window_))
        low = 1;
    else
        for (int iteration = 0; iteration < 20; ++iteration) {
            const double middle = (low + high) * 0.5;
            if (edges.at(base * middle).contains(window_))
                low = middle;
            else
                high = middle;
        }
    warp.crop_strength = warp.smoothed_crop_strength = low;
    // Safety reductions take effect immediately; recovery is rate limited.
    strength_ = std::min(base * low, strength_ + dt / 0.25);
    warp.strength = warp.planned_strength = strength_;
}

void plan_crops(std::span<const FrameMotion> frames, std::span<Warp> warps, Size size,
                const StabilizationSettings& options) {
    if (frames.size() != warps.size() || size.width < 2 || size.height < 2 || !std::isfinite(options.crop) ||
        options.crop < 0 || options.crop > 0.25 ||
        (options.crop_mode != CropMode::automatic && options.crop_mode != CropMode::fixed &&
         options.crop_mode != CropMode::adaptive))
        throw std::invalid_argument("Invalid crop planning geometry");
    size_t first = 0;
    for (size_t i = 0; i < frames.size(); ++i) {
        const bool start = i == 0 || frames[i].segment != frames[i - 1].segment || frames[i].motion.reset;
        if (!std::isfinite(frames[i].seconds) || !std::isfinite(frames[i].duration) || frames[i].duration < 0 ||
            (!start && frames[i].seconds <= frames[i - 1].seconds))
            throw std::invalid_argument("Invalid crop planning timestamps");
        if (start && i > first) {
            plan_shot(frames.subspan(first, i - first), warps.subspan(first, i - first), size, options);
            first = i;
        }
    }
    if (first < frames.size())
        plan_shot(frames.subspan(first), warps.subspan(first), size, options);
}
} // namespace stillflow
