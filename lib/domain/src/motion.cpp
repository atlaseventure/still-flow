#include "stillflow/domain/motion.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>

namespace stillflow {
namespace {
struct BandPosition {
    int low;
    double fraction;
};
BandPosition band_position(double coordinate, double extent, int bands) {
    const double position = std::clamp(coordinate / (extent - 1), 0.0, 1.0) * (bands - 1);
    const int low = std::min(static_cast<int>(position), bands - 2);
    return {low, position - low};
}
Point interpolate(const std::vector<Point>& values, Point p, Size size, bool horizontal) {
    if (values.empty())
        return {};
    const auto [low, t] =
        band_position(horizontal ? p.x : p.y, horizontal ? size.width : size.height, static_cast<int>(values.size()));
    return values[low] * (1 - t) + values[low + 1] * t;
}
// The constant/linear null space preserves stationary views and deliberate pans.
// A sparse second-derivative penalty also supports irregular presentation times.
Eigen::MatrixXd smooth(const Eigen::MatrixXd& path, const std::vector<double>& times,
                       const std::vector<double>& confidence, double seconds) {
    const int n = static_cast<int>(path.rows());
    if (n < 3 || seconds <= 0)
        return path;
    using Triplet = Eigen::Triplet<double>;
    std::vector<Triplet> entries;
    entries.reserve(static_cast<size_t>(n) * 10);
    Eigen::MatrixXd rhs = path;
    for (int i = 0; i < n; ++i) {
        const double weight = std::max(0.15, confidence[i]);
        entries.emplace_back(i, i, weight);
        rhs.row(i) *= weight;
    }
    const double lambda = std::pow(seconds / (2 * std::numbers::pi), 4);
    for (int i = 1; i + 1 < n; ++i) {
        const double before = std::max(0.001, times[i] - times[i - 1]);
        const double after = std::max(0.001, times[i + 1] - times[i]);
        const double scale = 2 / (before + after);
        const std::array<double, 3> d{scale / before, -scale * (1 / before + 1 / after), scale / after};
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                entries.emplace_back(i + a - 1, i + b - 1, lambda * d[a] * d[b]);
    }
    Eigen::SparseMatrix<double> system(n, n);
    system.setFromTriplets(entries.begin(), entries.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(system);
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("Trajectory factorization failed");
    Eigen::MatrixXd result = solver.solve(rhs);
    if (solver.info() != Eigen::Success || !result.allFinite())
        throw std::runtime_error("Trajectory optimization failed");
    return result;
}

Matrix3 scaled_global(const Warp& warp, Size size, double strength) {
    const Point center((size.width - 1) * 0.5, (size.height - 1) * 0.5);
    const auto shift = transform(warp.global, center) - center;
    const double angle = std::atan2(warp.global(1, 0), warp.global(0, 0)) * strength;
    const double scale = std::pow(std::hypot(warp.global(0, 0), warp.global(1, 0)), strength);
    const double a = scale * std::cos(angle), b = scale * std::sin(angle);
    return {a, -b, (1 - a) * center.x + b * center.y + strength * shift.x,
            b, a,  -b * center.x + (1 - a) * center.y + strength * shift.y,
            0, 0,  1};
}

Point inverse_point(Point target, const Matrix3& inverse, const Warp& warp, Size size, bool horizontal,
                    double strength) {
    auto p = transform(inverse, target);
    for (int iteration = 0; iteration < 5 && !warp.row_offset.empty(); ++iteration)
        p = transform(inverse, target - interpolate(warp.row_offset, p, size, horizontal) * strength);
    return p;
}

bool fits(const Warp& warp, Size size, double crop, bool horizontal, double strength) {
    const auto matrix = scaled_global(warp, size, strength);
    const auto inverse = matrix.inv();
    // Bound the derivative so inversion remains contractive and the warp cannot fold.
    if (!warp.row_offset.empty()) {
        const double spacing =
            static_cast<double>((horizontal ? size.width : size.height) - 1) / (warp.row_offset.size() - 1);
        for (size_t i = 1; i < warp.row_offset.size(); ++i)
            if (norm(warp.row_offset[i] - warp.row_offset[i - 1]) * strength / spacing > 0.20)
                return false;
    }
    const auto valid = [&](double x, double y) {
        const Point target((crop + x * (1 - 2 * crop)) * (size.width - 1),
                           (crop + y * (1 - 2 * crop)) * (size.height - 1));
        const auto p = inverse_point(target, inverse, warp, size, horizontal, strength);
        return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= 0.5 && p.x <= size.width - 1.5 && p.y >= 0.5 &&
               p.y <= size.height - 1.5;
    };
    for (int i = 0; i <= 64; ++i) {
        const double t = i / 64.0;
        if (!valid(t, 0) || !valid(t, 1) || !valid(0, t) || !valid(1, t))
            return false;
    }
    return true;
}
} // namespace

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size size, const StabilizationSettings& options,
                             bool horizontal, Profile& profile) {
    ScopeTimer timer(profile, "plan.optimize");
    std::vector<Warp> output(frames.size());
    const Point center((size.width - 1) * 0.5, (size.height - 1) * 0.5);
    for (size_t first = 0; first < frames.size();) {
        size_t last = first + 1;
        while (last < frames.size() && frames[last].segment == frames[first].segment)
            ++last;
        const int n = static_cast<int>(last - first);
        std::vector<Matrix3> cameras(n);
        Eigen::MatrixXd paths = Eigen::MatrixXd::Zero(n, 4);
        Eigen::MatrixXd rows = Eigen::MatrixXd::Zero(n, static_cast<Eigen::Index>(options.bands) * 2);
        std::vector<double> times(n), confidence(n);
        auto camera = Matrix3::eye();
        double last_angle = 0;
        for (int i = 0; i < n; ++i) {
            const auto& frame = frames[first + i];
            times[i] = frame.seconds;
            confidence[i] = i == 0 ? 1.0 : frame.motion.confidence;
            if (i)
                camera = frame.motion.step * camera;
            cameras[i] = camera;
            const auto position = transform(camera, center) - center;
            double angle = std::atan2(camera(1, 0), camera(0, 0));
            angle = last_angle + std::remainder(angle - last_angle, 2 * std::numbers::pi);
            last_angle = angle;
            paths.row(i) << position.x, position.y, angle, std::log(std::hypot(camera(0, 0), camera(1, 0)));
            if (i) {
                rows.row(i) = rows.row(i - 1);
                const auto inverse = camera.inv();
                for (Eigen::Index band = 0; band < options.bands; ++band) {
                    const auto d = frame.motion.row_delta[band];
                    rows(i, band * 2) += inverse(0, 0) * d.x + inverse(0, 1) * d.y;
                    rows(i, band * 2 + 1) += inverse(1, 0) * d.x + inverse(1, 1) * d.y;
                }
            }
        }
        const auto stable = smooth(paths, times, confidence, options.smooth_seconds);
        const auto stable_rows = smooth(rows, times, confidence, options.rs_seconds);
        for (int i = 0; i < n; ++i) {
            const double scale = std::exp(stable(i, 3));
            const double a = scale * std::cos(stable(i, 2)), b = scale * std::sin(stable(i, 2));
            const Matrix3 desired{a, -b, (1 - a) * center.x + b * center.y + stable(i, 0),
                                  b, a,  -b * center.x + (1 - a) * center.y + stable(i, 1),
                                  0, 0,  1};
            auto& warp = output[first + i];
            warp.global = desired * cameras[i].inv();
            if (options.rolling_shutter) {
                warp.row_offset.resize(options.bands);
                for (Eigen::Index band = 0; band < options.bands; ++band) {
                    const double dx = stable_rows(i, band * 2) - rows(i, band * 2);
                    const double dy = stable_rows(i, band * 2 + 1) - rows(i, band * 2 + 1);
                    warp.row_offset[band] = Point(a * dx - b * dy, b * dx + a * dy) * options.rs_strength;
                }
            }
            if (!fits(warp, size, options.crop, horizontal, 1)) {
                double low = 0, high = 1;
                for (int k = 0; k < 12; ++k) {
                    const double middle = (low + high) / 2;
                    if (fits(warp, size, options.crop, horizontal, middle))
                        low = middle;
                    else
                        high = middle;
                }
                warp.strength = low;
            }
        }
        // A bounded envelope eases into crop constraints without zoom breathing.
        for (size_t i = first + 1; i < last; ++i)
            output[i].strength = std::min(output[i].strength,
                                          output[i - 1].strength + (frames[i].seconds - frames[i - 1].seconds) / 0.25);
        for (size_t i = last - 1; i > first; --i)
            output[i - 1].strength = std::min(output[i - 1].strength,
                                              output[i].strength + (frames[i].seconds - frames[i - 1].seconds) / 0.25);
        first = last;
    }
    return output;
}

MapParameters prepare_map(const Warp& warp, Size size, Size output, double crop, bool horizontal) {
    MapParameters p;
    p.horizontal = horizontal;
    p.bands = static_cast<int>(warp.row_offset.size());
    if (p.bands == 1 || p.bands > 32)
        throw std::runtime_error("Scanline mapping requires zero or 2..32 bands");
    const auto inverse = scaled_global(warp, size, warp.strength).inv();
    const double sx = static_cast<double>(output.width - 1) / (size.width - 1);
    const double sy = static_cast<double>(output.height - 1) / (size.height - 1);
    // In global-inverse coordinates, the scan coordinate is a monotone piecewise
    // linear function. Invert that scalar function exactly, then subtract its
    // two-dimensional offset. No per-pixel iterative solver is needed.
    for (int i = 0; i < p.bands; ++i) {
        const auto r = warp.row_offset[i] * warp.strength;
        p.offset_x[i] = (inverse(0, 0) * r.x + inverse(0, 1) * r.y) * sx;
        p.offset_y[i] = (inverse(1, 0) * r.x + inverse(1, 1) * r.y) * sy;
        p.scan[i] = static_cast<double>((horizontal ? output.width : output.height) - 1) * i / (p.bands - 1) +
                    (horizontal ? p.offset_x[i] : p.offset_y[i]);
        if (i && p.scan[i] <= p.scan[i - 1])
            throw std::runtime_error("Scanline mapping is not invertible");
    }
    const double dx = (1 - 2 * crop) * (size.width - 1) / (output.width - 1);
    const double dy = (1 - 2 * crop) * (size.height - 1) / (output.height - 1);
    const auto origin = transform(inverse, {crop * (size.width - 1), crop * (size.height - 1)});
    p.origin_x = origin.x * sx;
    p.origin_y = origin.y * sy;
    p.xx = inverse(0, 0) * dx * sx;
    p.xy = inverse(0, 1) * dy * sx;
    p.yx = inverse(1, 0) * dx * sy;
    p.yy = inverse(1, 1) * dy * sy;
    return p;
}

} // namespace stillflow
