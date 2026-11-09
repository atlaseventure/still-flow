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
        const double weight = std::max(0.001, confidence[i]);
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
    return interpolate_similarity(warp.global, center, strength);
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

// Solve a small pose graph in centered, resolution-independent similarity
// coordinates. Each measurement constrains P_to = T * P_from exactly linearly.
// Weak velocity regularization fills unobserved interior poses; it cannot make
// a gap recoverable without independent measured connections across it.
bool recover_motion(std::span<FrameMotion> frames, const std::vector<TemporalLink>& links, Size size) {
    if (frames.size() < 2 || links.size() < 2)
        return false;
    const Eigen::Index n = static_cast<Eigen::Index>(frames.size()), dimensions = 4 * (n - 1);
    const double extent = std::max(size.width, size.height);
    const Point center((size.width - 1) * 0.5, (size.height - 1) * 0.5);
    const Matrix3 normalize{1 / extent, 0, -center.x / extent, 0, 1 / extent, -center.y / extent, 0, 0, 1};
    const auto denormalize = normalize.inv();
    struct Constraint {
        Eigen::Index from, to;
        Eigen::Matrix4d mapping;
        Eigen::Vector4d offset;
        double confidence;
        bool temporal;
    };
    std::vector<Constraint> constraints;
    const auto append = [&](Eigen::Index from, Eigen::Index to, const Motion& motion, bool temporal) {
        const auto t = normalize * motion.step * denormalize;
        const double a = t(0, 0), b = t(1, 0);
        Eigen::Matrix4d mapping;
        mapping << a, -b, 0, 0, b, a, 0, 0, 0, 0, a, -b, 0, 0, b, a;
        constraints.push_back({from, to, mapping, {0, 0, t(0, 2), t(1, 2)}, motion.confidence, temporal});
    };
    for (Eigen::Index i = 1; i < n; ++i) {
        if (frames[i].motion.reset)
            return false;
        if (frames[i].motion.state != MotionState::missing)
            append(i - 1, i, frames[i].motion, false);
    }
    for (const auto& link : links) {
        if (link.from >= link.to || link.to >= frames.size())
            return false;
        append(static_cast<Eigen::Index>(link.from), static_cast<Eigen::Index>(link.to), link.motion, true);
    }
    const Eigen::Vector4d origin(1, 0, 0, 0);
    std::vector<double> weights(constraints.size(), 1);
    Eigen::VectorXd poses = Eigen::VectorXd::Zero(dimensions);
    const auto pose = [&](Eigen::Index i) -> Eigen::Vector4d {
        return i ? Eigen::Vector4d(poses.segment<4>(4 * (i - 1))) : origin;
    };
    for (int iteration = 0; iteration < 5; ++iteration) {
        Eigen::MatrixXd lhs = Eigen::MatrixXd::Zero(dimensions, dimensions);
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dimensions);
        const auto add = [&](const Eigen::MatrixXd& design, const Eigen::Vector4d& target, double weight) {
            lhs.noalias() += weight * design.transpose() * design;
            rhs.noalias() += weight * design.transpose() * target;
        };
        for (size_t k = 0; k < constraints.size(); ++k) {
            const auto& c = constraints[k];
            Eigen::MatrixXd design = Eigen::MatrixXd::Zero(4, dimensions);
            design.block<4, 4>(0, 4 * (c.to - 1)).setIdentity();
            auto target = c.offset;
            if (c.from)
                design.block<4, 4>(0, 4 * (c.from - 1)) = -c.mapping;
            else
                target += c.mapping * origin;
            add(design, target, std::max(0.01, c.confidence * c.confidence) * weights[k]);
        }
        for (Eigen::Index i = 1; i + 1 < n; ++i) {
            const double before = frames[i].seconds - frames[i - 1].seconds;
            const double after = frames[i + 1].seconds - frames[i].seconds;
            if (before <= 0 || after <= 0)
                return false;
            const double t = before / (before + after);
            Eigen::MatrixXd design = Eigen::MatrixXd::Zero(4, dimensions);
            design.block<4, 4>(0, 4 * (i - 1)) = -Eigen::Matrix4d::Identity();
            design.block<4, 4>(0, 4 * i) = t * Eigen::Matrix4d::Identity();
            Eigen::Vector4d target = Eigen::Vector4d::Zero();
            if (i > 1)
                design.block<4, 4>(0, 4 * (i - 2)) = (1 - t) * Eigen::Matrix4d::Identity();
            else
                target = -(1 - t) * origin;
            const bool missing =
                frames[i].motion.state == MotionState::missing || frames[i + 1].motion.state == MotionState::missing;
            add(design, target, missing ? 0.01 : 0.0001);
        }
        Eigen::LDLT<Eigen::MatrixXd> solver(lhs);
        if (solver.info() != Eigen::Success || !solver.isPositive() || solver.vectorD().minCoeff() < 1e-10)
            return false;
        poses = solver.solve(rhs);
        if (!poses.allFinite())
            return false;
        for (size_t k = 0; k < constraints.size(); ++k) {
            const auto& c = constraints[k];
            const double error = (pose(c.to) - c.mapping * pose(c.from) - c.offset).norm();
            weights[k] = std::min(1.0, 0.003 / std::max(1e-9, error));
        }
    }
    std::vector<int> support(n, 0);
    std::vector<double> confidence(n, 0), residual(n, 0);
    int accepted_links = 0;
    for (const auto& c : constraints) {
        const double error = (pose(c.to) - c.mapping * pose(c.from) - c.offset).norm();
        if (!c.temporal) {
            if (error > 0.01)
                return false;
            continue;
        }
        if (error > 0.008)
            continue;
        ++accepted_links;
        for (Eigen::Index i = c.from + 1; i <= c.to; ++i) {
            ++support[i];
            confidence[i] += c.confidence;
            residual[i] += error;
        }
    }
    if (accepted_links < 2)
        return false;
    for (Eigen::Index i = 1; i < n; ++i)
        if (frames[i].motion.state == MotionState::missing && support[i] < 2)
            return false;
    std::vector<Matrix3> steps(n);
    auto previous = Matrix3::eye();
    for (Eigen::Index i = 1; i < n; ++i) {
        const auto p = pose(i);
        const Matrix3 normalized{p(0), -p(1), p(2), p(1), p(0), p(3), 0, 0, 1};
        const auto camera = denormalize * normalized * normalize;
        steps[i] = camera * previous.inv();
        const double scale = std::hypot(steps[i](0, 0), steps[i](1, 0));
        if (scale < 0.85 || scale > 1.18 || std::abs(std::atan2(steps[i](1, 0), steps[i](0, 0))) > 0.2 ||
            norm(transform(steps[i], center) - center) > extent * 0.2)
            return false;
        previous = camera;
    }
    for (Eigen::Index i = 1; i < n; ++i) {
        auto& motion = frames[i].motion;
        motion.step = steps[i];
        motion.temporal_links = support[i];
        motion.temporal_residual = support[i] ? residual[i] / support[i] : 0;
        if (motion.state == MotionState::missing) {
            motion.state = MotionState::bridged;
            motion.confidence = 0.75 * confidence[i] / support[i];
        }
    }
    return true;
}

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, Size size, const StabilizationSettings& options,
                             bool horizontal) {
    std::vector<Warp> output(frames.size());
    for (size_t first = 0; first < frames.size();) {
        size_t last = first + 1;
        while (last < frames.size() && frames[last].segment == frames[first].segment)
            ++last;
        const int n = static_cast<int>(last - first);
        std::vector<Matrix3> cameras(n);
        Eigen::MatrixXd paths = Eigen::MatrixXd::Zero(n, 4);
        Eigen::MatrixXd rows = Eigen::MatrixXd::Zero(n, static_cast<Eigen::Index>(options.bands) * 2);
        std::vector<double> times(n), confidence(n), row_strength(n, 1);
        auto camera = Matrix3::eye();
        for (int i = 0; i < n; ++i) {
            const auto& frame = frames[first + i];
            times[i] = frame.seconds;
            // The reported score is the geometric mean of independent quality
            // terms. Their product weights the observation in the trajectory fit.
            confidence[i] = i == 0 ? 1.0 : std::pow(frame.motion.confidence, 3);
            if (frame.motion.state == MotionState::missing)
                confidence[i] = 0;
            if (frame.motion.state == MotionState::missing || frame.motion.state == MotionState::global_fallback ||
                frame.motion.state == MotionState::descriptor_fallback || frame.motion.state == MotionState::bridged)
                row_strength[i] = 0;
            if (i)
                camera = frame.motion.step * camera;
            cameras[i] = camera;
            // Smooth the similarity coefficients together. This linear operation
            // commutes with changing the reference camera; separately smoothing
            // angle/log-scale and translation amplifies drift in long shots.
            paths.row(i) << camera(0, 2), camera(1, 2), camera(0, 0), camera(1, 0);
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
        // Global-only recovery also fades the scanline correction on either side
        // of the gap, so switching models cannot introduce a local deformation jump.
        for (int i = 1; i < n; ++i)
            row_strength[i] = std::min(row_strength[i], row_strength[i - 1] + (times[i] - times[i - 1]) / 0.15);
        for (int i = n - 1; i > 0; --i)
            row_strength[i - 1] = std::min(row_strength[i - 1], row_strength[i] + (times[i] - times[i - 1]) / 0.15);
        double missing_seconds = 0;
        for (int i = 0; i < n; ++i) {
            const double a = stable(i, 2), b = stable(i, 3);
            const Matrix3 desired{a, -b, stable(i, 0), b, a, stable(i, 1), 0, 0, 1};
            auto& warp = output[first + i];
            warp.global = desired * cameras[i].inv();
            if (options.rolling_shutter) {
                warp.row_offset.resize(options.bands);
                for (Eigen::Index band = 0; band < options.bands; ++band) {
                    const double dx = stable_rows(i, band * 2) - rows(i, band * 2);
                    const double dy = stable_rows(i, band * 2 + 1) - rows(i, band * 2 + 1);
                    warp.row_offset[band] =
                        Point(a * dx - b * dy, b * dx + a * dy) * (options.rs_strength * row_strength[i]);
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
                warp.crop_strength = low;
            }
            // Evidence loss accumulates over time. A single missing observation
            // only reduces trust locally; sustained failure reaches the input view.
            if (frames[first + i].motion.state == MotionState::missing) {
                missing_seconds += i ? times[i] - times[i - 1] : frames[first + i].duration;
                warp.tracking_strength = std::max(0.0, 1 - missing_seconds / 0.25);
            } else {
                missing_seconds = 0;
            }
        }
        // A bounded envelope eases into crop constraints without zoom breathing.
        for (auto member : {&Warp::crop_strength, &Warp::tracking_strength}) {
            for (size_t i = first + 1; i < last; ++i)
                output[i].*member = std::min(output[i].*member, output[i - 1].*member +
                                                                    (frames[i].seconds - frames[i - 1].seconds) / 0.25);
            for (size_t i = last - 1; i > first; --i)
                output[i - 1].*member = std::min(
                    output[i - 1].*member, output[i].*member + (frames[i].seconds - frames[i - 1].seconds) / 0.25);
        }
        for (size_t i = first; i < last; ++i)
            output[i].strength = std::min(output[i].crop_strength, output[i].tracking_strength);
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
