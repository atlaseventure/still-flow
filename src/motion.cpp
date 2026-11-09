#include "stillflow/motion.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCholesky>
#include <opencv2/features.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <numeric>
#include <stdexcept>

namespace stillflow {
namespace {
struct BandPosition { int low; double fraction; };
BandPosition band_position(double coordinate, double extent, int bands) {
    const double position = std::clamp(coordinate / (extent - 1), 0.0, 1.0) * (bands - 1);
    const int low = std::min(static_cast<int>(position), bands - 2);
    return {low, position - low};
}
cv::Point2d interpolate(const std::vector<cv::Point2d>& values, cv::Point2d p,
    cv::Size size, bool horizontal) {
    if (values.empty()) return {};
    const auto [low, t] = band_position(horizontal ? p.x : p.y,
        horizontal ? size.width : size.height, static_cast<int>(values.size()));
    return values[low] * (1 - t) + values[low + 1] * t;
}
double median(std::vector<double> values) {
    if (values.empty()) return 0;
    auto middle = values.begin() + static_cast<ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

// The constant/linear null space preserves stationary views and deliberate pans.
// A sparse second-derivative penalty also supports irregular presentation times.
Eigen::MatrixXd smooth(const Eigen::MatrixXd& path, const std::vector<double>& times,
    const std::vector<double>& confidence, double seconds) {
    const int n = static_cast<int>(path.rows());
    if (n < 3 || seconds <= 0) return path;
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
    if (solver.info() != Eigen::Success) throw std::runtime_error("Trajectory factorization failed");
    Eigen::MatrixXd result = solver.solve(rhs);
    if (solver.info() != Eigen::Success || !result.allFinite())
        throw std::runtime_error("Trajectory optimization failed");
    return result;
}

cv::Matx33d scaled_global(const Warp& warp, cv::Size size, double strength) {
    const cv::Point2d center((size.width - 1) * 0.5, (size.height - 1) * 0.5);
    const auto shift = transform(warp.global, center) - center;
    const double angle = std::atan2(warp.global(1, 0), warp.global(0, 0)) * strength;
    const double scale = std::pow(std::hypot(warp.global(0, 0), warp.global(1, 0)), strength);
    const double a = scale * std::cos(angle), b = scale * std::sin(angle);
    return {a, -b, (1 - a) * center.x + b * center.y + strength * shift.x,
            b, a, -b * center.x + (1 - a) * center.y + strength * shift.y, 0, 0, 1};
}

cv::Point2d inverse_point(cv::Point2d target, const cv::Matx33d& inverse,
    const Warp& warp, cv::Size size, bool horizontal, double strength) {
    auto p = transform(inverse, target);
    for (int iteration = 0; iteration < 5 && !warp.row_offset.empty(); ++iteration)
        p = transform(inverse, target - interpolate(warp.row_offset, p, size, horizontal) * strength);
    return p;
}

bool fits(const Warp& warp, cv::Size size, double crop, bool horizontal, double strength) {
    const auto matrix = scaled_global(warp, size, strength);
    const auto inverse = matrix.inv();
    // Bound the derivative so inversion remains contractive and the warp cannot fold.
    if (!warp.row_offset.empty()) {
        const double spacing = static_cast<double>((horizontal ? size.width : size.height) - 1)
            / (warp.row_offset.size() - 1);
        for (size_t i = 1; i < warp.row_offset.size(); ++i)
            if (cv::norm(warp.row_offset[i] - warp.row_offset[i - 1]) * strength / spacing > 0.20)
                return false;
    }
    const auto valid = [&](double x, double y) {
        const cv::Point2d target((crop + x * (1 - 2 * crop)) * (size.width - 1),
                                 (crop + y * (1 - 2 * crop)) * (size.height - 1));
        const auto p = inverse_point(target, inverse, warp, size, horizontal, strength);
        return std::isfinite(p.x) && std::isfinite(p.y) &&
            p.x >= 0.5 && p.x <= size.width - 1.5 && p.y >= 0.5 && p.y <= size.height - 1.5;
    };
    for (int i = 0; i <= 64; ++i) {
        const double t = i / 64.0;
        if (!valid(t, 0) || !valid(t, 1) || !valid(0, t) || !valid(1, t)) return false;
    }
    return true;
}
}

MotionEstimator::MotionEstimator(const Options& options, bool horizontal_scan)
    : options_(options), horizontal_(horizontal_scan) {}

Motion MotionEstimator::estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile) {
    Motion result;
    result.row_delta.resize(options_.bands);
    std::vector<cv::Point2f> source, destination, backward;
    {
        ScopeTimer timer(profile, "analysis.features");
        // Per-cell quotas stop one detailed foreground object taking all keypoints.
        constexpr int columns = 8, rows = 6;
        for (int y = 0; y < rows; ++y) for (int x = 0; x < columns; ++x) {
            cv::Rect roi(x * previous.cols / columns, y * previous.rows / rows,
                (x + 1) * previous.cols / columns - x * previous.cols / columns,
                (y + 1) * previous.rows / rows - y * previous.rows / rows);
            std::vector<cv::Point2f> points;
            cv::goodFeaturesToTrack(previous(roi), points, std::max(1, options_.features / (columns * rows)),
                0.015, 6);
            for (auto point : points) source.push_back(point + cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y)));
        }
    }
    if (source.size() < 24) { result.reset = true; return result; }
    std::vector<uchar> forward_status, backward_status;
    std::vector<float> forward_error, backward_error;
    {
        ScopeTimer timer(profile, "analysis.track");
        cv::calcOpticalFlowPyrLK(previous, current, source, destination, forward_status, forward_error,
            {21, 21}, 3);
        cv::calcOpticalFlowPyrLK(current, previous, destination, backward, backward_status, backward_error,
            {21, 21}, 3);
    }
    std::vector<cv::Point2f> p, q;
    for (size_t i = 0; i < source.size(); ++i) {
        if (forward_status[i] && backward_status[i] && forward_error[i] < 35 &&
            cv::norm(source[i] - backward[i]) < 1.5 &&
            destination[i].x >= 1 && destination[i].x < current.cols - 1 &&
            destination[i].y >= 1 && destination[i].y < current.rows - 1) {
            p.push_back(source[i]); q.push_back(destination[i]);
        }
    }
    result.tracks = static_cast<int>(p.size());
    if (p.size() < 24) { result.reset = true; return result; }
    ScopeTimer fit_timer(profile, "analysis.fit");
    cv::Mat inliers;
    const auto initial = cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3, 2000, 0.99, 10);
    if (initial.empty()) { result.reset = true; return result; }
    for (int r = 0; r < 2; ++r) for (int c = 0; c < 3; ++c) result.step(r, c) = initial.at<double>(r, c);
    // Keep motion estimation identical for RS on/off comparisons. The flag only
    // disables the correction, so RS residuals cannot poison the global baseline.
    const int bands = options_.bands;
    const int dimensions = 4 + 2 * bands;
    const int count = static_cast<int>(p.size());
    const double norm = std::max(previous.cols, previous.rows);
    const cv::Point2d center((previous.cols - 1) * 0.5, (previous.rows - 1) * 0.5);
    Eigen::MatrixXd design = Eigen::MatrixXd::Zero(count * 2, dimensions);
    Eigen::VectorXd observed(count * 2), weights(count), parameters = Eigen::VectorXd::Zero(dimensions);
    std::vector<int> band_counts(options_.bands, 0);
    std::array<int, 48> cells{};
    for (int i = 0; i < count; ++i) {
        const double x = (p[i].x - center.x) / norm, y = (p[i].y - center.y) / norm;
        design(2 * i, 0) = x; design(2 * i, 1) = -y; design(2 * i, 2) = 1;
        design(2 * i + 1, 0) = y; design(2 * i + 1, 1) = x; design(2 * i + 1, 3) = 1;
        observed(2 * i) = q[i].x - p[i].x;
        observed(2 * i + 1) = q[i].y - p[i].y;
        const double error = cv::norm(transform(result.step, p[i]) - cv::Point2d(q[i]));
        weights(i) = std::min(1.0, 3.0 / std::max(0.01, error));
        if (bands) {
            // Midpoint scan coordinates approximate source/target row correspondence
            // for small inter-frame motion; this is not a physical readout-time fit.
            const double coordinate = horizontal_ ? (p[i].x + q[i].x) * 0.5 : (p[i].y + q[i].y) * 0.5;
            const auto [low, t] = band_position(coordinate, horizontal_ ? previous.cols : previous.rows, bands);
            for (int axis = 0; axis < 2; ++axis) {
                design(2 * i + axis, 4 + 2 * low + axis) = 1 - t;
                design(2 * i + axis, 4 + 2 * (low + 1) + axis) = t;
            }
        }
    }
    Eigen::MatrixXd regularizer = Eigen::MatrixXd::Identity(dimensions, dimensions) * 1e-6;
    if (bands) {
        for (int axis = 0; axis < 2; ++axis) {
            Eigen::VectorXd mean = Eigen::VectorXd::Zero(dimensions);
            for (int b = 0; b < bands; ++b) {
                mean(4 + 2 * b + axis) = 1.0 / bands;
                regularizer(4 + 2 * b + axis, 4 + 2 * b + axis) += 0.25;
            }
            regularizer += 1000 * mean * mean.transpose();
            for (int b = 1; b + 1 < bands; ++b) {
                Eigen::VectorXd curvature = Eigen::VectorXd::Zero(dimensions);
                curvature(4 + 2 * (b - 1) + axis) = 1;
                curvature(4 + 2 * b + axis) = -2;
                curvature(4 + 2 * (b + 1) + axis) = 1;
                regularizer += 8 * curvature * curvature.transpose();
            }
        }
    }
    for (int iteration = 0; iteration < 5; ++iteration) {
        Eigen::VectorXd repeated(count * 2);
        for (int i = 0; i < count; ++i) repeated(2 * i) = repeated(2 * i + 1) = weights(i);
        const Eigen::MatrixXd weighted = repeated.asDiagonal() * design;
        const Eigen::MatrixXd lhs = design.transpose() * weighted + regularizer;
        parameters = lhs.ldlt().solve(weighted.transpose() * observed);
        if (!parameters.allFinite()) { result.reset = true; return result; }
        const Eigen::VectorXd residual = design * parameters - observed;
        for (int i = 0; i < count; ++i) {
            const double error = residual.segment<2>(2 * i).norm();
            weights(i) = std::min(1.0, 2.0 / std::max(0.01, error));
        }
    }
    const double a = 1 + parameters(0) / norm, b = parameters(1) / norm;
    result.step = {a, -b, (1 - a) * center.x + b * center.y + parameters(2),
                  b, a, -b * center.x + (1 - a) * center.y + parameters(3), 0, 0, 1};
    std::vector<double> residuals;
    const Eigen::VectorXd residual = design * parameters - observed;
    int reliable = 0;
    for (int i = 0; i < count; ++i) {
        const double error = residual.segment<2>(2 * i).norm();
        residuals.push_back(error);
        if (error > 3) continue;
        ++reliable;
        const int cx = std::clamp(static_cast<int>(p[i].x * 8 / previous.cols), 0, 7);
        const int cy = std::clamp(static_cast<int>(p[i].y * 6 / previous.rows), 0, 5);
        ++cells[cy * 8 + cx];
        const double location = horizontal_ ? p[i].x / previous.cols : p[i].y / previous.rows;
        ++band_counts[std::clamp(static_cast<int>(location * options_.bands), 0, options_.bands - 1)];
    }
    const double coverage = std::count_if(cells.begin(), cells.end(), [](int n) { return n >= 2; }) / 48.0;
    result.confidence = (static_cast<double>(reliable) / count) * std::min(1.0, coverage / 0.6);
    result.residual_px = median(residuals);
    const double scale = std::hypot(a, b);
    if (result.confidence < 0.25 || coverage < 0.20 || result.residual_px > 6 ||
        std::abs(std::atan2(b, a)) > 0.2 || scale < 0.85 || scale > 1.18 ||
        std::hypot(parameters(2), parameters(3)) > norm * 0.2) {
        result.step = cv::Matx33d::eye();
        result.reset = true;
        result.confidence = 0;
        return result;
    }
    if (bands) {
        const double row_coverage = std::count_if(band_counts.begin(), band_counts.end(), [](int n) { return n >= 4; })
            / static_cast<double>(bands);
        result.rs_confidence = result.confidence * row_coverage;
        for (int j = 0; j < bands; ++j) {
            const cv::Point2d delta(parameters(4 + 2 * j), parameters(5 + 2 * j));
            if (cv::norm(delta) > norm * 0.05) { result.rs_confidence = 0; break; }
        }
        const double trust = std::clamp((result.rs_confidence - 0.25) / 0.45, 0.0, 1.0);
        for (int j = 0; j < bands; ++j)
            result.row_delta[j] = cv::Point2d(parameters(4 + 2 * j), parameters(5 + 2 * j)) * trust;
    }
    return result;
}

std::vector<Warp> plan_warps(const std::vector<FrameMotion>& frames, cv::Size size,
    const Options& options, bool horizontal, Profile& profile) {
    ScopeTimer timer(profile, "plan.optimize");
    std::vector<Warp> output(frames.size());
    const cv::Point2d center((size.width - 1) * 0.5, (size.height - 1) * 0.5);
    for (size_t first = 0; first < frames.size();) {
        size_t last = first + 1;
        while (last < frames.size() && frames[last].segment == frames[first].segment) ++last;
        const int n = static_cast<int>(last - first);
        std::vector<cv::Matx33d> cameras(n);
        Eigen::MatrixXd paths = Eigen::MatrixXd::Zero(n, 4);
        Eigen::MatrixXd rows = Eigen::MatrixXd::Zero(n, options.bands * 2);
        std::vector<double> times(n), confidence(n);
        auto camera = cv::Matx33d::eye();
        double last_angle = 0;
        for (int i = 0; i < n; ++i) {
            const auto& frame = frames[first + i];
            times[i] = frame.seconds;
            confidence[i] = i == 0 ? 1.0 : frame.motion.confidence;
            if (i) camera = frame.motion.step * camera;
            cameras[i] = camera;
            const auto position = transform(camera, center) - center;
            double angle = std::atan2(camera(1, 0), camera(0, 0));
            angle = last_angle + std::remainder(angle - last_angle, 2 * std::numbers::pi);
            last_angle = angle;
            paths.row(i) << position.x, position.y, angle, std::log(std::hypot(camera(0, 0), camera(1, 0)));
            if (i) {
                rows.row(i) = rows.row(i - 1);
                const auto inverse = camera.inv();
                for (int band = 0; band < options.bands; ++band) {
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
            const cv::Matx33d desired{a, -b, (1 - a) * center.x + b * center.y + stable(i, 0),
                                    b, a, -b * center.x + (1 - a) * center.y + stable(i, 1), 0, 0, 1};
            auto& warp = output[first + i];
            warp.global = desired * cameras[i].inv();
            if (options.rolling_shutter) {
                warp.row_offset.resize(options.bands);
                for (int band = 0; band < options.bands; ++band) {
                    const double dx = stable_rows(i, band * 2) - rows(i, band * 2);
                    const double dy = stable_rows(i, band * 2 + 1) - rows(i, band * 2 + 1);
                    warp.row_offset[band] = cv::Point2d(a * dx - b * dy, b * dx + a * dy) * options.rs_strength;
                }
            }
            if (!fits(warp, size, options.crop, horizontal, 1)) {
                double low = 0, high = 1;
                for (int k = 0; k < 12; ++k) {
                    const double middle = (low + high) / 2;
                    if (fits(warp, size, options.crop, horizontal, middle)) low = middle;
                    else high = middle;
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

void build_maps(const Warp& warp, cv::Size size, cv::Size output, double crop,
    bool horizontal, cv::Mat& map_x, cv::Mat& map_y) {
    map_x.create(output, CV_32F);
    map_y.create(output, CV_32F);
    const auto inverse = scaled_global(warp, size, warp.strength).inv();
    const double sx = static_cast<double>(output.width - 1) / (size.width - 1);
    const double sy = static_cast<double>(output.height - 1) / (size.height - 1);
    // In global-inverse coordinates, the scan coordinate is a monotone piecewise
    // linear function. Invert that scalar function exactly, then subtract its
    // two-dimensional offset. No per-pixel iterative solver is needed.
    const int bands = static_cast<int>(warp.row_offset.size());
    std::vector<double> scan(bands);
    std::vector<cv::Point2d> offsets(bands);
    for (int i = 0; i < bands; ++i) {
        const auto r = warp.row_offset[i] * warp.strength;
        offsets[i] = {inverse(0, 0) * r.x + inverse(0, 1) * r.y,
                      inverse(1, 0) * r.x + inverse(1, 1) * r.y};
        scan[i] = static_cast<double>((horizontal ? size.width : size.height) - 1) * i / (bands - 1)
            + (horizontal ? offsets[i].x : offsets[i].y);
        if (i && scan[i] <= scan[i - 1]) throw std::runtime_error("Scanline mapping is not invertible");
    }
    const double dx = (1 - 2 * crop) * (size.width - 1) / (output.width - 1);
    const double tx = crop * (size.width - 1);
    cv::parallel_for_(cv::Range(0, output.height), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
            auto* mx = map_x.ptr<float>(y);
            auto* my = map_y.ptr<float>(y);
            const double ty = (crop + static_cast<double>(y) / (output.height - 1) * (1 - 2 * crop)) * (size.height - 1);
            auto u = transform(inverse, {tx, ty});
            int band = 0;
            for (int x = 0; x < output.width; ++x) {
                auto p = u;
                if (bands) {
                    const double coordinate = horizontal ? u.x : u.y;
                    while (band < bands - 2 && coordinate > scan[band + 1]) ++band;
                    while (band > 0 && coordinate < scan[band]) --band;
                    const double t = std::clamp((coordinate - scan[band]) / (scan[band + 1] - scan[band]), 0.0, 1.0);
                    p -= offsets[band] * (1 - t) + offsets[band + 1] * t;
                }
                mx[x] = static_cast<float>(p.x * sx);
                my[x] = static_cast<float>(p.y * sy);
                u.x += inverse(0, 0) * dx;
                u.y += inverse(1, 0) * dx;
            }
        }
    });
}
} // namespace stillflow
