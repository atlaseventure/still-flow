#include "motion.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <opencv2/features.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/video/tracking.hpp>

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
double median(std::vector<double> values) {
    if (values.empty())
        return 0;
    auto middle = values.begin() + static_cast<ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

} // namespace
MotionEstimator::MotionEstimator(const StabilizationSettings& options, bool horizontal_scan)
    : options_(options), horizontal_(horizontal_scan) {}

Motion MotionEstimator::estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile) {
    Motion result;
    result.row_delta.resize(options_.bands);
    std::vector<cv::Point2f> source, destination, backward;
    {
        ScopeTimer timer(profile, "analysis.features");
        // Per-cell quotas stop one detailed foreground object taking all keypoints.
        constexpr int columns = 8, rows = 6;
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < columns; ++x) {
                cv::Rect roi(x * previous.cols / columns, y * previous.rows / rows,
                             (x + 1) * previous.cols / columns - x * previous.cols / columns,
                             (y + 1) * previous.rows / rows - y * previous.rows / rows);
                std::vector<cv::Point2f> points;
                cv::goodFeaturesToTrack(previous(roi), points, std::max(1, options_.features / (columns * rows)), 0.015,
                                        6);
                for (auto point : points)
                    source.push_back(point + cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y)));
            }
    }
    if (source.size() < 24) {
        result.reset = true;
        return result;
    }
    std::vector<uchar> forward_status, backward_status;
    std::vector<float> forward_error, backward_error;
    {
        ScopeTimer timer(profile, "analysis.track");
        cv::calcOpticalFlowPyrLK(previous, current, source, destination, forward_status, forward_error, {21, 21}, 3);
        cv::calcOpticalFlowPyrLK(current, previous, destination, backward, backward_status, backward_error, {21, 21},
                                 3);
    }
    std::vector<cv::Point2f> p, q;
    for (size_t i = 0; i < source.size(); ++i) {
        if (forward_status[i] && backward_status[i] && forward_error[i] < 35 &&
            cv::norm(source[i] - backward[i]) < 1.5 && destination[i].x >= 1 && destination[i].x < current.cols - 1 &&
            destination[i].y >= 1 && destination[i].y < current.rows - 1) {
            p.push_back(source[i]);
            q.push_back(destination[i]);
        }
    }
    result.tracks = static_cast<int>(p.size());
    if (p.size() < 24) {
        result.reset = true;
        return result;
    }
    ScopeTimer fit_timer(profile, "analysis.fit");
    cv::Mat inliers;
    const auto initial = cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3, 2000, 0.99, 10);
    if (initial.empty()) {
        result.reset = true;
        return result;
    }
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result.step(r, c) = initial.at<double>(r, c);
    // Keep motion estimation identical for RS on/off comparisons. The flag only
    // disables the correction, so RS residuals cannot poison the global baseline.
    const int bands = options_.bands;
    const int dimensions = 4 + 2 * bands;
    const Eigen::Index count = static_cast<Eigen::Index>(p.size());
    const double norm = std::max(previous.cols, previous.rows);
    const Point center((previous.cols - 1) * 0.5, (previous.rows - 1) * 0.5);
    Eigen::MatrixXd design = Eigen::MatrixXd::Zero(count * 2, dimensions);
    Eigen::VectorXd observed(count * 2), weights(count), parameters = Eigen::VectorXd::Zero(dimensions);
    std::vector<int> band_counts(options_.bands, 0);
    std::array<int, 48> cells{};
    for (Eigen::Index i = 0; i < count; ++i) {
        const double x = (p[i].x - center.x) / norm, y = (p[i].y - center.y) / norm;
        design(2 * i, 0) = x;
        design(2 * i, 1) = -y;
        design(2 * i, 2) = 1;
        design(2 * i + 1, 0) = y;
        design(2 * i + 1, 1) = x;
        design(2 * i + 1, 3) = 1;
        observed(2 * i) = q[i].x - p[i].x;
        observed(2 * i + 1) = q[i].y - p[i].y;
        const double error = stillflow::norm(transform(result.step, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y});
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
        for (Eigen::Index i = 0; i < count; ++i)
            repeated(2 * i) = repeated(2 * i + 1) = weights(i);
        const Eigen::MatrixXd weighted = repeated.asDiagonal() * design;
        const Eigen::MatrixXd lhs = design.transpose() * weighted + regularizer;
        parameters = lhs.ldlt().solve(weighted.transpose() * observed);
        if (!parameters.allFinite()) {
            result.reset = true;
            return result;
        }
        const Eigen::VectorXd residual = design * parameters - observed;
        for (Eigen::Index i = 0; i < count; ++i) {
            const double error = residual.segment<2>(2 * i).norm();
            weights(i) = std::min(1.0, 2.0 / std::max(0.01, error));
        }
    }
    const double a = 1 + parameters(0) / norm, b = parameters(1) / norm;
    result.step = {a, -b, (1 - a) * center.x + b * center.y + parameters(2),
                   b, a,  -b * center.x + (1 - a) * center.y + parameters(3),
                   0, 0,  1};
    std::vector<double> residuals;
    const Eigen::VectorXd residual = design * parameters - observed;
    int reliable = 0;
    for (Eigen::Index i = 0; i < count; ++i) {
        const double error = residual.segment<2>(2 * i).norm();
        residuals.push_back(error);
        if (error > 3)
            continue;
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
    if (result.confidence < 0.25 || coverage < 0.20 || result.residual_px > 6 || std::abs(std::atan2(b, a)) > 0.2 ||
        scale < 0.85 || scale > 1.18 || std::hypot(parameters(2), parameters(3)) > norm * 0.2) {
        result.step = Matrix3::eye();
        result.reset = true;
        result.confidence = 0;
        return result;
    }
    if (bands) {
        const double row_coverage =
            std::count_if(band_counts.begin(), band_counts.end(), [](int n) { return n >= 4; }) /
            static_cast<double>(bands);
        result.rs_confidence = result.confidence * row_coverage;
        for (int j = 0; j < bands; ++j) {
            const Point delta(parameters(4 + 2 * j), parameters(5 + 2 * j));
            if (stillflow::norm(delta) > norm * 0.05) {
                result.rs_confidence = 0;
                break;
            }
        }
        const double trust = std::clamp((result.rs_confidence - 0.25) / 0.45, 0.0, 1.0);
        for (int j = 0; j < bands; ++j)
            result.row_delta[j] = Point(parameters(4 + 2 * j), parameters(5 + 2 * j)) * trust;
    }
    return result;
}

} // namespace stillflow
