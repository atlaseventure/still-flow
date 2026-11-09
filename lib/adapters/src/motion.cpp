#include "motion.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <opencv2/features.hpp>
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
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

std::string geometry_failure(const Matrix3& step, Point center, double extent) {
    const double a = step(0, 0), b = step(1, 0), scale = std::hypot(a, b);
    if (std::abs(std::atan2(b, a)) > 0.2)
        return "rotation";
    if (scale < 0.85 || scale > 1.18)
        return "scale";
    if (norm(transform(step, center) - center) > extent * 0.2)
        return "translation";
    return {};
}

struct DescriptorMotion {
    Matrix3 step = Matrix3::eye();
    bool textured = false;
    int inliers = 0;
    double confidence = 0, coverage = 0;
};

// Independent correspondences recover motion when large displacement, blur or
// parallax makes the optical-flow model unreliable. Mutual matches avoid
// counting multiple descriptors of the same destination as independent support.
DescriptorMotion descriptor_motion(const cv::Mat& previous, const cv::Mat& current, int features) {
    DescriptorMotion result;
    auto detector = cv::ORB::create(std::max(800, features));
    std::vector<cv::KeyPoint> keys_before, keys_after;
    cv::Mat descriptors_before, descriptors_after;
    detector->detectAndCompute(previous, {}, keys_before, descriptors_before);
    detector->detectAndCompute(current, {}, keys_after, descriptors_after);
    if (keys_before.size() < 64 || keys_after.size() < 64)
        return result;
    result.textured = true;
    std::vector<std::vector<cv::DMatch>> forward, backward;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(descriptors_before, descriptors_after, forward, 2);
    matcher.knnMatch(descriptors_after, descriptors_before, backward, 2);
    std::vector<cv::Point2f> p, q;
    for (const auto& pair : forward) {
        if (pair.size() != 2 || pair[0].distance >= 0.75 * pair[1].distance)
            continue;
        const auto& reverse = backward[pair[0].trainIdx];
        if (reverse.size() != 2 || reverse[0].distance >= 0.75 * reverse[1].distance ||
            reverse[0].trainIdx != pair[0].queryIdx)
            continue;
        p.push_back(keys_before[pair[0].queryIdx].pt);
        q.push_back(keys_after[pair[0].trainIdx].pt);
    }
    if (p.size() < 8)
        return result;
    cv::Mat inliers;
    const auto mapping = cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3, 2000, 0.99, 10);
    if (mapping.empty())
        return result;
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result.step(r, c) = mapping.at<double>(r, c);
    std::array<int, 48> cells{};
    for (size_t i = 0; i < p.size(); ++i) {
        if (norm(transform(result.step, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y}) > 3)
            continue;
        ++result.inliers;
        const int x = std::clamp(static_cast<int>(p[i].x * 8 / previous.cols), 0, 7);
        const int y = std::clamp(static_cast<int>(p[i].y * 6 / previous.rows), 0, 5);
        ++cells[y * 8 + x];
    }
    result.coverage = std::count_if(cells.begin(), cells.end(), [](int n) { return n >= 2; }) / 48.0;
    result.confidence = (static_cast<double>(result.inliers) / p.size()) * std::min(1.0, result.coverage / 0.6);
    return result;
}

// A cut needs both lost geometric correspondence and a substantial visual
// discontinuity. Low texture or blur alone is a missing observation.
bool scene_cut(const cv::Mat& previous, const cv::Mat& current, double support, const DescriptorMotion& descriptors) {
    if (support >= 0.10 || !descriptors.textured || descriptors.inliers >= 8)
        return false;
    cv::Mat before, after;
    cv::resize(previous, before, {64, 48}, 0, 0, cv::INTER_AREA);
    cv::resize(current, after, before.size(), 0, 0, cv::INTER_AREA);
    cv::Scalar mean, before_std, after_std;
    cv::meanStdDev(before, mean, before_std);
    cv::meanStdDev(after, mean, after_std);
    if (before_std[0] < 15 || after_std[0] < 15)
        return false;
    cv::Mat difference;
    cv::absdiff(before, after, difference);
    const double change = cv::mean(difference)[0] / 255;
    const int bins = 32, channel = 0;
    const float range[]{0, 256};
    const float* ranges[]{range};
    cv::Mat a, b;
    cv::calcHist(&before, 1, &channel, {}, a, 1, &bins, ranges);
    cv::calcHist(&after, 1, &channel, {}, b, 1, &bins, ranges);
    const double distance = cv::compareHist(a, b, cv::HISTCMP_BHATTACHARYYA);
    return change >= 0.12 || distance >= 0.5;
}

} // namespace
MotionEstimator::MotionEstimator(const StabilizationSettings& options, bool horizontal_scan)
    : options_(options), horizontal_(horizontal_scan) {}

Motion MotionEstimator::estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile) {
    Motion result;
    result.row_delta.resize(options_.bands);
    double global_support = 0;
    const Point center((previous.cols - 1) * 0.5, (previous.rows - 1) * 0.5);
    const double norm = std::max(previous.cols, previous.rows);
    const auto missing = [&](const std::string& reason) {
        result.step = Matrix3::eye();
        result.failure = reason;
        result.state = MotionState::missing;
        ScopeTimer timer(profile, "analysis.recover_features");
        const auto descriptors = descriptor_motion(previous, current, options_.features);
        if (descriptors.inliers >= 24 && descriptors.confidence >= 0.25 && descriptors.coverage >= 0.20 &&
            geometry_failure(descriptors.step, center, norm).empty()) {
            result.step = descriptors.step;
            result.confidence = descriptors.confidence;
            result.state = MotionState::descriptor_fallback;
        } else if (scene_cut(previous, current, global_support, descriptors)) {
            result.state = MotionState::cut;
            result.reset = true;
        }
        return result;
    };
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
        return missing("features");
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
        return missing("tracks");
    }
    ScopeTimer fit_timer(profile, "analysis.fit");
    cv::Mat inliers;
    const auto initial = cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3, 2000, 0.99, 10);
    if (initial.empty()) {
        return missing("model");
    }
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result.step(r, c) = initial.at<double>(r, c);
    const Matrix3 global = result.step;
    std::array<int, 48> global_cells{};
    int global_inliers = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        if (stillflow::norm(transform(global, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y}) > 3)
            continue;
        ++global_inliers;
        const int x = std::clamp(static_cast<int>(p[i].x * 8 / previous.cols), 0, 7);
        const int y = std::clamp(static_cast<int>(p[i].y * 6 / previous.rows), 0, 5);
        ++global_cells[y * 8 + x];
    }
    global_support = static_cast<double>(global_inliers) / source.size();
    const double global_coverage =
        std::count_if(global_cells.begin(), global_cells.end(), [](int n) { return n >= 2; }) / 48.0;
    const double global_confidence =
        (static_cast<double>(global_inliers) / p.size()) * std::min(1.0, global_coverage / 0.6);
    const auto fallback = [&](const std::string& reason) {
        if (global_inliers >= 24 && global_confidence >= 0.25 && global_coverage >= 0.20 &&
            geometry_failure(global, center, norm).empty()) {
            result.step = global;
            result.confidence = global_confidence;
            result.state = MotionState::global_fallback;
            result.failure = reason;
            return result;
        }
        return missing(reason);
    };
    // Keep motion estimation identical for RS on/off comparisons. The flag only
    // disables the correction, so RS residuals cannot poison the global baseline.
    const int bands = options_.bands;
    const int dimensions = 4 + 2 * bands;
    const Eigen::Index count = static_cast<Eigen::Index>(p.size());
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
            return fallback("nonfinite_fit");
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
    result.fit_confidence = result.confidence;
    result.coverage = coverage;
    result.residual_px = median(residuals);
    if (result.confidence < 0.25)
        return fallback("confidence");
    if (coverage < 0.20)
        return fallback("coverage");
    if (result.residual_px > 6)
        return fallback("residual");
    if (const auto reason = geometry_failure(result.step, center, norm); !reason.empty())
        return fallback(reason);
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
