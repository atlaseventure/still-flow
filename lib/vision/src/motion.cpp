#include "motion.hpp"
#include "stillflow/application/rendering.hpp"
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

// Residual scales are expressed at a 960-pixel analysis extent. The same
// evidence thresholds therefore apply when the proxy resolution changes.
double residual_scale(const cv::Mat& image) { return std::max(image.cols, image.rows) / 960.0; }
int cell_at(cv::Point2f point, const cv::Mat& image) {
    const int x = std::clamp(static_cast<int>(point.x * 8 / image.cols), 0, 7);
    const int y = std::clamp(static_cast<int>(point.y * 6 / image.rows), 0, 5);
    return y * 8 + x;
}
struct Quality {
    double matching = 0, consistency = 0, coverage = 0, span = 0, confidence = 0;
    int support = 0;
    bool reliable() const {
        return support >= 24 && matching >= 0.25 && consistency >= 0.25 && coverage >= 0.30 && span >= 0.08;
    }
};
Quality quality(const std::vector<cv::Point2f>& texture, const std::vector<cv::Point2f>& points,
                const std::vector<double>& reliability, const std::vector<double>& residuals, const cv::Mat& image) {
    Quality result;
    if (points.empty())
        return result;
    std::array<int, 48> textured{};
    std::array<double, 48> supported{};
    for (auto point : texture)
        ++textured[cell_at(point, image)];
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    Eigen::Matrix2d moment = Eigen::Matrix2d::Zero();
    double weight_sum = 0;
    const double scale = 3 * residual_scale(image);
    for (size_t i = 0; i < points.size(); ++i) {
        const double e = residuals[i] / scale;
        const double weight = 1 / (1 + e * e);
        result.matching += reliability[i];
        result.consistency += weight;
        supported[cell_at(points[i], image)] += weight;
        if (e > 2)
            continue;
        ++result.support;
        const Eigen::Vector2d location(points[i].x / image.cols, points[i].y / image.rows);
        mean += weight * location;
        moment += weight * location * location.transpose();
        weight_sum += weight;
    }
    result.matching /= points.size();
    result.matching *= std::min(1.0, 2.0 * points.size() / std::max(size_t{1}, texture.size()));
    result.consistency /= points.size();
    int available = 0;
    for (size_t i = 0; i < textured.size(); ++i) {
        if (textured[i] < 2)
            continue;
        ++available;
        result.coverage += std::min(1.0, supported[i] / 2);
    }
    result.coverage /= std::max(1, available);
    if (weight_sum > 0) {
        mean /= weight_sum;
        const Eigen::Matrix2d covariance = moment / weight_sum - mean * mean.transpose();
        result.span =
            std::sqrt(std::max(0.0, Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(covariance).eigenvalues()(0)));
    }
    const double spatial = std::min(1.0, result.coverage / 0.6) * std::min(1.0, result.span / 0.18);
    result.confidence = std::cbrt(result.matching * result.consistency * spatial);
    return result;
}
void record_quality(Motion& motion, const Quality& score) {
    motion.match_reliability = score.matching;
    motion.model_consistency = score.consistency;
    motion.coverage = score.coverage;
    motion.spatial_span = score.span;
    motion.confidence = score.confidence;
}

struct DescriptorMotion {
    Matrix3 step = Matrix3::eye();
    bool textured = false;
    int inliers = 0;
    Quality score;
};

// Independent correspondences recover motion when large displacement, blur or
// parallax makes the optical-flow model unreliable. Mutual matches avoid
// counting multiple descriptors of the same destination as independent support.
DescriptorMotion descriptor_motion(const cv::Mat& previous, const cv::Mat& current, int features,
                                   std::vector<cv::KeyPoint>& cached_keys, cv::Mat& cached_descriptors, bool cached,
                                   Profile& profile, FeatureFrame* before, FeatureFrame* after) {
    DescriptorMotion result;
    auto detector = cv::ORB::create(std::max(800, features));
    std::vector<cv::KeyPoint> keys_before, keys_after;
    cv::Mat descriptors_before, descriptors_after;
    {
        ScopeTimer timer(profile, "analysis.descriptors");
        if (before && before->descriptors_ready) {
            keys_before = before->keys;
            descriptors_before = before->descriptors;
        } else if (cached) {
            keys_before = std::move(cached_keys);
            descriptors_before = std::move(cached_descriptors);
        } else
            detector->detectAndCompute(previous, {}, keys_before, descriptors_before);
        if (before && !before->descriptors_ready) {
            before->keys = keys_before;
            before->descriptors = descriptors_before;
            before->descriptors_ready = true;
        }
        if (after && after->descriptors_ready) {
            keys_after = after->keys;
            descriptors_after = after->descriptors;
        } else {
            detector->detectAndCompute(current, {}, keys_after, descriptors_after);
            if (after) {
                after->keys = keys_after;
                after->descriptors = descriptors_after;
                after->descriptors_ready = true;
            }
        }
    }
    cached_keys = keys_after;
    cached_descriptors = descriptors_after;
    if (keys_before.size() < 64 || keys_after.size() < 64)
        return result;
    result.textured = true;
    std::vector<std::vector<cv::DMatch>> forward, backward;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    {
        ScopeTimer timer(profile, "analysis.match");
        matcher.knnMatch(descriptors_before, descriptors_after, forward, 2);
        matcher.knnMatch(descriptors_after, descriptors_before, backward, 2);
    }
    std::vector<cv::Point2f> p, q;
    std::vector<double> reliability;
    for (const auto& pair : forward) {
        if (pair.size() != 2 || pair[0].distance >= 0.75 * pair[1].distance)
            continue;
        const auto& reverse = backward[pair[0].trainIdx];
        if (reverse.size() != 2 || reverse[0].distance >= 0.75 * reverse[1].distance ||
            reverse[0].trainIdx != pair[0].queryIdx)
            continue;
        p.push_back(keys_before[pair[0].queryIdx].pt);
        q.push_back(keys_after[pair[0].trainIdx].pt);
        reliability.push_back(1 - std::pow(pair[0].distance / pair[1].distance, 2));
    }
    if (p.size() < 8)
        return result;
    cv::Mat inliers;
    const auto mapping =
        cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3 * residual_scale(previous), 2000, 0.99, 10);
    if (mapping.empty())
        return result;
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result.step(r, c) = mapping.at<double>(r, c);
    std::vector<cv::Point2f> texture;
    texture.reserve(keys_before.size());
    for (const auto& key : keys_before)
        texture.push_back(key.pt);
    std::vector<double> residuals;
    for (size_t i = 0; i < p.size(); ++i) {
        const double error = norm(transform(result.step, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y});
        residuals.push_back(error);
        if (error <= 3 * residual_scale(previous))
            ++result.inliers;
    }
    result.score = quality(texture, p, reliability, residuals, previous);
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
MotionEstimator::MotionEstimator(const StabilizationSettings& options, bool horizontal_scan, bool incremental,
                                 std::unique_ptr<SparseTracker> tracker)
    : tracker_(std::move(tracker)), options_(options), horizontal_(horizontal_scan), incremental_(incremental) {}

Motion MotionEstimator::estimate(const cv::Mat& previous, const cv::Mat& current, Profile& profile, bool consecutive,
                                 const Motion* prediction, FeatureFrame* before, FeatureFrame* after) {
    if (!consecutive) {
        pyramid_.clear();
        tracked_.clear();
        descriptors_ready_ = false;
        stable_pairs_ = 0;
        information_ = 0;
    }
    const bool cached_descriptors = descriptors_ready_;
    descriptors_ready_ = false;
    ++pairs_;
    const bool stable = adaptive_ && consecutive && prediction && prediction->state == MotionState::tracked &&
                        !prediction->reset && prediction->confidence >= 0.6 && prediction->model_consistency >= 0.7 &&
                        prediction->coverage >= 0.6 && prediction->spatial_span >= 0.14 && information_ >= 0.04 &&
                        (options_.bands == 0 || prediction->rs_confidence >= 0.6);
    if (prediction && (prediction->state != MotionState::tracked || prediction->confidence < .55 ||
                       prediction->model_consistency < .4))
        budget_cooldown_ = 60;
    else if (budget_cooldown_ > 0)
        --budget_cooldown_;
    stable_pairs_ = stable && budget_cooldown_ == 0 ? stable_pairs_ + 1 : 0;
    const bool budgeted = adaptive_ && stable_pairs_ >= 6;
    const int budget = std::min(options_.features, stable_pairs_ >= 12  ? 600
                                                   : stable_pairs_ >= 6 ? 900
                                                                        : options_.features);
    if (adaptive_)
        profile.add(budget <= 600   ? "analysis.budget_600"
                    : budget <= 900 ? "analysis.budget_900"
                                    : "analysis.budget_full",
                    0);
    const auto carried =
        incremental_ && (budgeted || pairs_ % 15 != 0) ? std::move(tracked_) : std::vector<cv::Point2f>{};
    tracked_.clear();
    Motion result;
    result.row_delta.resize(options_.bands);
    double global_support = 0;
    const Point center((previous.cols - 1) * 0.5, (previous.rows - 1) * 0.5);
    const double norm = std::max(previous.cols, previous.rows);
    const double pixel_scale = residual_scale(previous);
    const auto missing = [&](const std::string& reason) {
        result.step = Matrix3::eye();
        result.failure = reason;
        result.state = MotionState::missing;
        ScopeTimer timer(profile, "analysis.recover_features");
        const auto descriptors = descriptor_motion(previous, current, options_.features, descriptor_keys_, descriptors_,
                                                   cached_descriptors, profile, before, after);
        descriptors_ready_ = true;
        if (descriptors.score.reliable() && geometry_failure(descriptors.step, center, norm).empty()) {
            result.step = descriptors.step;
            record_quality(result, descriptors.score);
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
        if (before && before->corners_ready) {
            source = before->corners;
        } else {
            // Per-cell quotas stop one detailed foreground object taking all keypoints.
            constexpr int columns = 8, rows = 6;
            for (int y = 0; y < rows; ++y)
                for (int x = 0; x < columns; ++x) {
                    cv::Rect roi(x * previous.cols / columns, y * previous.rows / rows,
                                 (x + 1) * previous.cols / columns - x * previous.cols / columns,
                                 (y + 1) * previous.rows / rows - y * previous.rows / rows);
                    const size_t quota = static_cast<size_t>(std::max(1, budget / (columns * rows)));
                    std::vector<cv::Point2f> cell;
                    for (const auto point : carried)
                        if ((!budgeted || (y * columns + x) % 15 != pairs_ % 15) && cell.size() < quota &&
                            point.x >= roi.x && point.x < roi.x + roi.width && point.y >= roi.y &&
                            point.y < roi.y + roi.height)
                            cell.push_back(point);
                    if (cell.size() < quota / 2 || cell.empty()) {
                        std::vector<cv::Point2f> points;
                        cv::goodFeaturesToTrack(previous(roi), points, static_cast<int>(quota), 0.015, 6 * pixel_scale);
                        for (auto point : points) {
                            point += cv::Point2f(static_cast<float>(roi.x), static_cast<float>(roi.y));
                            if (cell.size() < quota && std::none_of(cell.begin(), cell.end(), [&](auto p) {
                                    return cv::norm(point - p) < 6 * pixel_scale;
                                }))
                                cell.push_back(point);
                        }
                    }
                    source.insert(source.end(), cell.begin(), cell.end());
                }
        }
        if (before) {
            before->corners = source;
            before->corners_ready = true;
        }
    }
    if (source.size() < 24) {
        pyramid_.clear();
        return missing("features");
    }
    std::vector<uchar> forward_status, backward_status;
    std::vector<float> forward_error, backward_error;
    {
        ScopeTimer timer(profile, "analysis.track");
        const auto build_pyramids = [&] {
            ScopeTimer pyramid_timer(profile, "analysis.pyramid");
            before_.swap(pyramid_);
            if (before && !before->pyramid.empty())
                before_ = before->pyramid;
            else if (!consecutive || before_.empty()) {
                if (before)
                    before_.clear();
                cv::buildOpticalFlowPyramid(previous, before_, {21, 21}, 3);
            }
            if (before)
                before->pyramid = before_;
            if (after && !after->pyramid.empty())
                pyramid_ = after->pyramid;
            else {
                if (after)
                    pyramid_.clear();
                cv::buildOpticalFlowPyramid(current, pyramid_, {21, 21}, 3);
            }
            if (after)
                after->pyramid = pyramid_;
        };
        if (!tracker_)
            build_pyramids();
        const bool predicted = incremental_ && consecutive && prediction && prediction->confidence >= 0.7 &&
                               !prediction->reset && prediction->state == MotionState::tracked;
        const bool mobile = options_.analysis_width <= 640 && options_.features <= 600;
        // Strong parallax needs the full coarse search even when most matches
        // pass the round-trip check. Reserve the shallow desktop search for
        // spatially supported, nearly rigid motion.
        const bool light = predicted &&
                           (mobile || (prediction->model_consistency >= 0.9 && prediction->coverage >= 0.8)) &&
                           stillflow::norm(transform(prediction->step, center) - center) < norm * 0.04;
        const auto track = [&](bool fast) {
            destination = source;
            if (predicted)
                for (size_t i = 0; i < source.size(); ++i) {
                    const auto p = transform(prediction->step, {source[i].x, source[i].y});
                    destination[i] = {static_cast<float>(p.x), static_cast<float>(p.y)};
                }
            const cv::Size window = fast && mobile ? cv::Size(15, 15) : cv::Size(21, 21);
            const int levels = fast ? 2 : 3;
            if (tracker_) {
                std::vector<Point> points, initial;
                for (size_t i = 0; i < source.size(); ++i) {
                    points.push_back({source[i].x, source[i].y});
                    initial.push_back({destination[i].x, destination[i].y});
                }
                try {
                    const auto matches = tracker_->track({{previous.cols, previous.rows}, previous.data, previous.step},
                                                         {{current.cols, current.rows}, current.data, current.step},
                                                         points, initial, levels, window.width, profile);
                    if (matches.size() != source.size())
                        throw BackendError("Invalid sparse tracking result count");
                    backward.resize(source.size());
                    forward_status.resize(source.size());
                    backward_status.resize(source.size());
                    forward_error.resize(source.size());
                    for (size_t i = 0; i < matches.size(); ++i) {
                        const auto& m = matches[i];
                        destination[i] = {float(m.destination.x), float(m.destination.y)};
                        backward[i] = {float(m.backward.x), float(m.backward.y)};
                        forward_error[i] = float(m.error);
                        forward_status[i] = backward_status[i] = m.valid;
                    }
                    return;
                } catch (const BackendError&) {
                    profile.add("analysis.gpu_fallback", 0);
                    tracker_.reset();
                    build_pyramids();
                }
            }
            const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
            {
                ScopeTimer forward_timer(profile, "analysis.lk_forward");
                cv::calcOpticalFlowPyrLK(before_, pyramid_, source, destination, forward_status, forward_error, window,
                                         levels, criteria, predicted ? cv::OPTFLOW_USE_INITIAL_FLOW : 0);
            }
            backward = source;
            {
                ScopeTimer backward_timer(profile, "analysis.lk_backward");
                cv::calcOpticalFlowPyrLK(pyramid_, before_, destination, backward, backward_status, backward_error,
                                         window, levels, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
            }
        };
        if (light)
            profile.add("analysis.predicted_track", 0);
        track(light);
        if (light) {
            size_t good = 0;
            for (size_t i = 0; i < source.size(); ++i)
                if (forward_status[i] && backward_status[i] && forward_error[i] < 35 &&
                    cv::norm(source[i] - backward[i]) < 1.5 * pixel_scale)
                    ++good;
            if (good < std::max(size_t{48}, source.size() * 3 / 5)) {
                profile.add("analysis.strong_retry", 0);
                track(false);
            }
        }
    }
    std::vector<cv::Point2f> p, q;
    std::vector<double> reliability;
    for (size_t i = 0; i < source.size(); ++i) {
        if (forward_status[i] && backward_status[i] && forward_error[i] < 35 &&
            cv::norm(source[i] - backward[i]) < 1.5 * pixel_scale && destination[i].x >= 1 &&
            destination[i].x < current.cols - 1 && destination[i].y >= 1 && destination[i].y < current.rows - 1) {
            p.push_back(source[i]);
            q.push_back(destination[i]);
            const double round_trip = cv::norm(source[i] - backward[i]) / (1.5 * pixel_scale);
            reliability.push_back(std::exp(-0.5 * round_trip * round_trip) / (1 + std::pow(forward_error[i] / 35, 2)));
        }
    }
    result.tracks = static_cast<int>(p.size());
    if (p.size() < 24) {
        return missing("tracks");
    }
    ScopeTimer fit_timer(profile, "analysis.fit");
    cv::Mat inliers;
    const auto initial =
        cv::estimateAffinePartial2D(p, q, inliers, cv::RANSAC, 3 * residual_scale(previous), 2000, 0.99, 10);
    if (initial.empty()) {
        return missing("model");
    }
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            result.step(r, c) = initial.at<double>(r, c);
    const Matrix3 global = result.step;
    std::vector<double> global_residuals;
    int global_inliers = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        const double error = stillflow::norm(transform(global, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y});
        global_residuals.push_back(error);
        if (error <= 3 * pixel_scale)
            ++global_inliers;
    }
    global_support = static_cast<double>(global_inliers) / source.size();
    const auto global_quality = quality(source, p, reliability, global_residuals, previous);
    const auto fallback = [&](const std::string& reason) {
        if (global_quality.reliable() && geometry_failure(global, center, norm).empty()) {
            result.step = global;
            record_quality(result, global_quality);
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
    struct SparseRow {
        std::array<int, 5> index{};
        std::array<double, 5> value{};
        double apply(const Eigen::VectorXd& x) const {
            double sum = 0;
            for (size_t j = 0; j < index.size(); ++j)
                sum += value[j] * x(index[j]);
            return sum;
        }
    };
    std::vector<SparseRow> design(static_cast<size_t>(count * 2));
    Eigen::VectorXd observed(count * 2), weights(count), parameters = Eigen::VectorXd::Zero(dimensions);
    std::vector<int> band_counts(options_.bands, 0);
    for (Eigen::Index i = 0; i < count; ++i) {
        const double x = (p[i].x - center.x) / norm, y = (p[i].y - center.y) / norm;
        design[2 * i] = {{0, 1, 2, 0, 0}, {x, -y, 1, 0, 0}};
        design[2 * i + 1] = {{0, 1, 3, 0, 0}, {y, x, 1, 0, 0}};
        observed(2 * i) = q[i].x - p[i].x;
        observed(2 * i + 1) = q[i].y - p[i].y;
        const double error = stillflow::norm(transform(result.step, {p[i].x, p[i].y}) - Point{q[i].x, q[i].y});
        weights(i) = std::min(1.0, 3.0 * pixel_scale / std::max(0.01, error));
        if (bands) {
            // Midpoint scan coordinates approximate source/target row correspondence
            // for small inter-frame motion; this is not a physical readout-time fit.
            const double coordinate = horizontal_ ? (p[i].x + q[i].x) * 0.5 : (p[i].y + q[i].y) * 0.5;
            const auto [low, t] = band_position(coordinate, horizontal_ ? previous.cols : previous.rows, bands);
            for (int axis = 0; axis < 2; ++axis) {
                auto& row = design[2 * i + axis];
                row.index[3] = 4 + 2 * low + axis;
                row.index[4] = 4 + 2 * (low + 1) + axis;
                row.value[3] = 1 - t;
                row.value[4] = t;
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
    Eigen::MatrixXd lhs(dimensions, dimensions);
    Eigen::VectorXd rhs(dimensions), residual(count * 2);
    const auto residuals_for = [&] {
        for (Eigen::Index i = 0; i < count * 2; ++i)
            residual(i) = design[i].apply(parameters) - observed(i);
    };
    for (int iteration = 0; iteration < 5; ++iteration) {
        lhs = regularizer;
        rhs.setZero();
        for (Eigen::Index i = 0; i < count * 2; ++i) {
            const auto& row = design[i];
            for (size_t a = 0; a < row.index.size(); ++a) {
                const double weighted = weights(i / 2) * row.value[a];
                rhs(row.index[a]) += weighted * observed(i);
                for (size_t b = 0; b < row.index.size(); ++b)
                    lhs(row.index[a], row.index[b]) += weighted * row.value[b];
            }
        }
        parameters = lhs.ldlt().solve(rhs);
        if (!parameters.allFinite()) {
            return fallback("nonfinite_fit");
        }
        residuals_for();
        for (Eigen::Index i = 0; i < count; ++i) {
            const double error = residual.segment<2>(2 * i).norm();
            weights(i) = std::min(1.0, 2.0 * pixel_scale / std::max(0.01, error));
        }
    }
    const double a = 1 + parameters(0) / norm, b = parameters(1) / norm;
    result.step = {a, -b, (1 - a) * center.x + b * center.y + parameters(2),
                   b, a,  -b * center.x + (1 - a) * center.y + parameters(3),
                   0, 0,  1};
    std::vector<double> residuals;
    residuals_for();
    for (Eigen::Index i = 0; i < count; ++i) {
        const double error = residual.segment<2>(2 * i).norm();
        residuals.push_back(error);
        if (error > 3 * pixel_scale || !bands)
            continue;
        const double location = horizontal_ ? p[i].x / previous.cols : p[i].y / previous.rows;
        ++band_counts[std::clamp(static_cast<int>(location * bands), 0, bands - 1)];
    }
    const auto score = quality(source, p, reliability, residuals, previous);
    record_quality(result, score);
    result.fit_confidence = result.confidence;
    result.residual_px = median(residuals);
    result.residual_normalized = result.residual_px / norm;
    if (!score.reliable())
        return fallback(score.coverage < 0.30 || score.span < 0.08 ? "distribution" : "consistency");
    if (result.residual_normalized > 6.0 / 960)
        return fallback("residual");
    if (const auto reason = geometry_failure(result.step, center, norm); !reason.empty())
        return fallback(reason);
    if (bands) {
        const double row_coverage =
            std::count_if(band_counts.begin(), band_counts.end(), [](int n) { return n >= 4; }) /
            static_cast<double>(bands);
        result.rs_confidence = score.consistency * std::min(1.0, score.coverage / 0.6) * row_coverage;
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
    if (adaptive_) {
        // Normalized similarity-model information. Keep the weakest direction
        // observable before reducing per-cell quotas; row-band coverage is gated separately.
        Eigen::Matrix4d information = Eigen::Matrix4d::Zero();
        double total = 0;
        for (size_t i = 0; i < p.size(); ++i) {
            const double x = (p[i].x - center.x) / norm, y = (p[i].y - center.y) / norm;
            Eigen::Matrix<double, 2, 4> jacobian;
            jacobian << x, -y, 1, 0, y, x, 0, 1;
            const double weight = reliability[i] / (1 + std::pow(residuals[i] / (3 * pixel_scale), 2));
            information.noalias() += weight * jacobian.transpose() * jacobian;
            total += weight;
        }
        information_ =
            total > 0 ? Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(information / total).eigenvalues()(0) : 0;
    }
    if (incremental_)
        for (size_t i = 0; i < q.size(); ++i)
            if (residuals[i] <= 3 * pixel_scale)
                tracked_.push_back(q[i]);
    return result;
}

} // namespace stillflow
