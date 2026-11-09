#include "stillflow/domain/motion.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace stillflow {
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

} // namespace stillflow
