#pragma once
#include "stillflow/application/vision.hpp"

namespace stillflow {
// Dependency-free translation estimator for constrained hosts.
std::unique_ptr<MotionAnalyzer> make_translation_analyzer(const StabilizationSettings& settings);
} // namespace stillflow
