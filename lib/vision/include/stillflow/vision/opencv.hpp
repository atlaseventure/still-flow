#pragma once
#include "stillflow/application/vision.hpp"
#include "stillflow/vision/tracking.hpp"

namespace stillflow {
// Optional OpenCV implementation of the full tracking algorithm.
std::unique_ptr<MotionAnalyzer> make_opencv_analyzer(const StabilizationSettings& settings, bool horizontal,
                                                     bool incremental = false,
                                                     std::unique_ptr<SparseTracker> tracker = {});
// Process-wide setting; configure before creating analysis workers.
void configure_opencv_threads(int threads);
} // namespace stillflow
