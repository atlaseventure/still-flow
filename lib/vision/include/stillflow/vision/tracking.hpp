#pragma once
#include "stillflow/application/image.hpp"
#include "stillflow/application/profile.hpp"
#include "stillflow/domain/geometry.hpp"
#include <span>

namespace stillflow {
struct TrackingPlane {
    Size size;
    const uint8_t* pixels;
    size_t stride;
};
struct TrackedPoint {
    Point destination, backward;
    double error = 0;
    bool valid = false;
};
// Coordinates and photometric errors are in source-proxy pixels and gray8 units.
// Implementations own retained resources; input storage is borrowed for the call.
class SparseTracker {
  public:
    virtual ~SparseTracker() = default;
    virtual std::vector<TrackedPoint> track(TrackingPlane previous, TrackingPlane current,
                                            std::span<const Point> source, std::span<const Point> initial, int levels,
                                            int window, Profile& profile) = 0;
};
} // namespace stillflow
