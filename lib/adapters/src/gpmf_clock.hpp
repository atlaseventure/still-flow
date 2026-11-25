#pragma once
#include "stillflow/domain/gyro.hpp"
#include <filesystem>
#include <optional>

namespace stillflow {
// Period of a continuous, packet-timed GPMF gyro series, from the camera's
// millisecond TICK at each packet's first sample. Other clock layouts use CSV time.
std::optional<double> gpmf_sample_period(const std::filesystem::path& path, std::span<const ImuSample> samples);
} // namespace stillflow
