#pragma once
#include "stillflow/application/options.hpp"

namespace stillflow {
GyroInput load_gyro_input(const std::filesystem::path& config_path, const std::filesystem::path& imu_override = {});
} // namespace stillflow
