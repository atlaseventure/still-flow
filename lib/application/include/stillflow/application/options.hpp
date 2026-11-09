#pragma once
#include "stillflow/domain/settings.hpp"
#include <filesystem>
#include <string>

namespace stillflow {
struct Options : StabilizationSettings {
    std::filesystem::path input, output, compare, report;
    int threads = 12;
    int crf = 18;
    std::string scan_axis = "auto";
    std::string preset = "fast";
    std::string backend = "auto";
    bool audio = true;
    bool overwrite = false;
};
} // namespace stillflow
