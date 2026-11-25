#pragma once
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace stillflow::test {
class TempDirectory {
  public:
    TempDirectory() {
        auto pattern = (std::filesystem::temp_directory_path() / "stillflow-test-XXXXXX").string();
        if (!mkdtemp(pattern.data()))
            throw std::runtime_error("Cannot create temporary test directory");
        path = pattern;
    }
    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    TempDirectory(const TempDirectory&) = delete;
    TempDirectory& operator=(const TempDirectory&) = delete;
    std::filesystem::path path;
};
} // namespace stillflow::test
