#include "stillflow/adapters/backends.hpp"
#include <opencv2/core.hpp>
#include <sys/resource.h>
extern "C" {
#include <libavutil/log.h>
}

namespace stillflow {
namespace {
class HostRuntime final : public Runtime {
public:
    void configure(int threads) override {
        cv::setNumThreads(threads);
        cv::setRNGSeed(42);
        av_log_set_level(AV_LOG_WARNING);
    }
    ResourceUsage resources() const override {
        struct rusage usage{};
        getrusage(RUSAGE_SELF, &usage);
        return {usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
                usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6,
                usage.ru_maxrss / 1024.0};
    }
};
}
std::unique_ptr<Runtime> make_runtime() { return std::make_unique<HostRuntime>(); }
std::string build_description() {
    return "stillflow " STILLFLOW_VERSION " (C++20, OpenCV " CV_VERSION
        ")";
}
} // namespace stillflow
