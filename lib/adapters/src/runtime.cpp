#include "pipeline.hpp"
#include "stillflow/adapters/backends.hpp"
#include <atomic>
#include <opencv2/core.hpp>
#include <sys/resource.h>
extern "C" {
#include <libavutil/log.h>
}

namespace stillflow {
namespace {
class ThreadExecutor final : public Executor {
  public:
    explicit ThreadExecutor(int workers) : workers_(workers) {
        if (workers < 1 || workers > 64)
            throw std::invalid_argument("Invalid worker count");
    }
    void for_each(size_t count, const std::function<void(size_t)>& work) override {
        std::atomic_size_t next{0};
        std::atomic_bool stopped{false};
        ThreadGroup group;
        for (size_t i = 0; i < std::min(count, static_cast<size_t>(workers_)); ++i)
            group.spawn([&] {
                try {
                    while (!stopped) {
                        const auto index = next.fetch_add(1);
                        if (index >= count)
                            break;
                        work(index);
                    }
                } catch (...) {
                    stopped = true;
                    throw;
                }
            });
        group.finish();
    }

  private:
    int workers_;
};
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
        return {usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 + usage.ru_stime.tv_sec +
                    usage.ru_stime.tv_usec / 1e6,
                usage.ru_maxrss / 1024.0};
    }
};
} // namespace
std::unique_ptr<Executor> make_thread_executor(int workers) { return std::make_unique<ThreadExecutor>(workers); }
std::unique_ptr<Runtime> make_runtime() { return std::make_unique<HostRuntime>(); }
std::string build_description() { return "stillflow " STILLFLOW_VERSION " (C++20, OpenCV " CV_VERSION ")"; }
} // namespace stillflow
