#include "stillflow/application/runtime.hpp"
#include "pipeline.hpp"
#include <algorithm>
#include <atomic>
#include <memory>

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
} // namespace
std::unique_ptr<Executor> make_thread_executor(int workers) { return std::make_unique<ThreadExecutor>(workers); }
} // namespace stillflow
