#pragma once
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string_view>

namespace stillflow {
class Cancelled : public std::runtime_error {
  public:
    Cancelled() : std::runtime_error("Operation cancelled") {}
};
class CancellationToken {
  public:
    virtual ~CancellationToken() = default;
    virtual bool cancelled() const noexcept = 0;
};
// A synchronous fork/join contract: every item completes before return; failures
// are rethrown after all workers stop. Hosts choose threads and affinity.
class Executor {
  public:
    virtual ~Executor() = default;
    virtual void for_each(size_t count, const std::function<void(size_t)>& work) = 0;
};
class Logger {
  public:
    virtual ~Logger() = default;
    virtual void log(std::string_view message) = 0;
};
struct ExecutionSettings {
    CancellationToken* cancellation = nullptr;
    Executor* executor = nullptr;
    Logger* logger = nullptr;
    void check() const {
        if (cancellation && cancellation->cancelled())
            throw Cancelled();
    }
    void for_each(size_t count, const std::function<void(size_t)>& work) const {
        check();
        auto checked = [&](size_t i) {
            check();
            work(i);
        };
        if (executor)
            executor->for_each(count, checked);
        else
            for (size_t i = 0; i < count; ++i)
                checked(i);
        check();
    }
};
} // namespace stillflow
