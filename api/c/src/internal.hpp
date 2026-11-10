#pragma once
#include "stillflow/application/engine.hpp"
#include "stillflow/stillflow.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <exception>
#include <memory>

namespace stillflow::c_api {
template <class T> void require(const T* value) {
    if (!value || value->struct_size < sizeof(T))
        throw std::invalid_argument("Missing or undersized structure");
}
inline void message(sf_error* error, const char* text) noexcept {
    if (error && error->struct_size >= sizeof(sf_error)) {
        std::strncpy(error->message, text, sizeof(error->message) - 1);
        error->message[sizeof(error->message) - 1] = '\0';
    }
}
template <class F> sf_status boundary(sf_error* error, F work) noexcept {
    if (error && error->struct_size < sizeof(sf_error))
        return SF_INVALID_ARGUMENT;
    message(error, "");
    try {
        work();
        return SF_OK;
    } catch (const Cancelled& e) {
        message(error, e.what());
        return SF_CANCELLED;
    } catch (const std::bad_alloc&) {
        message(error, "Allocation failed");
        return SF_OUT_OF_MEMORY;
    } catch (const std::invalid_argument& e) {
        message(error, e.what());
        return SF_INVALID_ARGUMENT;
    } catch (const std::out_of_range& e) {
        message(error, e.what());
        return SF_INVALID_ARGUMENT;
    } catch (const std::logic_error& e) {
        message(error, e.what());
        return SF_INVALID_STATE;
    } catch (const BackendError& e) {
        message(error, e.what());
        return SF_UNSUPPORTED;
    } catch (const std::exception& e) {
        message(error, e.what());
        return SF_INTERNAL_ERROR;
    } catch (...) {
        message(error, "Unknown failure");
        return SF_INTERNAL_ERROR;
    }
}
class Runtime final : public CancellationToken, public Logger, public Executor {
  public:
    explicit Runtime(const sf_runtime* callbacks) {
        if (callbacks) {
            require(callbacks);
            callbacks_ = *callbacks;
        }
    }
    bool cancelled() const noexcept override {
        return callbacks_.cancelled && callbacks_.cancelled(callbacks_.user_data);
    }
    void log(std::string_view text) override {
        if (callbacks_.log) {
            std::string copy(text);
            callbacks_.log(callbacks_.user_data, copy.c_str());
        }
    }
    void for_each(size_t count, const std::function<void(size_t)>& work) override {
        if (!callbacks_.execute) {
            for (size_t i = 0; i < count; ++i)
                work(i);
            return;
        }
        // Errors stay within C++; the host's C scheduler never receives exceptions.
        auto errors = std::make_unique<std::exception_ptr[]>(count);
        auto visited = std::make_unique<std::atomic_bool[]>(count);
        struct Context {
            const std::function<void(size_t)>& work;
            std::exception_ptr* errors;
            std::atomic_bool* visited;
            size_t count;
            std::atomic_bool invalid{false};
        } context{work, errors.get(), visited.get(), count};
        callbacks_.execute(
            callbacks_.user_data, count,
            [](void* data, size_t i) noexcept {
                auto& ctx = *static_cast<Context*>(data);
                if (i >= ctx.count || ctx.visited[i].exchange(true)) {
                    ctx.invalid = true;
                    return;
                }
                try {
                    ctx.work(i);
                } catch (...) {
                    ctx.errors[i] = std::current_exception();
                }
            },
            &context);
        if (context.invalid)
            throw std::invalid_argument("Executor returned an invalid index");
        for (size_t i = 0; i < count; ++i) {
            if (!visited[i])
                throw std::invalid_argument("Executor omitted work");
            if (errors[i])
                std::rethrow_exception(errors[i]);
        }
    }
    ExecutionSettings settings() { return {this, this, this}; }

  private:
    sf_runtime callbacks_{};
};
inline StabilizationSettings settings(const sf_settings* value) {
    require(value);
    StabilizationSettings result;
    result.analysis_width = value->analysis_width;
    result.features = value->features;
    result.bands = value->bands;
    result.crop_mode = static_cast<CropMode>(value->crop_mode);
    result.rolling_shutter = value->rolling_shutter != 0;
    result.smooth_seconds = value->smooth_seconds;
    result.rs_seconds = value->rs_seconds;
    result.crop = value->crop;
    result.rs_strength = value->rs_strength;
    validate_settings(result);
    return result;
}
} // namespace stillflow::c_api
