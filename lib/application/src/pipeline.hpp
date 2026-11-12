#pragma once
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace stillflow {
// Bounded FIFO between pipeline stages. close() ends the stream after queued
// items drain; cancel() discards them and releases blocked producers/consumers.
template <class T> class Channel {
  public:
    explicit Channel(size_t capacity) : capacity_(capacity ? capacity : 1) {}
    bool push(T value) {
        std::unique_lock lock(mutex_);
        writable_.wait(lock, [&] { return cancelled_ || items_.size() < capacity_; });
        if (cancelled_ || closed_)
            return false;
        items_.push_back(std::move(value));
        readable_.notify_one();
        return true;
    }
    std::optional<T> pop() {
        std::unique_lock lock(mutex_);
        readable_.wait(lock, [&] { return cancelled_ || closed_ || !items_.empty(); });
        if (cancelled_ || items_.empty())
            return std::nullopt;
        auto value = std::move(items_.front());
        items_.pop_front();
        writable_.notify_one();
        return value;
    }
    void close() {
        std::lock_guard lock(mutex_);
        closed_ = true;
        readable_.notify_all();
        writable_.notify_all();
    }
    bool cancelled() {
        std::lock_guard lock(mutex_);
        return cancelled_;
    }
    void cancel() {
        std::lock_guard lock(mutex_);
        cancelled_ = true;
        items_.clear();
        readable_.notify_all();
        writable_.notify_all();
    }

  private:
    std::mutex mutex_;
    std::condition_variable readable_, writable_;
    std::deque<T> items_;
    size_t capacity_;
    bool closed_ = false, cancelled_ = false;
};

// Owns stage threads. The first failure from any thread (or the caller via
// fail()) cancels every registered channel; rethrow() reports that failure.
class ThreadGroup {
  public:
    ThreadGroup() = default;
    ThreadGroup(const ThreadGroup&) = delete;
    ThreadGroup& operator=(const ThreadGroup&) = delete;
    ~ThreadGroup() {
        cancel();
        join();
    }
    // Register every channel before spawning threads.
    template <class T> void watch(Channel<T>& channel) {
        cancellers_.push_back([&channel] { channel.cancel(); });
    }
    template <class F> void spawn(F work) {
        threads_.emplace_back([this, work = std::move(work)]() mutable {
            try {
                work();
            } catch (...) {
                fail(std::current_exception());
            }
        });
    }
    void fail(std::exception_ptr error) {
        {
            std::lock_guard lock(mutex_);
            if (!error_)
                error_ = std::move(error);
        }
        cancel();
    }
    void join() {
        for (auto& thread : threads_)
            if (thread.joinable())
                thread.join();
    }
    // Joins all threads, then rethrows the first recorded failure.
    void finish() {
        join();
        if (error_)
            std::rethrow_exception(error_);
    }

  private:
    void cancel() {
        for (const auto& cancel : cancellers_)
            cancel();
    }
    std::vector<std::function<void()>> cancellers_;
    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::exception_ptr error_;
};
} // namespace stillflow
