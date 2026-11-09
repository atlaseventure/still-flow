#pragma once

#include "stillflow/application/media.hpp"
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
}

namespace stillflow {
// One CUDA device context, created on first use and shared by every reader and
// writer of a media factory so NVDEC and NVENC sessions share one context.
class HardwareDevices {
  public:
    HardwareDevices() = default;
    ~HardwareDevices();
    HardwareDevices(const HardwareDevices&) = delete;
    HardwareDevices& operator=(const HardwareDevices&) = delete;
    // Returns a new reference, or nullptr with the reason in error.
    AVBufferRef* cuda(std::string& error);

  private:
    std::mutex mutex_;
    AVBufferRef* cuda_ = nullptr;
    bool attempted_ = false;
    std::string error_;
};

struct DecodedFrame {
    cv::Mat bgr;
    int64_t pts_us = 0, duration_us = 0;
};

class Reader {
  public:
    Reader(const std::filesystem::path& path, int threads, const CodecRequest& decoder, HardwareDevices& devices);
    ~Reader();
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    bool next(DecodedFrame& frame);
    [[nodiscard]] const VideoInfo& info() const;
    [[nodiscard]] AVFormatContext* context() const;
    void set_audio_callback(std::function<void(const AVPacket*)> callback);
    [[nodiscard]] CodecReport codec() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class Writer {
  public:
    Writer(const std::filesystem::path& path, cv::Size size, double fps, AVFormatContext* source, int64_t origin_us,
           int threads, int crf, const std::string& preset, bool audio, const CodecRequest& encoder,
           HardwareDevices& devices);
    ~Writer();
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    void video(const cv::Mat& bgr, int64_t pts_us, int64_t duration_us);
    void audio(const AVPacket* packet);
    void finish();
    [[nodiscard]] const CodecReport& codec() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace stillflow
