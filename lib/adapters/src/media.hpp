#pragma once

#include "stillflow/application/media.hpp"
#include <filesystem>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
}

namespace stillflow {
struct DecodedFrame {
    cv::Mat bgr;
    int64_t pts_us = 0, duration_us = 0;
};

class Reader {
  public:
    Reader(const std::filesystem::path& path, int threads);
    ~Reader();
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    bool next(DecodedFrame& frame);
    [[nodiscard]] const VideoInfo& info() const;
    [[nodiscard]] AVFormatContext* context() const;
    void set_audio_callback(std::function<void(const AVPacket*)> callback);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class Writer {
  public:
    Writer(const std::filesystem::path& path, cv::Size size, double fps, AVFormatContext* source, int64_t origin_us,
           int threads, int crf, const std::string& preset, bool audio);
    ~Writer();
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    void video(const cv::Mat& bgr, int64_t pts_us, int64_t duration_us);
    void audio(const AVPacket* packet);
    void finish();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace stillflow
