#pragma once

#include "stillflow/application/media.hpp"
#include "stillflow/interop/cuda_image.hpp"
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
// writer of a media factory so NVDEC and NVENC sessions share one context. It is
// the device's primary context, which the CUDA runtime also uses, so decoded
// and rendered frames can be exchanged as device pointers.
class HardwareDevices {
  public:
    HardwareDevices();
    ~HardwareDevices();
    HardwareDevices(const HardwareDevices&) = delete;
    HardwareDevices& operator=(const HardwareDevices&) = delete;
    // Returns a new reference, or nullptr with the reason in error.
    AVBufferRef* cuda(std::string& error);
    // Waits for copies FFmpeg queued on the device context's stream.
    void synchronize();

  private:
    struct Driver;
    std::mutex mutex_;
    std::unique_ptr<Driver> driver_;
    AVBufferRef* cuda_ = nullptr;
    bool attempted_ = false;
    std::string error_;
};

// Converts decoded frames (host or CUDA) to packed, display-rotated BGR. Safe
// to call from several threads; conversions are serialized.
class HostConverter;

// An NVDEC frame kept in device memory. download() matches the host BGR or
// source-luma path and may run on any thread.
class DeviceFrame {
  public:
    // References (does not take) frame.
    DeviceFrame(const AVFrame* frame, std::shared_ptr<HostConverter> converter, int rotation);
    ~DeviceFrame();
    DeviceFrame(const DeviceFrame&) = delete;
    DeviceFrame& operator=(const DeviceFrame&) = delete;
    [[nodiscard]] const CudaImage& descriptor() const { return descriptor_; }
    [[nodiscard]] cv::Mat download(bool luma_only = false) const;

  private:
    AVFrame* frame_;
    std::shared_ptr<HostConverter> converter_;
    CudaImage descriptor_;
};

struct DecodedFrame {
    Image planar;
    cv::Mat luma;
    cv::Mat pixels;                            // Host BGR or source luma.
    std::shared_ptr<const DeviceFrame> device; // NVDEC frames left on the GPU.
    int64_t pts_us = 0, duration_us = 0;
};

class Reader {
  public:
    Reader(const std::filesystem::path& path, int threads, const CodecRequest& decoder, HardwareDevices& devices);
    ~Reader();
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    bool next(DecodedFrame& frame, bool luma_only = false);
    bool scan_timestamps(const std::function<void(FrameTimestamp)>& visit, const std::function<void()>& cancel);
    [[nodiscard]] const VideoInfo& info() const;
    [[nodiscard]] AVFormatContext* context() const;
    [[nodiscard]] AVRational video_time_base() const;
    void set_audio_callback(std::function<void(const AVPacket*)> callback);
    [[nodiscard]] CodecReport codec() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class Writer {
  public:
    Writer(const std::filesystem::path& path, cv::Size size, double fps, const Reader& reader, int64_t origin_us,
           int threads, int crf, const std::string& preset, AudioMode audio, const CodecRequest& encoder,
           HardwareDevices& devices);
    ~Writer();
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    void video(const cv::Mat& bgr, int64_t pts_us, int64_t duration_us);
    void video(const HostImageView& yuv, int64_t pts_us, int64_t duration_us);
    // Whether video(image, ...) can encode this device image without a copy.
    [[nodiscard]] bool accepts(const CudaImage& image) const;
    // Encodes device memory in place; lifetime is held until NVENC has read it.
    void video(const CudaImage& image, std::shared_ptr<const void> lifetime, int64_t pts_us, int64_t duration_us);
    void audio(const AVPacket* packet);
    void finish();
    [[nodiscard]] const CodecReport& codec() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace stillflow
