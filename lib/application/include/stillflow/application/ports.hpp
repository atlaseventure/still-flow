#pragma once
#include "stillflow/application/report.hpp"
#include <functional>

namespace stillflow {
class VideoReader {
public:
    virtual ~VideoReader() = default;
    virtual bool next(VideoFrame& frame) = 0;
    virtual const VideoInfo& info() const = 0;
    virtual void set_audio_callback(std::function<void(const AudioPacket&)> callback) = 0;
};
class VideoWriter {
public:
    virtual ~VideoWriter() = default;
    virtual void video(const Image& image, int64_t pts_us, int64_t duration_us) = 0;
    virtual void audio(const AudioPacket& packet) = 0;
    virtual void finish() = 0;
};
class MediaFactory {
public:
    virtual ~MediaFactory() = default;
    virtual std::unique_ptr<VideoReader> reader(const std::filesystem::path& path, int threads) = 0;
    // The source reader must outlive all writers created from it.
    virtual std::unique_ptr<VideoWriter> writer(const std::filesystem::path& path, Size size,
        double fps, VideoReader& source, int64_t origin_us, const Options& options) = 0;
};
class MotionAnalyzer {
public:
    virtual ~MotionAnalyzer() = default;
    virtual Motion estimate(const Image& previous, const Image& current, Profile& profile) = 0;
};
class ImageProcessing {
public:
    virtual ~ImageProcessing() = default;
    virtual std::unique_ptr<MotionAnalyzer> analyzer(const StabilizationSettings& settings, bool horizontal) = 0;
    virtual Image gray_proxy(const Image& image, Size size) = 0;
    virtual Image comparison(const Image& input, const Image& stabilized) = 0;
};
class OutputFile {
public:
    virtual ~OutputFile() = default;
    virtual const std::filesystem::path& temporary() const = 0;
    virtual void publish() = 0;
};
class OutputStorage {
public:
    virtual ~OutputStorage() = default;
    virtual void validate(const Options& options, const std::filesystem::path& frames_path) = 0;
    virtual std::unique_ptr<OutputFile> stage(const std::filesystem::path& target) = 0;
    virtual void write_report(const std::filesystem::path& json, const std::filesystem::path& csv,
        const RunReport& report) = 0;
};
class Runtime {
public:
    virtual ~Runtime() = default;
    virtual void configure(int threads) = 0;
    virtual ResourceUsage resources() const = 0;
};
class Progress {
public:
    virtual ~Progress() = default;
    virtual void backend(const RenderingReport& report) = 0;
    virtual void analyzing(const VideoInfo& info, Size analysis_size, bool horizontal) = 0;
    virtual void analyzed(size_t frames, bool complete) = 0;
    virtual void rendered(size_t frames, size_t total, bool complete) = 0;
};
struct Services {
    MediaFactory& media;
    ImageProcessing& images;
    RendererFactory& renderers;
    OutputStorage& outputs;
    Runtime& runtime;
    Progress& progress;
};
} // namespace stillflow
