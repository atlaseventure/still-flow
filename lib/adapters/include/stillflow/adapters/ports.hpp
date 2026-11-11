#pragma once
#include "stillflow/adapters/report.hpp"
#include "stillflow/vision/vision.hpp"
#include <functional>
#include <span>

namespace stillflow {
// Threading: stabilize() runs decoding, motion estimation, rendering and each
// encoder on separate threads. An object is never called concurrently, but it
// may be called from a thread other than the one that created it. Distinct
// MotionAnalyzer instances run concurrently, and RendererFactory::create may
// run concurrently with the analysis pass.
class VideoReader {
  public:
    virtual ~VideoReader() = default;
    virtual bool next(VideoFrame& frame) = 0;
    // Analysis uses unexpanded source luma where the decoder exposes it.
    virtual bool next_luma(VideoFrame& frame) { return next(frame); }
    virtual const VideoInfo& info() const = 0;
    // Invoked on the thread calling next(), before it returns the frame demuxed alongside.
    virtual void set_audio_callback(std::function<void(const AudioPacket&)> callback) = 0;
    // Valid after the last next() call; reflects every frame decoded so far.
    virtual CodecReport codec() const = 0;
};
class VideoWriter {
  public:
    virtual ~VideoWriter() = default;
    virtual void video(const Image& image, int64_t pts_us, int64_t duration_us) = 0;
    virtual void audio(const AudioPacket& packet) = 0;
    virtual void finish() = 0;
    virtual CodecReport codec() const = 0;
};
class MediaFactory {
  public:
    virtual ~MediaFactory() = default;
    // Check adapter-specific output constraints before decoding starts.
    virtual void validate_output(const std::filesystem::path& path) const = 0;
    virtual std::unique_ptr<VideoReader> reader(const std::filesystem::path& path, int threads,
                                                const CodecRequest& decoder) = 0;
    // The source reader must outlive all writers created from it. Writers run on
    // their own threads while the reader keeps decoding.
    virtual std::unique_ptr<VideoWriter> writer(const std::filesystem::path& path, Size size, double fps,
                                                VideoReader& source, int64_t origin_us,
                                                const EncodingSettings& settings, const CodecRequest& encoder) = 0;
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
    virtual void validate(const std::filesystem::path& input, std::span<const std::filesystem::path> outputs,
                          bool overwrite) = 0;
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
    // Called once per video stream after it is fully decoded or encoded.
    virtual void codec(const std::string& /*stream*/, const CodecReport& /*report*/) {}
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
