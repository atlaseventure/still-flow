#include "image.hpp"
#include "media.hpp"
#include "stillflow/adapters/backends.hpp"
#include <stdexcept>

namespace stillflow {
namespace {
class FfmpegAudioPacket final : public AudioPacket {
  public:
    explicit FfmpegAudioPacket(const AVPacket* value) : packet(value) {}
    ~FfmpegAudioPacket() override { av_packet_free(&owned_); }
    FfmpegAudioPacket(const FfmpegAudioPacket&) = delete;
    FfmpegAudioPacket& operator=(const FfmpegAudioPacket&) = delete;
    std::shared_ptr<const AudioPacket> retain() const override {
        auto copy = std::make_shared<FfmpegAudioPacket>(nullptr);
        copy->owned_ = av_packet_clone(packet);
        if (!copy->owned_)
            throw std::bad_alloc();
        copy->packet = copy->owned_;
        return copy;
    }
    const AVPacket* packet;

  private:
    AVPacket* owned_ = nullptr;
};
class FfmpegReader final : public VideoReader {
  public:
    FfmpegReader(const std::filesystem::path& path, int threads) : reader(path, threads) {
        if ((reader.info().width & 1) || (reader.info().height & 1))
            throw std::runtime_error("Output H.264 4:2:0 requires even input dimensions");
    }
    bool next(VideoFrame& frame) override {
        DecodedFrame decoded;
        if (!reader.next(decoded))
            return false;
        frame = {image(std::move(decoded.bgr)), decoded.pts_us, decoded.duration_us};
        return true;
    }
    const VideoInfo& info() const override { return reader.info(); }
    void set_audio_callback(std::function<void(const AudioPacket&)> callback) override {
        reader.set_audio_callback([callback = std::move(callback)](const AVPacket* packet) {
            if (callback)
                callback(FfmpegAudioPacket(packet));
        });
    }
    Reader reader;
};
class FfmpegWriter final : public VideoWriter {
  public:
    FfmpegWriter(const std::filesystem::path& path, Size size, double fps, VideoReader& source, int64_t origin,
                 const Options& options)
        : writer_(path, {size.width, size.height}, fps, dynamic_cast<FfmpegReader&>(source).reader.context(), origin,
                  options.threads, options.crf, options.preset, options.audio) {}
    void video(const Image& input, int64_t pts, int64_t duration) override {
        auto view = read_pixels(input);
        if (view.mapping.format != PixelFormat::bgr8)
            throw std::invalid_argument("Video writer requires BGR8 input");
        writer_.video(view.pixels, pts, duration);
    }
    void audio(const AudioPacket& packet) override {
        writer_.audio(dynamic_cast<const FfmpegAudioPacket&>(packet).packet);
    }
    void finish() override { writer_.finish(); }

  private:
    Writer writer_;
};
class FfmpegMedia final : public MediaFactory {
  public:
    std::unique_ptr<VideoReader> reader(const std::filesystem::path& path, int threads) override {
        return std::make_unique<FfmpegReader>(path, threads);
    }
    std::unique_ptr<VideoWriter> writer(const std::filesystem::path& path, Size size, double fps, VideoReader& source,
                                        int64_t origin, const Options& options) override {
        return std::make_unique<FfmpegWriter>(path, size, fps, source, origin, options);
    }
};
} // namespace
std::unique_ptr<MediaFactory> make_media_factory() { return std::make_unique<FfmpegMedia>(); }
} // namespace stillflow
