#include "image.hpp"
#include "media.hpp"
#include "stillflow/adapters/backends.hpp"
#include <mutex>
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
// A decoded frame left in device memory. Host mappings convert it once, on
// first use, with the same conversion the reader applies to host frames.
class DecodedDeviceImage final : public ImageData {
  public:
    DecodedDeviceImage(std::shared_ptr<const DeviceFrame> frame, Size size) : frame_(std::move(frame)), size_(size) {}
    Size size() const override { return size_; }
    PixelFormat format() const override { return PixelFormat::bgr8; }
    HostImageView map_read() const override {
        std::call_once(converted_, [&] { host_ = image(frame_->download()); });
        return host_->map_read();
    }
    std::optional<NativeImageView> native_read(std::string_view api) const override {
        if (api != cuda_image_api)
            return std::nullopt;
        return NativeImageView{std::string(api), &frame_->descriptor(), frame_};
    }

  private:
    std::shared_ptr<const DeviceFrame> frame_;
    Size size_;
    mutable std::once_flag converted_;
    mutable Image host_;
};
class FfmpegReader final : public VideoReader {
  public:
    FfmpegReader(const std::filesystem::path& path, int threads, const CodecRequest& decoder, HardwareDevices& devices)
        : reader(path, threads, decoder, devices) {
        if ((reader.info().width & 1) || (reader.info().height & 1))
            throw std::runtime_error("Output H.264 4:2:0 requires even input dimensions");
    }
    bool next(VideoFrame& frame) override {
        DecodedFrame decoded;
        if (!reader.next(decoded))
            return false;
        const Size size(reader.info().width, reader.info().height);
        auto pixels = decoded.device ? std::make_shared<DecodedDeviceImage>(std::move(decoded.device), size)
                                     : image(std::move(decoded.bgr));
        frame = {std::move(pixels), decoded.pts_us, decoded.duration_us};
        return true;
    }
    const VideoInfo& info() const override { return reader.info(); }
    CodecReport codec() const override { return reader.codec(); }
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
                 const Options& options, const CodecRequest& encoder, HardwareDevices& devices)
        : writer_(path, {size.width, size.height}, fps, dynamic_cast<FfmpegReader&>(source).reader.context(), origin,
                  options.threads, options.crf, options.preset, options.audio, encoder, devices) {}
    void video(const Image& input, int64_t pts, int64_t duration) override {
        if (!input)
            throw std::invalid_argument("Missing video frame");
        if (auto native = input->native_read(cuda_image_api); native && native->handle && native->lifetime) {
            const auto& device = *static_cast<const CudaImage*>(native->handle);
            if (writer_.accepts(device)) {
                writer_.video(device, std::move(native->lifetime), pts, duration);
                return;
            }
        }
        auto view = read_pixels(input);
        if (view.mapping.format != PixelFormat::bgr8)
            throw std::invalid_argument("Video writer requires BGR8 input");
        writer_.video(view.pixels, pts, duration);
    }
    void audio(const AudioPacket& packet) override {
        writer_.audio(dynamic_cast<const FfmpegAudioPacket&>(packet).packet);
    }
    void finish() override { writer_.finish(); }
    CodecReport codec() const override { return writer_.codec(); }

  private:
    Writer writer_;
};
class FfmpegMedia final : public MediaFactory {
  public:
    std::unique_ptr<VideoReader> reader(const std::filesystem::path& path, int threads,
                                        const CodecRequest& decoder) override {
        return std::make_unique<FfmpegReader>(path, threads, decoder, devices_);
    }
    std::unique_ptr<VideoWriter> writer(const std::filesystem::path& path, Size size, double fps, VideoReader& source,
                                        int64_t origin, const Options& options, const CodecRequest& encoder) override {
        return std::make_unique<FfmpegWriter>(path, size, fps, source, origin, options, encoder, devices_);
    }

  private:
    HardwareDevices devices_;
};
} // namespace
std::unique_ptr<MediaFactory> make_media_factory() { return std::make_unique<FfmpegMedia>(); }
} // namespace stillflow
