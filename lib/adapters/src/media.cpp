#include "media.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/display.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
// nv-codec-headers ship only with NVIDIA codec support; they load the driver
// API at runtime, so builds still need no CUDA Toolkit.
#if __has_include(<ffnvcodec/dynlink_loader.h>)
#define STILLFLOW_CUDA_DRIVER 1
#define FFNV_LOG_FUNC(logctx, msg, ...) av_log(logctx, AV_LOG_ERROR, msg, __VA_ARGS__)
#define FFNV_DEBUG_LOG_FUNC(logctx, msg, ...) av_log(logctx, AV_LOG_DEBUG, msg, __VA_ARGS__)
#include <ffnvcodec/dynlink_loader.h>
#include <libavutil/hwcontext_cuda.h>
#endif
}

namespace stillflow {
namespace {
constexpr AVRational microseconds{1, 1000000};
std::string error_text(int result) {
    char message[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(result, message, sizeof(message));
    return message;
}
void check(int result, const std::string& operation) {
    if (result < 0)
        throw std::runtime_error(operation + ": " + error_text(result));
}
// The device is reported by ordinal: FFmpeg loads the driver API privately.
constexpr const char* cuda_device = "cuda:0";
bool decodes_with_cuda(const AVCodec* codec) {
    for (int i = 0;; ++i) {
        const auto* config = avcodec_get_hw_config(codec, i);
        if (!config)
            return false;
        if (config->device_type == AV_HWDEVICE_TYPE_CUDA && (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX))
            return true;
    }
}
// Splits NV12/NV16 chroma into the planar layout software decoders emit, so the
// colour conversion below runs the same swscale path for both decoders.
AVPixelFormat planar_equivalent(AVPixelFormat format, bool full_range) {
    switch (format) {
    case AV_PIX_FMT_NV12:
        return full_range ? AV_PIX_FMT_YUVJ420P : AV_PIX_FMT_YUV420P;
    case AV_PIX_FMT_NV16:
        return full_range ? AV_PIX_FMT_YUVJ422P : AV_PIX_FMT_YUV422P;
    default:
        return AV_PIX_FMT_NONE;
    }
}
void deinterleave(const AVFrame* semi, AVFrame* planar) {
    const int chroma_width = (semi->width + 1) / 2;
    const int chroma_height = semi->format == AV_PIX_FMT_NV12 ? (semi->height + 1) / 2 : semi->height;
    av_image_copy_plane(planar->data[0], planar->linesize[0], semi->data[0], semi->linesize[0], semi->width,
                        semi->height);
    for (int y = 0; y < chroma_height; ++y) {
        const uint8_t* uv = semi->data[1] + static_cast<ptrdiff_t>(y) * semi->linesize[1];
        uint8_t* u = planar->data[1] + static_cast<ptrdiff_t>(y) * planar->linesize[1];
        uint8_t* v = planar->data[2] + static_cast<ptrdiff_t>(y) * planar->linesize[2];
        for (size_t x = 0; x < static_cast<size_t>(chroma_width); ++x) {
            u[x] = uv[2 * x];
            v[x] = uv[2 * x + 1];
        }
    }
}
// Maps x264 preset names onto NVENC's p1 (fastest) .. p7 (best) scale.
const char* nvenc_preset(const std::string& preset) {
    static const std::map<std::string, const char*> presets{
        {"ultrafast", "p1"}, {"superfast", "p1"}, {"veryfast", "p2"}, {"faster", "p3"},   {"fast", "p4"},
        {"medium", "p5"},    {"slow", "p6"},      {"slower", "p7"},   {"veryslow", "p7"}, {"placebo", "p7"},
        {"p1", "p1"},        {"p2", "p2"},        {"p3", "p3"},       {"p4", "p4"},       {"p5", "p5"},
        {"p6", "p6"},        {"p7", "p7"}};
    const auto found = presets.find(preset);
    if (found == presets.end())
        throw std::invalid_argument("Preset " + preset + " has no NVENC equivalent; use an x264 preset or p1..p7");
    return found->second;
}
int colorspace(AVColorSpace space) {
    switch (space) {
    case AVCOL_SPC_BT709:
        return SWS_CS_ITU709;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL:
        return SWS_CS_BT2020;
    case AVCOL_SPC_FCC:
        return SWS_CS_FCC;
    default:
        return SWS_CS_ITU601;
    }
}
// Explicitly initialized contexts use swscale's frame path, which splits each
// conversion into slices across threads with the same output as sws_scale().
SwsContext* make_scaler(int width, int height, AVPixelFormat source, AVPixelFormat target, int flags, int threads,
                        const char* operation) {
    SwsContext* sws = sws_alloc_context();
    if (!sws)
        throw std::bad_alloc();
    sws->flags = flags;
    sws->src_w = sws->dst_w = width;
    sws->src_h = sws->dst_h = height;
    sws->src_format = source;
    sws->dst_format = target;
    sws->threads = threads;
    if (sws_init_context(sws, nullptr, nullptr) < 0) {
        sws_freeContext(sws);
        throw std::runtime_error(operation);
    }
    return sws;
}
// Colour matrix and chroma siting of a decoded frame, as the device descriptor states them.
void describe_colour(const AVFrame* frame, CudaImage& image) {
    switch (frame->colorspace) {
    case AVCOL_SPC_BT709:
        image.kr = 0.2126f, image.kb = 0.0722f;
        break;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL:
        image.kr = 0.2627f, image.kb = 0.0593f;
        break;
    case AVCOL_SPC_FCC:
        image.kr = 0.30f, image.kb = 0.11f;
        break;
    default: // Matches colorspace(): everything else converts as BT.601.
        image.kr = 0.299f, image.kb = 0.114f;
    }
    image.full_range = frame->color_range == AVCOL_RANGE_JPEG;
    switch (frame->chroma_location) {
    case AVCHROMA_LOC_CENTER:
        image.chroma_x = 0.5f, image.chroma_y = 0.5f;
        break;
    case AVCHROMA_LOC_TOPLEFT:
        image.chroma_x = 0, image.chroma_y = 0;
        break;
    case AVCHROMA_LOC_TOP:
        image.chroma_x = 0.5f, image.chroma_y = 0;
        break;
    case AVCHROMA_LOC_BOTTOMLEFT:
        image.chroma_x = 0, image.chroma_y = 1;
        break;
    case AVCHROMA_LOC_BOTTOM:
        image.chroma_x = 0.5f, image.chroma_y = 1;
        break;
    default: // Left, the H.264/HEVC default when unspecified.
        image.chroma_x = 0, image.chroma_y = 0.5f;
    }
}
AVPixelFormat software_format(const AVFrame* frame) {
    if (frame->format == AV_PIX_FMT_CUDA && frame->hw_frames_ctx)
        return reinterpret_cast<const AVHWFramesContext*>(frame->hw_frames_ctx->data)->sw_format;
    return static_cast<AVPixelFormat>(frame->format);
}
void keep_pixels(void*, uint8_t*) {}
void release_lifetime(void* lifetime, uint8_t*) { delete static_cast<std::shared_ptr<const void>*>(lifetime); }
// References caller-owned packed pixels without copying; swscale requires
// reference-counted frames and would otherwise duplicate the image.
void borrow(AVFrame* frame, uint8_t* data, size_t stride, int width, int height, AVPixelFormat format) {
    av_frame_unref(frame);
    frame->width = width;
    frame->height = height;
    frame->format = format;
    frame->data[0] = data;
    frame->linesize[0] = static_cast<int>(stride);
    frame->buf[0] = av_buffer_create(data, stride * height, keep_pixels, nullptr, 0);
    if (!frame->buf[0])
        throw std::bad_alloc();
}
} // namespace

#ifdef STILLFLOW_CUDA_DRIVER
struct HardwareDevices::Driver {
    CudaFunctions* cu = nullptr;
    CUdevice device = 0;
    CUcontext context = nullptr;
    ~Driver() {
        if (context)
            cu->cuDevicePrimaryCtxRelease(device);
        cuda_free_functions(&cu);
    }
    void push() const {
        if (const auto result = cu->cuCtxPushCurrent(context))
            throw std::runtime_error("Activate CUDA context: driver error " + std::to_string(result));
    }
    void pop() const {
        CUcontext popped = nullptr;
        cu->cuCtxPopCurrent(&popped);
    }
};
#else
struct HardwareDevices::Driver {};
#endif

HardwareDevices::HardwareDevices() : driver_(std::make_unique<Driver>()) {}
HardwareDevices::~HardwareDevices() { av_buffer_unref(&cuda_); }
AVBufferRef* HardwareDevices::cuda(std::string& error) {
    std::lock_guard lock(mutex_);
    if (!attempted_) {
        attempted_ = true;
#ifdef STILLFLOW_CUDA_DRIVER
        // FFmpeg's own primary-context mode rejects a context the CUDA runtime
        // already activated with other flags, so retain it here and hand it
        // over as the current context instead.
        auto& d = *driver_;
        const auto failed = [](const char* operation, int result) {
            return std::string("CUDA device unavailable for video codecs: ") + operation + " failed (driver error " +
                   std::to_string(result) + ")";
        };
        if (cuda_load_functions(&d.cu, nullptr) < 0)
            error_ = "CUDA driver unavailable for video codecs";
        else if (const auto init = d.cu->cuInit(0))
            error_ = failed("cuInit", init);
        else if (const auto get = d.cu->cuDeviceGet(&d.device, 0))
            error_ = failed("cuDeviceGet", get);
        else if (const auto retain = d.cu->cuDevicePrimaryCtxRetain(&d.context, d.device)) {
            d.context = nullptr;
            error_ = failed("cuDevicePrimaryCtxRetain", retain);
        } else {
            AVDictionary* options = nullptr;
            av_dict_set(&options, "current_ctx", "1", 0);
            d.push();
            const int created = av_hwdevice_ctx_create(&cuda_, AV_HWDEVICE_TYPE_CUDA, "0", options, 0);
            d.pop();
            av_dict_free(&options);
            if (created < 0)
                error_ = "CUDA device unavailable for video codecs: " + error_text(created);
        }
#else
        error_ = "FFmpeg was built without NVIDIA codec support";
#endif
    }
    if (!cuda_) {
        error = error_;
        return nullptr;
    }
    auto* reference = av_buffer_ref(cuda_);
    if (!reference)
        throw std::bad_alloc();
    return reference;
}
void HardwareDevices::synchronize() {
#ifdef STILLFLOW_CUDA_DRIVER
    if (!cuda_)
        return;
    const auto& d = *driver_;
    const auto* device =
        static_cast<const AVCUDADeviceContext*>(reinterpret_cast<AVHWDeviceContext*>(cuda_->data)->hwctx);
    d.push();
    const auto result = d.cu->cuStreamSynchronize(device->stream);
    d.pop();
    if (result)
        throw std::runtime_error("Complete CUDA frame copy: driver error " + std::to_string(result));
#endif
}

class HostConverter {
  public:
    HostConverter(int threads, int rotation) : threads_(threads), rotation_(rotation) {
        converted_ = av_frame_alloc();
        transferred_ = av_frame_alloc();
        planar_ = av_frame_alloc();
        if (!converted_ || !transferred_ || !planar_) {
            release();
            throw std::bad_alloc();
        }
    }
    ~HostConverter() { release(); }
    HostConverter(const HostConverter&) = delete;
    HostConverter& operator=(const HostConverter&) = delete;
    cv::Mat convert(const AVFrame* frame);

  private:
    void release() {
        sws_freeContext(scaler_);
        av_frame_free(&planar_);
        av_frame_free(&transferred_);
        av_frame_free(&converted_);
    }
    std::mutex mutex_;
    AVFrame* converted_ = nullptr;
    AVFrame* transferred_ = nullptr; // NVDEC frame copied to host memory.
    AVFrame* planar_ = nullptr;      // transferred with chroma split into planes.
    SwsContext* scaler_ = nullptr;
    int scaler_format_ = AV_PIX_FMT_NONE;
    int threads_, rotation_;
};

// frame carries colour metadata; pixels is the host image to convert.
cv::Mat HostConverter::convert(const AVFrame* frame) {
    std::lock_guard lock(mutex_);
    const AVFrame* pixels = frame;
    if (frame->format == AV_PIX_FMT_CUDA) {
        av_frame_unref(transferred_);
        check(av_hwframe_transfer_data(transferred_, frame, 0), "Download NVDEC frame");
        pixels = transferred_;
        const auto planar =
            planar_equivalent(static_cast<AVPixelFormat>(pixels->format), frame->color_range == AVCOL_RANGE_JPEG);
        if (planar != AV_PIX_FMT_NONE) {
            if (planar_->format != planar || planar_->width != pixels->width || planar_->height != pixels->height) {
                av_frame_unref(planar_);
                planar_->format = planar;
                planar_->width = pixels->width;
                planar_->height = pixels->height;
                check(av_frame_get_buffer(planar_, 0), "Allocate NVDEC conversion frame");
            }
            deinterleave(pixels, planar_);
            pixels = planar_;
        }
    }
    // Frame dimensions are fixed by the reader; only the pixel format may change.
    if (!scaler_ || scaler_format_ != pixels->format) {
        sws_freeContext(scaler_);
        scaler_ = nullptr;
        scaler_ = make_scaler(pixels->width, pixels->height, static_cast<AVPixelFormat>(pixels->format),
                              AV_PIX_FMT_BGR24, SWS_BILINEAR, threads_, "Create decoder color converter");
        scaler_format_ = pixels->format;
    }
    const int* coefficients = sws_getCoefficients(colorspace(frame->colorspace));
    check(sws_setColorspaceDetails(scaler_, coefficients, frame->color_range == AVCOL_RANGE_JPEG, coefficients, 1, 0,
                                   1 << 16, 1 << 16),
          "Set input colorspace");
    cv::Mat raw(frame->height, frame->width, CV_8UC3), output;
    borrow(converted_, raw.data, raw.step, raw.cols, raw.rows, AV_PIX_FMT_BGR24);
    const int converted = sws_scale_frame(scaler_, converted_, pixels);
    av_frame_unref(converted_);
    check(converted, "Convert decoded frame");
    switch (rotation_) {
    case 90:
        cv::rotate(raw, output, cv::ROTATE_90_CLOCKWISE);
        return output;
    case 180:
        cv::rotate(raw, output, cv::ROTATE_180);
        return output;
    case 270:
        cv::rotate(raw, output, cv::ROTATE_90_COUNTERCLOCKWISE);
        return output;
    default:
        return raw;
    }
}

DeviceFrame::DeviceFrame(const AVFrame* frame, std::shared_ptr<HostConverter> converter, int rotation)
    : frame_(av_frame_clone(frame)), converter_(std::move(converter)) {
    if (!frame_)
        throw std::bad_alloc();
    descriptor_.coded = {frame_->width, frame_->height};
    descriptor_.rotation = rotation;
    descriptor_.luma = frame_->data[0];
    descriptor_.chroma = frame_->data[1];
    descriptor_.luma_pitch = static_cast<size_t>(frame_->linesize[0]);
    descriptor_.chroma_pitch = static_cast<size_t>(frame_->linesize[1]);
    describe_colour(frame_, descriptor_);
}
DeviceFrame::~DeviceFrame() { av_frame_free(&frame_); }
cv::Mat DeviceFrame::download() const { return converter_->convert(frame_); }

struct Reader::Impl {
    AVFormatContext* format = nullptr;
    AVCodecContext* decoder = nullptr;
    AVFrame* frame = nullptr;
    std::shared_ptr<HostConverter> converter;
    HardwareDevices* devices = nullptr;
    CodecReport codec;
    bool require_hardware = false, reported_software = false;
    AVPacket* packet = nullptr;
    int video_stream = -1;
    bool draining = false;
    int64_t last_pts = AV_NOPTS_VALUE;
    VideoInfo info;
    std::function<void(const AVPacket*)> audio_callback;
    ~Impl() {
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&decoder);
        avformat_close_input(&format);
    }
};

Reader::Reader(const std::filesystem::path& path, int threads, const CodecRequest& request, HardwareDevices& devices)
    : impl_(std::make_unique<Impl>()) {
    auto& s = *impl_;
    check(avformat_open_input(&s.format, path.c_str(), nullptr, nullptr), "Open input");
    check(avformat_find_stream_info(s.format, nullptr), "Read stream information");
    const AVCodec* codec = nullptr;
    s.video_stream = av_find_best_stream(s.format, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    check(s.video_stream, "Find video stream");
    auto* stream = s.format->streams[s.video_stream];
    s.decoder = avcodec_alloc_context3(codec);
    if (!s.decoder)
        throw std::bad_alloc();
    check(avcodec_parameters_to_context(s.decoder, stream->codecpar), "Set decoder parameters");
    s.decoder->thread_count = threads;
    s.devices = &devices;
    s.decoder->err_recognition = AV_EF_CAREFUL | AV_EF_EXPLODE;
    s.codec.request = request;
    s.codec.implementation = codec->name;
    if (request.name != "auto" && request.name != "cpu" && request.name != "nvdec")
        throw std::invalid_argument("Unknown decoder " + request.name + "; use auto, cpu or nvdec");
    s.require_hardware = request.name == "nvdec";
    if (s.require_hardware || (request.name == "auto" && request.prefer_hardware)) {
        std::string reason;
        if (!decodes_with_cuda(codec))
            reason = std::string("NVDEC does not support the ") + codec->name + " decoder";
        else if (auto* device = devices.cuda(reason)) {
            // libavcodec's default format negotiation picks CUDA frames when a
            // device is attached, and drops to software if NVDEC rejects the
            // stream (profile, chroma format or size).
            s.decoder->hw_device_ctx = device;
            s.codec.implementation += " + nvdec";
            s.codec.device = cuda_device;
        }
        if (!reason.empty()) {
            if (s.require_hardware)
                throw std::runtime_error("NVDEC unavailable: " + reason);
            s.codec.fallbacks.push_back(reason);
        }
    }
    check(avcodec_open2(s.decoder, codec, nullptr), "Open decoder");
    s.info.width = s.decoder->width;
    s.info.height = s.decoder->height;
    if (s.info.width < 32 || s.info.height < 32)
        throw std::runtime_error("Input must be at least 32 x 32 pixels");
    const auto sar = av_guess_sample_aspect_ratio(s.format, stream, nullptr);
    if (sar.num && sar.den && sar.num != sar.den)
        throw std::runtime_error("Non-square pixels: normalize the sample aspect ratio before processing");
    const auto rate = av_guess_frame_rate(s.format, stream, nullptr);
    s.info.fps = rate.num && rate.den ? av_q2d(rate) : 30.0;
    s.info.codec = codec->name;
    const auto* display = av_packet_side_data_get(stream->codecpar->coded_side_data,
                                                  stream->codecpar->nb_coded_side_data, AV_PKT_DATA_DISPLAYMATRIX);
    if (display && display->size >= 9 * sizeof(int32_t)) {
        const auto* matrix = reinterpret_cast<const int32_t*>(display->data);
        if (static_cast<double>(matrix[0]) * matrix[4] - static_cast<double>(matrix[1]) * matrix[3] < 0)
            throw std::runtime_error("Normalize mirrored display metadata before processing");
        const auto angle = -av_display_rotation_get(matrix);
        if (!std::isfinite(angle) || std::abs(angle - std::round(angle / 90) * 90) > 0.1)
            throw std::runtime_error("Only right-angle display rotations are supported");
        s.info.rotation = (static_cast<int>(std::lround(angle)) % 360 + 360) % 360;
    }
    if (s.info.rotation == 90 || s.info.rotation == 270) {
        std::swap(s.info.width, s.info.height);
        s.info.horizontal_scan = true;
    }
    s.converter = std::make_shared<HostConverter>(threads, s.info.rotation);
    s.frame = av_frame_alloc();
    s.packet = av_packet_alloc();
    if (!s.frame || !s.packet)
        throw std::bad_alloc();
}

Reader::~Reader() = default;
CodecReport Reader::codec() const {
    auto report = impl_->codec;
    // NVDEC was attached but rejected every frame: report what actually ran.
    if (!report.device.empty() && report.frames && !report.hardware_frames) {
        report.implementation = impl_->decoder->codec->name;
        report.device.clear();
    }
    return report;
}
const VideoInfo& Reader::info() const { return impl_->info; }
AVFormatContext* Reader::context() const { return impl_->format; }
void Reader::set_audio_callback(std::function<void(const AVPacket*)> callback) {
    impl_->audio_callback = std::move(callback);
}

bool Reader::next(DecodedFrame& output) {
    auto& s = *impl_;
    for (;;) {
        const int received = avcodec_receive_frame(s.decoder, s.frame);
        if (received == AVERROR_EOF)
            return false;
        if (received == 0)
            break;
        if (received != AVERROR(EAGAIN))
            check(received, "Decode video");
        if (s.draining)
            throw std::runtime_error("Decoder stalled while flushing");
        for (;;) {
            const int read = av_read_frame(s.format, s.packet);
            if (read == AVERROR_EOF) {
                check(avcodec_send_packet(s.decoder, nullptr), "Flush decoder");
                s.draining = true;
                break;
            }
            check(read, "Read input packet");
            if (s.packet->stream_index == s.video_stream) {
                const auto sent = avcodec_send_packet(s.decoder, s.packet);
                av_packet_unref(s.packet);
                check(sent, "Submit video packet");
                break;
            }
            if (s.audio_callback &&
                s.format->streams[s.packet->stream_index]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
                s.audio_callback(s.packet);
            av_packet_unref(s.packet);
        }
    }
    const AVFrame* frame = s.frame;
    ++s.codec.frames;
    if (frame->format == AV_PIX_FMT_CUDA)
        ++s.codec.hardware_frames;
    else if (s.decoder->hw_device_ctx && !s.reported_software) {
        if (s.require_hardware)
            throw std::runtime_error("NVDEC cannot decode this stream (unsupported profile, chroma format or size)");
        s.codec.fallbacks.push_back("NVDEC rejected the stream; decoding in software");
        s.reported_software = true;
    }
    const bool transposed = s.info.rotation == 90 || s.info.rotation == 270;
    if (frame->width != (transposed ? s.info.height : s.info.width) ||
        frame->height != (transposed ? s.info.width : s.info.height))
        throw std::runtime_error("Changing video resolution is not supported");
    const auto format = software_format(frame);
    const auto* description = av_pix_fmt_desc_get(format);
    if (!description || description->comp[0].depth > 8 || frame->color_trc == AVCOL_TRC_SMPTE2084 ||
        frame->color_trc == AVCOL_TRC_ARIB_STD_B67)
        throw std::runtime_error(
            "Host reference pipeline accepts 8-bit SDR; convert HDR/high-bit-depth input explicitly");
    if (frame->flags & AV_FRAME_FLAG_INTERLACED)
        throw std::runtime_error("Deinterlace input before stabilization");
    if (frame->best_effort_timestamp == AV_NOPTS_VALUE)
        throw std::runtime_error("Video frame has no presentation timestamp");
    const auto time_base = s.format->streams[s.video_stream]->time_base;
    output.pts_us = av_rescale_q(frame->best_effort_timestamp, time_base, microseconds);
    if (s.last_pts != AV_NOPTS_VALUE && output.pts_us <= s.last_pts)
        throw std::runtime_error("Video presentation timestamps must be strictly increasing");
    s.last_pts = output.pts_us;
    output.duration_us = frame->duration > 0 ? av_rescale_q(frame->duration, time_base, microseconds)
                                             : static_cast<int64_t>(std::llround(1000000 / s.info.fps));
    if (frame->format == AV_PIX_FMT_CUDA && format == AV_PIX_FMT_NV12) {
        // NV12 stays on the GPU; consumers that need host pixels convert lazily.
        s.devices->synchronize();
        output.device = std::make_shared<const DeviceFrame>(frame, s.converter, s.info.rotation);
        output.bgr.release();
    } else {
        output.device.reset();
        output.bgr = s.converter->convert(frame);
    }
    av_frame_unref(s.frame);
    return true;
}

struct Writer::Impl {
    AVFormatContext* format = nullptr;
    std::vector<AVRational> source_time_base; // Copied so audio muxing never reads the live demuxer.
    AVCodecContext* encoder = nullptr;
    AVStream* video = nullptr;
    AVFrame* frame = nullptr;    // Host frame in the encoder's software format.
    AVFrame* hardware = nullptr; // CUDA frame submitted to NVENC.
    AVFrame* source_frame = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* scaler = nullptr;
    HardwareDevices* devices = nullptr;
    std::vector<int> stream_map;
    std::map<int64_t, int64_t> durations;
    int64_t origin_us = 0;
    bool finished = false;
    CodecReport codec;
    ~Impl() {
        sws_freeContext(scaler);
        av_packet_free(&packet);
        av_frame_free(&source_frame);
        av_frame_free(&hardware);
        av_frame_free(&frame);
        avcodec_free_context(&encoder);
        if (format) {
            if (!(format->oformat->flags & AVFMT_NOFILE))
                avio_closep(&format->pb);
            avformat_free_context(format);
        }
    }
    void submit(AVFrame* input, int64_t pts_us, int64_t duration_us) {
        input->color_range = encoder->color_range;
        input->colorspace = encoder->colorspace;
        input->color_primaries = encoder->color_primaries;
        input->color_trc = encoder->color_trc;
        input->chroma_location = encoder->chroma_sample_location;
        input->pts = pts_us - origin_us;
        input->duration = duration_us;
        durations[input->pts] = duration_us;
        ++codec.frames;
        if (!codec.device.empty())
            ++codec.hardware_frames;
        check(avcodec_send_frame(encoder, input), "Submit output frame");
        drain();
    }
    void drain() {
        for (;;) {
            const int result = avcodec_receive_packet(encoder, packet);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
                break;
            check(result, "Receive encoded packet");
            if (auto it = durations.find(packet->pts); it != durations.end()) {
                packet->duration = it->second;
                durations.erase(it);
            }
            av_packet_rescale_ts(packet, encoder->time_base, video->time_base);
            packet->stream_index = video->index;
            const auto written = av_interleaved_write_frame(format, packet);
            av_packet_unref(packet);
            check(written, "Write video packet");
        }
    }
};

namespace {
struct EncoderSettings {
    cv::Size size;
    double fps;
    int threads, crf;
    std::string preset;
    bool global_header;
};
using EncoderContext =
    std::unique_ptr<AVCodecContext, decltype([](AVCodecContext* context) { avcodec_free_context(&context); })>;
EncoderContext allocate_encoder(const char* name, const EncoderSettings& settings) {
    const auto* codec = avcodec_find_encoder_by_name(name);
    if (!codec)
        throw std::runtime_error(std::string("FFmpeg was built without the ") + name + " encoder");
    EncoderContext encoder(avcodec_alloc_context3(codec));
    if (!encoder)
        throw std::bad_alloc();
    encoder->width = settings.size.width;
    encoder->height = settings.size.height;
    encoder->time_base = microseconds;
    encoder->framerate = av_d2q(settings.fps, 100000);
    encoder->pix_fmt = AV_PIX_FMT_YUV420P;
    encoder->sample_aspect_ratio = {1, 1};
    encoder->thread_count = settings.threads;
    encoder->gop_size = std::max(1, static_cast<int>(std::round(settings.fps * 2)));
    encoder->max_b_frames = 0;
    encoder->color_range = AVCOL_RANGE_MPEG;
    encoder->colorspace = AVCOL_SPC_BT709;
    encoder->color_primaries = AVCOL_PRI_BT709;
    encoder->color_trc = AVCOL_TRC_BT709;
    if (settings.global_header)
        encoder->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    return encoder;
}
EncoderContext open_x264(const EncoderSettings& settings) {
    auto encoder = allocate_encoder("libx264", settings);
    check(av_opt_set(encoder->priv_data, "preset", settings.preset.c_str(), 0), "Set encoder preset");
    check(av_opt_set_int(encoder->priv_data, "crf", settings.crf, 0), "Set CRF");
    check(avcodec_open2(encoder.get(), encoder->codec, nullptr), "Open encoder");
    return encoder;
}
// Encodes CUDA NV12 frames, so rendered frames are read in place and host frames
// are uploaded first. CRF maps to VBR constant quality (CQ) with no bitrate cap;
// CQ follows the same 0..51 QP scale but is not size-equivalent.
EncoderContext open_nvenc(const EncoderSettings& settings, AVBufferRef* device) {
    EncoderContext encoder;
    try {
        encoder = allocate_encoder("h264_nvenc", settings);
    } catch (...) {
        av_buffer_unref(&device);
        throw;
    }
    encoder->hw_device_ctx = device;
    encoder->hw_frames_ctx = av_hwframe_ctx_alloc(device);
    if (!encoder->hw_frames_ctx)
        throw std::bad_alloc();
    auto* frames = reinterpret_cast<AVHWFramesContext*>(encoder->hw_frames_ctx->data);
    frames->format = AV_PIX_FMT_CUDA;
    frames->sw_format = AV_PIX_FMT_NV12;
    frames->width = settings.size.width;
    frames->height = settings.size.height;
    check(av_hwframe_ctx_init(encoder->hw_frames_ctx), "Create NVENC frame pool");
    encoder->pix_fmt = AV_PIX_FMT_CUDA;
    encoder->sw_pix_fmt = AV_PIX_FMT_NV12;
    encoder->chroma_sample_location = AVCHROMA_LOC_LEFT;
    encoder->bit_rate = 0;
    auto* options = encoder->priv_data;
    check(av_opt_set(options, "preset", nvenc_preset(settings.preset), 0), "Set NVENC preset");
    if (settings.crf == 0)
        check(av_opt_set(options, "tune", "lossless", 0), "Set NVENC lossless tuning");
    else {
        check(av_opt_set(options, "tune", "hq", 0), "Set NVENC tuning");
        check(av_opt_set(options, "rc", "vbr", 0), "Set NVENC rate control");
        check(av_opt_set_double(options, "cq", settings.crf, 0), "Set NVENC constant quality");
    }
    const int opened = avcodec_open2(encoder.get(), encoder->codec, nullptr);
    if (opened < 0)
        throw std::runtime_error("NVENC rejected " + std::to_string(settings.size.width) + "x" +
                                 std::to_string(settings.size.height) +
                                 " H.264 (device limits or driver; see FFmpeg log): " + error_text(opened));
    return encoder;
}
} // namespace

Writer::Writer(const std::filesystem::path& path, cv::Size size, double fps, AVFormatContext* source, int64_t origin_us,
               int threads, int crf, const std::string& preset, bool audio, const CodecRequest& request,
               HardwareDevices& devices)
    : impl_(std::make_unique<Impl>()) {
    auto& s = *impl_;
    for (unsigned i = 0; i < source->nb_streams; ++i)
        s.source_time_base.push_back(source->streams[i]->time_base);
    s.origin_us = origin_us;
    s.devices = &devices;
    check(avformat_alloc_output_context2(&s.format, nullptr, nullptr, path.c_str()), "Create output container");
    if (!s.format)
        throw std::runtime_error("Unrecognized output container");
    if (request.name != "auto" && request.name != "x264" && request.name != "nvenc")
        throw std::invalid_argument("Unknown encoder " + request.name + "; use auto, x264 or nvenc");
    const bool global_header = (s.format->oformat->flags & AVFMT_GLOBALHEADER) != 0;
    const EncoderSettings settings{size, fps, threads, crf, preset, global_header};
    s.codec.request = request;
    EncoderContext encoder;
    if (request.name == "nvenc" || (request.name == "auto" && request.prefer_hardware)) {
        std::string reason;
        try {
            if (auto* device = devices.cuda(reason)) {
                encoder = open_nvenc(settings, device);
                s.codec.device = cuda_device;
            }
        } catch (const std::invalid_argument&) {
            throw; // Invalid settings are errors in auto mode too.
        } catch (const std::runtime_error& error) {
            reason = error.what();
        }
        if (!encoder) {
            if (request.name == "nvenc")
                throw std::runtime_error("NVENC unavailable: " + reason);
            s.codec.fallbacks.push_back(reason);
        }
    }
    if (!encoder)
        encoder = open_x264(settings);
    s.codec.implementation = encoder->codec->name;
    s.encoder = encoder.release();
    s.video = avformat_new_stream(s.format, nullptr);
    if (!s.video)
        throw std::bad_alloc();
    s.video->time_base = s.encoder->time_base;
    s.video->avg_frame_rate = s.encoder->framerate;
    check(avcodec_parameters_from_context(s.video->codecpar, s.encoder), "Copy encoder parameters");
    const bool cuda_input = s.encoder->pix_fmt == AV_PIX_FMT_CUDA;
    const auto software = cuda_input ? s.encoder->sw_pix_fmt : s.encoder->pix_fmt;
    if (cuda_input)
        s.video->codecpar->format = AV_PIX_FMT_YUV420P; // What decoders of the stream produce.
    s.stream_map.assign(source->nb_streams, -1);
    if (audio) {
        for (unsigned i = 0; i < source->nb_streams; ++i) {
            const auto* input = source->streams[i];
            if (input->codecpar->codec_type != AVMEDIA_TYPE_AUDIO)
                continue;
            if (avformat_query_codec(s.format->oformat, input->codecpar->codec_id, FF_COMPLIANCE_NORMAL) == 0)
                throw std::runtime_error("Audio codec is incompatible with output container; choose MKV or --no-audio");
            auto* stream = avformat_new_stream(s.format, nullptr);
            if (!stream)
                throw std::bad_alloc();
            check(avcodec_parameters_copy(stream->codecpar, input->codecpar), "Copy audio parameters");
            stream->codecpar->codec_tag = 0;
            stream->time_base = input->time_base;
            stream->disposition = input->disposition;
            av_dict_copy(&stream->metadata, input->metadata, 0);
            s.stream_map[i] = stream->index;
        }
    }
    if (!(s.format->oformat->flags & AVFMT_NOFILE))
        check(avio_open(&s.format->pb, path.c_str(), AVIO_FLAG_WRITE), "Open output file");
    check(avformat_write_header(s.format, nullptr), "Write output header");
    s.frame = av_frame_alloc();
    s.hardware = av_frame_alloc();
    s.source_frame = av_frame_alloc();
    s.packet = av_packet_alloc();
    if (!s.frame || !s.hardware || !s.source_frame || !s.packet)
        throw std::bad_alloc();
    s.frame->format = software;
    s.frame->width = size.width;
    s.frame->height = size.height;
    check(av_frame_get_buffer(s.frame, 32), "Allocate encoder frame");
    s.scaler = make_scaler(size.width, size.height, AV_PIX_FMT_BGR24, software, SWS_BICUBIC, threads,
                           "Create encoder color converter");
    const int* coefficients = sws_getCoefficients(SWS_CS_ITU709);
    check(sws_setColorspaceDetails(s.scaler, coefficients, 1, coefficients, 0, 0, 1 << 16, 1 << 16),
          "Set output colorspace");
}

Writer::~Writer() = default;
const CodecReport& Writer::codec() const { return impl_->codec; }
void Writer::video(const cv::Mat& bgr, int64_t pts_us, int64_t duration_us) {
    auto& s = *impl_;
    check(av_frame_make_writable(s.frame), "Make encoder frame writable");
    borrow(s.source_frame, bgr.data, bgr.step, bgr.cols, bgr.rows, AV_PIX_FMT_BGR24);
    const int converted = sws_scale_frame(s.scaler, s.frame, s.source_frame);
    av_frame_unref(s.source_frame);
    check(converted, "Convert output frame");
    if (s.encoder->pix_fmt != AV_PIX_FMT_CUDA) {
        s.submit(s.frame, pts_us, duration_us);
        return;
    }
    av_frame_unref(s.hardware);
    check(av_hwframe_get_buffer(s.encoder->hw_frames_ctx, s.hardware, 0), "Allocate NVENC input frame");
    check(av_hwframe_transfer_data(s.hardware, s.frame, 0), "Upload NVENC input frame");
    s.devices->synchronize(); // NVENC reads the frame outside FFmpeg's stream.
    s.submit(s.hardware, pts_us, duration_us);
    av_frame_unref(s.hardware);
}

bool Writer::accepts(const CudaImage& image) const {
    const auto& s = *impl_;
    // The descriptor must match what NVENC is told: unrotated BT.709 limited
    // range NV12 with left-sited chroma, chroma directly after a pitched luma plane.
    return s.encoder->pix_fmt == AV_PIX_FMT_CUDA && image.device == 0 && image.rotation == 0 &&
           image.coded.width == s.encoder->width && image.coded.height == s.encoder->height && !image.full_range &&
           image.kr == 0.2126f && image.kb == 0.0722f && image.chroma_x == 0 && image.chroma_y == 0.5f &&
           image.luma_pitch == image.chroma_pitch &&
           image.luma_pitch <= static_cast<size_t>(std::numeric_limits<int>::max()) &&
           image.chroma == image.luma + image.luma_pitch * image.coded.height;
}

void Writer::video(const CudaImage& image, std::shared_ptr<const void> lifetime, int64_t pts_us, int64_t duration_us) {
    auto& s = *impl_;
    if (!accepts(image) || !lifetime)
        throw std::invalid_argument("Device frame layout does not match the NVENC input");
    auto* hardware = s.hardware;
    av_frame_unref(hardware);
    hardware->format = AV_PIX_FMT_CUDA;
    hardware->width = image.coded.width;
    hardware->height = image.coded.height;
    hardware->data[0] = const_cast<uint8_t*>(image.luma);
    hardware->data[1] = const_cast<uint8_t*>(image.chroma);
    hardware->linesize[0] = hardware->linesize[1] = static_cast<int>(image.luma_pitch);
    hardware->hw_frames_ctx = av_buffer_ref(s.encoder->hw_frames_ctx);
    auto* owner = new std::shared_ptr<const void>(std::move(lifetime));
    // NVENC keeps a reference until it has finished reading; releasing it drops the lease.
    hardware->buf[0] = av_buffer_create(hardware->data[0], image.luma_pitch * image.coded.height * 3 / 2,
                                        release_lifetime, owner, AV_BUFFER_FLAG_READONLY);
    if (!hardware->buf[0])
        delete owner;
    if (!hardware->buf[0] || !hardware->hw_frames_ctx) {
        av_frame_unref(hardware);
        throw std::bad_alloc();
    }
    s.submit(hardware, pts_us, duration_us);
    av_frame_unref(hardware);
}

void Writer::audio(const AVPacket* packet) {
    auto& s = *impl_;
    const int index = s.stream_map.at(packet->stream_index);
    if (index < 0)
        return;
    auto* copy = av_packet_clone(packet);
    if (!copy)
        throw std::bad_alloc();
    const auto input_base = s.source_time_base[packet->stream_index];
    const int64_t offset = av_rescale_q(s.origin_us, microseconds, input_base);
    if (copy->pts != AV_NOPTS_VALUE)
        copy->pts -= offset;
    if (copy->dts != AV_NOPTS_VALUE)
        copy->dts -= offset;
    av_packet_rescale_ts(copy, input_base, s.format->streams[index]->time_base);
    copy->stream_index = index;
    copy->pos = -1;
    const int written = av_interleaved_write_frame(s.format, copy);
    av_packet_free(&copy);
    check(written, "Copy audio packet");
}

void Writer::finish() {
    auto& s = *impl_;
    if (s.finished)
        return;
    check(avcodec_send_frame(s.encoder, nullptr), "Flush encoder");
    s.drain();
    check(av_write_trailer(s.format), "Finalize output");
    s.finished = true;
}
} // namespace stillflow
