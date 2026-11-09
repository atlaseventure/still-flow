#include "media.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/display.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

namespace stillflow {
namespace {
constexpr AVRational microseconds{1, 1000000};
void check(int result, const std::string& operation) {
    if (result >= 0) return;
    char message[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(result, message, sizeof(message));
    throw std::runtime_error(operation + ": " + message);
}
int colorspace(AVColorSpace space) {
    switch (space) {
    case AVCOL_SPC_BT709: return SWS_CS_ITU709;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL: return SWS_CS_BT2020;
    case AVCOL_SPC_FCC: return SWS_CS_FCC;
    default: return SWS_CS_ITU601;
    }
}
}

struct Reader::Impl {
    AVFormatContext* format = nullptr;
    AVCodecContext* decoder = nullptr;
    AVFrame* frame = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* scaler = nullptr;
    int video_stream = -1;
    bool draining = false;
    int64_t last_pts = AV_NOPTS_VALUE;
    VideoInfo info;
    std::function<void(const AVPacket*)> audio_callback;
    ~Impl() {
        sws_freeContext(scaler);
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&decoder);
        avformat_close_input(&format);
    }
};

Reader::Reader(const std::filesystem::path& path, int threads) : impl_(std::make_unique<Impl>()) {
    auto& s = *impl_;
    check(avformat_open_input(&s.format, path.c_str(), nullptr, nullptr), "Open input");
    check(avformat_find_stream_info(s.format, nullptr), "Read stream information");
    const AVCodec* codec = nullptr;
    s.video_stream = av_find_best_stream(s.format, AVMEDIA_TYPE_VIDEO, -1, -1, &codec, 0);
    check(s.video_stream, "Find video stream");
    auto* stream = s.format->streams[s.video_stream];
    s.decoder = avcodec_alloc_context3(codec);
    if (!s.decoder) throw std::bad_alloc();
    check(avcodec_parameters_to_context(s.decoder, stream->codecpar), "Set decoder parameters");
    s.decoder->thread_count = threads;
    s.decoder->err_recognition = AV_EF_CAREFUL | AV_EF_EXPLODE;
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
    s.frame = av_frame_alloc();
    s.packet = av_packet_alloc();
    if (!s.frame || !s.packet) throw std::bad_alloc();
}

Reader::~Reader() = default;
const VideoInfo& Reader::info() const { return impl_->info; }
AVFormatContext* Reader::context() const { return impl_->format; }
void Reader::set_audio_callback(std::function<void(const AVPacket*)> callback) {
    impl_->audio_callback = std::move(callback);
}

bool Reader::next(DecodedFrame& output) {
    auto& s = *impl_;
    for (;;) {
        const int received = avcodec_receive_frame(s.decoder, s.frame);
        if (received == AVERROR_EOF) return false;
        if (received == 0) break;
        if (received != AVERROR(EAGAIN)) check(received, "Decode video");
        if (s.draining) throw std::runtime_error("Decoder stalled while flushing");
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
            if (s.audio_callback && s.format->streams[s.packet->stream_index]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
                s.audio_callback(s.packet);
            av_packet_unref(s.packet);
        }
    }
    const auto* frame = s.frame;
    const bool transposed = s.info.rotation == 90 || s.info.rotation == 270;
    if (frame->width != (transposed ? s.info.height : s.info.width) ||
        frame->height != (transposed ? s.info.width : s.info.height))
        throw std::runtime_error("Changing video resolution is not supported");
    const auto* description = av_pix_fmt_desc_get(static_cast<AVPixelFormat>(frame->format));
    if (!description || description->comp[0].depth > 8 ||
        frame->color_trc == AVCOL_TRC_SMPTE2084 || frame->color_trc == AVCOL_TRC_ARIB_STD_B67)
        throw std::runtime_error("Host reference pipeline accepts 8-bit SDR; convert HDR/high-bit-depth input explicitly");
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
    s.scaler = sws_getCachedContext(s.scaler, frame->width, frame->height,
        static_cast<AVPixelFormat>(frame->format), frame->width, frame->height,
        AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!s.scaler) throw std::runtime_error("Create decoder color converter");
    const int* coefficients = sws_getCoefficients(colorspace(frame->colorspace));
    check(sws_setColorspaceDetails(s.scaler, coefficients, frame->color_range == AVCOL_RANGE_JPEG,
        coefficients, 1, 0, 1 << 16, 1 << 16), "Set input colorspace");
    cv::Mat raw(frame->height, frame->width, CV_8UC3);
    uint8_t* data[] = {raw.data, nullptr, nullptr, nullptr};
    int stride[] = {static_cast<int>(raw.step), 0, 0, 0};
    check(sws_scale(s.scaler, frame->data, frame->linesize, 0, frame->height, data, stride), "Convert decoded frame");
    switch (s.info.rotation) {
    case 90: cv::rotate(raw, output.bgr, cv::ROTATE_90_CLOCKWISE); break;
    case 180: cv::rotate(raw, output.bgr, cv::ROTATE_180); break;
    case 270: cv::rotate(raw, output.bgr, cv::ROTATE_90_COUNTERCLOCKWISE); break;
    default: output.bgr = std::move(raw);
    }
    av_frame_unref(s.frame);
    return true;
}

struct Writer::Impl {
    AVFormatContext* format = nullptr;
    AVFormatContext* source = nullptr; // Reader must outlive writer.
    AVCodecContext* encoder = nullptr;
    AVStream* video = nullptr;
    AVFrame* frame = nullptr;
    AVPacket* packet = nullptr;
    SwsContext* scaler = nullptr;
    std::vector<int> stream_map;
    std::map<int64_t, int64_t> durations;
    int64_t origin_us = 0;
    bool finished = false;
    ~Impl() {
        sws_freeContext(scaler);
        av_packet_free(&packet);
        av_frame_free(&frame);
        avcodec_free_context(&encoder);
        if (format) {
            if (!(format->oformat->flags & AVFMT_NOFILE)) avio_closep(&format->pb);
            avformat_free_context(format);
        }
    }
    void drain() {
        for (;;) {
            const int result = avcodec_receive_packet(encoder, packet);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) break;
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

Writer::Writer(const std::filesystem::path& path, cv::Size size, double fps,
    AVFormatContext* source, int64_t origin_us, int threads, int crf,
    const std::string& preset, bool audio) : impl_(std::make_unique<Impl>()) {
    auto& s = *impl_;
    s.source = source;
    s.origin_us = origin_us;
    check(avformat_alloc_output_context2(&s.format, nullptr, nullptr, path.c_str()), "Create output container");
    if (!s.format) throw std::runtime_error("Unrecognized output container");
    const auto* codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) throw std::runtime_error("FFmpeg must include the libx264 encoder");
    s.encoder = avcodec_alloc_context3(codec);
    if (!s.encoder) throw std::bad_alloc();
    s.encoder->width = size.width;
    s.encoder->height = size.height;
    s.encoder->time_base = microseconds;
    s.encoder->framerate = av_d2q(fps, 100000);
    s.encoder->pix_fmt = AV_PIX_FMT_YUV420P;
    s.encoder->sample_aspect_ratio = {1, 1};
    s.encoder->thread_count = threads;
    s.encoder->gop_size = std::max(1, static_cast<int>(std::round(fps * 2)));
    s.encoder->max_b_frames = 0;
    s.encoder->color_range = AVCOL_RANGE_MPEG;
    s.encoder->colorspace = AVCOL_SPC_BT709;
    s.encoder->color_primaries = AVCOL_PRI_BT709;
    s.encoder->color_trc = AVCOL_TRC_BT709;
    if (s.format->oformat->flags & AVFMT_GLOBALHEADER) s.encoder->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    check(av_opt_set(s.encoder->priv_data, "preset", preset.c_str(), 0), "Set encoder preset");
    check(av_opt_set_int(s.encoder->priv_data, "crf", crf, 0), "Set CRF");
    check(avcodec_open2(s.encoder, codec, nullptr), "Open encoder");
    s.video = avformat_new_stream(s.format, nullptr);
    if (!s.video) throw std::bad_alloc();
    s.video->time_base = s.encoder->time_base;
    s.video->avg_frame_rate = s.encoder->framerate;
    check(avcodec_parameters_from_context(s.video->codecpar, s.encoder), "Copy encoder parameters");
    s.stream_map.assign(source->nb_streams, -1);
    if (audio) {
        for (unsigned i = 0; i < source->nb_streams; ++i) {
            const auto* input = source->streams[i];
            if (input->codecpar->codec_type != AVMEDIA_TYPE_AUDIO) continue;
            if (avformat_query_codec(s.format->oformat, input->codecpar->codec_id, FF_COMPLIANCE_NORMAL) == 0)
                throw std::runtime_error("Audio codec is incompatible with output container; choose MKV or --no-audio");
            auto* stream = avformat_new_stream(s.format, nullptr);
            if (!stream) throw std::bad_alloc();
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
    s.packet = av_packet_alloc();
    if (!s.frame || !s.packet) throw std::bad_alloc();
    s.frame->format = s.encoder->pix_fmt;
    s.frame->width = size.width;
    s.frame->height = size.height;
    s.frame->color_range = s.encoder->color_range;
    s.frame->colorspace = s.encoder->colorspace;
    s.frame->color_primaries = s.encoder->color_primaries;
    s.frame->color_trc = s.encoder->color_trc;
    check(av_frame_get_buffer(s.frame, 32), "Allocate encoder frame");
    s.scaler = sws_getContext(size.width, size.height, AV_PIX_FMT_BGR24,
        size.width, size.height, s.encoder->pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
    if (!s.scaler) throw std::runtime_error("Create encoder color converter");
    const int* coefficients = sws_getCoefficients(SWS_CS_ITU709);
    check(sws_setColorspaceDetails(s.scaler, coefficients, 1, coefficients, 0, 0, 1 << 16, 1 << 16), "Set output colorspace");
}

Writer::~Writer() = default;
void Writer::video(const cv::Mat& bgr, int64_t pts_us, int64_t duration_us) {
    auto& s = *impl_;
    check(av_frame_make_writable(s.frame), "Make encoder frame writable");
    const uint8_t* data[] = {bgr.data, nullptr, nullptr, nullptr};
    int stride[] = {static_cast<int>(bgr.step), 0, 0, 0};
    check(sws_scale(s.scaler, data, stride, 0, bgr.rows, s.frame->data, s.frame->linesize), "Convert output frame");
    s.frame->pts = pts_us - s.origin_us;
    s.frame->duration = duration_us;
    s.durations[s.frame->pts] = duration_us;
    check(avcodec_send_frame(s.encoder, s.frame), "Submit output frame");
    s.drain();
}

void Writer::audio(const AVPacket* packet) {
    auto& s = *impl_;
    const int index = s.stream_map.at(packet->stream_index);
    if (index < 0) return;
    auto* copy = av_packet_clone(packet);
    if (!copy) throw std::bad_alloc();
    const auto input_base = s.source->streams[packet->stream_index]->time_base;
    const int64_t offset = av_rescale_q(s.origin_us, microseconds, input_base);
    if (copy->pts != AV_NOPTS_VALUE) copy->pts -= offset;
    if (copy->dts != AV_NOPTS_VALUE) copy->dts -= offset;
    av_packet_rescale_ts(copy, input_base, s.format->streams[index]->time_base);
    copy->stream_index = index;
    copy->pos = -1;
    const int written = av_interleaved_write_frame(s.format, copy);
    av_packet_free(&copy);
    check(written, "Copy audio packet");
}

void Writer::finish() {
    auto& s = *impl_;
    if (s.finished) return;
    check(avcodec_send_frame(s.encoder, nullptr), "Flush encoder");
    s.drain();
    check(av_write_trailer(s.format), "Finalize output");
    s.finished = true;
}
} // namespace stillflow
