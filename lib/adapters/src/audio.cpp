#include "audio.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

namespace stillflow {
namespace {
void check_audio(int result, const char* operation) {
    if (result >= 0)
        return;
    char text[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(result, text, sizeof(text));
    throw std::runtime_error(std::string(operation) + ": " + text);
}
constexpr AVRational microseconds{1, 1000000};
} // namespace

struct AudioTranscoder::Impl {
    AVFormatContext* output = nullptr;
    AVStream* target = nullptr;
    AVRational input_time_base{};
    AVCodecContext *decoder = nullptr, *encoder = nullptr;
    AVFrame *decoded = nullptr, *converted = nullptr, *encoded = nullptr;
    AVPacket* packet = nullptr;
    SwrContext* resampler = nullptr;
    AVAudioFifo* fifo = nullptr;
    AVChannelLayout input_layout{};
    AVSampleFormat input_format = AV_SAMPLE_FMT_NONE;
    int input_rate = 0;
    int64_t origin = 0, fallback_start = 0, next_pts = AV_NOPTS_VALUE;
    bool finished = false;

    ~Impl() {
        av_channel_layout_uninit(&input_layout);
        av_audio_fifo_free(fifo);
        swr_free(&resampler);
        av_frame_free(&decoded);
        av_frame_free(&converted);
        av_frame_free(&encoded);
        av_packet_free(&packet);
        avcodec_free_context(&decoder);
        avcodec_free_context(&encoder);
    }
    void allocate(AVFrame* frame, int samples) const {
        av_frame_unref(frame);
        frame->format = encoder->sample_fmt;
        frame->sample_rate = encoder->sample_rate;
        frame->nb_samples = samples;
        check_audio(av_channel_layout_copy(&frame->ch_layout, &encoder->ch_layout), "Set audio channel layout");
        check_audio(av_frame_get_buffer(frame, 0), "Allocate audio samples");
    }
    void packets() {
        for (;;) {
            const int result = avcodec_receive_packet(encoder, packet);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
                return;
            check_audio(result, "Receive AAC packet");
            av_packet_rescale_ts(packet, encoder->time_base, target->time_base);
            packet->stream_index = target->index;
            const int written = av_interleaved_write_frame(output, packet);
            av_packet_unref(packet);
            check_audio(written, "Write AAC packet");
        }
    }
    void encode(bool flush) {
        while (av_audio_fifo_size(fifo) >= encoder->frame_size || (flush && av_audio_fifo_size(fifo) > 0)) {
            const int samples = std::min(encoder->frame_size, av_audio_fifo_size(fifo));
            allocate(encoded, samples);
            if (av_audio_fifo_read(fifo, reinterpret_cast<void**>(encoded->extended_data), samples) != samples)
                throw std::runtime_error("Read audio sample queue");
            encoded->pts = next_pts;
            next_pts += samples;
            check_audio(avcodec_send_frame(encoder, encoded), "Encode AAC frame");
            packets();
        }
    }
    void append(AVFrame* frame, int samples) {
        if (samples <= 0)
            return;
        if (av_audio_fifo_write(fifo, reinterpret_cast<void**>(frame->extended_data), samples) != samples)
            throw std::runtime_error("Write audio sample queue");
        encode(false);
    }
    void flush_resampler() {
        if (!resampler)
            return;
        for (;;) {
            allocate(converted, std::max(1, swr_get_out_samples(resampler, 0)));
            const int samples = swr_convert(resampler, converted->extended_data, converted->nb_samples, nullptr, 0);
            check_audio(samples, "Flush audio resampler");
            if (!samples)
                break;
            append(converted, samples);
        }
    }
    void frame() {
        if (!resampler) {
            input_rate = decoded->sample_rate;
            input_format = static_cast<AVSampleFormat>(decoded->format);
            check_audio(av_channel_layout_copy(&input_layout, &decoded->ch_layout), "Read audio channel layout");
            check_audio(swr_alloc_set_opts2(&resampler, &encoder->ch_layout, encoder->sample_fmt, encoder->sample_rate,
                                            &input_layout, input_format, input_rate, 0, nullptr),
                        "Create audio resampler");
            check_audio(swr_init(resampler), "Initialize audio resampler");
        } else if (decoded->sample_rate != input_rate || decoded->format != input_format ||
                   av_channel_layout_compare(&decoded->ch_layout, &input_layout)) {
            throw std::runtime_error("Changing audio format is not supported");
        }
        const int64_t pts = decoded->best_effort_timestamp;
        const int64_t position =
            pts == AV_NOPTS_VALUE ? AV_NOPTS_VALUE : av_rescale_q(pts, input_time_base, encoder->time_base) - origin;
        if (next_pts == AV_NOPTS_VALUE)
            next_pts = position == AV_NOPTS_VALUE ? fallback_start : position;
        if (position != AV_NOPTS_VALUE) {
            const int64_t expected = next_pts + av_audio_fifo_size(fifo) +
                                     av_rescale_rnd(swr_get_delay(resampler, input_rate), encoder->sample_rate,
                                                    input_rate, AV_ROUND_NEAR_INF);
            // Coarse container timestamps may round by a millisecond. Larger
            // forward gaps become silence, preserving synchronization with video.
            const int64_t tolerance = std::max(2, encoder->sample_rate / 500);
            if (position < expected - tolerance)
                throw std::runtime_error("Audio presentation timestamps overlap");
            if (position > expected + tolerance) {
                flush_resampler();
                int64_t gap = position - next_pts - av_audio_fifo_size(fifo);
                while (gap > 0) {
                    const int samples = static_cast<int>(std::min<int64_t>(gap, encoder->frame_size));
                    allocate(converted, samples);
                    check_audio(av_samples_set_silence(converted->extended_data, 0, samples,
                                                       encoder->ch_layout.nb_channels, encoder->sample_fmt),
                                "Fill audio timestamp gap");
                    append(converted, samples);
                    gap -= samples;
                }
                check_audio(swr_init(resampler), "Reset audio resampler after timestamp gap");
            }
        }
        const int capacity = swr_get_out_samples(resampler, decoded->nb_samples);
        check_audio(capacity, "Size converted audio");
        allocate(converted, std::max(1, capacity));
        const int samples = swr_convert(resampler, converted->extended_data, converted->nb_samples,
                                        const_cast<const uint8_t**>(decoded->extended_data), decoded->nb_samples);
        check_audio(samples, "Convert audio samples");
        append(converted, samples);
    }
    void receive() {
        for (;;) {
            const int result = avcodec_receive_frame(decoder, decoded);
            if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
                return;
            check_audio(result, "Decode audio frame");
            frame();
            av_frame_unref(decoded);
        }
    }
};

AudioTranscoder::AudioTranscoder(AVFormatContext* output, const AVStream* source, AVStream* target, int64_t origin_us,
                                 int threads)
    : impl_(std::make_unique<Impl>()) {
    auto& s = *impl_;
    s.output = output;
    s.target = target;
    s.input_time_base = source->time_base;
    const auto* decoder = avcodec_find_decoder(source->codecpar->codec_id);
    const auto* encoder = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!decoder || !encoder)
        throw std::runtime_error("Audio transcoding requires an input decoder and AAC encoder");
    s.decoder = avcodec_alloc_context3(decoder);
    s.encoder = avcodec_alloc_context3(encoder);
    s.decoded = av_frame_alloc();
    s.converted = av_frame_alloc();
    s.encoded = av_frame_alloc();
    s.packet = av_packet_alloc();
    if (!s.decoder || !s.encoder || !s.decoded || !s.converted || !s.encoded || !s.packet)
        throw std::bad_alloc();
    check_audio(avcodec_parameters_to_context(s.decoder, source->codecpar), "Set audio decoder parameters");
    s.decoder->pkt_timebase = source->time_base;
    s.decoder->thread_count = threads;
    check_audio(avcodec_open2(s.decoder, decoder, nullptr), "Open audio decoder");
    if (s.decoder->sample_rate <= 0 || s.decoder->ch_layout.nb_channels <= 0)
        throw std::runtime_error("Missing audio sample rate or channel layout");
    if (s.decoder->ch_layout.order == AV_CHANNEL_ORDER_UNSPEC) {
        const int channels = s.decoder->ch_layout.nb_channels;
        av_channel_layout_uninit(&s.decoder->ch_layout);
        av_channel_layout_default(&s.decoder->ch_layout, channels);
    }
    check_audio(av_channel_layout_copy(&s.encoder->ch_layout, &s.decoder->ch_layout), "Set AAC channel layout");
    const void* supported = nullptr;
    int count = 0;
    check_audio(avcodec_get_supported_config(s.encoder, encoder, AV_CODEC_CONFIG_SAMPLE_RATE, 0, &supported, &count),
                "Read AAC sample rates");
    s.encoder->sample_rate = s.decoder->sample_rate;
    if (supported && count > 0) {
        const auto* rates = static_cast<const int*>(supported);
        s.encoder->sample_rate = *std::min_element(rates, rates + count, [&](int a, int b) {
            return std::abs(a - s.decoder->sample_rate) < std::abs(b - s.decoder->sample_rate);
        });
    }
    s.encoder->sample_fmt = AV_SAMPLE_FMT_FLTP;
    s.encoder->time_base = {1, s.encoder->sample_rate};
    s.encoder->bit_rate = std::min(std::clamp(s.encoder->ch_layout.nb_channels * 64000, 96000, 512000),
                                   s.encoder->sample_rate * s.encoder->ch_layout.nb_channels * 6);
    s.encoder->thread_count = threads;
    if (output->oformat->flags & AVFMT_GLOBALHEADER)
        s.encoder->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    check_audio(avcodec_open2(s.encoder, encoder, nullptr), "Open AAC encoder");
    if (s.encoder->frame_size <= 0)
        throw std::runtime_error("AAC encoder has no frame size");
    target->time_base = s.encoder->time_base;
    check_audio(avcodec_parameters_from_context(target->codecpar, s.encoder), "Set AAC stream parameters");
    s.fifo = av_audio_fifo_alloc(s.encoder->sample_fmt, s.encoder->ch_layout.nb_channels, s.encoder->frame_size);
    if (!s.fifo)
        throw std::bad_alloc();
    s.origin = av_rescale_q(origin_us, microseconds, s.encoder->time_base);
    s.fallback_start = (source->start_time == AV_NOPTS_VALUE
                            ? 0
                            : av_rescale_q(source->start_time, source->time_base, s.encoder->time_base)) -
                       s.origin;
}
AudioTranscoder::~AudioTranscoder() = default;
void AudioTranscoder::push(const AVPacket* packet) {
    auto& s = *impl_;
    check_audio(avcodec_send_packet(s.decoder, packet), "Submit audio packet");
    s.receive();
}
void AudioTranscoder::finish() {
    auto& s = *impl_;
    if (s.finished)
        return;
    check_audio(avcodec_send_packet(s.decoder, nullptr), "Flush audio decoder");
    s.receive();
    s.flush_resampler();
    s.encode(true);
    check_audio(avcodec_send_frame(s.encoder, nullptr), "Flush AAC encoder");
    s.packets();
    s.finished = true;
}
} // namespace stillflow
