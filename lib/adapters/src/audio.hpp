#pragma once
#include <cstdint>
#include <memory>

extern "C" {
#include <libavformat/avformat.h>
}

namespace stillflow {
// One independently decoded/resampled AAC track. Called only by its writer.
class AudioTranscoder {
  public:
    AudioTranscoder(AVFormatContext* output, const AVStream* source, AVStream* target, int64_t origin_us, int threads);
    ~AudioTranscoder();
    AudioTranscoder(const AudioTranscoder&) = delete;
    AudioTranscoder& operator=(const AudioTranscoder&) = delete;
    void push(const AVPacket* packet);
    void finish();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace stillflow
