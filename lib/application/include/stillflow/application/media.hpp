#pragma once
#include "stillflow/application/image.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace stillflow {
struct VideoInfo {
    int width = 0, height = 0;
    int rotation = 0;
    double fps = 0;
    std::string codec;
    bool horizontal_scan = false;
    int64_t duration_us = 0; // Zero when the container does not provide a duration.
    size_t estimated_frames = 0;
};
struct VideoFrame {
    Image image;
    int64_t pts_us = 0, duration_us = 0;
    Image analysis_image;
};
// Codec choice handed to media adapters. "auto" lets the adapter choose,
// trying hardware first when prefer_hardware is set and falling back to
// software. Other names are adapter-defined and fail instead of falling back.
struct CodecRequest {
    std::string name = "auto";
    bool prefer_hardware = false;
    // Copy decoder surfaces into a separate device pool for delayed consumption.
    bool independent_frames = false;
};
enum class AudioMode { automatic, copy, aac, drop };
struct EncodingSettings {
    int threads = 12;
    int crf = 18;
    std::string preset = "fast";
    bool audio = true;
    AudioMode audio_mode = AudioMode::automatic;
};
// The codec a reader or writer actually used. A hardware decoder may hand
// individual streams back to software, so frames are counted separately.
struct AudioTrackReport {
    size_t input_stream;
    std::string input_codec, output_codec, action;
};
struct CodecReport {
    CodecRequest request;
    std::string implementation, device;
    size_t frames = 0, hardware_frames = 0;
    size_t reconstructed_timestamps = 0;
    std::vector<std::string> fallbacks;
    std::vector<AudioTrackReport> audio_tracks;
};
// Borrowed during the reader callback; consumers retain() packets they keep.
class AudioPacket {
  public:
    virtual ~AudioPacket() = default;
    // Returns an owning copy that stays valid after the callback returns and
    // may be read concurrently by several writers.
    virtual std::shared_ptr<const AudioPacket> retain() const = 0;
};
} // namespace stillflow
