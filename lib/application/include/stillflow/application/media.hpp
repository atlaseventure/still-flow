#pragma once
#include "stillflow/application/image.hpp"
#include <cstdint>
#include <memory>
#include <string>

namespace stillflow {
struct VideoInfo {
    int width = 0, height = 0;
    int rotation = 0;
    double fps = 0;
    std::string codec;
    bool horizontal_scan = false;
};
struct VideoFrame {
    Image bgr;
    int64_t pts_us = 0, duration_us = 0;
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
