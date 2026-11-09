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
// Borrowed during the reader callback; consumers copy packets they retain.
class AudioPacket {
  public:
    virtual ~AudioPacket() = default;
};
} // namespace stillflow
