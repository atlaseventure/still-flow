#pragma once
#include "stillflow/domain/geometry.hpp"
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
// Pixel storage belongs to the image adapter and stays alive across frame reads.
class ImageData {
public:
    virtual ~ImageData() = default;
    virtual Size size() const = 0;
};
using Image = std::shared_ptr<const ImageData>;
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
