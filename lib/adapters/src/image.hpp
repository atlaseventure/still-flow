#pragma once
#include "stillflow/application/media.hpp"
#include <opencv2/core.hpp>

namespace stillflow {
class OpenCvImage final : public ImageData {
public:
    explicit OpenCvImage(cv::Mat value) : pixels(std::move(value)) {}
    Size size() const override { return {pixels.cols, pixels.rows}; }
    cv::Mat pixels;
};
inline const cv::Mat& pixels(const Image& image) {
    return dynamic_cast<const OpenCvImage&>(*image).pixels;
}
inline Image image(cv::Mat pixels) { return std::make_shared<OpenCvImage>(std::move(pixels)); }
} // namespace stillflow
