#include "image.hpp"
#include "stillflow/adapters/backends.hpp"
#include "stillflow/vision/vision.hpp"
#include <opencv2/imgproc.hpp>

namespace stillflow {
namespace {
class OpenCvProcessing final : public ImageProcessing {
  public:
    std::unique_ptr<MotionAnalyzer> analyzer(const StabilizationSettings& settings, bool horizontal) override {
        return make_opencv_analyzer(settings, horizontal);
    }
    Image gray_proxy(const Image& input, Size size) override { return stillflow::gray_proxy(input, size); }

    Image comparison(const Image& input, const Image& stabilized) override {
        if (!font_)
            font_ = std::make_unique<cv::FontFace>("sans");
        cv::Mat combined;
        auto before = read_pixels(input), after = read_pixels(stabilized);
        cv::hconcat(before.pixels, after.pixels, combined);
        cv::putText(combined, "INPUT", {20, 35}, {255, 255, 255}, *font_, 24, 600);
        cv::putText(combined, "STABILIZED", {input->size().width + 20, 35}, {255, 255, 255}, *font_, 24, 600);
        return image(std::move(combined));
    }

  private:
    std::unique_ptr<cv::FontFace> font_;
};
} // namespace
std::unique_ptr<ImageProcessing> make_image_processing() { return std::make_unique<OpenCvProcessing>(); }
} // namespace stillflow
