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
    Image gray_proxy(const Image& input, Size size) override {
        if (input->format() != PixelFormat::bgr8)
            return stillflow::gray_proxy(input, size);
        cv::Mat gray, small;
        auto view = read_pixels(input);
        if (view.mapping.format != PixelFormat::bgr8)
            throw std::invalid_argument("Preprocessing requires BGR8 input");
        cv::cvtColor(view.pixels, gray, cv::COLOR_BGR2GRAY);
        const cv::Size target(size.width, size.height);
        if (gray.size() == target)
            return image(std::move(gray));
        cv::resize(gray, small, target, 0, 0, cv::INTER_AREA);
        return image(std::move(small));
    }

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
