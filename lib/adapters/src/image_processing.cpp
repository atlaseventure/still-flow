#include "stillflow/adapters/backends.hpp"
#include "image.hpp"
#include "motion.hpp"
#include "renderer.hpp"
#include <opencv2/imgproc.hpp>

namespace stillflow {
namespace {
class OpenCvMotionAnalyzer final : public MotionAnalyzer {
public:
    OpenCvMotionAnalyzer(const StabilizationSettings& settings, bool horizontal) : estimator_(settings, horizontal) {}
    Motion estimate(const Image& previous, const Image& current, Profile& profile) override {
        return estimator_.estimate(pixels(previous), pixels(current), profile);
    }
private:
    MotionEstimator estimator_;
};
class OpenCvRenderer final : public Renderer {
public:
    explicit OpenCvRenderer(const std::string& mode) : renderer_(mode) {}
    Image render(const Image& input, Warp& warp, Size size, double crop, bool horizontal, Profile& profile) override {
        cv::Mat output;
        renderer_.render(pixels(input), output, warp, size, crop, horizontal, profile);
        return image(std::move(output));
    }
    AccelerationReport report() const override { return renderer_.report(); }
private:
    FrameRenderer renderer_;
};
class OpenCvProcessing final : public ImageProcessing {
public:
    std::unique_ptr<MotionAnalyzer> analyzer(const StabilizationSettings& settings, bool horizontal) override {
        return std::make_unique<OpenCvMotionAnalyzer>(settings, horizontal);
    }
    std::unique_ptr<Renderer> renderer(const std::string& mode) override {
        return std::make_unique<OpenCvRenderer>(mode);
    }
    Image gray_proxy(const Image& input, Size size) override {
        cv::Mat gray, small;
        cv::cvtColor(pixels(input), gray, cv::COLOR_BGR2GRAY);
        const cv::Size target(size.width, size.height);
        if (gray.size() == target) return image(std::move(gray));
        cv::resize(gray, small, target, 0, 0, cv::INTER_AREA);
        return image(std::move(small));
    }
    Image comparison(const Image& input, const Image& stabilized) override {
        if (!font_) font_ = std::make_unique<cv::FontFace>("sans");
        cv::Mat combined;
        cv::hconcat(pixels(input), pixels(stabilized), combined);
        cv::putText(combined, "INPUT", {20, 35}, {255, 255, 255}, *font_, 24, 600);
        cv::putText(combined, "STABILIZED", {input->size().width + 20, 35}, {255, 255, 255}, *font_, 24, 600);
        return image(std::move(combined));
    }
private:
    std::unique_ptr<cv::FontFace> font_;
};
}
std::unique_ptr<ImageProcessing> make_image_processing() { return std::make_unique<OpenCvProcessing>(); }
} // namespace stillflow
