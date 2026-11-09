#include "stillflow/backends/cpu.hpp"
#include <algorithm>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace stillflow {
namespace {
// Builds remap tables and returns the number of samples outside the input,
// using the same float bounds as cv::remap's stored coordinates.
size_t build_maps(const MapParameters& m, cv::Size output, cv::Mat& map_x, cv::Mat& map_y) {
    map_x.create(output, CV_32F);
    map_y.create(output, CV_32F);
    const auto max_x = static_cast<float>(output.width - 1), max_y = static_cast<float>(output.height - 1);
    std::vector<size_t> invalid_rows(output.height);
    cv::parallel_for_(cv::Range(0, output.height), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
            auto* mx = map_x.ptr<float>(y);
            auto* my = map_y.ptr<float>(y);
            size_t invalid = 0;
            for (int x = 0; x < output.width; ++x) {
                const auto p = source_point(m, {m.origin_x + x * m.step_x, m.origin_y + y * m.step_y});
                const auto fx = static_cast<float>(p.x), fy = static_cast<float>(p.y);
                mx[x] = fx;
                my[x] = fy;
                invalid += !(fx >= 0 && fx <= max_x && fy >= 0 && fy <= max_y);
            }
            invalid_rows[y] = invalid;
        }
    });
    return std::accumulate(invalid_rows.begin(), invalid_rows.end(), size_t{0});
}

// Output storage is a reference-counted Mat, so it outlives later renders and
// the backend itself without a zero-filled staging copy.
class MatImage final : public ImageData {
  public:
    explicit MatImage(cv::Mat pixels) : pixels_(std::move(pixels)) {}
    Size size() const override { return {pixels_.cols, pixels_.rows}; }
    PixelFormat format() const override { return PixelFormat::bgr8; }
    HostImageView map_read() const override {
        const size_t bytes = (pixels_.rows - 1) * pixels_.step + pixels_.cols * pixels_.elemSize();
        return {size(), format(), pixels_.step, {pixels_.data, bytes}, std::make_shared<const cv::Mat>(pixels_)};
    }

  private:
    cv::Mat pixels_;
};

class CpuBackend final : public RenderBackend {
  public:
    BackendInfo info() const override {
        return {"cpu", "host", {{PixelFormat::bgr8}, {"host"}}, {{"opencv", CV_VERSION}}};
    }
    void upload(const Image& input) override {
        mapping_ = map_host_image(input);
        if (mapping_.format != PixelFormat::bgr8)
            throw BackendError("CPU rendering requires BGR8 input");
        input_ = cv::Mat(mapping_.size.height, mapping_.size.width, CV_8UC3,
                         const_cast<uint8_t*>(mapping_.bytes.data()), mapping_.row_stride);
    }
    double warp(const MapParameters& map) override {
        try {
            const auto invalid = build_maps(map, input_.size(), map_x_, map_y_);
            return static_cast<double>(invalid) / static_cast<double>(input_.total());
        } catch (const cv::Exception& error) {
            throw BackendError(error.what());
        }
    }
    Image download() override {
        try {
            cv::Mat output(input_.rows, input_.cols, CV_8UC3);
            cv::remap(input_, output, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            return std::make_shared<MatImage>(std::move(output));
        } catch (const cv::Exception& error) {
            throw BackendError(error.what());
        }
    }

  private:
    HostImageView mapping_{};
    cv::Mat input_, map_x_, map_y_;
};
} // namespace
std::unique_ptr<RenderBackend> make_cpu_backend() { return std::make_unique<CpuBackend>(); }
} // namespace stillflow
