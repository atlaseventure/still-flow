#include "stillflow/backends/cpu.hpp"
#include <algorithm>
#include <cstring>
#include <opencv2/imgproc.hpp>

namespace stillflow {
namespace {
void build_maps(const MapParameters& m, cv::Size output, cv::Mat& map_x, cv::Mat& map_y) {
    map_x.create(output, CV_32F);
    map_y.create(output, CV_32F);
    cv::parallel_for_(cv::Range(0, output.height), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
            auto* mx = map_x.ptr<float>(y);
            auto* my = map_y.ptr<float>(y);
            cv::Point2d u(m.origin_x + y * m.xy, m.origin_y + y * m.yy);
            int band = 0;
            for (int x = 0; x < output.width; ++x) {
                auto p = u;
                if (m.bands) {
                    const double coordinate = m.horizontal ? u.x : u.y;
                    while (band < m.bands - 2 && coordinate > m.scan[band + 1])
                        ++band;
                    while (band > 0 && coordinate < m.scan[band])
                        --band;
                    const double t =
                        std::clamp((coordinate - m.scan[band]) / (m.scan[band + 1] - m.scan[band]), 0.0, 1.0);
                    p.x -= m.offset_x[band] * (1 - t) + m.offset_x[band + 1] * t;
                    p.y -= m.offset_y[band] * (1 - t) + m.offset_y[band + 1] * t;
                }
                mx[x] = static_cast<float>(p.x);
                my[x] = static_cast<float>(p.y);
                u.x += m.xx;
                u.y += m.yx;
            }
        }
    });
}

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
            build_maps(map, input_.size(), map_x_, map_y_);
            cv::Mat valid = (map_x_ >= 0) & (map_x_ <= input_.cols - 1) & (map_y_ >= 0) & (map_y_ <= input_.rows - 1);
            return 1 - static_cast<double>(cv::countNonZero(valid)) / input_.total();
        } catch (const cv::Exception& error) {
            throw BackendError(error.what());
        }
    }
    Image download() override {
        try {
            const size_t stride = static_cast<size_t>(input_.cols) * 3;
            std::vector<uint8_t> bytes(stride * input_.rows);
            cv::Mat output(input_.rows, input_.cols, CV_8UC3, bytes.data(), stride);
            cv::remap(input_, output, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
            return make_host_image(mapping_.size, PixelFormat::bgr8, std::move(bytes), stride);
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
