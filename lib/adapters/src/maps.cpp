#include "motion.hpp"
#include <algorithm>

namespace stillflow {
void build_maps(const Warp& warp, Size size, cv::Size output, double crop,
    bool horizontal, cv::Mat& map_x, cv::Mat& map_y) {
    map_x.create(output, CV_32F);
    map_y.create(output, CV_32F);
    const auto m = prepare_map(warp, size, {output.width, output.height}, crop, horizontal);
    cv::parallel_for_(cv::Range(0, output.height), [&](const cv::Range& range) {
        for (int y = range.start; y < range.end; ++y) {
            auto* mx = map_x.ptr<float>(y);
            auto* my = map_y.ptr<float>(y);
            cv::Point2d u(m.origin_x + y * m.xy, m.origin_y + y * m.yy);
            int band = 0;
            for (int x = 0; x < output.width; ++x) {
                auto p = u;
                if (m.bands) {
                    const double coordinate = horizontal ? u.x : u.y;
                    while (band < m.bands - 2 && coordinate > m.scan[band + 1]) ++band;
                    while (band > 0 && coordinate < m.scan[band]) --band;
                    const double t = std::clamp((coordinate - m.scan[band]) / (m.scan[band + 1] - m.scan[band]), 0.0, 1.0);
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
} // namespace stillflow
