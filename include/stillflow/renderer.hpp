#pragma once

#include "stillflow/motion.hpp"
#include <memory>

namespace stillflow {
// Device buffers persist across frames. Only decoded and rendered pixels cross
// the host/device boundary; dense maps are never transferred.
class GpuRenderer {
public:
    virtual ~GpuRenderer() = default;
    virtual std::string device() const = 0;
    virtual void upload(const cv::Mat& input) = 0;
    virtual double warp(const MapParameters& parameters) = 0;
    virtual void download(cv::Mat& output) = 0;
};
std::unique_ptr<GpuRenderer> make_cuda_renderer();

class FrameRenderer {
public:
    explicit FrameRenderer(const std::string& mode);
    void render(const cv::Mat& input, cv::Mat& output, Warp& warp,
        cv::Size analysis_size, double crop, bool horizontal, Profile& profile);
    nlohmann::json report() const;
private:
    std::string mode_, device_, fallback_;
    std::unique_ptr<GpuRenderer> gpu_;
    cv::Mat map_x_, map_y_;
    size_t gpu_frames_ = 0, cpu_frames_ = 0;
};
} // namespace stillflow
