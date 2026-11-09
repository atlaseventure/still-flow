#include "stillflow/renderer.hpp"

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>

namespace stillflow {
#ifndef STILLFLOW_HAVE_CUDA
std::unique_ptr<GpuRenderer> make_cuda_renderer() {
    throw std::runtime_error("CUDA rendering is not included in this build");
}
#endif

namespace {
void reduce_strength(Warp& warp, int attempt) {
    // Dense validation remains authoritative after sparse crop planning.
    if (attempt > 4) throw std::runtime_error("Cannot construct a valid crop");
    warp.strength = attempt < 3 ? warp.strength * 0.75 : 0;
}
}

FrameRenderer::FrameRenderer(const std::string& mode) : mode_(mode) {
    if (mode_ != "auto" && mode_ != "cuda" && mode_ != "off")
        throw std::runtime_error("Unknown GPU mode: " + mode_);
    if (mode_ != "off") {
        try {
            gpu_ = make_cuda_renderer();
            device_ = gpu_->device();
        } catch (const std::exception& error) {
            if (mode_ == "cuda") throw;
            fallback_ = error.what();
        }
    }
    std::cerr << "Render backend: " << (gpu_ ? "CUDA (" + device_ + ")" : "CPU");
    if (!fallback_.empty()) std::cerr << " (" << fallback_ << ')';
    std::cerr << '\n';
}

void FrameRenderer::render(const cv::Mat& input, cv::Mat& output, Warp& warp,
    cv::Size analysis_size, double crop, bool horizontal, Profile& profile) {
    if (gpu_) {
        const double original_strength = warp.strength;
        try {
            // Includes transfers and synchronization, so this measures completed
            // work and can be compared with the CPU map + remap stages.
            ScopeTimer timer(profile, "render.cuda_warp");
            gpu_->upload(input);
            for (int attempt = 0; ; ++attempt) {
                warp.invalid_fraction = gpu_->warp(prepare_map(warp, analysis_size, input.size(), crop, horizontal));
                if (warp.invalid_fraction == 0) break;
                reduce_strength(warp, attempt);
            }
            gpu_->download(output);
            ++gpu_frames_;
            return;
        } catch (const std::exception& error) {
            if (mode_ == "cuda") throw;
            fallback_ = error.what();
            std::cerr << "CUDA rendering failed; using CPU: " << fallback_ << '\n';
            gpu_.reset();
            warp.strength = original_strength;
        }
    }
    {
        ScopeTimer timer(profile, "render.map");
        for (int attempt = 0; ; ++attempt) {
            build_maps(warp, analysis_size, input.size(), crop, horizontal, map_x_, map_y_);
            // Negating the valid predicate also catches NaN coordinates.
            cv::Mat valid = (map_x_ >= 0) & (map_x_ <= input.cols - 1)
                & (map_y_ >= 0) & (map_y_ <= input.rows - 1);
            warp.invalid_fraction = 1 - static_cast<double>(cv::countNonZero(valid)) / input.total();
            if (warp.invalid_fraction == 0) break;
            reduce_strength(warp, attempt);
        }
    }
    {
        ScopeTimer timer(profile, "render.remap");
        cv::remap(input, output, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
    ++cpu_frames_;
}

nlohmann::json FrameRenderer::report() const {
    return {{"requested", mode_}, {"backend", gpu_frames_ ? (cpu_frames_ ? "mixed" : "cuda") : "cpu"},
        {"device", device_}, {"gpu_frames", gpu_frames_}, {"cpu_frames", cpu_frames_},
#ifdef STILLFLOW_CUDA_VERSION
        {"cuda_compiler", STILLFLOW_CUDA_VERSION},
#endif
        {"fallback_reason", fallback_}};
}
} // namespace stillflow
