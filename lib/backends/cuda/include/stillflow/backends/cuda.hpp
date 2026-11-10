#pragma once
#include "stillflow/application/rendering.hpp"
#include "stillflow/vision/vision.hpp"

namespace stillflow {
std::unique_ptr<RenderBackend> make_cuda_backend();
// Builds analysis proxies on the GPU for images with CUDA device views and
// delegates everything else, including any GPU failure, to host.
std::unique_ptr<ImageProcessing> make_cuda_image_processing(std::unique_ptr<ImageProcessing> host);
} // namespace stillflow
