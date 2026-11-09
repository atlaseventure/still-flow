#pragma once
#include "stillflow/application/rendering.hpp"

namespace stillflow {
std::unique_ptr<RenderBackend> make_cuda_backend();
} // namespace stillflow
