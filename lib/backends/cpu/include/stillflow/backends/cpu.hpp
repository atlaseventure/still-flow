#pragma once
#include "stillflow/application/rendering.hpp"

namespace stillflow {
std::unique_ptr<RenderBackend> make_cpu_backend();
} // namespace stillflow
