#pragma once
#include "stillflow/application/rendering.hpp"
#include "stillflow/application/runtime.hpp"

namespace stillflow {
std::unique_ptr<RenderBackend> make_cpu_backend(ExecutionSettings execution = {});
} // namespace stillflow
