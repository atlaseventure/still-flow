#pragma once
#include "stillflow/application/rendering.hpp"

namespace stillflow {
// Private EGL context; operations restore the caller's EGL context and surfaces.
std::unique_ptr<RenderBackend> make_gles_backend();
} // namespace stillflow
