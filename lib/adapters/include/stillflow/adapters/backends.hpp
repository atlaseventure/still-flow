#pragma once
#include "stillflow/application/ports.hpp"

namespace stillflow {
std::unique_ptr<MediaFactory> make_media_factory();
std::unique_ptr<ImageProcessing> make_image_processing();
std::unique_ptr<OutputStorage> make_output_storage();
std::unique_ptr<Runtime> make_runtime();
std::string build_description();
} // namespace stillflow
