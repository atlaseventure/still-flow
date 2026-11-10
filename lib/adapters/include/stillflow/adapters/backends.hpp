#pragma once
#include "stillflow/adapters/ports.hpp"

namespace stillflow {
std::unique_ptr<MediaFactory> make_media_factory();
std::unique_ptr<ImageProcessing> make_image_processing();
std::unique_ptr<OutputStorage> make_output_storage();
std::unique_ptr<Runtime> make_runtime();
std::string build_description();
std::unique_ptr<Executor> make_thread_executor(int workers);
} // namespace stillflow
