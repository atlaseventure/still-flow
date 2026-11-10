#pragma once
#include "stillflow/application/image.hpp"
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace stillflow {
// NativeImageView::api for CudaImage handles. The descriptor only names device
// memory, so producers and consumers need no shared CUDA headers.
inline constexpr std::string_view cuda_image_api = "cuda";

// 8-bit Y'CbCr 4:2:0 with interleaved chroma (NV12) resident in the primary
// context of CUDA device `device`. Planes are stored unrotated (`coded` size);
// the logical image is the stored one rotated clockwise by `rotation` degrees.
// Pixels are complete when native_read() returns; consumers keep the view's
// lifetime until their own device work has finished reading them.
struct CudaImage {
    int device = 0;
    Size coded;
    int rotation = 0; // 0, 90, 180 or 270.
    const uint8_t* luma = nullptr;
    const uint8_t* chroma = nullptr; // Cb, Cr pairs at half resolution.
    size_t luma_pitch = 0, chroma_pitch = 0;
    ColorInfo color;
};
} // namespace stillflow
