#pragma once
#include "stillflow/application/cuda_image.hpp"
#include "stillflow/application/rendering.hpp"

#include <cuda_runtime.h>
#include <optional>
#include <string>

namespace stillflow::cuda {
inline void check(cudaError_t result, const char* operation) {
    if (result != cudaSuccess)
        throw BackendError(std::string(operation) + ": " + cudaGetErrorString(result));
}

// Selects device 0 on the calling thread; media adapters use the same ordinal
// and primary context, so their device pointers are valid here.
inline cudaDeviceProp select_device() {
    int count = 0;
    check(cudaGetDeviceCount(&count), "Detect CUDA device");
    if (count == 0)
        throw BackendError("No CUDA device available");
    check(cudaSetDevice(0), "Select CUDA device");
    cudaDeviceProp properties{};
    check(cudaGetDeviceProperties(&properties, 0), "Read CUDA device");
    return properties;
}

template <class T, bool pinned = false> class Buffer {
  public:
    ~Buffer() { release(data_); }
    Buffer() = default;
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;
    void reserve(size_t count) {
        if (count <= capacity_)
            return;
        T* next = nullptr;
        if constexpr (pinned)
            check(cudaMallocHost(reinterpret_cast<void**>(&next), count * sizeof(T)), "Allocate CUDA staging buffer");
        else
            check(cudaMalloc(reinterpret_cast<void**>(&next), count * sizeof(T)), "Allocate CUDA buffer");
        release(data_);
        data_ = next;
        capacity_ = count;
    }
    T* data() const { return data_; }

  private:
    static void release(T* pointer) {
        if (!pointer)
            return;
        if constexpr (pinned)
            cudaFreeHost(pointer);
        else
            cudaFree(pointer);
    }
    T* data_ = nullptr;
    size_t capacity_ = 0;
};

class Stream {
  public:
    Stream() { check(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking), "Create CUDA stream"); }
    ~Stream() { cudaStreamDestroy(value); }
    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;
    cudaStream_t value{};
};

// A device image this library can read in place, with the lease that keeps it valid.
struct NativeSource {
    CudaImage image;
    std::shared_ptr<const void> lifetime;
};
inline std::optional<NativeSource> native_source(const Image& input) {
    if (!input)
        return std::nullopt;
    auto native = input->native_read(cuda_image_api);
    if (!native || !native->handle || !native->lifetime)
        return std::nullopt;
    const auto& image = *static_cast<const CudaImage*>(native->handle);
    const auto logical = input->size();
    const bool transposed = image.rotation == 90 || image.rotation == 270;
    const int width = transposed ? image.coded.height : image.coded.width;
    const int height = transposed ? image.coded.width : image.coded.height;
    if (image.device != 0 || (image.rotation % 90) || image.rotation < 0 || image.rotation > 270 ||
        width != logical.width || height != logical.height || !image.luma || !image.chroma ||
        image.luma_pitch < static_cast<size_t>(image.coded.width) ||
        image.chroma_pitch < static_cast<size_t>(image.coded.width + 1) / 2 * 2)
        return std::nullopt;
    return NativeSource{image, std::move(native->lifetime)};
}

// Maps a logical (displayed) coordinate to the stored, unrotated plane.
__device__ inline float2 coded_position(float x, float y, int rotation, int coded_width, int coded_height) {
    switch (rotation) {
    case 90:
        return {y, coded_height - 1 - x};
    case 180:
        return {coded_width - 1 - x, coded_height - 1 - y};
    case 270:
        return {coded_width - 1 - y, x};
    default:
        return {x, y};
    }
}
// Either packed BGR uploaded from the host or a decoder's NV12 planes.
struct DeviceSource {
    const unsigned char* bgr;
    const unsigned char *luma, *chroma;
    size_t luma_pitch, chroma_pitch;
    int coded_width, coded_height, rotation;
    float kr, kb, chroma_x, chroma_y;
    int full_range;
};
inline DeviceSource yuv_source(const CudaImage& image) {
    return {nullptr,           image.luma,         image.chroma,    image.luma_pitch, image.chroma_pitch,
            image.coded.width, image.coded.height, image.rotation,  image.kr,         image.kb,
            image.chroma_x,    image.chroma_y,     image.full_range};
}

__device__ inline unsigned char to_byte(float value) {
    return static_cast<unsigned char>(__float2uint_rn(fminf(fmaxf(value, 0), 255)));
}

__device__ inline float bilinear(const unsigned char* plane, size_t pitch, int stride, int width, int height, float x,
                                 float y) {
    x = fminf(fmaxf(x, 0), width - 1);
    y = fminf(fmaxf(y, 0), height - 1);
    const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
    const int x1 = min(x0 + 1, width - 1), y1 = min(y0 + 1, height - 1);
    const float fx = x - x0, fy = y - y0;
    const unsigned char* top = plane + static_cast<size_t>(y0) * pitch;
    const unsigned char* bottom = plane + static_cast<size_t>(y1) * pitch;
    const auto left = static_cast<ptrdiff_t>(x0) * stride, right = static_cast<ptrdiff_t>(x1) * stride;
    const float upper = top[left] * (1 - fx) + top[right] * fx;
    const float lower = bottom[left] * (1 - fx) + bottom[right] * fx;
    return upper * (1 - fy) + lower * fy;
}

// Samples the logical width x height image at (x, y); returns B, G, R in [0, 255].
template <bool yuv> __device__ float3 sample(const DeviceSource& s, float x, float y, int width, int height) {
    if constexpr (!yuv) {
        // OpenCV 5 uses full floating-point weights for float remap inputs.
        // Neighbours at the last row/column have zero extra weight.
        const size_t pitch = static_cast<size_t>(width) * 3;
        return {bilinear(s.bgr, pitch, 3, width, height, x, y), bilinear(s.bgr + 1, pitch, 3, width, height, x, y),
                bilinear(s.bgr + 2, pitch, 3, width, height, x, y)};
    } else {
        const float2 p = coded_position(x, y, s.rotation, s.coded_width, s.coded_height);
        const int chroma_width = (s.coded_width + 1) / 2, chroma_height = (s.coded_height + 1) / 2;
        const float cx = (p.x - s.chroma_x) * 0.5f, cy = (p.y - s.chroma_y) * 0.5f;
        float luma = bilinear(s.luma, s.luma_pitch, 1, s.coded_width, s.coded_height, p.x, p.y);
        float cb = bilinear(s.chroma, s.chroma_pitch, 2, chroma_width, chroma_height, cx, cy) - 128;
        float cr = bilinear(s.chroma + 1, s.chroma_pitch, 2, chroma_width, chroma_height, cx, cy) - 128;
        if (s.full_range) {
            luma /= 255, cb /= 255, cr /= 255;
        } else {
            luma = (luma - 16) / 219, cb /= 224, cr /= 224;
        }
        const float r = luma + 2 * (1 - s.kr) * cr;
        const float b = luma + 2 * (1 - s.kb) * cb;
        const float g = (luma - s.kr * r - s.kb * b) / (1 - s.kr - s.kb);
        return {b * 255, g * 255, r * 255};
    }
}
} // namespace stillflow::cuda
