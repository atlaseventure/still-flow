#include "stillflow/backends/cuda.hpp"

#include <cstring>
#include <cuda_runtime.h>

namespace stillflow {
namespace {
void check(cudaError_t result, const char* operation) {
    if (result != cudaSuccess)
        throw BackendError(std::string(operation) + ": " + cudaGetErrorString(result));
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

struct DeviceMap {
    float origin_x, origin_y, xx, xy, yx, yy;
    float scan[32], offset_x[32], offset_y[32];
    int bands, horizontal;
};

__global__ void render_kernel(const unsigned char* input, unsigned char* output, int width, int height, DeviceMap m,
                              unsigned int* invalid_count) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < width && y < height) {
        float px = m.origin_x + x * m.xx + y * m.xy;
        float py = m.origin_y + x * m.yx + y * m.yy;
        if (m.bands) {
            const float coordinate = m.horizontal ? px : py;
            int band = 0;
            while (band < m.bands - 2 && coordinate > m.scan[band + 1])
                ++band;
            const float t = fminf(1, fmaxf(0, (coordinate - m.scan[band]) / (m.scan[band + 1] - m.scan[band])));
            px -= m.offset_x[band] * (1 - t) + m.offset_x[band + 1] * t;
            py -= m.offset_y[band] * (1 - t) + m.offset_y[band + 1] * t;
        }
        invalid = !(px >= 0 && px <= width - 1 && py >= 0 && py <= height - 1);
        if (!invalid) {
            // OpenCV 5 uses full floating-point weights for float remap inputs.
            // Neighbours at the last row/column have zero extra weight.
            const int x0 = static_cast<int>(px), y0 = static_cast<int>(py);
            const int x1 = min(x0 + 1, width - 1), y1 = min(y0 + 1, height - 1);
            const float fx = px - x0, fy = py - y0;
            const size_t out = (static_cast<size_t>(y) * width + x) * 3;
            const size_t a = (static_cast<size_t>(y0) * width + x0) * 3;
            const size_t b = (static_cast<size_t>(y0) * width + x1) * 3;
            const size_t c = (static_cast<size_t>(y1) * width + x0) * 3;
            const size_t d = (static_cast<size_t>(y1) * width + x1) * 3;
            for (int channel = 0; channel < 3; ++channel) {
                const float top = input[a + channel] * (1 - fx) + input[b + channel] * fx;
                const float bottom = input[c + channel] * (1 - fx) + input[d + channel] * fx;
                output[out + channel] = static_cast<unsigned char>(__float2uint_rn(top * (1 - fy) + bottom * fy));
            }
        }
    }
    const int invalid_in_block = __syncthreads_count(invalid);
    if (threadIdx.x == 0 && threadIdx.y == 0 && invalid_in_block)
        atomicAdd(invalid_count, static_cast<unsigned int>(invalid_in_block));
}

class CudaBackend final : public RenderBackend {
  public:
    explicit CudaBackend(std::string name) : name_(std::move(name)) { invalid_.reserve(1); }
    ~CudaBackend() override { cudaStreamSynchronize(stream_.value); }
    BackendInfo info() const override {
        return {"cuda", name_, {{PixelFormat::bgr8}, {"host"}}, {{"compiler", STILLFLOW_CUDA_VERSION}}};
    }
    void upload(const Image& input) override {
        const auto mapping = map_host_image(input);
        if (mapping.format != PixelFormat::bgr8)
            throw BackendError("CUDA rendering requires BGR8 input");
        size_ = mapping.size;
        const size_t stride = static_cast<size_t>(size_.width) * 3;
        const size_t bytes = stride * size_.height;
        input_.reserve(bytes);
        output_.reserve(bytes);
        staging_.reserve(bytes);
        for (int y = 0; y < size_.height; ++y)
            std::memcpy(staging_.data() + y * stride, mapping.bytes.data() + y * mapping.row_stride, stride);
        check(cudaMemcpyAsync(input_.data(), staging_.data(), bytes, cudaMemcpyHostToDevice, stream_.value),
              "Upload CUDA frame");
    }
    double warp(const MapParameters& p) override {
        DeviceMap m{};
        m.origin_x = p.origin_x;
        m.origin_y = p.origin_y;
        m.xx = p.xx;
        m.xy = p.xy;
        m.yx = p.yx;
        m.yy = p.yy;
        m.bands = p.bands;
        m.horizontal = p.horizontal;
        for (int i = 0; i < p.bands; ++i) {
            m.scan[i] = p.scan[i];
            m.offset_x[i] = p.offset_x[i];
            m.offset_y[i] = p.offset_y[i];
        }
        check(cudaMemsetAsync(invalid_.data(), 0, sizeof(unsigned int), stream_.value), "Reset CUDA crop check");
        const dim3 block(16, 16), grid((size_.width + 15) / 16, (size_.height + 15) / 16);
        render_kernel<<<grid, block, 0, stream_.value>>>(input_.data(), output_.data(), size_.width, size_.height, m,
                                                         invalid_.data());
        check(cudaGetLastError(), "Launch CUDA warp");
        unsigned int invalid = 0;
        check(cudaMemcpyAsync(&invalid, invalid_.data(), sizeof(invalid), cudaMemcpyDeviceToHost, stream_.value),
              "Read CUDA crop check");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA warp");
        return static_cast<double>(invalid) / (static_cast<double>(size_.width) * size_.height);
    }
    Image download() override {
        const size_t bytes = static_cast<size_t>(size_.width) * size_.height * 3;
        check(cudaMemcpyAsync(staging_.data(), output_.data(), bytes, cudaMemcpyDeviceToHost, stream_.value),
              "Download CUDA frame");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA download");
        std::vector<uint8_t> pixels(staging_.data(), staging_.data() + bytes);
        return make_host_image(size_, PixelFormat::bgr8, std::move(pixels));
    }

  private:
    std::string name_;
    Size size_;
    Stream stream_;
    Buffer<unsigned char> input_, output_;
    // Reuse one pinned host buffer after each synchronized upload/warp/download.
    Buffer<unsigned char, true> staging_;
    Buffer<unsigned int> invalid_;
};
} // namespace

std::unique_ptr<RenderBackend> make_cuda_backend() {
    int count = 0;
    check(cudaGetDeviceCount(&count), "Detect CUDA device");
    if (count == 0)
        throw BackendError("No CUDA device available");
    check(cudaSetDevice(0), "Select CUDA device");
    cudaDeviceProp properties{};
    check(cudaGetDeviceProperties(&properties, 0), "Read CUDA device");
    return std::make_unique<CudaBackend>(properties.name);
}
} // namespace stillflow
