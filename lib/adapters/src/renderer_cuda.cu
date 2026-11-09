#include "renderer.hpp"

#include <cuda_runtime.h>
#include <stdexcept>

namespace stillflow {
namespace {
void check(cudaError_t result, const char* operation) {
    if (result != cudaSuccess)
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
}

template<class T, bool pinned = false> class Buffer {
public:
    ~Buffer() { release(data_); }
    Buffer() = default;
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;
    void reserve(size_t count) {
        if (count <= capacity_) return;
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
        if (!pointer) return;
        if constexpr (pinned) cudaFreeHost(pointer);
        else cudaFree(pointer);
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

__global__ void render_kernel(const unsigned char* input, unsigned char* output,
    int width, int height, DeviceMap m, unsigned int* invalid_count) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < width && y < height) {
        float px = m.origin_x + x * m.xx + y * m.xy;
        float py = m.origin_y + x * m.yx + y * m.yy;
        if (m.bands) {
            const float coordinate = m.horizontal ? px : py;
            int band = 0;
            while (band < m.bands - 2 && coordinate > m.scan[band + 1]) ++band;
            const float t = fminf(1, fmaxf(0,
                (coordinate - m.scan[band]) / (m.scan[band + 1] - m.scan[band])));
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

class CudaRenderer final : public GpuRenderer {
public:
    explicit CudaRenderer(std::string name) : name_(std::move(name)) { invalid_.reserve(1); }
    ~CudaRenderer() override { cudaStreamSynchronize(stream_.value); }
    std::string device() const override { return name_; }
    void upload(const cv::Mat& input) override {
        if (input.type() != CV_8UC3) throw std::runtime_error("CUDA renderer requires BGR8 input");
        size_ = input.size();
        const size_t bytes = input.total() * 3;
        input_.reserve(bytes);
        output_.reserve(bytes);
        staging_.reserve(bytes);
        cv::Mat host(size_, CV_8UC3, staging_.data());
        input.copyTo(host);
        check(cudaMemcpyAsync(input_.data(), staging_.data(), bytes,
            cudaMemcpyHostToDevice, stream_.value), "Upload CUDA frame");
    }
    double warp(const MapParameters& p) override {
        DeviceMap m{};
        m.origin_x = p.origin_x; m.origin_y = p.origin_y;
        m.xx = p.xx; m.xy = p.xy; m.yx = p.yx; m.yy = p.yy;
        m.bands = p.bands; m.horizontal = p.horizontal;
        for (int i = 0; i < p.bands; ++i) {
            m.scan[i] = p.scan[i];
            m.offset_x[i] = p.offset_x[i]; m.offset_y[i] = p.offset_y[i];
        }
        check(cudaMemsetAsync(invalid_.data(), 0, sizeof(unsigned int), stream_.value), "Reset CUDA crop check");
        const dim3 block(16, 16), grid((size_.width + 15) / 16, (size_.height + 15) / 16);
        render_kernel<<<grid, block, 0, stream_.value>>>(input_.data(), output_.data(),
            size_.width, size_.height, m, invalid_.data());
        check(cudaGetLastError(), "Launch CUDA warp");
        unsigned int invalid = 0;
        check(cudaMemcpyAsync(&invalid, invalid_.data(), sizeof(invalid), cudaMemcpyDeviceToHost,
            stream_.value), "Read CUDA crop check");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA warp");
        return static_cast<double>(invalid) / (static_cast<double>(size_.width) * size_.height);
    }
    void download(cv::Mat& output) override {
        output.create(size_, CV_8UC3);
        check(cudaMemcpyAsync(staging_.data(), output_.data(), output.total() * 3,
            cudaMemcpyDeviceToHost, stream_.value), "Download CUDA frame");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA download");
        cv::Mat(size_, CV_8UC3, staging_.data()).copyTo(output);
    }
private:
    std::string name_;
    cv::Size size_;
    Stream stream_;
    Buffer<unsigned char> input_, output_;
    // Reuse one pinned host buffer after each synchronized upload/warp/download.
    Buffer<unsigned char, true> staging_;
    Buffer<unsigned int> invalid_;
};
}

std::unique_ptr<GpuRenderer> make_cuda_renderer() {
    int count = 0;
    check(cudaGetDeviceCount(&count), "Detect CUDA device");
    if (count == 0) throw std::runtime_error("No CUDA device available");
    check(cudaSetDevice(0), "Select CUDA device");
    cudaDeviceProp properties{};
    check(cudaGetDeviceProperties(&properties, 0), "Read CUDA device");
    return std::make_unique<CudaRenderer>(properties.name);
}
} // namespace stillflow
