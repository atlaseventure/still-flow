#include "frames.cuh"
#include "stillflow/backends/cuda.hpp"

#include <cstring>

namespace stillflow {
namespace {
using cuda::Buffer;
using cuda::check;
using cuda::DeviceSource;
using cuda::FramePool;
using cuda::OutputFrame;
using cuda::Stream;

struct DeviceMap {
    float origin_x, origin_y, step_x, step_y, grid_x, grid_y;
    float offset_x[max_mesh_vertices], offset_y[max_mesh_vertices];
    int columns, rows;
};

__device__ float sample_mesh(const float* values, int a, int b, float fx, float fy) {
    return (values[a] * (1 - fx) + values[a + 1] * fx) * (1 - fy) + (values[b] * (1 - fx) + values[b + 1] * fx) * fy;
}

template <bool yuv>
__global__ void render_kernel(DeviceSource source, unsigned char* output, int width, int height, const DeviceMap* map,
                              unsigned int* invalid_count) {
    const auto& m = *map;
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < width && y < height) {
        const float target_x = m.origin_x + x * m.step_x;
        const float target_y = m.origin_y + y * m.step_y;
        float px = target_x, py = target_y;
        for (int iteration = 0; iteration < mesh_inverse_iterations; ++iteration) {
            const float gx = fminf(m.columns - 1, fmaxf(0, px * m.grid_x));
            const float gy = fminf(m.rows - 1, fmaxf(0, py * m.grid_y));
            const int cx = min(static_cast<int>(gx), m.columns - 2);
            const int cy = min(static_cast<int>(gy), m.rows - 2);
            const float fx = gx - cx, fy = gy - cy;
            const int a = cy * m.columns + cx, b = a + m.columns;
            px = target_x - sample_mesh(m.offset_x, a, b, fx, fy);
            py = target_y - sample_mesh(m.offset_y, a, b, fx, fy);
        }
        invalid = !(px >= 0 && px <= width - 1 && py >= 0 && py <= height - 1);
        if (!invalid) {
            const float3 bgr = cuda::sample<yuv>(source, px, py, width, height);
            unsigned char* out = output + (static_cast<size_t>(y) * width + x) * 3;
            out[0] = cuda::to_byte(bgr.x);
            out[1] = cuda::to_byte(bgr.y);
            out[2] = cuda::to_byte(bgr.z);
        }
    }
    const int invalid_in_block = __syncthreads_count(invalid);
    if (threadIdx.x == 0 && threadIdx.y == 0 && invalid_in_block)
        atomicAdd(invalid_count, static_cast<unsigned int>(invalid_in_block));
}

class CudaBackend final : public RenderBackend {
  public:
    explicit CudaBackend(std::string name) : name_(std::move(name)) {
        invalid_.reserve(1);
        map_.reserve(1);
    }
    ~CudaBackend() override { cudaStreamSynchronize(stream_.value); }
    BackendInfo info() const override {
        return {"cuda",
                name_,
                {{PixelFormat::bgr8}, {std::string(cuda_image_api), "host"}},
                {{"compiler", STILLFLOW_CUDA_VERSION}, {"output", "bgr8 + nv12 bt709 (device)"}}};
    }
    void upload(const Image& input) override {
        lease_.reset();
        size_ = input->size();
        source_ = {};
        if (auto native = cuda::native_source(input)) {
            // Decoded frames are sampled where the decoder left them.
            source_ = cuda::yuv_source(native->image);
            lease_ = std::move(native->lifetime);
            yuv_ = true;
        } else {
            const auto mapping = map_host_image(input);
            if (mapping.format != PixelFormat::bgr8)
                throw BackendError("CUDA rendering requires BGR8 input");
            const size_t stride = static_cast<size_t>(size_.width) * 3;
            const size_t bytes = stride * size_.height;
            input_.reserve(bytes);
            staging_.reserve(bytes);
            for (int y = 0; y < size_.height; ++y)
                std::memcpy(staging_.data() + y * stride, mapping.bytes.data() + y * mapping.row_stride, stride);
            check(cudaMemcpyAsync(input_.data(), staging_.data(), bytes, cudaMemcpyHostToDevice, stream_.value),
                  "Upload CUDA frame");
            source_.bgr = input_.data();
            yuv_ = false;
        }
        frame_ = pool_->acquire(size_);
    }
    double warp(const MapParameters& p) override {
        DeviceMap m{};
        m.origin_x = p.origin_x;
        m.origin_y = p.origin_y;
        m.step_x = p.step_x;
        m.step_y = p.step_y;
        m.grid_x = p.grid_x;
        m.grid_y = p.grid_y;
        m.columns = p.columns;
        m.rows = p.rows;
        for (int i = 0; i < p.columns * p.rows; ++i) {
            m.offset_x[i] = p.offset_x[i];
            m.offset_y[i] = p.offset_y[i];
        }
        check(cudaMemcpyAsync(map_.data(), &m, sizeof(m), cudaMemcpyHostToDevice, stream_.value),
              "Upload displacement mesh");
        check(cudaMemsetAsync(invalid_.data(), 0, sizeof(unsigned int), stream_.value), "Reset CUDA crop check");
        const dim3 block(16, 16), grid((size_.width + 15) / 16, (size_.height + 15) / 16);
        if (yuv_)
            render_kernel<true><<<grid, block, 0, stream_.value>>>(source_, frame_->bgr, size_.width, size_.height,
                                                                   map_.data(), invalid_.data());
        else
            render_kernel<false><<<grid, block, 0, stream_.value>>>(source_, frame_->bgr, size_.width, size_.height,
                                                                    map_.data(), invalid_.data());
        check(cudaGetLastError(), "Launch CUDA warp");
        unsigned int invalid = 0;
        check(cudaMemcpyAsync(&invalid, invalid_.data(), sizeof(invalid), cudaMemcpyDeviceToHost, stream_.value),
              "Read CUDA crop check");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA warp");
        return static_cast<double>(invalid) / (static_cast<double>(size_.width) * size_.height);
    }
    Image download() override {
        lease_.reset();
        return cuda::publish(std::move(frame_), stream_.value);
    }

  private:
    std::string name_;
    Size size_;
    Stream stream_;
    DeviceSource source_{};
    bool yuv_ = false;
    std::shared_ptr<const void> lease_; // Keeps a decoded source valid until the frame is rendered.
    std::shared_ptr<FramePool> pool_ = std::make_shared<FramePool>();
    std::shared_ptr<OutputFrame> frame_;
    Buffer<unsigned char> input_;
    // Reuse one pinned host buffer after each synchronized upload.
    Buffer<unsigned char, true> staging_;
    Buffer<unsigned int> invalid_;
    Buffer<DeviceMap> map_;
};
} // namespace

std::unique_ptr<RenderBackend> make_cuda_backend() { return std::make_unique<CudaBackend>(cuda::select_device().name); }
} // namespace stillflow
