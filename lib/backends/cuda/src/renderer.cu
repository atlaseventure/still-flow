#include "frames.cuh"
#include "stillflow/backends/cuda.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

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

struct DeviceGyroMap {
    int input_width, input_height, output_width, output_height;
    int lens_model, row_count, sensor_rows;
    float in_fx, in_fy, in_cx, in_cy;
    float k0, k1, k2, k3, lens_correction_amount;
    float out_fx, out_fy, out_cx, out_cy;
};

DeviceGyroMap pack_gyro_map(const GyroFrameMap& map, Size uploaded) {
    const auto& lens = map.lens;
    const auto finite = [](double value) { return std::isfinite(static_cast<float>(value)); };
    const auto focal = [&](double value) { return finite(value) && static_cast<float>(value) > 0; };
    if (uploaded.width < 2 || uploaded.height < 2 || map.input_size.width != uploaded.width ||
        map.input_size.height != uploaded.height || map.output_size.width < 2 || map.output_size.height < 2 ||
        map.row_rotations.empty() || map.row_rotations.size() > std::numeric_limits<int>::max() / 9 ||
        !focal(map.out_fx) || !focal(map.out_fy) || !focal(lens.fx) || !focal(lens.fy) || !finite(map.out_cx) ||
        !finite(map.out_cy) || !finite(lens.cx) || !finite(lens.cy) ||
        !std::all_of(lens.k.begin(), lens.k.end(), finite) ||
        !(lens.lens_correction_amount >= 0 && lens.lens_correction_amount <= 1) ||
        (lens.model != LensModel::none && lens.model != LensModel::opencv_fisheye))
        throw std::invalid_argument("Invalid CUDA gyro map");
    return {uploaded.width,
            uploaded.height,
            map.output_size.width,
            map.output_size.height,
            static_cast<int>(lens.model),
            static_cast<int>(map.row_rotations.size()),
            map.sensor_rows,
            static_cast<float>(lens.fx),
            static_cast<float>(lens.fy),
            static_cast<float>(lens.cx),
            static_cast<float>(lens.cy),
            static_cast<float>(lens.k[0]),
            static_cast<float>(lens.k[1]),
            static_cast<float>(lens.k[2]),
            static_cast<float>(lens.k[3]),
            static_cast<float>(lens.lens_correction_amount),
            static_cast<float>(map.out_fx),
            static_cast<float>(map.out_fy),
            static_cast<float>(map.out_cx),
            static_cast<float>(map.out_cy)};
}

__device__ bool gyro_project(const DeviceGyroMap& m, const float* row_matrices, float x, float y, float fraction,
                             float2& p) {
    const float nx = (x - m.out_cx) / m.out_fx, ny = (y - m.out_cy) / m.out_fy;
    const float row = fminf(fmaxf(fraction, 0), 1) * (m.row_count - 1);
    const int lo = min(static_cast<int>(row), m.row_count - 1), hi = min(lo + 1, m.row_count - 1);
    const float* a = row_matrices + static_cast<size_t>(lo) * 9;
    const float* b = row_matrices + static_cast<size_t>(hi) * 9;
    const auto component = [&](int i) {
        const float first = a[i] * nx + a[i + 1] * ny + a[i + 2];
        const float second = b[i] * nx + b[i + 1] * ny + b[i + 2];
        return first + (row - lo) * (second - first);
    };
    const float rz = component(6);
    if (!(rz > 1e-6f && isfinite(rz)))
        return false;
    float px = component(0) / rz, py = component(3) / rz;
    if (m.lens_model == 1 && m.lens_correction_amount > 0) {
        const float r = hypotf(px, py);
        if (r > 0) {
            const float theta = atanf(r), u = theta * theta;
            const float theta_d = theta * (1 + u * (m.k0 + u * (m.k1 + u * (m.k2 + u * m.k3))));
            const float scale = (r + (theta_d - r) * m.lens_correction_amount) / r;
            px *= scale;
            py *= scale;
        }
    }
    // Preserve exact border coordinates when the ray and intrinsics are unchanged.
    p.x = px == nx && m.in_fx == m.out_fx && m.in_cx == m.out_cx ? x : px * m.in_fx + m.in_cx;
    p.y = py == ny && m.in_fy == m.out_fy && m.in_cy == m.out_cy ? y : py * m.in_fy + m.in_cy;
    return isfinite(p.x) && isfinite(p.y);
}

__device__ bool gyro_source_position(const DeviceGyroMap& m, const float* rows, float x, float y, float2& p) {
    float fraction = y / (m.output_height - 1);
    for (int i = 0; i < (m.sensor_rows ? 3 : 1); ++i) {
        if (!gyro_project(m, rows, x, y, fraction, p))
            return false;
        fraction = p.y / (m.input_height - 1);
    }
    return p.x >= 0 && p.x <= m.input_width - 1 && p.y >= 0 && p.y <= m.input_height - 1;
}

__device__ float sample_mesh(const float* values, int a, int b, float fx, float fy) {
    return (values[a] * (1 - fx) + values[a + 1] * fx) * (1 - fy) + (values[b] * (1 - fx) + values[b + 1] * fx) * fy;
}

__device__ float2 source_position(const DeviceMap& m, float x, float y) {
    const float2 target{m.origin_x + x * m.step_x, m.origin_y + y * m.step_y};
    float2 p = target;
    for (int iteration = 0; iteration < mesh_inverse_iterations; ++iteration) {
        const float gx = fminf(m.columns - 1, fmaxf(0, p.x * m.grid_x));
        const float gy = fminf(m.rows - 1, fmaxf(0, p.y * m.grid_y));
        const int cx = min(static_cast<int>(gx), m.columns - 2), cy = min(static_cast<int>(gy), m.rows - 2);
        const float fx = gx - cx, fy = gy - cy;
        const int a = cy * m.columns + cx, b = a + m.columns;
        p = {target.x - sample_mesh(m.offset_x, a, b, fx, fy), target.y - sample_mesh(m.offset_y, a, b, fx, fy)};
    }
    return p;
}

// Keep matching YUV samples in their original code space, including headroom.
// Other matrices/ranges are converted in float without an 8-bit RGB intermediate.
template <bool gyro = false> __device__ float3 output_yuv(const DeviceSource& s, float2 at, int width, int height) {
    if (s.kr == 0.2126f && s.kb == 0.0722f && (!s.full_range || gyro)) {
        const auto p = cuda::coded_position(at.x, at.y, s.rotation, s.coded_width, s.coded_height);
        const float cx = (p.x - s.chroma_x) * 0.5f, cy = (p.y - s.chroma_y) * 0.5f;
        const int cw = (s.coded_width + 1) / 2, ch = (s.coded_height + 1) / 2;
        const float y = cuda::reconstruct<gyro>(s.luma, s.luma_pitch, 1, s.coded_width, s.coded_height, p.x, p.y);
        const float u = cuda::bilinear(s.chroma, s.chroma_pitch, 2, cw, ch, cx, cy);
        const float v = cuda::bilinear(s.chroma + 1, s.chroma_pitch, 2, cw, ch, cx, cy);
        if constexpr (gyro) {
            if (s.full_range)
                return {16 + y * (219.0f / 255), 128 + (u - 128) * (224.0f / 255), 128 + (v - 128) * (224.0f / 255)};
        }
        return {y, u, v};
    }
    const auto rgb = cuda::sample<true, gyro>(s, at.x, at.y, width, height);
    const float y = 0.2126f * rgb.z + 0.7152f * rgb.y + 0.0722f * rgb.x;
    return {16 + y * (219.0f / 255), 128 + (rgb.x - y) * (224.0f / (255 * 1.8556f)),
            128 + (rgb.z - y) * (224.0f / (255 * 1.5748f))};
}

__global__ void render_yuv_kernel(DeviceSource source, unsigned char* output, size_t pitch, int width, int height,
                                  const DeviceMap* map, unsigned int* invalid_count) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < width && y < height) {
        const auto p = source_position(*map, x, y);
        invalid = !(p.x >= 0 && p.x <= width - 1 && p.y >= 0 && p.y <= height - 1);
        output[static_cast<size_t>(y) * pitch + x] = cuda::to_byte(output_yuv(source, p, width, height).x);
        if (!(x & 1) && !(y & 1)) {
            // Left-sited chroma lives between the two luma rows.
            const auto uv = output_yuv(source, source_position(*map, x, fminf(y + 0.5f, height - 1)), width, height);
            auto* pixel = output + pitch * height + static_cast<size_t>(y / 2) * pitch + x;
            pixel[0] = cuda::to_byte(uv.y), pixel[1] = cuda::to_byte(uv.z);
        }
    }
    const int count = __syncthreads_count(invalid);
    if (threadIdx.x == 0 && threadIdx.y == 0 && count)
        atomicAdd(invalid_count, static_cast<unsigned int>(count));
}

__global__ void render_bgr_kernel(DeviceSource source, unsigned char* output, int width, int height,
                                  const DeviceMap* map, unsigned int* invalid_count) {
    const auto& m = *map;
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < width && y < height) {
        const auto p = source_position(m, x, y);
        const float px = p.x, py = p.y;
        invalid = !(px >= 0 && px <= width - 1 && py >= 0 && py <= height - 1);
        if (!invalid) {
            const float3 bgr = cuda::sample<false>(source, px, py, width, height);
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

__global__ void render_gyro_bgr_kernel(DeviceSource source, unsigned char* output, const DeviceGyroMap* map,
                                       const float* rows, unsigned int* invalid_count) {
    const auto& m = *map;
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < m.output_width && y < m.output_height) {
        float2 p;
        invalid = !gyro_source_position(m, rows, x, y, p);
        const float3 bgr =
            invalid ? float3{} : cuda::sample<false, true>(source, p.x, p.y, m.input_width, m.input_height);
        auto* pixel = output + (static_cast<size_t>(y) * m.output_width + x) * 3;
        pixel[0] = cuda::to_byte(bgr.x);
        pixel[1] = cuda::to_byte(bgr.y);
        pixel[2] = cuda::to_byte(bgr.z);
    }
    const int count = __syncthreads_count(invalid);
    if (threadIdx.x == 0 && threadIdx.y == 0 && count)
        atomicAdd(invalid_count, static_cast<unsigned int>(count));
}

__global__ void render_gyro_yuv_kernel(DeviceSource source, unsigned char* output, size_t pitch,
                                       const DeviceGyroMap* map, const float* rows, unsigned int* invalid_count) {
    const auto& m = *map;
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    bool invalid = false;
    if (x < m.output_width && y < m.output_height) {
        float2 p;
        invalid = !gyro_source_position(m, rows, x, y, p);
        output[static_cast<size_t>(y) * pitch + x] =
            invalid ? 16 : cuda::to_byte(output_yuv<true>(source, p, m.input_width, m.input_height).x);
        if (!(x & 1) && !(y & 1)) {
            const bool valid = gyro_source_position(m, rows, x, fminf(y + 0.5f, m.output_height - 1), p);
            invalid |= !valid;
            const float3 uv = valid ? output_yuv<true>(source, p, m.input_width, m.input_height) : float3{16, 128, 128};
            auto* pixel = output + pitch * m.output_height + static_cast<size_t>(y / 2) * pitch + x;
            pixel[0] = cuda::to_byte(uv.y);
            pixel[1] = cuda::to_byte(uv.z);
        }
    }
    const int count = __syncthreads_count(invalid);
    if (threadIdx.x == 0 && threadIdx.y == 0 && count)
        atomicAdd(invalid_count, static_cast<unsigned int>(count));
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
                {{"compiler", STILLFLOW_CUDA_VERSION},
                 {"sampler", "visual Catmull-Rom, gyro Lanczos4 luma/RGB, bilinear chroma"},
                 {"output", "nv12 bt709 (device), lazy host bgr8"}}};
    }
    void upload(const Image& input) override {
        lease_.reset();
        frame_.reset();
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
                std::memcpy(staging_.data() + y * stride,
                            mapping.planes[0].bytes.data() + y * mapping.planes[0].row_stride, stride);
            check(cudaMemcpyAsync(input_.data(), staging_.data(), bytes, cudaMemcpyHostToDevice, stream_.value),
                  "Upload CUDA frame");
            source_.bgr = input_.data();
            yuv_ = false;
        }
    }
    double warp(const MapParameters& p) override {
        prepare_output(size_);
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
            render_yuv_kernel<<<grid, block, 0, stream_.value>>>(source_, frame_->nv12, frame_->pitch, size_.width,
                                                                 size_.height, map_.data(), invalid_.data());
        else
            render_bgr_kernel<<<grid, block, 0, stream_.value>>>(source_, frame_->bgr, size_.width, size_.height,
                                                                 map_.data(), invalid_.data());
        check(cudaGetLastError(), "Launch CUDA warp");
        return invalid_fraction(size_);
    }
    double warp_gyro(const GyroFrameMap& map) override {
        if (!source_.bgr && !lease_)
            throw std::invalid_argument("No uploaded CUDA gyro input");
        const auto m = pack_gyro_map(map, size_);
        std::vector<float> rows;
        rows.reserve(map.row_rotations.size() * 9);
        for (const auto& rotation : map.row_rotations)
            rows.insert(rows.end(), rotation.values.begin(), rotation.values.end());
        prepare_output(map.output_size);
        row_matrices_.reserve(rows.size());
        gyro_map_.reserve(1);
        check(cudaMemcpyAsync(row_matrices_.data(), rows.data(), rows.size() * sizeof(float), cudaMemcpyHostToDevice,
                              stream_.value),
              "Upload gyro row rotations");
        check(cudaMemcpyAsync(gyro_map_.data(), &m, sizeof(m), cudaMemcpyHostToDevice, stream_.value),
              "Upload gyro map");
        check(cudaMemsetAsync(invalid_.data(), 0, sizeof(unsigned int), stream_.value), "Reset CUDA crop check");
        const dim3 block(16, 16), grid((m.output_width + 15) / 16, (m.output_height + 15) / 16);
        if (yuv_)
            render_gyro_yuv_kernel<<<grid, block, 0, stream_.value>>>(
                source_, frame_->nv12, frame_->pitch, gyro_map_.data(), row_matrices_.data(), invalid_.data());
        else
            render_gyro_bgr_kernel<<<grid, block, 0, stream_.value>>>(source_, frame_->bgr, gyro_map_.data(),
                                                                      row_matrices_.data(), invalid_.data());
        check(cudaGetLastError(), "Launch CUDA gyro warp");
        return invalid_fraction(map.output_size);
    }
    Image download() override {
        lease_.reset();
        return cuda::publish(std::move(frame_), stream_.value, yuv_);
    }

  private:
    void prepare_output(Size output) {
        // Crop retries on the same upload reuse the existing allocation.
        if (!frame_ || frame_->size.width != output.width || frame_->size.height != output.height)
            frame_ = pool_->acquire(output);
        if (!yuv_)
            frame_->require_bgr();
    }
    double invalid_fraction(Size output) {
        unsigned int invalid = 0;
        check(cudaMemcpyAsync(&invalid, invalid_.data(), sizeof(invalid), cudaMemcpyDeviceToHost, stream_.value),
              "Read CUDA crop check");
        check(cudaStreamSynchronize(stream_.value), "Complete CUDA warp");
        return static_cast<double>(invalid) / (static_cast<double>(output.width) * output.height);
    }
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
    Buffer<DeviceGyroMap> gyro_map_;
    Buffer<float> row_matrices_;
};
} // namespace

std::unique_ptr<RenderBackend> make_cuda_backend() { return std::make_unique<CudaBackend>(cuda::select_device().name); }
} // namespace stillflow
