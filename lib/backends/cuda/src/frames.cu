#include "frames.cuh"

namespace stillflow::cuda {
namespace {
// One 2x2 block per thread. Chroma is sited left (between rows, on even
// columns) with a [1 2 1] horizontal filter.
__global__ void nv12_kernel(const unsigned char* bgr, unsigned char* luma, unsigned char* chroma, size_t pitch,
                            int width, int height) {
    constexpr float kr = 0.2126f, kb = 0.0722f, kg = 1 - kr - kb;
    const int bx = blockIdx.x * blockDim.x + threadIdx.x, by = blockIdx.y * blockDim.y + threadIdx.y;
    const int x = bx * 2, y = by * 2;
    if (x >= width || y >= height)
        return;
    float3 sum{0, 0, 0};
    for (int row = 0; row < 2; ++row) {
        for (int column = -1; column <= 1; ++column) {
            const int px = min(max(x + column, 0), width - 1), py = min(y + row, height - 1);
            const unsigned char* pixel = bgr + (static_cast<size_t>(py) * width + px) * 3;
            const float b = pixel[0], g = pixel[1], r = pixel[2];
            if (column >= 0 && x + column < width && y + row < height)
                luma[static_cast<size_t>(y + row) * pitch + x + column] =
                    to_byte(16 + 219 * (kr * r + kg * g + kb * b) / 255);
            const float weight = column == 0 ? 0.5f : 0.25f;
            sum.x += weight * b, sum.y += weight * g, sum.z += weight * r;
        }
    }
    const float b = sum.x / 2, g = sum.y / 2, r = sum.z / 2;
    const float yy = kr * r + kg * g + kb * b;
    unsigned char* uv = chroma + static_cast<size_t>(by) * pitch + x;
    uv[0] = to_byte(128 + 224 * (b - yy) / (2 * (1 - kb)) / 255);
    uv[1] = to_byte(128 + 224 * (r - yy) / (2 * (1 - kr)) / 255);
}

__global__ void bgr_kernel(DeviceSource source, unsigned char* output, int width, int height) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height)
        return;
    const auto value = sample<true>(source, x, y, width, height);
    auto* pixel = output + (static_cast<size_t>(y) * width + x) * 3;
    pixel[0] = to_byte(value.x), pixel[1] = to_byte(value.y), pixel[2] = to_byte(value.z);
}

class PublishedImage final : public ImageData {
  public:
    PublishedImage(std::shared_ptr<const OutputFrame> frame, bool native) : frame_(std::move(frame)), native_(native) {}
    Size size() const override { return frame_->size; }
    PixelFormat format() const override { return PixelFormat::bgr8; }
    HostImageView map_read() const override {
        std::call_once(downloaded_, [&] {
            const size_t bytes = static_cast<size_t>(frame_->size.width) * frame_->size.height * 3;
            std::vector<uint8_t> pixels(bytes);
            check(cudaSetDevice(0), "Select CUDA device");
            Buffer<unsigned char> converted;
            const unsigned char* bgr = frame_->bgr;
            if (native_) {
                converted.reserve(bytes);
                const dim3 block(16, 16), grid((frame_->size.width + 15) / 16, (frame_->size.height + 15) / 16);
                bgr_kernel<<<grid, block>>>(yuv_source(frame_->descriptor), converted.data(), frame_->size.width,
                                            frame_->size.height);
                check(cudaGetLastError(), "Convert CUDA host mapping");
                bgr = converted.data();
            }
            check(cudaMemcpy(pixels.data(), bgr, bytes, cudaMemcpyDeviceToHost), "Download CUDA frame");
            host_ = make_host_image(frame_->size, PixelFormat::bgr8, std::move(pixels));
        });
        return host_->map_read();
    }
    std::optional<NativeImageView> native_read(std::string_view api) const override {
        if (api != cuda_image_api)
            return std::nullopt;
        return NativeImageView{std::string(api), &frame_->descriptor, frame_};
    }

  private:
    std::shared_ptr<const OutputFrame> frame_;
    bool native_;
    mutable std::once_flag downloaded_;
    mutable Image host_;
};
} // namespace

OutputFrame::OutputFrame(Size value) : size(value) {
    const auto width = static_cast<size_t>(size.width), height = static_cast<size_t>(size.height);
    const cudaError_t allocated =
        cudaMallocPitch(reinterpret_cast<void**>(&nv12), &pitch, width, height + (height + 1) / 2);
    check(allocated, "Allocate CUDA NV12 frame");
    descriptor.coded = size;
    descriptor.luma = nv12;
    descriptor.chroma = nv12 + pitch * height;
    descriptor.luma_pitch = descriptor.chroma_pitch = pitch;
    descriptor.color.kr = 0.2126f, descriptor.color.kb = 0.0722f;
    descriptor.color.full_range = false;
    descriptor.color.chroma_x = 0, descriptor.color.chroma_y = 0.5f;
}
void OutputFrame::require_bgr() {
    if (!bgr)
        check(cudaMalloc(reinterpret_cast<void**>(&bgr), static_cast<size_t>(size.width) * size.height * 3),
              "Allocate CUDA BGR frame");
}
OutputFrame::~OutputFrame() {
    cudaFree(nv12);
    cudaFree(bgr);
}

std::shared_ptr<OutputFrame> FramePool::acquire(Size size) {
    std::unique_ptr<OutputFrame> frame;
    {
        std::lock_guard lock(mutex_);
        while (!free_.empty() && !frame) {
            frame = std::move(free_.back());
            free_.pop_back();
            if (frame->size.width != size.width || frame->size.height != size.height)
                frame.reset();
        }
    }
    if (!frame)
        frame = std::make_unique<OutputFrame>(size);
    return {frame.release(), [pool = shared_from_this()](OutputFrame* released) { pool->release(released); }};
}
void FramePool::release(OutputFrame* frame) {
    std::lock_guard lock(mutex_);
    free_.emplace_back(frame);
}

Image publish(std::shared_ptr<OutputFrame> frame, cudaStream_t stream, bool nv12_ready) {
    const auto [width, height] = frame->size;
    const dim3 block(16, 16), grid((width + 31) / 32, (height + 31) / 32);
    if (!nv12_ready)
        nv12_kernel<<<grid, block, 0, stream>>>(frame->bgr, frame->nv12, frame->nv12 + frame->pitch * height,
                                                frame->pitch, width, height);
    check(cudaGetLastError(), "Launch CUDA NV12 conversion");
    check(cudaStreamSynchronize(stream), "Complete CUDA frame");
    return std::make_shared<PublishedImage>(std::move(frame), nv12_ready);
}
} // namespace stillflow::cuda
