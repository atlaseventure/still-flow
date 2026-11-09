#include "frames.cuh"
#include "stillflow/backends/cuda.hpp"

#include <vector>

namespace stillflow {
namespace {
using cuda::Buffer;
using cuda::check;
using cuda::Stream;

// Area-averages the displayed luma plane (like INTER_AREA when shrinking) and
// expands limited range to full, approximating the host BGR->gray proxy.
__global__ void luma_proxy_kernel(const unsigned char* luma, size_t pitch, int coded_width, int coded_height,
                                  int rotation, int full_range, int width, int height, unsigned char* output,
                                  int output_width, int output_height) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= output_width || y >= output_height)
        return;
    const float scale_x = static_cast<float>(width) / output_width,
                scale_y = static_cast<float>(height) / output_height;
    const float left = x * scale_x, right = fminf((x + 1) * scale_x, width);
    const float top = y * scale_y, bottom = fminf((y + 1) * scale_y, height);
    float sum = 0, area = 0;
    for (int row = static_cast<int>(top); row < bottom; ++row) {
        const float weight_y = fminf(row + 1, bottom) - fmaxf(row, top);
        for (int column = static_cast<int>(left); column < right; ++column) {
            const float weight = weight_y * (fminf(column + 1, right) - fmaxf(column, left));
            const float2 p = cuda::coded_position(column, row, rotation, coded_width, coded_height);
            sum += weight * luma[static_cast<size_t>(p.y) * pitch + static_cast<int>(p.x)];
            area += weight;
        }
    }
    float value = sum / area;
    if (!full_range)
        value = (value - 16) * (255.0f / 219);
    output[static_cast<size_t>(y) * output_width + x] =
        static_cast<unsigned char>(__float2uint_rn(fminf(fmaxf(value, 0), 255)));
}

// Places input and stabilized frames side by side and blends white labels
// with the given coverage (0..255) over them.
__global__ void compose_kernel(cuda::DeviceSource input, cuda::DeviceSource stabilized, int width, int height,
                               const unsigned char* labels, unsigned char* output) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x, y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width * 2 || y >= height)
        return;
    const bool left = x < width;
    const float3 bgr = cuda::sample<true>(left ? input : stabilized, left ? x : x - width, y, width, height);
    const size_t index = static_cast<size_t>(y) * width * 2 + x;
    const float alpha = labels[index] / 255.0f;
    unsigned char* out = output + index * 3;
    out[0] = cuda::to_byte(bgr.x * (1 - alpha) + 255 * alpha);
    out[1] = cuda::to_byte(bgr.y * (1 - alpha) + 255 * alpha);
    out[2] = cuda::to_byte(bgr.z * (1 - alpha) + 255 * alpha);
}

class CudaImageProcessing final : public ImageProcessing {
  public:
    explicit CudaImageProcessing(std::unique_ptr<ImageProcessing> host) : host_(std::move(host)) {
        if (!host_)
            throw std::invalid_argument("Missing host image processing");
    }
    ~CudaImageProcessing() override {
        if (stream_)
            cudaStreamSynchronize(stream_->value);
    }
    std::unique_ptr<MotionAnalyzer> analyzer(const StabilizationSettings& settings, bool horizontal) override {
        return host_->analyzer(settings, horizontal);
    }
    Image gray_proxy(const Image& input, Size size) override {
        if (!disabled_ && size.width > 0 && size.height > 0) {
            if (auto source = cuda::native_source(input)) {
                try {
                    return device_proxy(source->image, input->size(), size);
                } catch (const BackendError&) {
                    disabled_ = true; // Keep analysing on the host path.
                }
            }
        }
        return host_->gray_proxy(input, size);
    }
    Image comparison(const Image& input, const Image& stabilized) override {
        if (!disabled_ && input && stabilized && input->size().width == stabilized->size().width &&
            input->size().height == stabilized->size().height) {
            auto left = cuda::native_source(input), right = cuda::native_source(stabilized);
            if (left && right) {
                try {
                    return device_comparison(left->image, right->image, input->size());
                } catch (const BackendError&) {
                    disabled_ = true;
                }
            }
        }
        return host_->comparison(input, stabilized);
    }

  private:
    void prepare() {
        if (!stream_) {
            cuda::select_device();
            stream_ = std::make_unique<Stream>();
        }
        check(cudaSetDevice(0), "Select CUDA device");
    }
    // Only the reduced luma plane leaves the GPU.
    Image device_proxy(const CudaImage& image, Size logical, Size size) {
        prepare();
        const size_t bytes = static_cast<size_t>(size.width) * size.height;
        output_.reserve(bytes);
        staging_.reserve(bytes);
        const dim3 block(16, 16), grid((size.width + 15) / 16, (size.height + 15) / 16);
        luma_proxy_kernel<<<grid, block, 0, stream_->value>>>(
            image.luma, image.luma_pitch, image.coded.width, image.coded.height, image.rotation, image.full_range,
            logical.width, logical.height, output_.data(), size.width, size.height);
        check(cudaGetLastError(), "Launch CUDA analysis proxy");
        check(cudaMemcpyAsync(staging_.data(), output_.data(), bytes, cudaMemcpyDeviceToHost, stream_->value),
              "Download CUDA analysis proxy");
        check(cudaStreamSynchronize(stream_->value), "Complete CUDA analysis proxy");
        return make_host_image(size, PixelFormat::gray8,
                               std::vector<uint8_t>(staging_.data(), staging_.data() + bytes));
    }
    // The composite stays on the GPU for the encoder. Labels come from the host
    // composite of two black frames, so text matches the host path; this relies
    // on it drawing white labels over the concatenated frames.
    Image device_comparison(const CudaImage& input, const CudaImage& stabilized, Size size) {
        prepare();
        const Size combined{size.width * 2, size.height};
        const size_t pixels = static_cast<size_t>(combined.width) * combined.height;
        if (label_size_.width != combined.width || label_size_.height != combined.height) {
            const auto black = make_host_image(size, PixelFormat::bgr8,
                                               std::vector<uint8_t>(static_cast<size_t>(size.width) * size.height * 3));
            const auto composite = map_host_image(host_->comparison(black, black));
            if (composite.size.width != combined.width || composite.size.height != combined.height ||
                composite.format != PixelFormat::bgr8)
                throw BackendError("Host comparison layout is not a side-by-side BGR composite");
            std::vector<uint8_t> coverage(pixels);
            for (int y = 0; y < combined.height; ++y)
                for (int x = 0; x < combined.width; ++x)
                    coverage[static_cast<size_t>(y) * combined.width + x] =
                        composite.bytes[static_cast<size_t>(y) * composite.row_stride + static_cast<size_t>(x) * 3];
            labels_.reserve(pixels);
            check(cudaMemcpy(labels_.data(), coverage.data(), pixels, cudaMemcpyHostToDevice), "Upload CUDA labels");
            label_size_ = combined;
        }
        auto frame = pool_->acquire(combined);
        const dim3 block(16, 16), grid((combined.width + 15) / 16, (combined.height + 15) / 16);
        compose_kernel<<<grid, block, 0, stream_->value>>>(cuda::yuv_source(input), cuda::yuv_source(stabilized),
                                                           size.width, size.height, labels_.data(), frame->bgr);
        check(cudaGetLastError(), "Launch CUDA comparison");
        return cuda::publish(std::move(frame), stream_->value);
    }
    std::unique_ptr<ImageProcessing> host_;
    std::unique_ptr<Stream> stream_;
    std::shared_ptr<cuda::FramePool> pool_ = std::make_shared<cuda::FramePool>();
    Buffer<unsigned char> labels_;
    Size label_size_;
    Buffer<unsigned char> output_;
    Buffer<unsigned char, true> staging_;
    bool disabled_ = false;
};
} // namespace

std::unique_ptr<ImageProcessing> make_cuda_image_processing(std::unique_ptr<ImageProcessing> host) {
    return std::make_unique<CudaImageProcessing>(std::move(host));
}
} // namespace stillflow
