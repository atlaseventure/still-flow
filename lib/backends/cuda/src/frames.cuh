#pragma once
#include "device.cuh"

#include <memory>
#include <mutex>
#include <vector>

namespace stillflow::cuda {
// NV12 output (BT.709 limited range, left-sited chroma). Packed RGB callers
// allocate BGR on demand; native YUV rendering only allocates NV12.
struct OutputFrame {
    explicit OutputFrame(Size value);
    ~OutputFrame();
    void require_bgr();
    OutputFrame(const OutputFrame&) = delete;
    OutputFrame& operator=(const OutputFrame&) = delete;
    Size size;
    unsigned char* bgr = nullptr;
    unsigned char* nv12 = nullptr;
    size_t pitch = 0;
    CudaImage descriptor;
};

// Frames return to the pool once encoders and other consumers drop them.
class FramePool : public std::enable_shared_from_this<FramePool> {
  public:
    std::shared_ptr<OutputFrame> acquire(Size size);

  private:
    void release(OutputFrame* frame);
    std::mutex mutex_;
    std::vector<std::unique_ptr<OutputFrame>> free_;
};

// Convert packed output unless NV12 is already rendered, then wait for readers
// on other streams/engines. Host BGR mappings are materialized on first access.
Image publish(std::shared_ptr<OutputFrame> frame, cudaStream_t stream, bool nv12_ready = false);
} // namespace stillflow::cuda
