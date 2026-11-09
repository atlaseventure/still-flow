#pragma once
#include "device.cuh"

#include <memory>
#include <mutex>
#include <vector>

namespace stillflow::cuda {
// A device frame: packed BGR written by kernels, plus the NV12 copy encoders
// read in place (BT.709 limited range, left-sited chroma).
struct OutputFrame {
    explicit OutputFrame(Size value);
    ~OutputFrame();
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

// Fills the NV12 planes from BGR on stream, waits for completion (consumers use
// other streams and engines) and returns an image exposing both: native reads
// see NV12 in place, host mappings download BGR once.
Image publish(std::shared_ptr<OutputFrame> frame, cudaStream_t stream);
} // namespace stillflow::cuda
