#include "stillflow/adapters/backends.hpp"
#include "stillflow/application/stabilize.hpp"
#include "stillflow/backends/cpu.hpp"
#ifdef STILLFLOW_CLI_CUDA
#include "stillflow/backends/cuda.hpp"
#endif

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {
class ConsoleProgress final : public stillflow::Progress {
  public:
    void backend(const stillflow::RenderingReport& report) override {
        std::cerr << "Render backend: " << report.backend;
        if (!report.backends.empty())
            std::cerr << " (" << report.backends.back().info.device << ')';
        for (const auto& failure : report.failures)
            std::cerr << "; " << failure.backend << ": " << failure.reason;
        std::cerr << '\n';
    }
    void codec(const std::string& stream, const stillflow::CodecReport& report) override {
        std::cerr << "Codec " << stream << ": " << report.implementation;
        if (!report.device.empty())
            std::cerr << " (" << report.device << ", " << report.hardware_frames << '/' << report.frames
                      << " frames in hardware)";
        for (const auto& reason : report.fallbacks)
            std::cerr << "; " << reason;
        std::cerr << '\n';
    }
    void analyzing(const stillflow::VideoInfo& info, stillflow::Size size, bool horizontal) override {
        std::cerr << "Analyze " << info.width << 'x' << info.height << " at " << size.width << 'x' << size.height
                  << ", scan axis " << (horizontal ? 'x' : 'y') << '\n';
    }
    void analyzed(size_t frames, bool complete) override {
        std::cerr << "\rAnalyzed " << frames << " frames";
        if (complete)
            std::cerr << '\n';
        else
            std::cerr << std::flush;
    }
    void rendered(size_t frames, size_t total, bool complete) override {
        std::cerr << "\rRendered " << frames << '/' << total;
        if (complete)
            std::cerr << '\n';
        else
            std::cerr << std::flush;
    }
};
constexpr std::string_view help = R"(stillflow - offline video stabilization with scanline wobble suppression

Usage: stillflow INPUT OUTPUT [options]

  --compare PATH          Side-by-side input/output video (adds encoding cost)
  --report PATH           JSON profile; default OUTPUT.profile.json
  --rs on|off             Scanline correction (default on)
  --scan-axis auto|x|y    Scan axis in displayed pixels (default auto)
  --analysis-width N      Proxy width, no upscaling (default 960)
  --features N            Feature budget (default 1200)
  --bands N               Scanline control bands, 4..32 (default 12)
  --smooth SECONDS        Global trajectory smoothing scale (default 0.45)
  --rs-smooth SECONDS     Scanline trajectory smoothing scale (default 0.30)
  --rs-strength N         Scanline correction gain, 0..1 (default 1)
  --crop FRACTION         Fixed crop on EACH edge, 0..0.25 (default 0.08)
  --threads N             Threads per OpenCV/codec worker pool (default 12)
  --backend auto|NAME     Rendering backend (default auto)
  --decoder NAME          Video decoder: auto, cpu or nvdec (default auto)
  --encoder NAME          H.264 encoder: auto, x264 or nvenc (default auto)
  --crf N                 H.264 quality, 0..51 (default 18); NVENC constant quality
  --preset NAME           x264 preset (default fast); NVENC maps it or takes p1..p7
  --no-audio              Omit audio instead of stream copying
  --overwrite             Replace existing output/report files
  --version               Print version
  --help                  Show this help

Accepts progressive 8-bit SDR video with square pixels and even dimensions.
Writes H.264 MP4/MKV; keeps VFR presentation times and compatible audio.
auto assumes sensor rows and adjusts for display-rotation metadata.
Crop limits reduce correction strength. No IMU, training or network required.
Codec auto uses NVDEC/NVENC when rendering on CUDA and falls back to CPU
codecs; nvdec/nvenc require the NVIDIA driver and fail instead.
)";

double number(const std::string& text, const std::string& option) {
    size_t used = 0;
    double result;
    try {
        result = std::stod(text, &used);
    } catch (...) {
        throw std::runtime_error("Invalid number for " + option);
    }
    if (used != text.size() || !std::isfinite(result))
        throw std::runtime_error("Invalid number for " + option);
    return result;
}
double bounded(const std::string& text, const std::string& option, double low, double high) {
    const double value = number(text, option);
    if (value < low || value > high)
        throw std::runtime_error(option + " is out of range");
    return value;
}
int integer(const std::string& text, const std::string& option, int low, int high) {
    const double value = bounded(text, option, low, high);
    if (std::floor(value) != value)
        throw std::runtime_error(option + " requires an integer");
    return static_cast<int>(value);
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--version") {
            std::cout << stillflow::build_description() << '\n';
            return EXIT_SUCCESS;
        }
        if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) {
            std::cout << help;
            return argc == 1 ? EXIT_FAILURE : EXIT_SUCCESS;
        }
        if (argc < 3)
            throw std::runtime_error("Expected INPUT and OUTPUT; see --help");
        stillflow::Options o;
        o.input = argv[1];
        o.output = argv[2];
        bool backend_selected = false;
        for (int i = 3; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--no-audio") {
                o.audio = false;
                continue;
            }
            if (option == "--overwrite") {
                o.overwrite = true;
                continue;
            }
            if (option == "--help") {
                std::cout << help;
                return EXIT_SUCCESS;
            }
            if (i + 1 >= argc)
                throw std::runtime_error("Missing value for " + option);
            const std::string value = argv[++i];
            if (option == "--compare")
                o.compare = value;
            else if (option == "--report")
                o.report = value;
            else if (option == "--analysis-width")
                o.analysis_width = integer(value, option, 160, 4096);
            else if (option == "--features")
                o.features = integer(value, option, 96, 10000);
            else if (option == "--bands")
                o.bands = integer(value, option, 4, 32);
            else if (option == "--threads")
                o.threads = integer(value, option, 1, 64);
            else if (option == "--crf")
                o.crf = integer(value, option, 0, 51);
            else if (option == "--smooth")
                o.smooth_seconds = bounded(value, option, 0, 10);
            else if (option == "--rs-smooth")
                o.rs_seconds = bounded(value, option, 0, 10);
            else if (option == "--rs-strength")
                o.rs_strength = bounded(value, option, 0, 1);
            else if (option == "--crop")
                o.crop = bounded(value, option, 0, 0.25);
            else if (option == "--preset")
                o.preset = value;
            else if (option == "--decoder") {
                if (value != "auto" && value != "cpu" && value != "nvdec")
                    throw std::runtime_error("--decoder must be auto, cpu or nvdec");
                o.decoder = value;
            } else if (option == "--encoder") {
                if (value != "auto" && value != "x264" && value != "nvenc")
                    throw std::runtime_error("--encoder must be auto, x264 or nvenc");
                o.encoder = value;
            } else if (option == "--backend") {
                if (backend_selected && o.backend != value)
                    throw std::runtime_error("Conflicting rendering backend selections");
                o.backend = value;
                backend_selected = true;
            } else if (option == "--scan-axis") {
                if (value != "auto" && value != "x" && value != "y")
                    throw std::runtime_error("--scan-axis must be auto, x or y");
                o.scan_axis = value;
            } else if (option == "--rs") {
                if (value != "on" && value != "off")
                    throw std::runtime_error("--rs must be on or off");
                o.rolling_shutter = value == "on";
            } else
                throw std::runtime_error("Unknown option: " + option);
        }
        if (o.report.empty())
            o.report = o.output.string() + ".profile.json";
        auto media = stillflow::make_media_factory();
        auto images = stillflow::make_image_processing();
        std::vector<stillflow::BackendProvider> providers;
#ifdef STILLFLOW_CLI_CUDA
        providers.push_back({"cuda", stillflow::make_cuda_backend, true});
#endif
        providers.push_back({"cpu", stillflow::make_cpu_backend});
        auto renderers = stillflow::make_renderer_factory(std::move(providers));
        auto outputs = stillflow::make_output_storage();
        auto runtime = stillflow::make_runtime();
        ConsoleProgress progress;
        const auto result = stillflow::stabilize(o, {*media, *images, *renderers, *outputs, *runtime, progress});
        std::cout << std::fixed << std::setprecision(2) << result.frames << " frames, " << result.wall_ms / 1000
                  << " s, " << result.frames * 1000.0 / result.wall_ms << " end-to-end fps, "
                  << result.resources.peak_rss_mib << " MiB peak RSS\nReport: " << o.report << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "stillflow: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
