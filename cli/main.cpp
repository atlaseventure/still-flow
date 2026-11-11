#include "stillflow/desktop.h"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {
constexpr std::string_view help = R"(stillflow - video stabilization with scanline wobble suppression

Usage: stillflow INPUT OUTPUT [options]

  --mode offline|realtime Single-pass fixed-lag streaming (default offline)
  --lookahead SECONDS     Streaming lookahead, 0..10 (default 1.5)
  --buffer-frames N       Maximum delayed frames, 2..600 (default 120)
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
  --crop FRACTION         Maximum equivalent per-edge crop, 0..0.25 (default 0.08)
  --crop-mode MODE        auto (shot-fixed), fixed or adaptive (default auto)
  --threads N             Threads per OpenCV/codec worker pool (default 12)
  --backend auto|NAME     Rendering backend (default auto)
  --decoder NAME          Video decoder: auto, cpu or nvdec (default auto)
  --encoder NAME          H.264 encoder: auto, openh264 or nvenc (default auto)
  --crf N                 H.264 quality, 0..51 (default 18); OpenH264 QP / NVENC CQ
  --preset NAME           NVENC speed preset (default fast); NVENC maps it or takes p1..p7
  --no-audio              Omit audio instead of stream copying
  --overwrite             Replace existing output/report files
  --version               Print version
  --help                  Show this help

Accepts progressive 8-bit SDR video with square pixels and even dimensions.
Writes H.264 MP4/MKV; keeps VFR presentation times and compatible audio.
auto assumes sensor rows and adjusts for display-rotation metadata.
Crop limits reduce correction strength. No IMU, training or network required.
Codec auto uses NVDEC/NVENC when rendering on CUDA and falls back to CPU
OpenH264; nvdec/nvenc require the NVIDIA driver and fail instead.
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
int integer(const std::string& text, const std::string& option) {
    const double value = number(text, option);
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max())
        throw std::runtime_error(option + " is out of range");
    if (std::floor(value) != value)
        throw std::runtime_error(option + " requires an integer");
    return static_cast<int>(value);
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--version") {
            std::cout << sf_desktop_build_description() << '\n';
            return EXIT_SUCCESS;
        }
        if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) {
            std::cout << help;
            return argc == 1 ? EXIT_FAILURE : EXIT_SUCCESS;
        }
        if (argc < 3)
            throw std::runtime_error("Expected INPUT and OUTPUT; see --help");
        sf_file_options o{};
        o.struct_size = sizeof(o);
        if (sf_file_options_default(&o) != SF_OK)
            throw std::runtime_error("Cannot initialize options");
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
            if (option == "--mode") {
                if (value != "offline" && value != "realtime")
                    throw std::runtime_error("--mode must be offline or realtime");
                o.realtime = value == "realtime";
            } else if (option == "--lookahead")
                o.lookahead_seconds = number(value, option);
            else if (option == "--buffer-frames") {
                const int capacity = integer(value, option);
                if (capacity < 2 || capacity > 600)
                    throw std::runtime_error("--buffer-frames must be 2..600");
                o.max_buffered_frames = static_cast<size_t>(capacity);
            } else if (option == "--compare")
                o.compare = argv[i];
            else if (option == "--report")
                o.report = argv[i];
            else if (option == "--analysis-width")
                o.stabilization.analysis_width = integer(value, option);
            else if (option == "--features")
                o.stabilization.features = integer(value, option);
            else if (option == "--bands")
                o.stabilization.bands = integer(value, option);
            else if (option == "--threads")
                o.threads = integer(value, option);
            else if (option == "--crf")
                o.crf = integer(value, option);
            else if (option == "--smooth")
                o.stabilization.smooth_seconds = number(value, option);
            else if (option == "--rs-smooth")
                o.stabilization.rs_seconds = number(value, option);
            else if (option == "--rs-strength")
                o.stabilization.rs_strength = number(value, option);
            else if (option == "--crop")
                o.stabilization.crop = number(value, option);
            else if (option == "--crop-mode") {
                if (value == "auto")
                    o.stabilization.crop_mode = SF_CROP_AUTO;
                else if (value == "fixed")
                    o.stabilization.crop_mode = SF_CROP_FIXED;
                else if (value == "adaptive")
                    o.stabilization.crop_mode = SF_CROP_ADAPTIVE;
                else
                    throw std::runtime_error("--crop-mode must be auto, fixed or adaptive");
            } else if (option == "--preset")
                o.preset = argv[i];
            else if (option == "--decoder") {
                if (value != "auto" && value != "cpu" && value != "nvdec")
                    throw std::runtime_error("--decoder must be auto, cpu or nvdec");
                o.decoder = argv[i];
            } else if (option == "--encoder") {
                if (value != "auto" && value != "openh264" && value != "nvenc")
                    throw std::runtime_error("--encoder must be auto, openh264 or nvenc");
                o.encoder = argv[i];
            } else if (option == "--backend") {
                if (backend_selected && value != o.backend)
                    throw std::runtime_error("Conflicting rendering backend selections");
                o.backend = argv[i];
                backend_selected = true;
            } else if (option == "--scan-axis") {
                o.scan_axis = argv[i];
            } else if (option == "--rs") {
                if (value != "on" && value != "off")
                    throw std::runtime_error("--rs must be on or off");
                o.stabilization.rolling_shutter = value == "on";
            } else
                throw std::runtime_error("Unknown option: " + option);
        }
        sf_run_summary result{};
        result.struct_size = sizeof(result);
        sf_error error{};
        error.struct_size = sizeof(error);
        sf_progress progress{sizeof(progress), nullptr, [](void*, const char* text) { std::cerr << text << '\n'; }};
        if (sf_stabilize_file(&o, nullptr, &progress, &result, &error) != SF_OK)
            throw std::runtime_error(error.message);
        const std::string report = o.report ? o.report : std::string(o.output) + ".profile.json";
        std::cout << std::fixed << std::setprecision(2) << result.frames << " frames, " << result.wall_ms / 1000
                  << " s, " << result.frames * 1000.0 / result.wall_ms << " end-to-end fps, " << result.peak_rss_mib
                  << " MiB peak RSS\nReport: " << report << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "stillflow: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
