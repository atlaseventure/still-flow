#include "stillflow/stabilizer.hpp"

#include <opencv2/core/version.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <cstdlib>

namespace {
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
  --crf N                 H.264 quality, 0..51 (default 18)
  --preset NAME           x264 encoding preset (default fast)
  --no-audio              Omit audio instead of stream copying
  --overwrite             Replace existing output/report files
  --version               Print version
  --help                  Show this help

Accepts progressive 8-bit SDR video with square pixels and even dimensions.
Writes H.264 MP4/MKV; keeps VFR presentation times and compatible audio.
auto assumes sensor rows and adjusts for display-rotation metadata.
Crop limits reduce correction strength. No IMU, training or network required.
)";

double number(const std::string& text, const std::string& option) {
    size_t used = 0;
    double result;
    try { result = std::stod(text, &used); }
    catch (...) { throw std::runtime_error("Invalid number for " + option); }
    if (used != text.size() || !std::isfinite(result)) throw std::runtime_error("Invalid number for " + option);
    return result;
}
double bounded(const std::string& text, const std::string& option, double low, double high) {
    const double value = number(text, option);
    if (value < low || value > high) throw std::runtime_error(option + " is out of range");
    return value;
}
int integer(const std::string& text, const std::string& option, int low, int high) {
    const double value = bounded(text, option, low, high);
    if (std::floor(value) != value) throw std::runtime_error(option + " requires an integer");
    return static_cast<int>(value);
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--version") {
            std::cout << "stillflow " << STILLFLOW_VERSION << " (C++20, OpenCV " << CV_VERSION << ")\n";
            return EXIT_SUCCESS;
        }
        if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) {
            std::cout << help; return argc == 1 ? EXIT_FAILURE : EXIT_SUCCESS;
        }
        if (argc < 3) throw std::runtime_error("Expected INPUT and OUTPUT; see --help");
        stillflow::Options o;
        o.input = argv[1]; o.output = argv[2];
        for (int i = 3; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--no-audio") { o.audio = false; continue; }
            if (option == "--overwrite") { o.overwrite = true; continue; }
            if (option == "--help") { std::cout << help; return EXIT_SUCCESS; }
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + option);
            const std::string value = argv[++i];
            if (option == "--compare") o.compare = value;
            else if (option == "--report") o.report = value;
            else if (option == "--analysis-width") o.analysis_width = integer(value, option, 160, 4096);
            else if (option == "--features") o.features = integer(value, option, 96, 10000);
            else if (option == "--bands") o.bands = integer(value, option, 4, 32);
            else if (option == "--threads") o.threads = integer(value, option, 1, 64);
            else if (option == "--crf") o.crf = integer(value, option, 0, 51);
            else if (option == "--smooth") o.smooth_seconds = bounded(value, option, 0, 10);
            else if (option == "--rs-smooth") o.rs_seconds = bounded(value, option, 0, 10);
            else if (option == "--rs-strength") o.rs_strength = bounded(value, option, 0, 1);
            else if (option == "--crop") o.crop = bounded(value, option, 0, 0.25);
            else if (option == "--preset") o.preset = value;
            else if (option == "--scan-axis") {
                if (value != "auto" && value != "x" && value != "y")
                    throw std::runtime_error("--scan-axis must be auto, x or y");
                o.scan_axis = value;
            } else if (option == "--rs") {
                if (value != "on" && value != "off") throw std::runtime_error("--rs must be on or off");
                o.rolling_shutter = value == "on";
            } else throw std::runtime_error("Unknown option: " + option);
        }
        if (o.report.empty()) o.report = o.output.string() + ".profile.json";
        return stillflow::run(o);
    } catch (const std::exception& error) {
        std::cerr << "stillflow: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
