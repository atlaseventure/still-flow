#include "stillflow/desktop.h"
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unistd.h>

namespace {
constexpr std::string_view help = R"(stillflow - video stabilization with scanline wobble suppression

Usage: stillflow INPUT OUTPUT [options]

  --mode offline|realtime Single-pass fixed-lag streaming (default offline)
  --lookahead SECONDS     Streaming lookahead, 0..10 (default 1.35)
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
  --gyro-config PATH      Offline CUDA stabilization using stabilization.json or .gyroflow
  --imu PATH              Override the camera-frame IMU CSV
  --output-size WxH       Gyro output dimensions
  --gyro-smooth N         Gyro smoothness, 0..1
  --max-zoom N            Maximum zoom ratio, or percentage above 10
  --zoom-window SECONDS   Adaptive zoom window, 0 for static zoom
  --fov N                 Positive field-of-view multiplier
  --lens-correction N     Lens correction amount, 0..1
  --readout-ms N          Rolling shutter readout in milliseconds
  --gyro-offset MS        Additional IMU time offset
  --imu-integration MODE  auto, gyro or complementary
  --backend auto|NAME     Rendering backend (default auto)
  --decoder NAME          Video decoder: auto, cpu or nvdec (default auto)
  --encoder NAME          H.264 encoder: auto, openh264 or nvenc (default auto)
  --crf N                 H.264 quality, 0..51 (default 18); OpenH264 QP / NVENC CQ
  --preset NAME           NVENC speed preset (default fast); NVENC maps it or takes p1..p7
  --audio MODE            auto (copy compatible tracks), copy, aac or none
  --no-audio              Alias for --audio none
  --overwrite             Replace existing output/report files
  --version               Print version
  --help                  Show this help

Accepts progressive 8-bit SDR video with square pixels and even dimensions.
Writes H.264 MP4/MKV; keeps VFR presentation times and compatible audio.
auto assumes sensor rows and adjusts for display-rotation metadata.
Visual stabilization runs without IMU data. Crop limits reduce visual correction strength.
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
bool gyro_option(sf_gyro_options& gyro, const std::string& option, const char* argument) {
    const std::string value = argument;
    auto bounded = [&](double low, double high) {
        const double result = number(value, option);
        if (result < low || result > high)
            throw std::runtime_error(option + " is out of range");
        return result;
    };
    if (option == "--gyro-config")
        gyro.config = argument;
    else if (option == "--imu")
        gyro.imu = argument;
    else if (option == "--output-size") {
        const auto sep = value.find('x');
        if (sep == value.npos || value.find_first_not_of("0123456789x") != value.npos)
            throw std::runtime_error("--output-size requires WIDTHxHEIGHT");
        gyro.output_width = integer(value.substr(0, sep), option);
        gyro.output_height = integer(value.substr(sep + 1), option);
        if (gyro.output_width <= 0 || gyro.output_height <= 0)
            throw std::runtime_error("--output-size requires positive dimensions");
    } else if (option == "--gyro-smooth")
        gyro.smoothness = bounded(0, 1);
    else if (option == "--max-zoom") {
        gyro.max_zoom = number(value, option);
        if (gyro.max_zoom < 1 || (gyro.max_zoom > 10 && gyro.max_zoom < 100))
            throw std::runtime_error("--max-zoom must be at least 1x or 100 percent");
    } else if (option == "--zoom-window")
        gyro.zoom_window = bounded(0, std::numeric_limits<double>::max());
    else if (option == "--fov") {
        gyro.fov = number(value, option);
        if (gyro.fov <= 0)
            throw std::runtime_error("--fov must be positive");
    } else if (option == "--lens-correction")
        gyro.lens_correction = bounded(0, 1);
    else if (option == "--readout-ms")
        gyro.readout_ms = bounded(0, std::numeric_limits<double>::max());
    else if (option == "--gyro-offset")
        gyro.time_offset_ms = number(value, option);
    else if (option == "--imu-integration") {
        if (value == "auto")
            gyro.integration = SF_IMU_AUTO;
        else if (value == "gyro")
            gyro.integration = SF_IMU_GYRO;
        else if (value == "complementary")
            gyro.integration = SF_IMU_COMPLEMENTARY;
        else
            throw std::runtime_error("--imu-integration must be auto, gyro or complementary");
    } else
        return false;
    return true;
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
        auto o = sf_file_options_default();
        o.input = argv[1];
        o.output = argv[2];
        bool gyro_selected = false;
        bool backend_selected = false;
        for (int i = 3; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--no-audio") {
                o.audio = SF_AUDIO_DROP;
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
            if (gyro_option(o.gyro, option, argv[i])) {
                gyro_selected = true;
                continue;
            }
            if (option == "--mode") {
                if (value != "offline" && value != "realtime")
                    throw std::runtime_error("--mode must be offline or realtime");
                o.mode = value == "realtime" ? SF_FILE_STREAM : SF_FILE_OFFLINE;
            } else if (option == "--lookahead")
                o.stream.lookahead_seconds = number(value, option);
            else if (option == "--buffer-frames") {
                const int capacity = integer(value, option);
                if (capacity < 2 || capacity > 600)
                    throw std::runtime_error("--buffer-frames must be 2..600");
                o.stream.max_buffered_frames = static_cast<size_t>(capacity);
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
                o.decoder = value == "auto" ? SF_DECODER_AUTO : value == "cpu" ? SF_DECODER_CPU : SF_DECODER_NVDEC;
            } else if (option == "--audio") {
                if (value == "auto")
                    o.audio = SF_AUDIO_AUTO;
                else if (value == "copy")
                    o.audio = SF_AUDIO_COPY;
                else if (value == "aac")
                    o.audio = SF_AUDIO_AAC;
                else if (value == "none")
                    o.audio = SF_AUDIO_DROP;
                else
                    throw std::runtime_error("--audio must be auto, copy, aac or none");
            } else if (option == "--encoder") {
                if (value != "auto" && value != "openh264" && value != "nvenc")
                    throw std::runtime_error("--encoder must be auto, openh264 or nvenc");
                o.encoder = value == "auto"       ? SF_ENCODER_AUTO
                            : value == "openh264" ? SF_ENCODER_OPENH264
                                                  : SF_ENCODER_NVENC;
            } else if (option == "--backend") {
                if (value != "auto" && value != "cpu" && value != "cuda" && value != "gles")
                    throw std::runtime_error("Unknown rendering backend: " + value);
                const sf_backend backend = value == "auto"   ? SF_BACKEND_AUTO
                                           : value == "cpu"  ? SF_BACKEND_CPU
                                           : value == "gles" ? SF_BACKEND_GLES
                                                             : SF_BACKEND_CUDA;
                if (backend_selected && backend != o.backend)
                    throw std::runtime_error("Conflicting rendering backend selections");
                o.backend = backend;
                backend_selected = true;
            } else if (option == "--scan-axis") {
                if (value != "auto" && value != "x" && value != "y")
                    throw std::runtime_error("--scan-axis must be auto, x or y");
                o.scan_axis = value == "auto" ? SF_SCAN_AUTO : value == "x" ? SF_SCAN_X : SF_SCAN_Y;
            } else if (option == "--rs") {
                if (value != "on" && value != "off")
                    throw std::runtime_error("--rs must be on or off");
                o.stabilization.rolling_shutter = value == "on";
            } else
                throw std::runtime_error("Unknown option: " + option);
        }
        if (gyro_selected && !o.gyro.config)
            throw std::runtime_error("Gyro options require --gyro-config");
        sf_run_summary result{};
        sf_error error{};
        struct Display {
            bool terminal = isatty(STDERR_FILENO);
            int percent = -1, stage = -1;
        } display;
        sf_file_callbacks callbacks{
            {&display, nullptr,
             [](void* context, const char* text) {
                 if (static_cast<Display*>(context)->terminal)
                     std::cerr << "\r\033[K";
                 std::cerr << text << '\n';
             },
             nullptr},
            [](void* context, const sf_progress_event* event) {
                auto& state = *static_cast<Display*>(context);
                const int percent = static_cast<int>(event->fraction * 100);
                if (state.percent == percent && state.stage == event->stage && !event->stage_complete)
                    return;
                state.percent = percent;
                state.stage = event->stage;
                constexpr const char* stages[]{"analyze", "recover", "plan", "render", "finalize", "complete"};
                if (state.terminal)
                    std::cerr << "\r\033[K";
                std::cerr << '[' << std::string(percent / 5, '=') << std::string(20 - percent / 5, ' ') << "] "
                          << percent << "% " << stages[event->stage];
                if (event->processed_frames || event->total_frames) {
                    std::cerr << ' ' << event->processed_frames;
                    if (event->total_frames)
                        std::cerr << '/' << event->total_frames;
                }
                if (!state.terminal || event->stage == SF_PROGRESS_COMPLETE)
                    std::cerr << '\n';
                std::cerr.flush();
            }};
        if (sf_stabilize_file(&o, &callbacks, &result, &error) != SF_OK)
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
