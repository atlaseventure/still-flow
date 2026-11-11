#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <string>
TEST_CASE("Synthetic video retains frames and audio and reduces reference-relative jitter", "[video][global]") {
    std::ifstream metrics(STILLFLOW_VIDEO_METRICS);
    int frames = 0, audio = 0, reported = 0, lgpl = 0;
    std::string encoder;
    double before = 0, after = 0;
    REQUIRE(bool(metrics >> frames >> audio >> before >> after >> reported >> lgpl >> encoder));
    REQUIRE(lgpl == 1);
    REQUIRE(encoder == "libopenh264");
    REQUIRE(frames == 60);
    REQUIRE(reported == frames);
    REQUIRE(audio == 1);
    REQUIRE(before > 0.5);
    REQUIRE(after < before * 0.65);
    REQUIRE(after < 1.5);
}

namespace {
void check_startup_deformation(const std::string& mode) {
    std::ifstream metrics(std::string(STILLFLOW_RS_VIDEO_METRICS) + "/" + mode + ".txt");
    int frames = 0;
    double input = 0, first = 0, first_ten = 0, first_thirty = 0, middle = 0;
    REQUIRE(bool(metrics >> frames >> input >> first >> first_ten >> first_thirty >> middle));
    CAPTURE(mode, input, first, first_ten, first_thirty, middle);
    REQUIRE(frames == 90);
    // All errors are reference-relative local deformation in 1280x720 pixels,
    // after removing global translation, rotation and scale. Check the first
    // frame and short windows so a bad opening cannot hide in the full clip mean.
    REQUIRE(input > 1);
    REQUIRE(first < 0.5);
    REQUIRE(first_ten < 0.25);
    REQUIRE(first_thirty < 0.22);
    REQUIRE(middle < 0.22);
    REQUIRE(first_ten < input * 0.25);
}
} // namespace

TEST_CASE("Offline rolling shutter correction works from the first frame", "[video][rolling-shutter-offline]") {
    check_startup_deformation("offline");
}
TEST_CASE("Realtime rolling shutter correction works from the first frame", "[video][rolling-shutter-realtime]") {
    check_startup_deformation("realtime");
}
