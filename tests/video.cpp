#include <catch2/catch_test_macros.hpp>
#include <fstream>
TEST_CASE("Synthetic video retains frames and audio and reduces reference-relative jitter", "[video]") {
    std::ifstream metrics(STILLFLOW_VIDEO_METRICS);
    int frames = 0, audio = 0, reported = 0;
    double before = 0, after = 0;
    REQUIRE(bool(metrics >> frames >> audio >> before >> after >> reported));
    REQUIRE(frames == 60);
    REQUIRE(reported == frames);
    REQUIRE(audio == 1);
    REQUIRE(before > 0.5);
    REQUIRE(after < before * 0.65);
    REQUIRE(after < 1.5);
}
