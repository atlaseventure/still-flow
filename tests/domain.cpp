#include "fixture.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fstream>

TEST_CASE("Trajectory and crop plans match the validated golden sample", "[domain][golden]") {
    std::ifstream input(STILLFLOW_GOLDEN_FILE);
    REQUIRE(input.good());
    const auto actual = golden_values();
    size_t index = 0;
    for (double expected; input >> expected; ++index) {
        REQUIRE(index < actual.size());
        CAPTURE(index);
        REQUIRE(actual[index] == Catch::Approx(expected).margin(1e-8).epsilon(1e-8));
    }
    REQUIRE(index == actual.size());
}
TEST_CASE("Identity and translation invert with the documented crop", "[domain]") {
    stillflow::Warp warp;
    warp.columns = warp.rows = 2;
    warp.displacement.assign(4, {3, -2});
    auto map = stillflow::prepare_map(warp, {100, 100}, {100, 100}, 0.1, false);
    auto source = stillflow::source_point(map, {50, 50});
    REQUIRE(source.x == Catch::Approx(47));
    REQUIRE(source.y == Catch::Approx(52));
    REQUIRE(map.origin_x == Catch::Approx(9.9));
    REQUIRE(map.step_x == Catch::Approx(0.8));
    warp.displacement.clear();
    REQUIRE_THROWS_AS(stillflow::prepare_map(warp, {100, 100}, {100, 100}, 0, false), std::invalid_argument);
}
