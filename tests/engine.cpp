#include "stillflow/application/engine.hpp"
#include "stillflow/backends/cpu.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {
struct Token : stillflow::CancellationToken {
    bool stop = false;
    bool cancelled() const noexcept override { return stop; }
};
struct Executor : stillflow::Executor {
    size_t calls = 0;
    void for_each(size_t count, const std::function<void(size_t)>& work) override {
        ++calls;
        for (size_t i = 0; i < count; ++i)
            work(i);
    }
};
struct Logger : stillflow::Logger {
    size_t calls = 0;
    void log(std::string_view) override { ++calls; }
};
} // namespace
TEST_CASE("Padded YUV planes retain ownership and expose zero-copy luma", "[image]") {
    for (auto format : {stillflow::PixelFormat::nv12, stillflow::PixelFormat::nv21, stillflow::PixelFormat::i420}) {
        auto owner = std::make_shared<std::vector<uint8_t>>(size_t{64} * 32, 127);
        stillflow::HostImageView view{{33, 33}, format, {{40, {owner->data(), size_t{40} * 33}}}, owner, {}};
        if (format == stillflow::PixelFormat::i420) {
            view.planes.push_back({20, {owner->data(), size_t{20} * 17}});
            view.planes.push_back({20, {owner->data(), size_t{20} * 17}});
        } else
            view.planes.push_back({36, {owner->data(), size_t{36} * 17}});
        auto image = stillflow::borrow_host_image(view);
        auto luma = stillflow::gray_proxy(image, {33, 33});
        REQUIRE(stillflow::map_host_image(luma).planes[0].bytes.data() == owner->data());
        owner.reset();
        image.reset();
        REQUIRE(stillflow::map_host_image(luma).planes[0].bytes[0] == 127);
        view.planes.back().row_stride = 1;
        REQUIRE_THROWS_AS(stillflow::borrow_host_image(view), std::invalid_argument);
    }
}
TEST_CASE("In-memory analysis plans and renders translated frames through host services", "[engine]") {
    stillflow::StabilizationSettings settings;
    settings.bands = 4;
    Token token;
    Executor executor;
    Logger logger;
    stillflow::ExecutionSettings execution{&token, &executor, &logger};
    stillflow::Analyzer analyzer(settings, {64, 64}, 30, false, stillflow::make_translation_analyzer(settings),
                                 execution);
    stillflow::Profile profile;
    std::vector<uint8_t> pixels(size_t{64} * 64), shifted(size_t{64} * 64);
    uint32_t state = 42;
    for (auto& value : pixels) {
        state = state * 1664525 + 1013904223;
        value = state >> 24;
    }
    for (int y = 0; y < 64; ++y)
        for (int x = 1; x < 64; ++x)
            shifted[static_cast<size_t>(y) * 64 + x] = pixels[static_cast<size_t>(y) * 64 + x - 1];
    auto first = stillflow::make_host_image({64, 64}, stillflow::PixelFormat::gray8, pixels);
    auto second = stillflow::make_host_image({64, 64}, stillflow::PixelFormat::gray8, shifted);
    analyzer.push(first, 100000, 33333, profile);
    analyzer.push(second, 133333, 33333, profile);
    REQUIRE_THROWS_AS(analyzer.push(first, 133333, 33333, profile), std::invalid_argument);
    auto plan = analyzer.finish();
    REQUIRE(plan.analysis.frames[1].motion.step(0, 2) == Catch::Approx(1));
    REQUIRE(plan.analysis.origin_us == 100000);
    REQUIRE(logger.calls == 1);
    REQUIRE(executor.calls == 1);
    REQUIRE_THROWS_AS(analyzer.finish(), std::logic_error);
    auto factory = stillflow::make_renderer_factory({{"cpu", [&] { return stillflow::make_cpu_backend(execution); }}});
    auto renderer = factory->create("cpu");
    stillflow::RenderSession session(plan, *renderer, execution);
    auto output = session.render(0, first, profile);
    REQUIRE(output->size().width == 64);
    REQUIRE(executor.calls > 1);
    token.stop = true;
    REQUIRE_THROWS_AS(session.render(1, second, profile), stillflow::Cancelled);
    REQUIRE_THROWS_AS(stillflow::Planner::plan(plan.analysis, settings, execution), stillflow::Cancelled);
}
TEST_CASE("Timestamp gaps begin a new segment", "[engine]") {
    stillflow::StabilizationSettings settings;
    stillflow::Analyzer analyzer(settings, {64, 64}, 30, false, nullptr);
    analyzer.push_observation({}, 0, 33333);
    analyzer.push_observation({}, 1000000, 33333);
    auto result = analyzer.finish_analysis();
    REQUIRE(result.frames[1].segment == 1);
    REQUIRE(result.frames[1].motion.state == stillflow::MotionState::time_gap);
    REQUIRE(result.frames[0].duration == 1);
}
