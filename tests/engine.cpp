#include "stillflow/application/engine.hpp"
#include "stillflow/application/stream.hpp"
#include "stillflow/backends/cpu.hpp"
#include "stillflow/vision/portable.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>

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

TEST_CASE("Streaming anticipates a growing shake envelope without leaking high-frequency motion", "[stream]") {
    constexpr stillflow::Size size{320, 180};
    stillflow::StabilizationSettings settings;
    settings.rolling_shutter = false;
    std::vector<stillflow::FrameMotion> frames(450);
    std::vector<double> observed(frames.size());
    for (size_t i = 0; i < frames.size(); ++i) {
        const double t = i / 30.0;
        const double amplitude = 1 + 8 * std::clamp((t - 5) / 0.5, 0.0, 1.0);
        observed[i] = 3 * t + amplitude * std::sin(8 * std::numbers::pi * t);
        auto& f = frames[i];
        f.seconds = t;
        f.duration = 1 / 30.0;
        f.motion.confidence = 1;
        f.motion.step(0, 2) = i ? observed[i] - observed[i - 1] : 0;
    }
    const auto offline = stillflow::plan_warps(frames, size, settings, false);
    stillflow::StreamAnalyzer stream(settings, {}, size, 30, false);
    std::vector<stillflow::Warp> streaming;
    const auto drain = [&] {
        stillflow::ReadyWarp ready;
        while (stream.take_ready(ready)) {
            REQUIRE(ready.index == streaming.size());
            REQUIRE(ready.pts_us == std::llround(frames[ready.index].seconds * 1000000));
            REQUIRE(ready.warp.window.left >= 0);
            REQUIRE(ready.warp.window.left <= settings.crop);
            streaming.push_back(ready.warp);
        }
    };
    for (const auto& f : frames) {
        REQUIRE(stream.push_observation(f.motion, std::llround(f.seconds * 1000000), 33333));
        drain();
    }
    stream.finish();
    drain();
    REQUIRE(streaming.size() == frames.size());
    const auto energy = [&](const std::vector<stillflow::Warp>& warps) {
        std::vector<double> path = observed;
        for (size_t i = 0; i < path.size() && !warps.empty(); ++i)
            path[i] += warps[i].global_offset.x * warps[i].strength;
        double sum = 0;
        for (size_t i = 151; i < 390; ++i) {
            const double acceleration = path[i + 1] - 2 * path[i] + path[i - 1];
            sum += acceleration * acceleration;
        }
        return std::sqrt(sum);
    };
    const double input = energy({});
    REQUIRE(1 - energy(streaming) / input >= 0.85 * (1 - energy(offline) / input));
    for (size_t i = 2; i < streaming.size(); ++i) {
        const auto time = [&](size_t j) { return std::llround(frames[j].seconds * 1000000) / 1000000.0; };
        const double dt = time(i) - time(i - 1), previous_dt = time(i - 1) - time(i - 2);
        const double speed = (streaming[i].window.scale - streaming[i - 1].window.scale) / dt;
        const double previous = (streaming[i - 1].window.scale - streaming[i - 2].window.scale) / previous_dt;
        REQUIRE(std::abs(speed) <= 0.04001);
        REQUIRE(2 * std::abs(speed - previous) / (dt + previous_dt) <= 0.08001);
    }
}
