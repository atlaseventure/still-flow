#include "gpmf_clock.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <string_view>

namespace stillflow {
namespace {
uint32_t be32(std::string_view bytes) {
    uint32_t value = 0;
    for (unsigned char byte : bytes)
        value = (value << 8) | byte;
    return value;
}
struct Record {
    std::string_view key, data;
    char type;
    size_t stride, count;
};
size_t payload_size(std::string_view header) {
    return static_cast<size_t>(static_cast<unsigned char>(header[5])) * be32(header.substr(6, 2));
}
template <class Visit> bool records(std::string_view bytes, Visit visit) {
    while (bytes.size() >= 8) {
        const auto header = bytes.substr(0, 8);
        bytes.remove_prefix(8);
        const size_t size = payload_size(header), padded = (size + 3) & ~size_t{3};
        if (bytes.size() < padded || !visit(Record{header.substr(0, 4), bytes.substr(0, size), header[4],
                                                   static_cast<unsigned char>(header[5]), be32(header.substr(6, 2))}))
            return false;
        bytes.remove_prefix(padded);
    }
    return bytes.empty();
}
struct Anchor {
    size_t index;
    double time;
};
} // namespace
std::optional<double> gpmf_sample_period(const std::filesystem::path& path, std::span<const ImuSample> samples) {
    std::ifstream input(path, std::ios::binary);
    std::vector<Anchor> anchors;
    size_t seen = 0;
    std::array<char, 8> header{};
    while (input.read(header.data(), header.size())) {
        const std::string_view h(header.data(), header.size());
        if (h.substr(0, 4) != "DEVC" || h[4] != 0)
            return {};
        const size_t size = payload_size(h);
        std::string block((size + 3) & ~size_t{3}, '\0');
        if (!input.read(block.data(), static_cast<std::streamsize>(block.size())))
            return {};
        const bool ok = records(std::string_view(block).substr(0, size), [&](Record stream) {
            if (stream.key != "STRM")
                return true;
            size_t count = 0;
            uint32_t tick = 0, total = 0;
            bool microsecond_clock = false;
            if (!records(stream.data, [&](Record r) {
                    if (r.key == "TICK" && r.type == 'L' && r.data.size() == 4)
                        tick = be32(r.data);
                    if (r.key == "TSMP" && r.type == 'L' && r.data.size() == 4)
                        total = be32(r.data);
                    if (r.key == "STMP")
                        microsecond_clock = true;
                    if (r.key == "GYRO" && r.type == 's' && r.stride == 6)
                        count = r.count;
                    return true;
                }))
                return false;
            if (!count)
                return true;
            if (!tick || microsecond_clock || total != seen + count || total > samples.size() ||
                (!anchors.empty() && tick * 0.001 <= anchors.back().time))
                return false;
            // Match the CSV to the packet interpolation before changing its clock.
            // The final packet has no following packet start. Its last CSV sample
            // still anchors the interpolation of all intermediate samples.
            const size_t end = total < samples.size() ? total : total - 1;
            if (end > seen + 1) {
                const double dt = (samples[end].time_s - samples[seen].time_s) / (end - seen);
                for (size_t i = 1; i < end - seen; ++i)
                    if (std::abs(samples[seen + i].time_s - (samples[seen].time_s + i * dt)) > 2e-6)
                        return false;
            }
            anchors.push_back({seen, tick * 0.001});
            seen = total;
            return true;
        });
        if (!ok)
            return {};
    }
    if (!input.eof() || input.gcount() || seen != samples.size() || anchors.size() < 3)
        return {};
    double mean_index = 0, mean_time = 0;
    for (const auto& a : anchors) {
        mean_index += a.index;
        mean_time += a.time;
    }
    mean_index /= anchors.size();
    mean_time /= anchors.size();
    double xx = 0, xy = 0;
    for (const auto& a : anchors) {
        xx += (a.index - mean_index) * (a.index - mean_index);
        xy += (a.index - mean_index) * (a.time - mean_time);
    }
    const double period = xy / xx;
    for (const auto& a : anchors)
        if (std::abs(a.time - mean_time - (a.index - mean_index) * period) > period)
            return {};
    return period;
}
} // namespace stillflow
