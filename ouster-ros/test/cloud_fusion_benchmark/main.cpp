#include "benchmark.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
using namespace cloud_bench;
static volatile uint32_t sink;
std::vector<cloud_test::Point48> bytes(const Variant& v) {
    if (v.output.point_step != 48 || v.output.data.size() % 48) throw std::runtime_error("Wrong point stride");
    std::vector<cloud_test::Point48> result(v.output.data.size() / 48);
    std::memcpy(result.data(), v.output.data.data(), v.output.data.size()); return result;
}
int main(int argc, char** argv) try {
    if (argc < 2 || argc > 6) throw std::runtime_error("usage: cloud_fusion_benchmark fixtures [iterations=64] [rounds=9] [origin=0] [width=full]");
    const auto frames = cloud_test::read_frames(argv[1]); const auto& f = frames.front();
    const unsigned iterations = argc > 2 ? std::stoul(argv[2]) : 64;
    const unsigned rounds = argc > 3 ? std::stoul(argv[3]) : 9;
    cloud_test::Roi roi{argc > 4 ? uint32_t(std::stoul(argv[4])) : 0, argc > 5 ? uint32_t(std::stoul(argv[5])) : f.width};
    if (!iterations || !rounds || !roi.width || roi.width > f.width || roi.start >= f.width) throw std::runtime_error("Invalid arguments");
    Calibration calibration(f); std::vector<Input> inputs; inputs.reserve(frames.size());
    for (const auto& frame : frames) {
        if (frame.width != f.width || frame.height != f.height || frame.direction != f.direction || frame.offset != f.offset || frame.shifts != f.shifts)
            throw std::runtime_error("Fixture calibration differs");
        inputs.emplace_back(frame);
    }
    const std::vector<std::string> names{"production_3bf8744", "fused", "tile32", "tile64", "tile128", "neon32", "neon64", "neon128"};
    std::vector<std::unique_ptr<Variant>> variants; variants.push_back(make_baseline(calibration, roi));
    for (int i = 0; i < 7; ++i) variants.push_back(make_candidate(calibration, roi, i));
    for (size_t frame = 0; frame < frames.size(); ++frame) {
        for (size_t v = 0; v < variants.size(); ++v) {
            variants[v]->run(inputs[frame]);
            cloud_test::compare(cloud_test::crop_reference(frames[frame], roi), bytes(*variants[v]), names[v]);
            if (variants[v]->output != variants.front()->output) throw std::runtime_error(names[v] + ": metadata/data differ from actual production headers");
        }
    }
    std::cerr << "Exact fixture/header validation PASS: " << frames.size() << " frames x " << variants.size() << " variants\n";
    for (auto& v : variants) for (unsigned i = 0; i < 16; ++i) v->run(inputs[i % inputs.size()]);
    std::cout << "crop_start,crop_width,round,order,implementation,iterations,wall_ms,cpu_ms,checksum\n" << std::setprecision(12);
    for (unsigned round = 0; round < rounds; ++round) for (size_t order = 0; order < variants.size(); ++order) {
        const size_t index = (round + (round % 2 ? variants.size() - 1 - order : order)) % variants.size();
        auto& v = *variants[index]; const auto wall = std::chrono::steady_clock::now(); const auto cpu = std::clock();
        for (unsigned i = 0; i < iterations; ++i) { v.run(inputs[i % inputs.size()]); sink = v.output.width; }
        const double cpu_ms = 1000.0 * (std::clock() - cpu) / CLOCKS_PER_SEC;
        const double wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall).count();
        auto result = bytes(v); cloud_test::compare(cloud_test::crop_reference(frames[(iterations - 1) % inputs.size()], roi), result, names[index]);
        std::cout << roi.start << ',' << roi.width << ',' << round << ',' << order << ',' << names[index] << ',' << iterations << ',' << wall_ms << ',' << cpu_ms << ',' << cloud_test::checksum(result) << '\n';
    }
    return 0;
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
