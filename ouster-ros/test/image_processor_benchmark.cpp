// Benchmarks actual current/frozen image processors; no middleware publication.
#include "../src/image_processor.h"
#include "frozen_image_processor.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <ctime>

namespace core = ouster::sdk::core;
using Clock = std::chrono::steady_clock;

template <typename T>
std::vector<T> read(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream) throw std::runtime_error("Cannot read " + path.string());
    const auto size = stream.tellg();
    if (size % sizeof(T)) throw std::runtime_error("Invalid fixture size");
    std::vector<T> result(static_cast<size_t>(size) / sizeof(T));
    stream.seekg(0); stream.read(reinterpret_cast<char*>(result.data()), size);
    if (!stream) throw std::runtime_error("Truncated fixture");
    return result;
}

int main(int argc, char** argv) try {
    if (argc < 2) throw std::runtime_error("usage: benchmark fixture-directory [frames=120] [rounds=6] [all|signal|nearir|range|rgb]");
    const size_t frames = argc > 2 ? std::stoul(argv[2]) : 120;
    const size_t rounds = argc > 3 ? std::stoul(argv[3]) : 6;
    const std::string wanted = argc > 4 ? argv[4] : "all";
    auto info = core::default_sensor_info(core::LidarMode::_2048x10);
    info.format.pixels_per_column = 128;
    info.format.columns_per_frame = 2048;
    info.format.udp_profile_lidar = wanted == "rgb" ?
        core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 : core::UDPProfileLidar::LEGACY;
    std::vector<core::LidarScan> scans;
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(argv[1]))
        if (entry.is_directory()) paths.push_back(entry.path());
    std::sort(paths.begin(), paths.end());
    for (const auto& path : paths) {
        const std::filesystem::directory_entry entry(path);
        auto ranges = read<uint32_t>(entry.path() / "range.u32");
        auto signal = read<uint32_t>(entry.path() / "signal.u32");
        auto reflec = read<uint16_t>(entry.path() / "reflectivity.u16");
        auto nearir = read<uint16_t>(entry.path() / "near_ir.u16");
        auto shifts = read<int32_t>(entry.path() / "shifts.i32");
        if (ranges.size() != 128 * 2048 || signal.size() != ranges.size() ||
            reflec.size() != ranges.size() || nearir.size() != ranges.size() || shifts.size() != 128)
            throw std::runtime_error("Unexpected fixture dimensions");
        info.format.pixel_shift_by_row.assign(shifts.begin(), shifts.end());
        scans.emplace_back(2048, 128, info.format.udp_profile_lidar);
        auto& scan = scans.back();
        std::copy(ranges.begin(), ranges.end(), scan.field<uint32_t>(core::ChanField::RANGE).data());
        std::copy(signal.begin(), signal.end(), scan.field<uint16_t>(core::ChanField::SIGNAL).data());
        std::copy(reflec.begin(), reflec.end(), scan.field<uint8_t>(core::ChanField::REFLECTIVITY).data());
        std::copy(nearir.begin(), nearir.end(), scan.field<uint16_t>(core::ChanField::NEAR_IR).data());
        if (wanted == "rgb") {
            // Synthetic tone-mapped RGB only; the captured sensor is LEGACY.
            for (const auto& channel : {core::ChanField::R_U8, core::ChanField::G_U8, core::ChanField::B_U8})
                scan.add_field(core::FieldType(channel, core::ChanFieldType::UINT8));
            std::copy(signal.begin(), signal.end(), scan.field<uint8_t>(core::ChanField::R_U8).data());
            std::copy(reflec.begin(), reflec.end(), scan.field<uint8_t>(core::ChanField::G_U8).data());
            std::copy(nearir.begin(), nearir.end(), scan.field<uint8_t>(core::ChanField::B_U8).data());
        }
    }
    if (scans.empty() || !frames || !rounds) throw std::runtime_error("Empty workload");
    auto demand = [&](const std::string& channel) {
        return wanted == "all" || (wanted == "signal" && channel == core::ChanField::SIGNAL) ||
            (wanted == "nearir" && channel == core::ChanField::NEAR_IR) ||
            (wanted == "range" && channel == core::ChanField::RANGE) ||
            (wanted == "rgb" && channel == core::ChanField::RGB);
    };
    ouster_ros::ImageProcessor::OutputType old_output, new_output;
    auto old = ouster_ros::FrozenImageProcessor::create(info, "frame", "", [&](auto output) { old_output = std::move(output); }, demand);
    auto current = ouster_ros::ImageProcessor::create(info, "frame", "", [&](auto output) { new_output = std::move(output); }, demand);
    // Cross exposure refresh boundaries while checking every output byte.
    for (size_t i = 0; i < 36; ++i) {
        old(scans[i % scans.size()], 0, rclcpp::Time(i));
        current(scans[i % scans.size()], 0, rclcpp::Time(i));
        if (old_output.size() != new_output.size()) throw std::runtime_error("Output keys differ");
        for (const auto& item : old_output)
            if (*item.second != *new_output.at(item.first)) throw std::runtime_error("Image differs: " + item.first);
    }
    std::cout << "round,order,processor,demand,frames,wall_ms,cpu_ms\n" << std::setprecision(12);
    for (size_t round = 0; round < rounds; ++round)
        for (size_t order = 0; order < 2; ++order) {
            const bool frozen = (round + order) % 2 == 0;
            auto& processor = frozen ? old : current;
            const auto start = Clock::now();
            const auto cpu_start = std::clock();
            for (size_t i = 0; i < frames; ++i) processor(scans[i % scans.size()], 0, rclcpp::Time(i));
            const auto ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
            const auto cpu_ms = 1000.0 * (std::clock() - cpu_start) / CLOCKS_PER_SEC;
            std::cout << round << ',' << order << ',' << (frozen ? "frozen" : "candidate") << ',' << wanted << ',' << frames << ',' << ms << ',' << cpu_ms << '\n';
        }
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
}
