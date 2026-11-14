#pragma once
#include "cloud_common.hpp"
#include <memory>
#include <ouster/lidar_scan.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace cloud_bench {
namespace core = ouster::sdk::core;
struct Input {
    core::LidarScan scan;
    uint64_t scan_ts;
    uint32_t minimum, maximum;
    explicit Input(const cloud_test::Frame& f)
        : scan(f.width, f.height, core::UDPProfileLidar::LEGACY), scan_ts(f.scan_ts),
          minimum(f.min_range), maximum(f.max_range) {
        std::copy(f.range.begin(), f.range.end(), scan.field<uint32_t>(core::ChanField::RANGE).data());
        auto signal = scan.field<uint16_t>(core::ChanField::SIGNAL);
        auto nir = scan.field<uint16_t>(core::ChanField::NEAR_IR);
        auto reflectivity = scan.field<uint8_t>(core::ChanField::REFLECTIVITY);
        for (size_t i = 0; i < f.range.size(); ++i) {
            signal.data()[i] = f.signal[i]; nir.data()[i] = f.near_ir[i];
            reflectivity.data()[i] = f.reflectivity[i];
        }
        std::copy(f.timestamps.begin(), f.timestamps.end(), scan.timestamp().data());
    }
};
struct Calibration {
    core::ArrayX3fR direction, offset;
    std::vector<int> shifts;
    uint32_t width, height;
    explicit Calibration(const cloud_test::Frame& f)
        : direction(f.range.size(), 3), offset(f.range.size(), 3),
          shifts(f.shifts.begin(), f.shifts.end()), width(f.width), height(f.height) {
        std::copy(f.direction.begin(), f.direction.end(), direction.data());
        std::copy(f.offset.begin(), f.offset.end(), offset.data());
    }
};
struct Variant {
    sensor_msgs::msg::PointCloud2 output;
    virtual ~Variant() = default;
    virtual void run(const Input&) = 0;
};
std::unique_ptr<Variant> make_baseline(const Calibration&, cloud_test::Roi);
std::unique_ptr<Variant> make_candidate(const Calibration&, cloud_test::Roi, int);
}
