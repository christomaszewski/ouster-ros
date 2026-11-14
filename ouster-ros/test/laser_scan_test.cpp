#include <gtest/gtest.h>
#include <climits>
#include <cstring>
#include <type_traits>
#include "frozen_laser_scan.h"

using namespace ouster::sdk::core;
namespace {
std::shared_ptr<SensorInfo> scan_info(size_t width, size_t height) {
    auto info = std::make_shared<SensorInfo>(default_sensor_info(LidarMode::_2048x10));
    info->format.columns_per_frame = width;
    info->format.pixels_per_column = height;
    return info;
}
void fill(LidarScan& scan) {
    for (const auto& field : scan.field_types())
        impl::visit_field(scan, field.name, [&](auto values) {
            using Scalar = typename decltype(values)::Scalar;
            for (Eigen::Index i = 0; i < values.size(); ++i) {
                if constexpr (std::is_floating_point_v<Scalar>)
                    values.data()[i] = static_cast<Scalar>((i % 997) * 65431.0 + 0.75);
                else if constexpr (std::is_signed_v<Scalar>)
                    values.data()[i] = static_cast<Scalar>((i % 7 == 0 ? -1 : 1) * (i * 257 + 13));
                else
                    values.data()[i] = static_cast<Scalar>(uint64_t(i) * 4294967311ULL + 16777217ULL);
            }
        });
    for (const auto& name : {ChanField::RANGE, ChanField::RANGE2})
        if (scan.has_field(name)) {
            auto ranges = scan.field<uint32_t>(name);
            for (Eigen::Index i = 0; i < ranges.size(); ++i)
                ranges.data()[i] = i % 5 == 0 ? 0U : i % 7 == 0 ? UINT32_MAX : uint32_t(i * 1001 + 2);
        }
}
void equivalent(const LidarScan& scan, int shift, uint16_t ring, int ret,
                LidarMode mode = LidarMode::_2048x10) {
    std::vector<int> shifts(scan.h, shift);
    const rclcpp::Time stamp(123456789123LL);
    const auto expected = ouster_ros::frozen_lidar_scan_to_laser_scan_msg(
        scan, stamp, "laser_frame", mode, ring, shifts, ret);
    const auto actual = ouster_ros::lidar_scan_to_laser_scan_msg(
        scan, stamp, "laser_frame", mode, ring, shifts, ret);
    ASSERT_EQ(actual, expected);
    ASSERT_EQ(actual.ranges.size(), scan.w);
    ASSERT_EQ(actual.intensities.size(), scan.w);
    if (scan.w) {
        EXPECT_EQ(std::memcmp(actual.ranges.data(), expected.ranges.data(), scan.w * sizeof(float)), 0);
        EXPECT_EQ(std::memcmp(actual.intensities.data(), expected.intensities.data(), scan.w * sizeof(float)), 0);
    }
}
std::vector<int> shifts(size_t width) {
    const int w = static_cast<int>(width);
    return {0, 1, -1, w - 1, w, -w, 3 * w, -3 * w,
            w + w / 2 + 1, w + w / 2 + w - 1, INT_MIN, INT_MAX};
}
}

TEST(LaserScanRow, ProfilesReturnsAndRingsMatchFrozenBytes) {
    for (const auto profile : {UDPProfileLidar::LEGACY,
             UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
             UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
             UDPProfileLidar::RNG15_RFL8_NIR8,
             UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL,
             UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16,
             UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL}) {
        LidarScan scan(512, 32, profile); fill(scan);
        for (int ret : {0, 1}) {
            if (ret && !scan.has_field(ChanField::RANGE2)) continue;
            for (uint16_t ring : {0, 15, 31})
                for (int shift : shifts(scan.w)) {
                    SCOPED_TRACE(::testing::Message() << int(profile) << '/' << ret << '/' << ring << '/' << shift);
                    equivalent(scan, shift, ring, ret, LidarMode::_512x20);
                }
        }
    }
}
TEST(LaserScanRow, Full128By2048MatchesFrozenBytes) {
    LidarScan scan(2048, 128, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL); fill(scan);
    for (int ret : {0, 1})
        for (uint16_t ring : {0, 63, 127})
            for (int shift : shifts(scan.w)) equivalent(scan, shift, ring, ret);
}
TEST(LaserScanRow, AllSignalStorageTypesAndUnsignedShiftWrapMatch) {
    for (auto type : {ChanFieldType::UINT8, ChanFieldType::UINT16, ChanFieldType::UINT32,
                     ChanFieldType::UINT64, ChanFieldType::INT8, ChanFieldType::INT16,
                     ChanFieldType::INT32, ChanFieldType::INT64, ChanFieldType::FLOAT32,
                     ChanFieldType::FLOAT64})
        for (size_t width : {7U, 512U}) {
            LidarScan scan(scan_info(width, 8), std::vector<FieldType>{
                {ChanField::RANGE, ChanFieldType::UINT32}, {ChanField::RANGE2, ChanFieldType::UINT32},
                {ChanField::SIGNAL, type}, {ChanField::SIGNAL2, type}});
            fill(scan);
            for (int ret : {0, 1, 3})
                for (uint16_t ring : {0, 7})
                    for (int shift : shifts(width)) {
                        SCOPED_TRACE(::testing::Message() << int(type) << '/' << width << '/' << ret << '/' << ring << '/' << shift);
                        equivalent(scan, shift, ring, ret);
                    }
        }
}
TEST(LaserScanRow, MissingSignalRemainsZeroForBothReturns) {
    for (size_t width : {7U, 2048U}) {
        LidarScan scan(scan_info(width, 128), std::vector<FieldType>{
            {ChanField::RANGE, ChanFieldType::UINT32}, {ChanField::RANGE2, ChanFieldType::UINT32}});
        fill(scan);
        for (int ret : {0, 1})
            for (int shift : shifts(width)) equivalent(scan, shift, 127, ret);
    }
}
TEST(LaserScanRow, MissingSelectedRangeStillThrows) {
    LidarScan scan(scan_info(512, 8), std::vector<FieldType>{{ChanField::RANGE, ChanFieldType::UINT32}});
    std::vector<int> shifts(8, 0);
    EXPECT_THROW(ouster_ros::frozen_lidar_scan_to_laser_scan_msg(scan, rclcpp::Time(0), "frame", LidarMode::_512x10, 0, shifts, 1), std::exception);
    EXPECT_THROW(ouster_ros::lidar_scan_to_laser_scan_msg(scan, rclcpp::Time(0), "frame", LidarMode::_512x10, 0, shifts, 1), std::exception);
}
