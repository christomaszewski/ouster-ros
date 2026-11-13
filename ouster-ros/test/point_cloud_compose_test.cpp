#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>

#include <array>
#include <cmath>
#include <limits>
#include <tuple>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster_ros/os_point.h"
#include "../src/point_meta_helpers.h"
#include "../src/point_cloud_compose.h"
#include "../src/point_cloud_processor.h"

class PointCloudComposeTest : public ::testing::Test {
   protected:
    void SetUp() override {}

    void TearDown() override {}
};

using namespace std;
using namespace ouster_ros;
using namespace ouster::sdk::core;

// TODO: generalize the test case!

TEST_F(PointCloudComposeTest, MapLidarScanFields) {
    const auto WIDTH = 5U;
    const auto HEIGHT = 3U;
    const auto SAMPLES = WIDTH * HEIGHT;
    UDPProfileLidar lidar_udp_profile =
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;

    LidarScan ls(WIDTH, HEIGHT, lidar_udp_profile);

    auto fill_data = [](auto& img, auto base, auto count) {
        auto* p = img.data();
        for (auto i = 0U; i < count; ++i)
            p[i] =
                static_cast<std::remove_reference_t<decltype(p[0])>>(base + i);
    };

    auto range = ls.field<uint32_t>(ChanField::RANGE);
    auto signal = ls.field<uint16_t>(ChanField::SIGNAL);
    auto reflect = ls.field<uint8_t>(ChanField::REFLECTIVITY);
    auto near_ir = ls.field<uint16_t>(ChanField::NEAR_IR);

    // choose a base value that could ultimately wrap around
    fill_data(range, static_cast<uint32_t>(1 + (1ULL << 32) - SAMPLES / 2),
              SAMPLES);
    fill_data(signal, static_cast<uint16_t>(3 + (1 << 16) - SAMPLES / 2),
              SAMPLES);
    fill_data(reflect, static_cast<uint8_t>(5 + (1 << 8) - SAMPLES / 2),
              SAMPLES);
    fill_data(near_ir, static_cast<uint16_t>(7 + (1 << 16) - SAMPLES / 2),
              SAMPLES);

    ouster_ros::Cloud<Point_RNG19_RFL8_SIG16_NIR16_DUAL> cloud{WIDTH, HEIGHT};

    auto ls_tuple =
        make_lidar_scan_tuple<0, Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
                              Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(ls);

    ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_DUAL pt;

    for (auto src_idx = 0U; src_idx < SAMPLES; ++src_idx) {
        copy_lidar_scan_fields_to_point<0>(pt, ls_tuple, src_idx);
        EXPECT_EQ(point::get<5>(pt), range(0, src_idx));
        EXPECT_EQ(point::get<6>(pt), signal(0, src_idx));
        EXPECT_EQ(point::get<7>(pt), reflect(0, src_idx));
        EXPECT_EQ(point::get<8>(pt), near_ir(0, src_idx));
    }
}

TEST_F(PointCloudComposeTest, ScanToCloudDestaggerHandlesNegativePixelShift) {
    const auto WIDTH = 8;
    const auto HEIGHT = 6;
    const auto SAMPLES = WIDTH * HEIGHT;

    LidarScan ls(WIDTH, HEIGHT, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    auto range = ls.field<uint32_t>(ChanField::RANGE);
    for (auto i = 0; i < SAMPLES; ++i) range.data()[i] = i + 1;

    PointCloudXYZf points = PointCloudXYZf::Zero(SAMPLES, 3);

    // mix of positive, zero, negative and boundary shifts within [-w, w)
    const std::vector<int> pixel_shift_by_row{3, 0, -3, WIDTH - 1, -WIDTH, -1};

    ouster_ros::Cloud<Point_RNG19_RFL8_SIG16_NIR16> cloud{WIDTH, HEIGHT};
    Point_RNG19_RFL8_SIG16_NIR16 staging_pt;
    scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16.size(),
                    Profile_RNG19_RFL8_SIG16_NIR16>(
        cloud, staging_pt, points, 0, ls, pixel_shift_by_row, true, true);

    for (auto u = 0; u < HEIGHT; ++u) {
        const auto s = ((pixel_shift_by_row[u] % WIDTH) + WIDTH) % WIDTH;
        for (auto v = 0; v < WIDTH; ++v) {
            const auto expected_v = (v + WIDTH - s) % WIDTH;
            EXPECT_EQ(cloud.points[u * WIDTH + v].range,
                      range(u, expected_v))
                << "u=" << u << " v=" << v;
        }
    }
}

namespace {
PointCloudProcessor<Point_RNG19_RFL8_SIG16_NIR16> make_processor(
    const SensorInfo& info) {
    return PointCloudProcessor<Point_RNG19_RFL8_SIG16_NIR16>(
        info, "os_lidar", false, 0, 1000000, 1, "",
        [](auto&, const auto&, auto, const auto&, const auto&, auto) {},
        [](auto) {});
}
}  // namespace

TEST_F(PointCloudComposeTest, PointCloudProcessorAcceptsPixelShiftInRange) {
    auto info = default_sensor_info(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);
    shifts.front() = -w;
    shifts.back() = w - 1;
    EXPECT_NO_THROW(make_processor(info));
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftOutOfRange) {
    auto info = default_sensor_info(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);

    shifts[1] = w;
    EXPECT_THROW(make_processor(info), std::invalid_argument);

    shifts[1] = -w - 1;
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftSizeMismatch) {
    auto info = default_sensor_info(LidarMode::_512x10);
    info.format.pixel_shift_by_row.pop_back();
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}

namespace {
using OriginalCloudCase = std::tuple<int, int, bool, bool>;

std::vector<OriginalCloudCase> original_cloud_cases() {
    std::vector<OriginalCloudCase> cases;
    // Cover the deployed 128-beam sizes and a width that cannot use a bit mask
    // in place of modulo, with every supported power-of-two row reduction.
    for (int width : {7, 1024, 2048})
        for (int rows_step : {1, 2, 4, 8, 16, 32, 64, 128})
            for (bool organized : {false, true})
                for (bool destagger : {false, true})
                    cases.emplace_back(width, rows_step, organized, destagger);
    return cases;
}
}  // namespace

class OriginalCloudEquivalenceTest
    : public ::testing::TestWithParam<OriginalCloudCase> {};

TEST_P(OriginalCloudEquivalenceTest, PreservesFieldsOrderingAndSchema) {
    const auto [width, rows_step, organized, destagger] = GetParam();
    constexpr int height = 128;
    constexpr uint64_t scan_ts = 1000000000000ULL;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    LidarScan scan(width, height, UDPProfileLidar::LEGACY);
    auto range = scan.field<uint32_t>(ChanField::RANGE);
    auto signal = scan.field<uint16_t>(ChanField::SIGNAL);
    auto reflectivity = scan.field<uint8_t>(ChanField::REFLECTIVITY);
    auto near_ir = scan.field<uint16_t>(ChanField::NEAR_IR);
    scan.field<uint8_t>(ChanField::FLAGS).setZero();
    auto timestamps = scan.timestamp();
    PointCloudXYZf xyz(width * height, 3);
    for (int i = 0; i < width * height; ++i) {
        range.data()[i] = static_cast<uint32_t>(100000 + i);
        signal.data()[i] = static_cast<uint16_t>(65535 - i);
        reflectivity.data()[i] = static_cast<uint8_t>(3 * i + 7);
        near_ir.data()[i] = static_cast<uint16_t>(5 * i + 11);
        xyz(i, 0) = i % 17 == 0 ? nan : static_cast<float>(i);
        xyz(i, 1) = i % 19 == 0 ? nan : -static_cast<float>(i);
        xyz(i, 2) = i % 23 == 0 ? nan : 0.25f * i;
    }
    for (int v = 0; v < width; ++v) timestamps[v] = scan_ts + 1000 * v;
    timestamps[0] = scan_ts - 1;  // Timestamp clamping at the scan boundary.
    timestamps[1] = scan_ts;
    timestamps[2] = scan_ts + uint64_t{UINT32_MAX} + 123;

    const std::array<int, 6> shift_values{
        {-width, 0, width - 1, -1, width / 2, -width + 1}};
    std::vector<int> shifts(height);
    for (int u = 0; u < height; ++u) shifts[u] = shift_values[u % 6];

    Cloud<Point> cloud{static_cast<uint32_t>(width),
                       static_cast<uint32_t>(height / rows_step)};
    Point_LEGACY staging_point;
    scan_to_cloud_f<Profile_LEGACY.size(), Profile_LEGACY>(
        cloud, staging_point, xyz, scan_ts, scan, shifts, organized, destagger,
        rows_step);

    size_t expected_size = 0;
    bool expected_dense = true;
    for (int u = 0; u < height; u += rows_step) {
        for (int v = 0; v < width; ++v) {
            // Independent oracle: the pre-backport modulo expression. Compare
            // named fields instead of padding bytes in the aligned PCL point.
            const int source_v = destagger ? (v + width - shifts[u]) % width : v;
            const int source = u * width + source_v;
            const bool invalid = xyz.row(source).hasNaN();
            if (!organized && invalid) continue;
            ASSERT_LT(expected_size, cloud.size());
            const auto& point = cloud.points[expected_size++];
            expected_dense &= !invalid;
            const std::array<float, 3> actual_xyz{{point.x, point.y, point.z}};
            for (int axis = 0; axis < 3; ++axis) {
                if (std::isnan(xyz(source, axis)))
                    EXPECT_TRUE(std::isnan(actual_xyz[axis]));
                else
                    EXPECT_EQ(actual_xyz[axis], xyz(source, axis));
            }
            const uint64_t stamp = timestamps[source_v];
            EXPECT_EQ(point.t, static_cast<uint32_t>(
                                   stamp > scan_ts ? stamp - scan_ts : 0));
            EXPECT_EQ(point.ring, u);
            EXPECT_EQ(point.range, range.data()[source]);
            EXPECT_EQ(point.intensity, static_cast<float>(signal.data()[source]));
            EXPECT_EQ(point.reflectivity, reflectivity.data()[source]);
            EXPECT_EQ(point.ambient, near_ir.data()[source]);
        }
    }
    EXPECT_EQ(cloud.size(), expected_size);
    EXPECT_EQ(cloud.is_dense, expected_dense);

    pcl::PCLPointCloud2 staging_message;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toPCLPointCloud2(cloud, staging_message);
    pcl_conversions::moveFromPCL(staging_message, message);
    EXPECT_EQ(message.width,
              organized ? static_cast<size_t>(width) : expected_size);
    EXPECT_EQ(message.height, organized ? height / rows_step : 1);
    EXPECT_EQ(message.is_dense, expected_dense);
    EXPECT_FALSE(message.is_bigendian);
    EXPECT_EQ(message.point_step, 48U);
    EXPECT_EQ(message.row_step, message.width * message.point_step);
    EXPECT_EQ(message.data.size(), expected_size * message.point_step);

    using Field = sensor_msgs::msg::PointField;
    const std::array<const char*, 9> names{
        {"x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"}};
    const std::array<uint32_t, 9> offsets{{0, 4, 8, 16, 20, 24, 26, 28, 32}};
    const std::array<uint8_t, 9> types{
        {Field::FLOAT32, Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
         Field::UINT32, Field::UINT16, Field::UINT16, Field::UINT16, Field::UINT32}};
    ASSERT_EQ(message.fields.size(), names.size());
    for (size_t i = 0; i < names.size(); ++i) {
        EXPECT_EQ(message.fields[i].name, names[i]);
        EXPECT_EQ(message.fields[i].offset, offsets[i]);
        EXPECT_EQ(message.fields[i].datatype, types[i]);
        EXPECT_EQ(message.fields[i].count, 1U);
    }
}

INSTANTIATE_TEST_SUITE_P(AllLayouts, OriginalCloudEquivalenceTest,
                        ::testing::ValuesIn(original_cloud_cases()));
