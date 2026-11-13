#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <fstream>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ouster_ros/os_ros.h"
#include "../src/point_cloud_processor_factory.h"

using namespace ouster_ros;
using namespace ouster::sdk::core;

namespace {
constexpr int WIDTH = 16;
constexpr int HEIGHT = 8;
constexpr uint64_t SCAN_TS = 1000000000000ULL;

SensorInfo test_info(UDPProfileLidar profile) {
    auto info = default_sensor_info(LidarMode::_512x10);
    info.format.columns_per_frame = WIDTH;
    info.format.pixels_per_column = HEIGHT;
    info.format.udp_profile_lidar = profile;
    info.format.pixel_shift_by_row = {-16, -1, 0, 1, 15, -9, 8, 3};
    info.beam_azimuth_angles.resize(HEIGHT);
    info.beam_altitude_angles.resize(HEIGHT);
    info.lidar_to_sensor_transform = mat4d::Identity();
    info.lidar_to_sensor_transform(0, 0) = 0;
    info.lidar_to_sensor_transform(1, 1) = 0;
    info.lidar_to_sensor_transform(0, 1) = -1;
    info.lidar_to_sensor_transform(1, 0) = 1;
    info.lidar_to_sensor_transform(0, 3) = 120;
    info.lidar_to_sensor_transform(2, 3) = -35;
    return info;
}

template <typename T>
void fill_field(LidarScan& scan, const std::string& field, int frame) {
    auto values = scan.field<T>(field);
    for (int i = 0; i < values.size(); ++i)
        values.data()[i] = static_cast<T>(1000 + 131 * i + 29 * frame +
                                         11 * field.size());
}

void fill_scan(LidarScan& scan, int frame) {
    for (const auto& field : scan.field_types()) {
        switch (field.element_type) {
            case ChanFieldType::UINT8: fill_field<uint8_t>(scan, field.name, frame); break;
            case ChanFieldType::UINT16: fill_field<uint16_t>(scan, field.name, frame); break;
            case ChanFieldType::UINT32: fill_field<uint32_t>(scan, field.name, frame); break;
            case ChanFieldType::UINT64: fill_field<uint64_t>(scan, field.name, frame); break;
            default: FAIL() << "unexpected test field type";
        }
    }
    for (const auto& name : {ChanField::RANGE, ChanField::RANGE2}) {
        if (!scan.has_field(name)) continue;
        auto range = scan.field<uint32_t>(name);
        for (int i = 0; i < range.size(); ++i)
            range.data()[i] = (i + frame) % 5 == 0 ? 0 :
                             1000 + 1000 * ((i + 3 * frame) % 20);
    }
    for (int col = 0; col < WIDTH; ++col)
        scan.timestamp()[col] = SCAN_TS + 1000 * col + frame;
    scan.timestamp()[0] = SCAN_TS - 1;
    scan.timestamp()[1] = SCAN_TS + uint64_t{UINT32_MAX} + 123;
}

template <typename PointT>
void expect_same_fields(const sensor_msgs::msg::PointCloud2& expected,
                        const sensor_msgs::msg::PointCloud2& actual) {
    ASSERT_EQ(expected.header, actual.header);
    ASSERT_EQ(expected.fields, actual.fields);
    ASSERT_EQ(expected.height, actual.height);
    ASSERT_EQ(expected.width, actual.width);
    ASSERT_EQ(expected.point_step, actual.point_step);
    ASSERT_EQ(expected.row_step, actual.row_step);
    ASSERT_EQ(expected.is_bigendian, actual.is_bigendian);
    ASSERT_EQ(expected.is_dense, actual.is_dense);
    ASSERT_EQ(expected.data.size(), actual.data.size());
    // Compare named field bytes, including NaN payloads and packed RGB, plus
    // PCL's initialized fourth XYZ float. True alignment padding is unspecified.
    const size_t scalar_sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
    for (size_t point = 0; point < expected.width * expected.height; ++point) {
        const auto fourth_float = point * expected.point_step + 12;
        ASSERT_EQ(0, std::memcmp(expected.data.data() + fourth_float,
                                 actual.data.data() + fourth_float, 4));
        for (const auto& field : expected.fields) {
            ASSERT_LT(field.datatype, std::size(scalar_sizes));
            const auto offset = point * expected.point_step + field.offset;
            ASSERT_EQ(0, std::memcmp(expected.data.data() + offset,
                                     actual.data.data() + offset,
                                     scalar_sizes[field.datatype] * field.count))
                << "point=" << point << " field=" << field.name;
        }
    }
    // PCL coalesces adjacent mappings, including gaps. Preserve its initialized
    // homogeneous-coordinate slot through a real ROS -> PCL roundtrip too.
    Cloud<PointT> expected_pcl, actual_pcl;
    pcl::fromROSMsg(expected, expected_pcl);
    pcl::fromROSMsg(actual, actual_pcl);
    ASSERT_EQ(expected_pcl.size(), actual_pcl.size());
    for (size_t point = 0; point < expected_pcl.size(); ++point)
        ASSERT_EQ(0, std::memcmp(
            reinterpret_cast<const uint8_t*>(&expected_pcl[point]) + 12,
            reinterpret_cast<const uint8_t*>(&actual_pcl[point]) + 12, 4));
}

class MaskFile {
   public:
    MaskFile() : path(testing::TempDir() + "cloud_mask_" +
                     std::to_string(::getpid()) + ".pgm") {
        std::ofstream file(path, std::ios::binary);
        file << "P5\n" << WIDTH << " " << HEIGHT << "\n255\n";
        for (int i = 0; i < WIDTH * HEIGHT; ++i)
            file.put(i % 7 == 0 ? 0 : 255);
    }
    ~MaskFile() { std::remove(path.c_str()); }
    std::string path;
};

template <typename PointT, typename NativeT, auto& Profile, auto& SecondProfile = Profile>
void compare_processors(const std::string& point_type, UDPProfileLidar profile) {
    const auto info = test_info(profile);
    LidarScan scan(WIDTH, HEIGHT, profile);
    if (profile == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 ||
        profile == UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL) {
        // The packet handler supplies these exposed color channels before
        // invoking cloud processors; raw SDK scans do not contain them.
        for (const auto& name : {ChanField::R_U8, ChanField::G_U8, ChanField::B_U8})
            scan.add_field(name, fd_array<uint8_t>(HEIGHT, WIDTH));
    }
    MaskFile mask;
    for (bool organized : {false, true})
        for (bool destagger : {false, true})
            for (int rows_step : {1, 2, 4, 8})
                for (bool filtered : {false, true}) {
                    SCOPED_TRACE(point_type + " profile=" + to_string(profile) +
                                 " organized=" + std::to_string(organized) +
                                 " destagger=" + std::to_string(destagger) +
                                 " rows=" + std::to_string(rows_step) +
                                 " filtered=" + std::to_string(filtered));
                    PointCloudProcessor_OutputType expected;
                    PointCloudProcessor_OutputType actual;
                    auto reference = PointCloudProcessor<PointT>::create(
                        info, "os_sensor", filtered, filtered ? 5000 : 0,
                        filtered ? 15000 : UINT32_MAX, rows_step,
                        filtered ? mask.path : "",
                        [=](Cloud<PointT>& cloud, const PointCloudXYZf& xyz,
                            uint64_t stamp, const LidarScan& ls,
                            const std::vector<int>& shifts, int return_index) {
                            NativeT point;
                            if (return_index == 0)
                                scan_to_cloud_f<Profile.size(), Profile>(
                                    cloud, point, xyz, stamp, ls, shifts,
                                    organized, destagger, rows_step);
                            else
                                scan_to_cloud_f<SecondProfile.size(), SecondProfile>(
                                    cloud, point, xyz, stamp, ls, shifts,
                                    organized, destagger, rows_step);
                        },
                        [&](auto messages) { expected = std::move(messages); });
                    auto candidate = PointCloudProcessorFactory::create_point_cloud_processor(
                        point_type, info, "os_sensor", filtered, organized,
                        destagger, filtered ? 5000 : 0,
                        filtered ? 15000 : UINT32_MAX, rows_step,
                        filtered ? mask.path : "",
                        [&](auto messages) { actual = std::move(messages); });
                    for (int frame : {0, 3, 1}) {
                        fill_scan(scan, frame);
                        const auto stamp = rclcpp::Time(SCAN_TS + frame);
                        reference(scan, SCAN_TS, stamp);
                        candidate(scan, SCAN_TS, stamp);
                        ASSERT_EQ(expected.size(), info.num_returns());
                        ASSERT_EQ(actual.size(), expected.size());
                        for (size_t i = 0; i < actual.size(); ++i)
                            expect_same_fields<PointT>(*expected[i], *actual[i]);
                        // Release callbacks' ownership before the next frame,
                        // exercising capacity reuse across changed validity.
                        expected.clear();
                        actual.clear();
                    }
                }
}

template <typename NativeT, auto& Profile, auto& SecondProfile = Profile>
void compare_all_point_types(UDPProfileLidar profile) {
    compare_processors<Point, NativeT, Profile, SecondProfile>("original", profile);
    compare_processors<pcl::PointXYZ, NativeT, Profile, SecondProfile>("xyz", profile);
    compare_processors<pcl::PointXYZI, NativeT, Profile, SecondProfile>("xyzi", profile);
    compare_processors<PointXYZI, NativeT, Profile, SecondProfile>("o_xyzi", profile);
    compare_processors<PointXYZIR, NativeT, Profile, SecondProfile>("xyzir", profile);
    compare_processors<pcl::PointXYZRGB, NativeT, Profile, SecondProfile>("xyzrgb", profile);
    compare_processors<pcl::PointXYZRGBA, NativeT, Profile, SecondProfile>("xyzrgba", profile);
    compare_processors<ColorPoint, NativeT, Profile, SecondProfile>("color_point", profile);
    compare_processors<NativeT, NativeT, Profile, SecondProfile>("native", profile);
}
}  // namespace

TEST(PointCloud2Builder, LegacyAllPointTypesAndLayouts) {
    compare_all_point_types<Point_LEGACY, Profile_LEGACY>(UDPProfileLidar::LEGACY);
}

TEST(PointCloud2Builder, DualReturnAllPointTypesAndLayouts) {
    compare_all_point_types<Point_RNG19_RFL8_SIG16_NIR16_DUAL,
                           Profile_RNG19_RFL8_SIG16_NIR16_DUAL,
                           Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN>(
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
}

TEST(PointCloud2Builder, RgbDualReturnAllPointTypesAndLayouts) {
    compare_all_point_types<Point_RNG19_RFL8_SIG16_NIR16_RGB16_DUAL,
                           Profile_RNG19_RFL8_SIG16_NIR16_RGB16_DUAL,
                           Profile_RNG19_RFL8_SIG16_NIR16_RGB16_DUAL_2ND_RETURN>(
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL);
}

TEST(PointCloud2Builder, RemainingNativeProfilesAndLayouts) {
    compare_processors<Point_RNG19_RFL8_SIG16_NIR16,
                       Point_RNG19_RFL8_SIG16_NIR16, Profile_RNG19_RFL8_SIG16_NIR16>(
        "native", UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    compare_processors<Point_RNG15_RFL8_NIR8, Point_RNG15_RFL8_NIR8,
                       Profile_RNG15_RFL8_NIR8>("native", UDPProfileLidar::RNG15_RFL8_NIR8);
    compare_processors<Point_RNG15_RFL8_NIR8_DUAL, Point_RNG15_RFL8_NIR8_DUAL,
                       Profile_RNG15_RFL8_NIR8_DUAL,
                       Profile_RNG15_RFL8_NIR8_DUAL_2ND_RETURN>(
        "native", UDPProfileLidar::RNG15_RFL8_NIR8_DUAL);
    compare_processors<Point_RNG15_RFL8_NIR8_DUAL, Point_RNG15_RFL8_NIR8_DUAL,
                       Profile_RNG15_RFL8_NIR8_DUAL,
                       Profile_RNG15_RFL8_NIR8_DUAL_2ND_RETURN>(
        "native", UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL);
    compare_processors<Point_RNG15_RFL8_WIN8, Point_RNG15_RFL8_WIN8,
                       Profile_RNG15_RFL8_WIN8>("native", UDPProfileLidar::RNG15_RFL8_WIN8);
    compare_processors<Point_RNG15_RFL8_NIR8_ZONE16, Point_RNG15_RFL8_NIR8_ZONE16,
                       Profile_RNG15_RFL8_NIR8_ZONE16>(
        "native", UDPProfileLidar::RNG15_RFL8_NIR8_ZONE16);
    compare_processors<Point_RNG19_RFL8_SIG16_NIR16_ZONE16,
                       Point_RNG19_RFL8_SIG16_NIR16_ZONE16,
                       Profile_RNG19_RFL8_SIG16_NIR16_ZONE16>(
        "native", UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_ZONE16);
    compare_processors<Point_RNG19_RFL8_SIG16_NIR16_RGB16,
                       Point_RNG19_RFL8_SIG16_NIR16_RGB16,
                       Profile_RNG19_RFL8_SIG16_NIR16_RGB16>(
        "native", UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16);
}

TEST(PointCloud2Builder, RetainedMessagesAndSerializedPublicationStayUnchanged) {
    const auto info = test_info(UDPProfileLidar::LEGACY);
    LidarScan scan(WIDTH, HEIGHT, UDPProfileLidar::LEGACY);
    PointCloudProcessor_OutputType retained;
    auto processor = PointCloudProcessorFactory::create_point_cloud_processor(
        "original", info, "os_sensor", true, true, true, 0, UINT32_MAX, 1, "",
        [&](auto messages) { retained.push_back(messages.front()); });
    fill_scan(scan, 0);
    processor(scan, SCAN_TS, rclcpp::Time(SCAN_TS));
    const auto snapshot = *retained.front();
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    rclcpp::SerializedMessage serialized;
    serialization.serialize_message(retained.front().get(), &serialized);
    for (int frame = 1; frame < 4; ++frame) {
        fill_scan(scan, frame);
        processor(scan, SCAN_TS, rclcpp::Time(SCAN_TS + frame));
        EXPECT_NE(retained[frame].get(), retained[frame - 1].get());
        EXPECT_NE(retained[frame]->data.data(), retained[frame - 1]->data.data());
        EXPECT_EQ(snapshot, *retained.front());
    }
    sensor_msgs::msg::PointCloud2 deserialized;
    serialization.deserialize_message(&serialized, &deserialized);
    EXPECT_EQ(snapshot, deserialized);
}

TEST(PointCloud2Builder, ExclusiveMessageReusesStorageAndHandlesEmptyFrames) {
    const auto info = test_info(UDPProfileLidar::LEGACY);
    for (bool organized : {false, true}) {
        LidarScan scan(WIDTH, HEIGHT, UDPProfileLidar::LEGACY);
        const uint8_t* first_data = nullptr;
        size_t previous_size = 0;
        auto processor = PointCloudProcessorFactory::create_point_cloud_processor(
            "original", info, "os_sensor", false, organized, true, 0, UINT32_MAX, 1, "",
            [&](auto messages) {
                const auto& message = *messages.front();
                if (!first_data) first_data = message.data.data();
                EXPECT_EQ(first_data, message.data.data());
                EXPECT_EQ(message.point_step, 48u);
                previous_size = message.data.size();
            });
        for (int frame : {0, 1, 2}) {
            fill_scan(scan, frame);
            processor(scan, SCAN_TS, rclcpp::Time(SCAN_TS + frame));
        }
        scan.field<uint32_t>(ChanField::RANGE).setZero();
        processor(scan, SCAN_TS, rclcpp::Time(SCAN_TS));
        EXPECT_EQ(previous_size, organized ? WIDTH * HEIGHT * 48u : 0u);
        fill_scan(scan, 3);
        processor(scan, SCAN_TS, rclcpp::Time(SCAN_TS + 3));
    }
}

TEST(PointCloud2Builder, RejectsInvalidDimensionsAndRowStepsBeforeAllocation) {
    auto info = test_info(UDPProfileLidar::LEGACY);
    const auto create = [&](int rows_step) {
        return PointCloudProcessorFactory::create_point_cloud_processor(
            "original", info, "os_sensor", false, true, true, 0, UINT32_MAX,
            rows_step, "", [](auto) {});
    };
    EXPECT_THROW(create(0), std::invalid_argument);
    EXPECT_THROW(create(-1), std::invalid_argument);
    EXPECT_THROW(create(3), std::invalid_argument);
    EXPECT_THROW(create(16), std::invalid_argument);
    EXPECT_NO_THROW(create(8));
    info.format.pixels_per_column = 0;
    EXPECT_THROW(create(1), std::invalid_argument);
    info = test_info(UDPProfileLidar::LEGACY);
    info.format.columns_per_frame = 0;
    EXPECT_THROW(create(1), std::invalid_argument);
}
