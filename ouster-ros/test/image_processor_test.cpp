#include <gtest/gtest.h>

#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <thread>
#include <tuple>

#include "../src/image_processor.h"
#include "legacy_image_processor.h"

namespace {
using namespace ouster::sdk::core;
using ouster_ros::ImageProcessor;
using ouster_ros::LegacyImageProcessor;
using Images = std::map<std::string, sensor_msgs::msg::Image>;
using Image = sensor_msgs::msg::Image;

auto collect(Images& images) {
    return [&images](ImageProcessor::OutputType output) {
        images.clear();
        for (const auto& entry : output) images.emplace(entry.first, *entry.second);
    };
}

void expect_image(const Image& actual, const Image& expected) {
    EXPECT_EQ(actual.header, expected.header);
    EXPECT_EQ(actual.width, expected.width);
    EXPECT_EQ(actual.height, expected.height);
    EXPECT_EQ(actual.encoding, expected.encoding);
    EXPECT_EQ(actual.is_bigendian, expected.is_bigendian);
    EXPECT_EQ(actual.step, expected.step);
    EXPECT_EQ(actual.data, expected.data);
}

SensorInfo image_info(UDPProfileLidar profile, int width = 64) {
    auto info = default_sensor_info(LidarMode::_1024x10);
    info.format.udp_profile_lidar = profile;
    info.format.columns_per_frame = width;
    info.format.pixels_per_column = 32;
    const std::array<int, 6> shifts{{-width, -1, 0, width - 1, width / 2, -width + 1}};
    info.format.pixel_shift_by_row.resize(32);
    for (size_t u = 0; u < 32; ++u)
        info.format.pixel_shift_by_row[u] = shifts[u % shifts.size()];
    return info;
}

template <typename T>
void fill_field(LidarScan& scan, const std::string& name, int frame, int seed) {
    auto field = scan.field<T>(name);
    for (Eigen::Index i = 0; i < field.size(); ++i) {
        // Include zeros, row bias for BUC, changing distributions for AE, and
        // values that exercise the exact integer casts in the old processor.
        uint32_t value = (i % 13 == 0) ? 0 :
            static_cast<uint32_t>(1 + (i * (seed + frame + 3) +
                                      (i / scan.w) * 19 + frame * 113) % 120000);
        if (name == ChanField::RANGE || name == ChanField::RANGE2) {
            if (i % 17 == 0) value = 262143;
            if (i % 19 == 0) value = UINT32_MAX;
        }
        field.data()[i] = static_cast<T>(value);
    }
}

LidarScan image_scan(const SensorInfo& info, int frame) {
    LidarScan scan(info.format.columns_per_frame, info.format.pixels_per_column,
                   info.format.udp_profile_lidar);
    const std::vector<std::string> fields{
        ChanField::RANGE, ChanField::SIGNAL, ChanField::REFLECTIVITY,
        ChanField::NEAR_IR, ChanField::RANGE2, ChanField::SIGNAL2,
        ChanField::REFLECTIVITY2};
    for (size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        if (!scan.has_field(field)) continue;
        switch (scan.field_type(field).element_type) {
            case ChanFieldType::UINT8: fill_field<uint8_t>(scan, field, frame, i); break;
            case ChanFieldType::UINT16: fill_field<uint16_t>(scan, field, frame, i); break;
            case ChanFieldType::UINT32: fill_field<uint32_t>(scan, field, frame, i); break;
            default: throw std::runtime_error("Unexpected fixture channel type");
        }
    }
    // The packet handler supplies these tone-mapped channels to ImageProcessor.
    for (const auto& channel : {ChanField::R_U8, ChanField::G_U8, ChanField::B_U8}) {
        scan.add_field(FieldType(channel, ChanFieldType::UINT8));
        fill_field<uint8_t>(scan, channel, frame,
                           channel == ChanField::R_U8 ? 3 :
                           channel == ChanField::G_U8 ? 7 : 11);
    }
    return scan;
}

class MaskFile {
   public:
    explicit MaskFile(const SensorInfo& info, bool enabled) {
        if (!enabled) return;
        char pattern[] = "/tmp/ouster_image_XXXXXX.pgm";
        const int fd = mkstemps(pattern, 4);
        if (fd < 0) throw std::runtime_error("Unable to create image mask");
        close(fd);
        path = pattern;
        std::ofstream out(path, std::ios::binary);
        out << "P5\n" << info.format.columns_per_frame << " "
            << info.format.pixels_per_column << "\n255\n";
        for (size_t u = 0; u < info.format.pixels_per_column; ++u)
            for (size_t v = 0; v < info.format.columns_per_frame; ++v)
                out.put((u * 3 + v) % 7 == 0 ? 0 : 255);
    }
    ~MaskFile() { if (!path.empty()) std::remove(path.c_str()); }
    std::string path;
};

using ImageCase = std::tuple<UDPProfileLidar, int, bool>;
std::vector<ImageCase> image_cases() {
    std::vector<ImageCase> result;
    for (auto profile : {UDPProfileLidar::LEGACY,
                         UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
                         UDPProfileLidar::RNG15_RFL8_NIR8,
                         UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16,
                         UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL})
        for (int width : {17, 64})
            for (bool mask : {false, true}) result.emplace_back(profile, width, mask);
    return result;
}

class ImageEquivalenceTest : public ::testing::TestWithParam<ImageCase> {};

TEST_P(ImageEquivalenceTest, AllDemandedImagesMatchOldBytesOverMultipleFrames) {
    const auto [profile, width, masked] = GetParam();
    const auto info = image_info(profile, width);
    MaskFile mask(info, masked);
    Images expected, actual;
    auto old = LegacyImageProcessor::create(info, "test_frame", mask.path, collect(expected));
    auto current = ImageProcessor::create(
        info, "test_frame", mask.path, collect(actual),
        [](const std::string&) { return true; });
    for (int frame = 0; frame < 18; ++frame) {
        SCOPED_TRACE(frame);
        auto scan = image_scan(info, frame);
        const rclcpp::Time stamp(1000000000LL + frame * 100000000LL);
        old(scan, 0, stamp);
        current(scan, 0, stamp);
        ASSERT_EQ(actual.size(), expected.size());
        for (const auto& entry : expected) {
            SCOPED_TRACE(entry.first);
            ASSERT_EQ(actual.count(entry.first), 1U);
            expect_image(actual.at(entry.first), entry.second);
        }
    }
}

INSTANTIATE_TEST_SUITE_P(ProfilesAndMasks, ImageEquivalenceTest,
                        ::testing::ValuesIn(image_cases()));

TEST(ImageDemandTest, ZeroDemandAndUnavailableRgbNeverReadScanFields) {
    const auto info = image_info(UDPProfileLidar::LEGACY);
    int callbacks = 0;
    int queries = 0;
    auto processor = ImageProcessor::create(
        info, "frame", "", [&](auto) { ++callbacks; },
        [&](const std::string& field) { ++queries; return field == ChanField::RGB; });
    // An invalid empty scan would throw if the processor fetched any fields.
    EXPECT_NO_THROW(processor(LidarScan{}, 0, rclcpp::Time(0)));
    EXPECT_EQ(callbacks, 0);
    EXPECT_EQ(queries, 4);  // One demand snapshot, only supported channels.
}

TEST(ImageDemandTest, SecondReturnOnlyUsesFirstReturnExposureHistory) {
    const auto info = image_info(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    const std::set<std::string> wanted{
        ChanField::RANGE2, ChanField::SIGNAL2, ChanField::REFLECTIVITY2};
    Images expected, actual;
    auto old = LegacyImageProcessor::create(info, "frame", "", collect(expected));
    auto current = ImageProcessor::create(info, "frame", "", collect(actual),
        [&](const std::string& channel) { return wanted.count(channel) != 0; });
    for (int frame = 0; frame < 18; ++frame) {
        SCOPED_TRACE(frame);
        auto scan = image_scan(info, frame);
        old(scan, 0, rclcpp::Time(frame));
        current(scan, 0, rclcpp::Time(frame));
        ASSERT_EQ(actual.size(), wanted.size());
        for (const auto& field : wanted) expect_image(actual.at(field), expected.at(field));
    }
}

TEST(ImageDemandTest, SignalOnlyDoesNotReadRangeOrOtherChannels) {
    const auto info = image_info(UDPProfileLidar::LEGACY);
    LidarScan scan(std::make_shared<SensorInfo>(info),
                   std::vector<FieldType>{{ChanField::SIGNAL, ChanFieldType::UINT16}});
    scan.field<uint16_t>(ChanField::SIGNAL).setConstant(123);
    Images actual;
    auto current = ImageProcessor::create(info, "frame", "", collect(actual),
        [](const std::string& field) { return field == ChanField::SIGNAL; });
    EXPECT_NO_THROW(current(scan, 0, rclcpp::Time(0)));
    ASSERT_EQ(actual.size(), 1U);
    EXPECT_EQ(actual.count(ChanField::SIGNAL), 1U);
}

TEST(ImageDemandTest, PausesEachUnusedFamilyAndResumesWithoutReset) {
    const auto info = image_info(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    std::set<std::string> wanted;
    Images actual, signal, reflectivity, nearir;
    auto current = ImageProcessor::create(info, "frame", "", collect(actual),
        [&](const std::string& field) { return wanted.count(field) != 0; });
    auto signal_reference = LegacyImageProcessor::create(info, "frame", "", collect(signal));
    auto reflectivity_reference = LegacyImageProcessor::create(info, "frame", "", collect(reflectivity));
    auto nearir_reference = LegacyImageProcessor::create(info, "frame", "", collect(nearir));
    for (int frame = 0; frame < 24; ++frame) {
        SCOPED_TRACE(frame);
        wanted.clear();
        if (frame % 6 < 2) wanted = {ChanField::SIGNAL, ChanField::SIGNAL2};
        if (frame % 6 == 2) wanted = {ChanField::NEAR_IR};
        if (frame % 6 == 3) wanted = {ChanField::SIGNAL2, ChanField::REFLECTIVITY2};
        if (frame % 6 == 5) wanted = {ChanField::SIGNAL, ChanField::REFLECTIVITY,
                                     ChanField::NEAR_IR};
        auto scan = image_scan(info, frame);
        const rclcpp::Time stamp(frame);
        if (wanted.count(ChanField::SIGNAL) || wanted.count(ChanField::SIGNAL2))
            signal_reference(scan, 0, stamp);
        if (wanted.count(ChanField::REFLECTIVITY) || wanted.count(ChanField::REFLECTIVITY2))
            reflectivity_reference(scan, 0, stamp);
        if (wanted.count(ChanField::NEAR_IR)) nearir_reference(scan, 0, stamp);
        actual.clear();
        current(scan, 0, stamp);
        ASSERT_EQ(actual.size(), wanted.size());
        for (const auto& field : wanted) {
            const auto& reference = field == ChanField::NEAR_IR ? nearir :
                (field == ChanField::SIGNAL || field == ChanField::SIGNAL2 ? signal : reflectivity);
            expect_image(actual.at(field), reference.at(field));
        }
    }
}

class ImageSubscriberTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ImageSubscriberTest, OrdinaryAndIntraProcessSubscribersAndOverride) {
    auto publisher_node = std::make_shared<rclcpp::Node>(
        "image_demand_publisher", rclcpp::NodeOptions().use_intra_process_comms(true));
    auto external_node = std::make_shared<rclcpp::Node>(
        "image_demand_external", rclcpp::NodeOptions().use_intra_process_comms(false));
    auto publisher = publisher_node->create_publisher<Image>("image_demand_test", rclcpp::SensorDataQoS());
    std::map<std::string, decltype(publisher)> publishers{{ChanField::RANGE, publisher}};
    auto demand = ImageProcessor::subscriber_demand(publishers, false);
    auto always = ImageProcessor::subscriber_demand(publishers, true);
    EXPECT_FALSE(demand(ChanField::RANGE));
    EXPECT_FALSE(demand(ChanField::RGB));
    EXPECT_TRUE(always(ChanField::RANGE));

    auto local = publisher_node->create_subscription<Image>(
        "image_demand_test", rclcpp::SensorDataQoS(), [](Image::ConstSharedPtr) {});
    EXPECT_EQ(publisher->get_intra_process_subscription_count(), 1U);
    EXPECT_TRUE(demand(ChanField::RANGE));
    local.reset();
    auto external = external_node->create_subscription<Image>(
        "image_demand_test", rclcpp::SensorDataQoS(), [](Image::ConstSharedPtr) {});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(publisher->get_intra_process_subscription_count(), 0U);
    EXPECT_GT(publisher->get_subscription_count(), 0U);
    EXPECT_TRUE(demand(ChanField::RANGE));
}
}  // namespace
