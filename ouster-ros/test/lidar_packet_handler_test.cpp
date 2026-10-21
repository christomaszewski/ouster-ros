// Keep os_ros.h first: it enables PCL_NO_PRECOMPILE.
#include "ouster_ros/os_ros.h"

#include <gtest/gtest.h>
#include <ouster/impl/packet_writer.h>

#include <condition_variable>
#include <cstring>
#include <mutex>

#include "../src/lidar_packet_handler.h"
#include "../src/point_cloud_processor.h"

namespace {

using namespace ouster::sdk::core;
using namespace ouster_ros;
using namespace std::chrono_literals;

constexpr uint64_t SENSOR_START_NS = 100000000000ULL;
constexpr uint64_t HOST_START_NS = 200000000000ULL;
constexpr uint64_t FRAME_PERIOD_NS = 100000000ULL;
constexpr int64_t PTP_OFFSET_NS = -37000000000LL;

uint32_t point_field(const sensor_msgs::msg::PointCloud2& cloud, size_t index,
                     const std::string& name) {
    const auto field = std::find_if(
        cloud.fields.begin(), cloud.fields.end(),
        [&name](const auto& field) { return field.name == name; });
    EXPECT_NE(field, cloud.fields.end());
    if (field == cloud.fields.end()) return 0;
    EXPECT_EQ(field->datatype, sensor_msgs::msg::PointField::UINT32);
    const auto offset = index * cloud.point_step + field->offset;
    EXPECT_LE(offset + sizeof(uint32_t), cloud.data.size());
    if (offset + sizeof(uint32_t) > cloud.data.size()) return 0;
    uint32_t result;
    std::memcpy(&result, cloud.data.data() + offset, sizeof(result));
    return result;
}

class LidarPacketHandlerTimestampTest
    : public ::testing::TestWithParam<std::string> {
   protected:
    struct Output {
        int64_t frame_id;
        uint64_t sensor_ns;
        int64_t header_ns;
        uint32_t first_point_ns;
        uint32_t last_point_ns;
        uint32_t width;
        uint32_t height;
        uint32_t first_range;
        uint32_t last_range;
    };

    SensorInfo info = default_sensor_info(LidarMode(512, 10));
    size_t last_test_column = info.format.columns_per_packet - 1;
    std::mutex mutex;
    std::condition_variable condition;
    bool consumer_entered = false;
    bool consumer_released = false;
    std::vector<Output> outputs;
    int64_t current_frame_id = -1;
    uint64_t current_sensor_ns = 0;

    // By default, one valid packet per frame leaves each scan incomplete.
    // The next frame releases it, giving ROS-time mode an unambiguous origin.
    // Complete-frame tests supply each packet's starting measurement ID.
    LidarPacket packet(uint32_t frame_id, uint32_t first_column = 0) {
        const auto& format = get_format(info);
        ouster::sdk::core::impl::PacketWriter writer(format);
        LidarPacket result(format.lidar_packet_size);
        result.format = std::make_shared<PacketFormat>(format);
        result.host_timestamp = HOST_START_NS + frame_id * FRAME_PERIOD_NS +
                                first_column * FRAME_PERIOD_NS /
                                    info.format.columns_per_frame;
        writer.set_frame_id(result.buf.data(), frame_id);
        for (int column = 0; column < format.columns_per_packet; ++column) {
            auto* data = writer.nth_col(column, result.buf.data());
            const auto measurement_id = first_column + column;
            writer.set_col_measurement_id(data, measurement_id);
            writer.set_col_status(data, 1);
            writer.set_col_timestamp(
                data, SENSOR_START_NS + frame_id * FRAME_PERIOD_NS +
                          measurement_id * FRAME_PERIOD_NS /
                              info.format.columns_per_frame);
        }
        // PacketWriter indexes this source by absolute measurement ID, so it
        // needs a full-width array even though only one packet is emitted.
        std::vector<uint32_t> ranges(
            format.pixels_per_column * info.format.columns_per_frame,
            1000 + frame_id);
        writer.set_block(ranges.data(), info.format.columns_per_frame,
                         ChanField::RANGE, result.buf.data());
        return result;
    }

    LidarPacketHandler::HandlerType handler() {
        auto gate = [this](const LidarScan& scan, uint64_t scan_ns,
                           const rclcpp::Time&) {
            std::unique_lock<std::mutex> lock(mutex);
            current_frame_id = scan.frame_id;
            current_sensor_ns = scan_ns;
            if (!consumer_entered) {
                consumer_entered = true;
                condition.notify_all();
                // Bound failure duration, including assertions in the producer.
                EXPECT_TRUE(condition.wait_for(
                    lock, 5s, [this] { return consumer_released; }));
            }
        };

        auto cloud = PointCloudProcessor<Point>::create(
            info, "os_lidar", false, 0, 1000000, 1, "",
            [](Cloud<Point>& cloud, const PointCloudXYZf& points,
               uint64_t scan_ns, const LidarScan& scan,
               const std::vector<int>& pixel_shift_by_row, int) {
                Point_LEGACY staging;
                scan_to_cloud_f<Profile_LEGACY.size(), Profile_LEGACY>(
                    cloud, staging, points, scan_ns, scan, pixel_shift_by_row,
                    true, false);
            },
            [this](PointCloudProcessor_OutputType messages) {
                ASSERT_EQ(messages.size(), 1u);
                const auto& cloud = *messages.front();
                const auto last_point =
                    (info.format.pixels_per_column - 1) * cloud.width +
                    last_test_column;
                std::lock_guard<std::mutex> lock(mutex);
                outputs.push_back(
                    {current_frame_id, current_sensor_ns,
                     rclcpp::Time(cloud.header.stamp).nanoseconds(),
                     point_field(cloud, 0, "t"),
                     point_field(cloud, last_point, "t"), cloud.width,
                     cloud.height, point_field(cloud, 0, "range"),
                     point_field(cloud, last_point, "range")});
                condition.notify_all();
            });
        return LidarPacketHandler::create(
            info, {gate, cloud}, GetParam(), PTP_OFFSET_NS, 0.0f);
    }

    bool wait_for_consumer() {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, 5s, [this] { return consumer_entered; });
    }

    void release_consumer() {
        std::lock_guard<std::mutex> lock(mutex);
        consumer_released = true;
        condition.notify_all();
    }

    bool wait_for_outputs(size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, 5s,
                                  [this, count] { return outputs.size() >= count; });
    }

    void expect_outputs(const std::vector<int64_t>& frame_ids) {
        std::lock_guard<std::mutex> lock(mutex);
        ASSERT_EQ(outputs.size(), frame_ids.size());
        for (size_t i = 0; i < frame_ids.size(); ++i) {
            SCOPED_TRACE("frame " + std::to_string(frame_ids[i]));
            const auto sensor_ns = SENSOR_START_NS + frame_ids[i] * FRAME_PERIOD_NS;
            const auto corrected_ns = GetParam() == "TIME_FROM_PTP_1588"
                                          ? sensor_ns + PTP_OFFSET_NS
                                          : sensor_ns;
            const auto header_ns = GetParam() == "TIME_FROM_ROS_TIME"
                                       ? HOST_START_NS + frame_ids[i] * FRAME_PERIOD_NS
                                       : corrected_ns;
            EXPECT_EQ(outputs[i].frame_id, frame_ids[i]);
            EXPECT_EQ(outputs[i].sensor_ns, corrected_ns);
            EXPECT_EQ(outputs[i].header_ns, static_cast<int64_t>(header_ns));
            EXPECT_EQ(outputs[i].first_point_ns, 0u);
            EXPECT_EQ(outputs[i].last_point_ns,
                      last_test_column * FRAME_PERIOD_NS /
                          info.format.columns_per_frame);
            EXPECT_EQ(outputs[i].width, info.format.columns_per_frame);
            EXPECT_EQ(outputs[i].height, info.format.pixels_per_column);
            EXPECT_EQ(outputs[i].first_range, 1000u + frame_ids[i]);
            EXPECT_EQ(outputs[i].last_range, 1000u + frame_ids[i]);
        }
    }
};

TEST_P(LidarPacketHandlerTimestampTest, QueuedScansRetainTheirOwnTimestamps) {
    auto consume = handler();
    consume(packet(0));
    consume(packet(1));  // scan A reaches the blocked first processor
    ASSERT_TRUE(wait_for_consumer());
    consume(packet(2));  // queue scan B
    consume(packet(3));  // queue scan C
    release_consumer();
    ASSERT_TRUE(wait_for_outputs(3));
    expect_outputs({0, 1, 2});
}

TEST_P(LidarPacketHandlerTimestampTest,
       TimestampsStayWithScansAcrossThrottlingDroppingAndSlotReuse) {
    auto consume = handler();
    consume(packet(0));
    consume(packet(1));
    ASSERT_TRUE(wait_for_consumer());
    for (uint32_t frame = 2; frame <= 11; ++frame) consume(packet(frame));
    // Nine queued scans fill the ring: packets 10 and 11 are dropped. Reading
    // the first scan skips scan 1 at the existing 70% throttle threshold.
    release_consumer();
    ASSERT_TRUE(wait_for_outputs(8));
    expect_outputs({0, 2, 3, 4, 5, 6, 7, 8});
    // Scan 9 was cached by ScanBatcher before overflow. New packets flush it
    // and scan 12, wrapping the queue back onto previously consumed slots.
    consume(packet(12));
    consume(packet(13));
    ASSERT_TRUE(wait_for_outputs(10));
    expect_outputs({0, 2, 3, 4, 5, 6, 7, 8, 9, 12});
}

INSTANTIATE_TEST_SUITE_P(
    ClockModes, LidarPacketHandlerTimestampTest,
    ::testing::Values("TIME_FROM_SYNC_PULSE_IN", "TIME_FROM_PTP_1588",
                      "TIME_FROM_ROS_TIME"),
    [](const auto& info) { return info.param; });

class CompleteLidarPacketHandlerTimestampTest
    : public LidarPacketHandlerTimestampTest {
   protected:
    void SetUp() override {
        // Synthetic geometry matches the larger legacy packet dimensions,
        // without embedding any hardware identity or calibration data.
        info = default_sensor_info(LidarMode(2048, 10));
        info.format.pixels_per_column = 128;
        info.format.pixel_shift_by_row.assign(128, 0);
        info.beam_altitude_angles.assign(128, 0.0);
        info.beam_azimuth_angles.assign(128, 0.0);
        last_test_column = info.format.columns_per_frame - 1;
        ASSERT_EQ(info.format.columns_per_packet, 16);
        ASSERT_EQ(get_format(info).lidar_packet_size, 24896u);
    }

    void consume_frame(const LidarPacketHandler::HandlerType& consume,
                       uint32_t frame_id) {
        for (uint32_t column = 0; column < info.format.columns_per_frame;
             column += info.format.columns_per_packet) {
            consume(packet(frame_id, column));
        }
    }
};

TEST_P(CompleteLidarPacketHandlerTimestampTest,
       Complete128BeamScansRetainTheirTimestampsAndRangesWhileQueued) {
    auto consume = handler();
    // Complete scans are released by their final packet, without a packet
    // from the following frame. Keep three such scans queued concurrently.
    consume_frame(consume, 0);
    ASSERT_TRUE(wait_for_consumer());
    consume_frame(consume, 1);
    consume_frame(consume, 2);
    release_consumer();
    ASSERT_TRUE(wait_for_outputs(3));
    expect_outputs({0, 1, 2});
}

// Complete-frame ROS_TIME origin estimation is a separate upstream issue;
// these cases verify ownership of sensor time and PTP-adjusted time only.
INSTANTIATE_TEST_SUITE_P(
    ClockModes, CompleteLidarPacketHandlerTimestampTest,
    ::testing::Values("TIME_FROM_INTERNAL_OSC", "TIME_FROM_PTP_1588"),
    [](const auto& info) { return info.param; });

}  // namespace
