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

uint32_t point_time(const sensor_msgs::msg::PointCloud2& cloud, size_t column) {
    const auto field = std::find_if(
        cloud.fields.begin(), cloud.fields.end(),
        [](const auto& field) { return field.name == "t"; });
    EXPECT_NE(field, cloud.fields.end());
    if (field == cloud.fields.end()) return 0;
    uint32_t result;
    std::memcpy(&result,
                cloud.data.data() + column * cloud.point_step + field->offset,
                sizeof(result));
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
    };

    SensorInfo info = default_sensor_info(LidarMode(512, 10));
    std::mutex mutex;
    std::condition_variable condition;
    bool consumer_entered = false;
    bool consumer_released = false;
    std::vector<Output> outputs;
    int64_t current_frame_id = -1;
    uint64_t current_sensor_ns = 0;

    // One valid packet per frame deliberately leaves each scan incomplete.
    // The next frame's packet releases it, exercising the real ScanBatcher
    // boundary path and giving ROS-time mode an unambiguous next-frame origin.
    LidarPacket packet(uint32_t frame_id) {
        const auto& format = get_format(info);
        ouster::sdk::core::impl::PacketWriter writer(format);
        LidarPacket result(format.lidar_packet_size);
        result.format = std::make_shared<PacketFormat>(format);
        result.host_timestamp = HOST_START_NS + frame_id * FRAME_PERIOD_NS;
        writer.set_frame_id(result.buf.data(), frame_id);
        for (int column = 0; column < format.columns_per_packet; ++column) {
            auto* data = writer.nth_col(column, result.buf.data());
            writer.set_col_measurement_id(data, column);
            writer.set_col_status(data, 1);
            writer.set_col_timestamp(
                data, SENSOR_START_NS + frame_id * FRAME_PERIOD_NS +
                          column * FRAME_PERIOD_NS / info.format.columns_per_frame);
        }
        std::vector<uint32_t> ranges(
            format.pixels_per_column * format.columns_per_packet, 1000);
        writer.set_block(ranges.data(), format.columns_per_packet,
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
                std::lock_guard<std::mutex> lock(mutex);
                outputs.push_back(
                    {current_frame_id, current_sensor_ns,
                     rclcpp::Time(cloud.header.stamp).nanoseconds(),
                     point_time(cloud, 0),
                     point_time(cloud, info.format.columns_per_packet - 1)});
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
                      (info.format.columns_per_packet - 1) * FRAME_PERIOD_NS /
                          info.format.columns_per_frame);
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

}  // namespace
