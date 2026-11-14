// Frozen exact ROS53b2b4 LaserScan converter; only function name changed.
#pragma once
#include "ouster_ros/os_ros.h"
namespace ouster_ros {
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::LidarMode;
namespace ChanField = ouster::sdk::core::ChanField;
// TODO: provide a method that accepts sensor_msgs::msg::LaserScan object
sensor_msgs::msg::LaserScan frozen_lidar_scan_to_laser_scan_msg(
    const LidarScan& ls, const rclcpp::Time& timestamp,
    const std::string& frame, const LidarMode ld_mode,
    const uint16_t ring, const std::vector<int>& pixel_shift_by_row,
    const int return_index) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame;
    msg.angle_min = -M_PI;   // TODO: configure to match the actual scan window
    msg.angle_max = M_PI;    // TODO: configure to match the actual scan window
    msg.range_min = 0.1f;    // TODO: fill per product type
    msg.range_max = 120.0f;  // TODO: fill per product type

    const auto scan_width = ouster::sdk::core::n_cols_of_lidar_mode(ld_mode);
    const auto scan_frequency = ouster::sdk::core::frequency_of_lidar_mode(ld_mode);
    msg.scan_time = 1.0f / scan_frequency;
    msg.time_increment = 1.0f / (scan_width * scan_frequency);
    msg.angle_increment = 2 * M_PI / scan_width;

    auto which_range = return_index == 0 ? ChanField::RANGE
                                         : ChanField::RANGE2;
    ouster::sdk::core::img_t<uint32_t> range = ls.field<uint32_t>(which_range);
    auto which_signal = return_index == 0 ? ChanField::SIGNAL
                                          : ChanField::SIGNAL2;
    ouster::sdk::core::img_t<uint32_t> signal =
        impl::get_or_fill_zero<uint32_t>(which_signal, ls);
    const auto rg = range.data();
    const auto sg = signal.data();
    msg.ranges.resize(ls.w);
    msg.intensities.resize(ls.w);

    uint16_t u = ring;
    for (int v =  0; v < static_cast<int>(ls.w); ++v) {
        auto v_shift = (v + ls.w - pixel_shift_by_row[u] + ls.w / 2) % ls.w;
        auto src_idx = u * ls.w + v_shift;
        auto tgt_idx = ls.w - 1 - v;
        msg.ranges[tgt_idx] = rg[src_idx] * ouster::sdk::core::RANGE_UNIT;
        msg.intensities[tgt_idx] = static_cast<float>(sg[src_idx]);
    }

    return msg;
}

}  // namespace ouster_ros
