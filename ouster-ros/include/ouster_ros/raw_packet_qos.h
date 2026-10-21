#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace ouster_ros {

// A packet is only part of a scan: dropping the previous sample is not a useful
// default here. Keep transport policy independent of processed cloud/image QoS.
template <typename NodeT>
rclcpp::QoS raw_packet_qos(const NodeT& node) {
    const auto depth = node.get_parameter("raw_packet_qos_depth").as_int();
    const auto reliability =
        node.get_parameter("raw_packet_qos_reliability").as_string();
    if (depth < 1 || depth > 65536) {
        throw std::invalid_argument(
            "raw_packet_qos_depth must be between 1 and 65536 packets");
    }
    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
    qos.durability_volatile();
    if (reliability == "reliable") {
        qos.reliable();
    } else if (reliability == "best_effort") {
        qos.best_effort();
    } else {
        throw std::invalid_argument(
            "raw_packet_qos_reliability must be reliable or best_effort");
    }
    return qos;
}

template <typename NodeT>
void declare_raw_packet_qos_parameters(NodeT& node) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description =
        "Raw packet KeepLast depth (1..65536 packets); restart to change";
    node.template declare_parameter<int64_t>("raw_packet_qos_depth", 1024,
                                              descriptor);
    descriptor.description =
        "Raw packet reliability: reliable or best_effort; restart to change";
    node.template declare_parameter<std::string>(
        "raw_packet_qos_reliability", "reliable", descriptor);
    // Fail before a lifecycle configure/activate can change a physical sensor.
    (void)raw_packet_qos(node);
}

}  // namespace ouster_ros
